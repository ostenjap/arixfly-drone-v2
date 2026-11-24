"""Learned motion model + Kalman filter for the HS420, fitted on recorded flights (OFFLINE ONLY: no drone, no Wi-Fi).

What it does
  build : per flight folder, reconstruct per-frame tag pose (range/along, sideways, height; IPPE ambiguity resolved
          with flight_review's Viterbi) + visual-odometry heading; cached in captures/motion_model/cache/<folder>.json
  fit   : per air segment, line up those positions with the sticks actually sent (logged bytes + trim from events.csv)
          and fit a first-order model per body axis:  dv/dt = (gain * stick(t - delay) + drift - v) / tau
          (forward <- pitch, right <- roll, up <- throttle) and  yaw rate = gain * yaw(t - delay) + own turn.
          Leave-one-flight-out refits give the +- uncertainty. Writes calib/motion_model.json.
  eval  : Kalman filter (position + velocity per axis in the tag frame, heading + rate + VO offset) with that model
          as predict and tag sightings as update. Hides the tag for 0.5/1/2/3 s at many points of held-out flights
          (model fitted without that flight) and scores the predicted position vs the true sighting after the gap,
          against two naive baselines: last-seen position, constant velocity. Plots + table in captures/motion_model/.
Frames used: tag 2 (bottom tag) only; the top tag is not used here.

Coordinates (tag frame, gravity aligned, from flight_review.pose_candidates):
  side  = cm right of the tag normal (as seen from the drone facing the tag), along = cm out along the normal,
  height = cm camera above the tag centre.  psi = drone heading, 0 = facing the tag along its normal, + = right.
  forward unit vector (side, along) = (sin psi, -cos psi), right unit vector = (cos psi, sin psi).

Usage:
  python tools/motion_model.py build captures/flights/2026111* captures/flights/20261109_2*   # slow (~minutes)
  python tools/motion_model.py fit   captures/flights/2026111* captures/flights/20261109_2*
  python tools/motion_model.py eval  captures/flights/2026111* captures/flights/20261109_2*
  (fit / eval build missing caches themselves; --jobs N for parallel builds)
  --setup tilt : flights after 2026-11-10 22:39 (camera tilted down; own params calib/motion_model_tilt.json)
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
OUT_DIR = os.path.join(ROOT, "captures", "motion_model")
CACHE_DIR = os.path.join(OUT_DIR, "cache")
PARAMS = os.path.join(ROOT, "calib", "motion_model.json")
TILT_FROM = "20261110_2239"   # camera tilted down by the user at 22:39 -> other geometry, fitted separately
SUFFIX = ""                   # "_tilt" for the tilted-camera setup (own params / outputs)

SKIP_AFTER_TAKEOFF = 3.0   # one-key takeoff climbs on its own for ~3 s (ACQUIRE starts ~3.1 s after TAKEOFF)
DT = 0.05                  # model / filter step (s)
GAPS = (0.5, 1.0, 2.0, 3.0)


# ================================================================ build: per-frame dataset (reuses flight_review)
def build_folder(folder, force=False):
    import csv
    import flight_review as fr
    from aruco_distance import load_focal
    name = os.path.basename(os.path.normpath(folder))
    path = os.path.join(CACHE_DIR, name + ".json")
    if os.path.exists(path) and not force:
        return path
    rows = list(csv.DictReader(open(os.path.join(folder, "frames.csv"))))
    events = fr.load_events(folder)
    out = {"folder": name, "flights": fr.flights(events), "frames": []}
    if out["flights"]:
        f = load_focal(1280)[0]
        frames = fr.measure(folder, rows, f)
        fr.resolve_sideways(frames)
        for d in frames:
            e = {k: d[k] for k in ("frame", "t", "state", "heading", "vo_ok", "blur", "seen")}
            e["vo_ok"] = bool(e["vo_ok"])
            if d["seen"] and "side" in d:
                best = min(d["cands"], key=lambda c: abs(c[0][0] - d["side"]) + abs(c[0][2] - d["height"]))
                e.update(dist=d["dist"], bearing=d["bearing"], elev=d["elev"], side=d["side"],
                         along=float(best[0][1]), height=d["height"], reproj=float(best[1]), edge=d["edge"])
            else:
                e["seen"] = False
            out["frames"].append(e)
    os.makedirs(CACHE_DIR, exist_ok=True)
    with open(path, "w") as fh:
        json.dump(out, fh)
    return path


def _build_one(args):
    folder, force = args
    try:
        build_folder(folder, force)
        return folder, None
    except Exception as ex:  # keep going with the other folders
        return folder, repr(ex)


def expand(patterns):
    out = []
    for p in patterns:
        for folder in sorted(glob.glob(p)) or [p]:
            if os.path.exists(os.path.join(folder, "frames.csv")) and folder not in out:
                out.append(folder)
    return out


def build_all(folders, jobs=4, force=False):
    todo = [f for f in folders if force or not os.path.exists(
        os.path.join(CACHE_DIR, os.path.basename(os.path.normpath(f)) + ".json"))]
    if not todo:
        return
    print(f"building {len(todo)} folder cache(s) with {jobs} job(s) ...", flush=True)
    if jobs > 1 and len(todo) > 1:
        from multiprocessing import Pool
        with Pool(min(jobs, len(todo))) as pool:
            for folder, err in pool.imap_unordered(_build_one, [(f, force) for f in todo]):
                print(f"  {os.path.basename(folder)}: {'ok' if err is None else 'FAILED ' + err}", flush=True)
    else:
        for f in todo:
            folder, err = _build_one((f, force))
            print(f"  {os.path.basename(folder)}: {'ok' if err is None else 'FAILED ' + err}", flush=True)


# ================================================================ segments: tag positions + sticks per air segment
def psi_tag_deg(side, along, bearing):
    """Drone heading in the tag frame from one sighting: direction to the tag minus where it appears in the image."""
    return np.degrees(np.arctan2(-np.asarray(side), np.asarray(along))) - np.asarray(bearing)


def wrap(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


GROUND_SHARP = 400.0       # picture sharpness (flight_review) above this = sitting on the ground (air 30-300, ground ~2000)
LAND_MAX_S = 9.0           # auto-land descent lasts ~8 s ("landing time over")


def load_segments(folders):
    """One dict per takeoff: air time from SKIP_AFTER_TAKEOFF s after TAKEOFF through the auto-land descent
    (LANDING frames), cut where the picture turns ground-sharp. Inputs per control row: throttle, yaw, roll, pitch
    fractions incl. trim + a 5th column = 1 while auto-landing (no control rows then: sticks centred + trim)."""
    import flight_review as fr
    segs = []
    for folder in folders:
        name = os.path.basename(os.path.normpath(folder))
        p = os.path.join(CACHE_DIR, name + ".json")
        if not os.path.exists(p):
            continue
        d = json.load(open(p))
        ctl = fr.load_control(folder)
        if not ctl or not d["flights"]:
            continue
        trim_at = fr.trims_at(fr.load_events(folder))
        frames = d["frames"]
        rows = []   # (t, throttle, yaw, roll, pitch bytes sent, landing flag, raw offsets x4)
        for r in ctl:
            t = float(r["t"])
            tr = trim_at(t)
            raw = [int(r[k]) - 128 for k in ("throttle", "yaw", "roll", "pitch")]
            rows.append((t, 128 + raw[0], min(255, max(0, 128 + raw[1] + tr["yaw"])),
                         min(255, max(0, 128 + raw[2] + tr["roll"])), min(255, max(0, 128 + raw[3] + tr["pitch"])),
                         0, *raw))
        ends = {}
        for k, (t0, t1, reason) in enumerate(d["flights"], 1):
            t1 = t1 if t1 is not None else max(f["t"] for f in frames)
            tr = trim_at(t1)
            rows.append((t1, 128, min(255, max(0, 128 + tr["yaw"])), min(255, max(0, 128 + tr["roll"])),
                         min(255, max(0, 128 + tr["pitch"])), 1, 0, 0, 0, 0))
            land = [f["t"] for f in frames if t1 <= f["t"] <= t1 + LAND_MAX_S and f["state"] == "LANDING"]
            end = max(land) if land else t1
            sharp = [f["t"] for f in frames if t1 + 0.5 <= f["t"] <= end and f["blur"] > GROUND_SHARP]
            ends[k] = (t1, min(sharp) - 0.3 if sharp else end)
        rows.sort(key=lambda r: r[0])
        A = np.array(rows, float)
        ct = A[:, 0]
        U = np.column_stack([(A[:, 1:5] - 128.0) / 127.0, A[:, 5]])   # 1.0 = full stick
        Uraw = A[:, 6:10]
        for k, (t0, _, reason) in enumerate(d["flights"], 1):
            t1, b = ends[k]
            a = t0 + SKIP_AFTER_TAKEOFF
            fr_ = [f for f in frames if a <= f["t"] <= b and f["blur"] <= GROUND_SHARP]
            sg = [f for f in fr_ if f["seen"]]
            if b - a < 2.0 or len(sg) < 10:
                continue
            seg = dict(folder=name, k=k, t0=t0, t1=t1, start=a, end=b, reason=reason, ct=ct, U=U, Uraw=Uraw,
                       vt=np.array([f["t"] for f in fr_]), vh=np.array([f["heading"] for f in fr_]),
                       vok=np.array([f["vo_ok"] for f in fr_], bool),
                       st=np.array([f["t"] for f in sg]),
                       z=np.array([[f["side"], f["along"], f["height"]] for f in sg]),
                       bearing=np.array([f["bearing"] for f in sg]), dist=np.array([f["dist"] for f in sg]))
            seg["psi_tag"] = psi_tag_deg(seg["z"][:, 0], seg["z"][:, 1], seg["bearing"])
            segs.append(seg)
    return segs


def sticks_at(seg, t):
    """Stick fractions sent at time(s) t (zero-order hold of control.csv incl. trim; 0 before the log starts)."""
    t = np.atleast_1d(t)
    i = np.searchsorted(seg["ct"], t, "right") - 1
    out = seg["U"][np.clip(i, 0, None)].copy()
    out[i < 0] = 0.0
    return out


def vo_at(seg, t):
    return np.interp(t, seg["vt"], seg["vh"])


# ================================================================ measurement noise (from consecutive sightings)
def noise_model(segs):
    """sigma per axis at 200 cm, scaled with (range/200)^2 (pixel noise -> distance^2). From pairs of sightings
    < 0.12 s apart: motion in that time is ~1-2 cm, so their difference is mostly noise (robust MAD / sqrt 2)."""
    D, R = [], []
    for s in segs:
        dt = np.diff(s["st"])
        ok = dt < 0.12
        dz = np.diff(np.column_stack([s["z"], s["psi_tag"]]), axis=0)[ok]
        dz[:, 3] = wrap(dz[:, 3])
        D.append(dz)
        R.append(((s["dist"][1:] + s["dist"][:-1]) / 2)[ok])
    D, R = np.vstack(D), np.concatenate(R)
    scale = (np.clip(R, 80, 400) / 200.0) ** 2
    sig = 1.4826 * np.median(np.abs(D / scale[:, None]), axis=0) / math.sqrt(2)
    bins = []
    for lo, hi in ((0, 150), (150, 250), (250, 1000)):
        m = (R >= lo) & (R < hi)
        if m.sum() > 20:
            bins.append(dict(range_cm=f"{lo}-{hi}", n=int(m.sum()),
                             sigma=[round(float(x), 1) for x in 1.4826 * np.median(np.abs(D[m]), axis=0) / math.sqrt(2)]))
    return dict(sigma_at_200cm=dict(side=float(sig[0]), along=float(sig[1]), height=float(sig[2]), psi_deg=float(sig[3])),
                scaling="sigma * (clip(range, 80, 400) / 200)^2", n_pairs=int(len(R)), by_range=bins)


def meas_sigma(noise, dist):
    sc = (min(max(dist, 80.0), 400.0) / 200.0) ** 2
    s = noise["sigma_at_200cm"]
    return np.array([s["side"], s["along"], s["height"]]) * sc, s["psi_deg"] * sc


# ================================================================ fitting (windows, nuisance initial state removed)
WIN = 6.0
TAU_GRID = (0.15, 0.3, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0)
DELAY_GRID = (0.0, 0.1, 0.2, 0.3, 0.4, 0.6)
YAW_TAU_GRID = (0.05, 0.15, 0.3, 0.5, 0.8)


def first_order_basis(F, a):
    """Position response (zero initial state) to forcing F (n x m) for v[k+1] = a v[k] + (1-a) F[k],
    p[k+1] = p[k] + DT v[k]. Same discretisation as the Kalman filter."""
    n = len(F)
    key = (n, a)
    M = _BASIS.get(key)
    if M is None:   # v[k] = (1-a) sum_{i<k} a^(k-1-i) F[i]
        e = np.arange(n)[:, None] - 1 - np.arange(n)[None, :]
        M = np.where(e >= 0, (1.0 - a) * a ** np.maximum(e, 0), 0.0)
        _BASIS[key] = M
    v = M @ F
    p = np.zeros_like(v)
    p[1:] = DT * np.cumsum(v[:-1], axis=0)
    return p


_BASIS = {}


def windows_pos(seg):
    """Sighting windows of <= WIN s (each starts at a sighting); heading offset VO -> tag frame per window."""
    out, i, st = [], 0, seg["st"]
    while i < len(st):
        j = np.searchsorted(st, st[i] + WIN, "right")
        if j - i >= 6 and st[j - 1] - st[i] >= 1.0:
            idx = np.arange(i, j)
            off = float(np.median(wrap(seg["psi_tag"][idx] - vo_at(seg, st[idx]))))
            out.append(dict(idx=idx, t0=st[i], off=off))
        i = j
    return out


def windows_yaw(seg):
    """Runs of valid VO heading (split at invalid frames / gaps > 0.2 s), cut to <= WIN s."""
    out, t, ok = [], seg["vt"], seg["vok"]
    runs, cur = [], []
    for k in range(len(t)):
        if ok[k] and (not cur or t[k] - t[cur[-1]] < 0.2):
            cur.append(k)
        else:
            if cur:
                runs.append(cur)
            cur = [k] if ok[k] else []
    if cur:
        runs.append(cur)
    for r in runs:
        r = np.array(r)
        while len(r):
            j = np.searchsorted(t[r], t[r[0]] + WIN, "right")
            w = r[:j]
            if len(w) >= 10 and t[w[-1]] - t[w[0]] >= 1.0:
                out.append(dict(idx=w, t0=t[w[0]]))
            r = r[j:]
    return out


def design(seg, w, axis, tau, delay, noise):
    """(y, L, G, sigma) for one window. axis 'h': rows side/along, globals [K_fwd, K_right, drift_fwd, drift_right];
    'z': rows height, globals [K_up, drift_up, extra up while auto-landing]; 'yaw': rows VO heading, globals [K_yaw, own_turn]."""
    if axis == "yaw":
        tm = seg["vt"][w["idx"]]
    else:
        tm = seg["st"][w["idx"]]
    n = int(round((tm[-1] - w["t0"]) / DT)) + 2
    tg = w["t0"] + DT * np.arange(n)
    k = np.clip(np.round((tm - w["t0"]) / DT).astype(int), 0, n - 1)
    u = sticks_at(seg, tg - delay)
    a = math.exp(-DT / tau)
    g = DT * (1 - a ** k) / (1 - a)                   # position response to initial velocity
    if axis == "yaw":
        y = seg["vh"][w["idx"]]
        L = np.column_stack([np.ones(len(k)), g])
        G = first_order_basis(np.column_stack([u[:, 1], np.ones(n)]), a)[k]
        return y, L, G, np.full(len(y), 0.5)
    sig = np.array([meas_sigma(noise, d)[0] for d in seg["dist"][w["idx"]]])
    if axis == "z":
        y = seg["z"][w["idx"], 2]
        L = np.column_stack([np.ones(len(k)), g])
        G = first_order_basis(np.column_stack([u[:, 0], np.ones(n), u[:, 4]]), a)[k]
        return y, L, G, sig[:, 2]
    psi = np.radians(vo_at(seg, tg) + w["off"])
    fwd = np.column_stack([np.sin(psi), -np.cos(psi)])
    rgt = np.column_stack([np.cos(psi), np.sin(psi)])
    F = [fwd * u[:, 3:4], rgt * u[:, 2:3], fwd, rgt]   # pitch -> forward, roll -> right, drift fwd, drift right
    P = [first_order_basis(f, a)[k] for f in F]        # each (m x 2): side, along
    m = len(k)
    y = np.concatenate([seg["z"][w["idx"], 0], seg["z"][w["idx"], 1]])
    G = np.column_stack([np.concatenate([p[:, 0], p[:, 1]]) for p in P])
    z0, o1 = np.zeros(m), np.ones(m)
    L = np.column_stack([np.concatenate([o1, z0]), np.concatenate([z0, o1]),
                         np.concatenate([g, z0]), np.concatenate([z0, g])])
    return y, L, G, np.concatenate([sig[:, 0], sig[:, 1]])


def project(y, L, G, wts):
    """Weighted Frisch-Waugh: remove the window's own initial position/velocity from y and G."""
    Lw, Gw, yw = L * wts[:, None], G * wts[:, None], y * wts
    Q, _ = np.linalg.qr(Lw)
    return yw - Q @ (Q.T @ yw), Gw - Q @ (Q.T @ Gw)


def huber(r, c=1.5):
    a = np.abs(r)
    return np.where(a <= c, 0.5 * r * r, c * (a - 0.5 * c))


def fit_grid_point(items, iters=4):
    """items: [(fold, y, L, G, sigma)] -> theta, per-window projected (yr, Gr) with final robust weights."""
    w8 = [1.0 / s for (_, _, _, _, s) in items]
    theta = None
    for _ in range(iters):
        pr = [project(y, L, G, w) for (_, y, L, G, _), w in zip(items, w8)]
        Y = np.concatenate([p[0] for p in pr])
        X = np.vstack([p[1] for p in pr])
        theta = np.linalg.lstsq(X, Y, rcond=None)[0]
        new = []
        for (_, y, L, G, s), (yr, Gr) in zip(items, pr):
            r = (yr - Gr @ theta)        # already in sigma units
            new.append((1.0 / s) * np.sqrt(np.minimum(1.0, 1.5 / np.maximum(np.abs(r), 1e-9))))
        w8 = new
    return theta, pr


def fit_axis(segs, axis, noise, taus, delays):
    """Grid over (tau, delay), linear LS inside. Returns best + per-fold (leave one folder out) refits."""
    folds = sorted({s["folder"] for s in segs})
    wins = [(s, w) for s in segs for w in (windows_yaw(s) if axis == "yaw" else windows_pos(s))]
    res = {}
    for tau in taus:
        for delay in delays:
            items = [(s["folder"],) + design(s, w, axis, tau, delay, noise) for s, w in wins]
            theta, pr = fit_grid_point(items)
            per = [(it[0], yr, Gr) for it, (yr, Gr) in zip(items, pr)]
            res[(tau, delay)] = per
    def solve(excl):
        best = None
        for key, per in res.items():
            X = np.vstack([Gr for f, _, Gr in per if f != excl])
            Y = np.concatenate([yr for f, yr, _ in per if f != excl])
            th = np.linalg.lstsq(X, Y, rcond=None)[0]
            loss = float(huber(Y - X @ th).sum())
            if best is None or loss < best[2]:
                best = (key, th, loss, len(Y))
        return best
    full = solve(None)
    jk = {f: solve(f) for f in folds}
    return dict(full=full, folds=jk, n_windows=len(wins), n_rows=full[3], n_folders=len(folds))


def jk_se(vals):
    v = np.array(vals, float)
    n = len(v)
    return float(math.sqrt((n - 1) / n * ((v - v.mean(axis=0)) ** 2).sum(axis=0))) if n > 1 else float("nan")


AXES = {"h": ["gain_forward_cm_s_per_pitch", "gain_right_cm_s_per_roll", "drift_forward_cm_s", "drift_right_cm_s"],
        "z": ["gain_up_cm_s_per_throttle", "drift_up_cm_s", "landing_extra_up_cm_s"],
        "yaw": ["gain_yaw_deg_s_per_yaw", "own_turn_deg_s"]}


def fit(segs, noise, verbose=True):
    """Fit all axes. Returns the params dict (+ per-fold params for held-out evaluation)."""
    out, folds = {}, {}
    for axis, taus in (("h", TAU_GRID), ("z", TAU_GRID), ("yaw", YAW_TAU_GRID)):
        r = fit_axis(segs, axis, noise, taus, DELAY_GRID)
        (tau, delay), th, loss, nrow = r["full"]
        ent = dict(tau_s=tau, delay_s=delay, n_windows=r["n_windows"], n_rows=nrow, n_folders=r["n_folders"])
        for name, v, j in zip(AXES[axis], th, range(len(th))):
            ent[name] = round(float(v), 2)
            ent[name + "_se"] = round(jk_se([fv[1][j] for fv in r["folds"].values()]), 2)
        ent["tau_s_se"] = round(jk_se([fv[0][0] for fv in r["folds"].values()]), 2)
        ent["delay_s_se"] = round(jk_se([fv[0][1] for fv in r["folds"].values()]), 2)
        out[axis] = ent
        for f, ((tau_f, del_f), th_f, _, _) in r["folds"].items():
            folds.setdefault(f, {})[axis] = dict(tau_s=tau_f, delay_s=del_f, **{n: float(v) for n, v in zip(AXES[axis], th_f)})
        if verbose:
            print(f"  {axis}: " + ", ".join(f"{k}={v}" for k, v in ent.items()), flush=True)
    return out, folds


# ================================================================ Kalman filter
class MotionKF:
    """Position/velocity (tag frame, cm) + heading filter driven by the fitted model.
      position state [side, along, height, v_side, v_along, v_height]: predict with
        v += (1 - a) * (R(psi) [K_fwd*pitch + drift_fwd, K_right*roll + drift_right] - v),  a = exp(-dt / tau)
      heading state [psi, rate, vo_offset] (deg): rate follows K_yaw * yaw + own_turn with its own tau;
        VO heading measures psi - vo_offset, tag sightings measure psi directly.
    Inputs: sticks as fractions (byte - 128) / 127 incl. trim, order throttle, yaw, roll, pitch, + landing flag
    (1 while the drone's own auto-land runs).
    Measurements: update_tag() for drone-camera sightings, update_position() for any other position sensor
    (e.g. a fixed outside camera giving the drone in the same tag frame), update_vo() for the VO heading.
    params=None -> constant-velocity filter (no stick model, no drift): the 'KF without model' baseline."""

    def __init__(self, params, noise, q_h=400.0, q_z=100.0, q_rate=400.0):
        self.p, self.noise = params, noise
        self.q_h, self.q_z, self.q_rate = q_h, q_z, q_rate
        self.x = None
        self.hist = []            # (t, sticks) for the input delay
        self.t = None
        self.vo_last_ok = None

    def copy(self):
        import copy
        c = copy.copy(self)
        c.x, c.P, c.h, c.Ph = self.x.copy(), self.P.copy(), self.h.copy(), self.Ph.copy()
        c.hist = list(self.hist)
        return c

    # ---- inputs
    def set_sticks(self, t, u):
        self.hist.append((t, np.asarray(u, float)))
        if self.t is not None and len(self.hist) > 300:     # keep ~2 s before the filter time (input delay)
            cut = self.t - 2.0
            i = next((k for k, (th, _) in enumerate(self.hist) if th > cut), len(self.hist))
            self.hist = self.hist[max(0, i - 1):]

    def _u(self, t):
        u = np.zeros(5)
        for th, uh in reversed(self.hist):
            if th <= t:
                return uh
        return u

    # ---- init / predict
    def init(self, t, z, psi, vo=None):
        self.t = t
        self.x = np.array([z[0], z[1], z[2], 0.0, 0.0, 0.0])
        sxy, _ = meas_sigma(self.noise, float(np.linalg.norm(z)))
        self.P = np.diag([sxy[0] ** 2, sxy[1] ** 2, sxy[2] ** 2, 30.0 ** 2, 30.0 ** 2, 20.0 ** 2])
        self.h = np.array([psi, 0.0, psi - vo if vo is not None else 0.0])
        self.Ph = np.diag([25.0, 100.0, 25.0 if vo is not None else 1e6])

    def _step(self, dt):
        p = self.p
        if p is None:
            a_h = a_z = a_y = 1.0
            Kf = Kr = df = dr = Ku = du = Kl = Ky = dy = 0.0
            uh = uz = uy = np.zeros(5)
        else:
            a_h, a_z, a_y = (math.exp(-dt / p[k]["tau_s"]) for k in ("h", "z", "yaw"))
            Kf, Kr, df, dr = (p["h"][n] for n in AXES["h"])
            Ku, du, Kl = (p["z"][n] for n in AXES["z"])
            Ky, dy = (p["yaw"][n] for n in AXES["yaw"])
            uh, uz, uy = (self._u(self.t - p[k]["delay_s"]) for k in ("h", "z", "yaw"))
        # heading
        h, Ph = self.h, self.Ph
        Ah = np.array([[1.0, dt, 0.0], [0.0, a_y, 0.0], [0.0, 0.0, 1.0]])
        bh = np.array([0.0, (1 - a_y) * (Ky * uy[1] + dy), 0.0])
        self.h = Ah @ h + bh
        self.Ph = Ah @ Ph @ Ah.T + np.diag([0.05, self.q_rate, 0.02]) * dt
        # position
        psi = math.radians(h[0])
        fwd, rgt = np.array([math.sin(psi), -math.cos(psi)]), np.array([math.cos(psi), math.sin(psi)])
        Fh = fwd * (Kf * uh[3] + df) + rgt * (Kr * uh[2] + dr)
        Fz = Ku * uz[0] + du + Kl * uz[4]
        A = np.eye(6)
        A[0, 3] = A[1, 4] = A[2, 5] = dt
        A[3, 3] = A[4, 4] = a_h
        A[5, 5] = a_z
        b = np.array([0, 0, 0, (1 - a_h) * Fh[0], (1 - a_h) * Fh[1], (1 - a_z) * Fz])
        self.x = A @ self.x + b
        self.P = A @ self.P @ A.T + np.diag([0.5, 0.5, 0.5, self.q_h, self.q_h, self.q_z]) * dt
        self.t += dt

    def predict_to(self, t):
        while self.t < t - 1e-9:
            self._step(min(DT, t - self.t))

    # ---- updates
    def _kf(self, x, P, z, H, R, gate):
        y = z - H @ x
        S = H @ P @ H.T + R
        Si = np.linalg.inv(S)
        if float(y @ Si @ y) > gate:
            return x, P, False
        K = P @ H.T @ Si
        return x + K @ y, (np.eye(len(x)) - K @ H) @ P, True

    def update_position(self, z, sig, gate=25.0):
        H = np.hstack([np.eye(3), np.zeros((3, 3))])
        self.x, self.P, ok = self._kf(self.x, self.P, np.asarray(z, float), H, np.diag(np.asarray(sig) ** 2), gate)
        return ok

    def update_tag(self, z, bearing, dist):
        sxy, spsi = meas_sigma(self.noise, dist)
        ok = self.update_position(z, sxy)
        if ok:
            psi = float(psi_tag_deg(z[0], z[1], bearing))
            H = np.array([[1.0, 0.0, 0.0]])
            zz = np.array([self.h[0] + wrap(psi - self.h[0])])
            self.h, self.Ph, _ = self._kf(self.h, self.Ph, zz, H, np.array([[spsi ** 2]]), 16.0)
        return ok

    def update_vo(self, t, heading, ok):
        if not ok:
            return
        if self.vo_last_ok is not None and t - self.vo_last_ok > 0.3:   # VO froze meanwhile -> offset unknown-ish
            self.Ph[2, 2] += 100.0 * (t - self.vo_last_ok)
        self.vo_last_ok = t
        H = np.array([[1.0, 0.0, -1.0]])
        self.h, self.Ph, _ = self._kf(self.h, self.Ph, np.array([heading]), H, np.array([[0.5 ** 2]]), 1e9)

    def bearing(self):
        """Predicted tag bearing in the image (deg, + right), from the estimated position and heading."""
        return float(wrap(psi_tag_deg(self.x[0], self.x[1], 0.0) - self.h[0]))


def run_events(seg):
    """Time-ordered events of a segment: ('ctl', t, u), ('vo', t, heading, ok), ('tag', t, index)."""
    ev = []
    m = (seg["ct"] >= seg["start"] - 1.0) & (seg["ct"] <= seg["end"])
    for t, u in zip(seg["ct"][m], seg["U"][m]):
        ev.append((t, 0, "ctl", u))
    for t, h, ok in zip(seg["vt"], seg["vh"], seg["vok"]):
        ev.append((t, 1, "vo", (h, ok)))
    for i, t in enumerate(seg["st"]):
        ev.append((t, 2, "tag", i))
    ev.sort(key=lambda e: (e[0], e[1]))
    return ev


def apply(kf, e, seg, use_tag=True):
    t, _, kind, val = e
    if kind == "ctl":
        kf.set_sticks(t, val)
        return
    if kf.x is None:
        if kind == "tag":
            i = val
            kf.init(t, seg["z"][i], float(seg["psi_tag"][i]), float(vo_at(seg, t)))
        return
    kf.predict_to(t)
    if kind == "vo":
        kf.update_vo(t, val[0], val[1])
    elif kind == "tag" and use_tag:
        i = val
        kf.update_tag(seg["z"][i], seg["bearing"][i], seg["dist"][i])


Q_H = 400.0   # process noise on horizontal velocity, (cm/s)^2 per s  (--q-h)


def gap_tests(seg, params, noise, gaps=GAPS, spacing=0.5, natural_min=0.5):
    """Artificial gaps after sightings + the real tag losses. Returns a list of result dicts."""
    ev = run_events(seg)
    kfs = {"model": MotionKF(params, noise, q_h=Q_H, q_z=Q_H / 4),
           "kf_cv": MotionKF(None, noise, q_h=Q_H, q_z=Q_H / 4)}
    st, z = seg["st"], seg["z"]
    results, last_start, n_seen, tr = [], -1e9, 0, []
    for ei, e in enumerate(ev):
        for kf in kfs.values():
            apply(kf, e, seg)
        if e[2] == "tag" and kfs["model"].x is not None:
            i = e[3]
            n_seen += 1
            tr.append((e[0], kfs["model"].x[:3].copy(), kfs["model"].h[0]))
            starts = []
            if n_seen >= 5 and e[0] - seg["start"] >= 1.0 and e[0] - last_start >= spacing:
                last_start = e[0]
                for G in gaps:
                    j = np.searchsorted(st, e[0] + G - 1e-9)
                    if j < len(st) and st[j] - (e[0] + G) < 0.25:
                        starts.append(("hidden", G, j))
            if i + 1 < len(st) and st[i + 1] - st[i] >= natural_min:
                starts.append(("natural", st[i + 1] - st[i], i + 1))
            for kind, G, j in starts:
                tj = st[j]
                pred = {}
                for name, kf in kfs.items():
                    c = kf.copy()
                    for e2 in ev[ei + 1:]:
                        if e2[0] >= tj:
                            break
                        apply(c, e2, seg, use_tag=False)
                    c.predict_to(tj)
                    pred[name] = (c.x[:3].copy(), c.bearing(), math.sqrt(c.P[0, 0] + c.P[1, 1]))
                # naive baselines from raw sightings before the gap
                m = (st > e[0] - 0.2) & (st <= e[0])
                last = z[m].mean(axis=0)
                m5 = (st > e[0] - 0.5) & (st <= e[0])
                if m5.sum() >= 3 and st[m5][-1] - st[m5][0] >= 0.2:
                    A = np.column_stack([st[m5] - e[0], np.ones(m5.sum())])
                    coef = np.linalg.lstsq(A, z[m5], rcond=None)[0]
                    cv = coef[1] + coef[0] * (tj - e[0])
                else:
                    cv = last
                # current tracker-style bearing: remembered position + VO heading change
                psi_last = float(np.median(wrap(seg["psi_tag"][m] - vo_at(seg, st[m])))) + vo_at(seg, tj)
                b_last = float(wrap(psi_tag_deg(last[0], last[1], 0.0) - psi_last))
                truth = z[j]
                mc = (seg["ct"] >= e[0] - 0.3) & (seg["ct"] <= tj)
                active = bool(mc.any() and np.abs(seg["Uraw"][mc][:, [0, 2, 3]]).max() >= 5)
                row = dict(folder=seg["folder"], k=seg["k"], kind=kind, gap=G, t=e[0], range=float(seg["dist"][j]),
                           truth_bearing=float(seg["bearing"][j]), active=active,
                           moved=float(np.linalg.norm((truth - last)[:2])))
                for name, p in (("last_seen", last), ("const_vel", cv), ("kf_cv", pred["kf_cv"][0]),
                                ("model", pred["model"][0])):
                    row[name + "_h"] = float(np.linalg.norm((p - truth)[:2]))
                    row[name + "_z"] = float(abs(p[2] - truth[2]))
                    row[name + "_along"] = float(p[1] - truth[1])
                    row[name + "_side"] = float(p[0] - truth[0])
                row["last_seen_b"] = row["const_vel_b"] = abs(float(wrap(b_last - row["truth_bearing"])))
                row["kf_cv_b"] = abs(float(wrap(pred["kf_cv"][1] - row["truth_bearing"])))
                row["model_b"] = abs(float(wrap(pred["model"][1] - row["truth_bearing"])))
                row["model_sig_h"] = pred["model"][2]
                sxy, _ = meas_sigma(noise, float(seg["dist"][j]))
                row["noise_side"], row["noise_along"] = 0.674 * sxy[0], 0.674 * sxy[1]
                results.append(row)
    return results, tr


# ================================================================ commands
METHODS = (("last_seen", "last seen"), ("const_vel", "const. velocity"), ("kf_cv", "KF, no model"),
           ("model", "KF + model"))


def cmd_fit(folders):
    segs = load_segments(folders)
    print(f"{len(segs)} air segments from {len({s['folder'] for s in segs})} folders, "
          f"{sum(len(s['st']) for s in segs)} tag sightings, {sum(s['end'] - s['start'] for s in segs):.0f} s in the air")
    noise = noise_model(segs)
    print("  measurement noise:", json.dumps(noise))
    params, folds = fit(segs, noise)
    out = dict(created="motion_model.py fit", folders=sorted({s["folder"] for s in segs}), n_segments=len(segs),
               units="stick = (byte sent incl. trim - 128) / 127 (1 = full stick); positions in cm, tag frame; "
                     "+-_se = leave-one-folder-out jackknife standard error",
               model="dv/dt = (gain * stick(t - delay) + drift - v) / tau per body axis; yaw rate likewise",
               noise=noise, **params)
    params_path = PARAMS.replace(".json", SUFFIX + ".json")
    os.makedirs(os.path.dirname(params_path), exist_ok=True)
    with open(params_path, "w") as fh:
        json.dump(out, fh, indent=1)
    print(f"wrote {os.path.relpath(params_path, ROOT)}")
    return segs, noise, params, folds


def table(rows, key="h", kinds=("hidden", "natural"), tag=""):
    lines = []
    for kind in kinds:
        sub = [r for r in rows if r["kind"] == kind]
        if not sub:
            continue
        bins = [(g, g) for g in GAPS] if kind == "hidden" else [(0.5, 1.0), (1.0, 2.0), (2.0, 4.0), (4.0, 99.0)]
        for lo, hi in bins:
            s = [r for r in sub if (r["gap"] == lo if kind == "hidden" else lo <= r["gap"] < hi)]
            if len(s) < 3:
                continue
            label = f"{lo:.1f} s" if kind == "hidden" else f"{lo:.1f}-{hi:.0f} s"
            cells = []
            for m, _ in METHODS:
                v = np.array([r[f"{m}_{key}"] for r in s])
                cells.append(f"{np.median(v):5.1f} / {np.percentile(v, 90):5.1f}")
            lines.append(f"| {kind}{tag} {label} | {len(s)} | " + " | ".join(cells) + " |")
    return lines


def cmd_eval(folders, q_h=None):
    segs, noise, params, folds = cmd_fit(folders)
    rows, traces = [], {}
    for s in segs:
        p = folds.get(s["folder"], params)      # model fitted WITHOUT this folder
        r, tr = gap_tests(s, p, noise)
        rows += r
        traces[(s["folder"], s["k"])] = (s, tr)
    os.makedirs(OUT_DIR, exist_ok=True)
    import csv
    with open(os.path.join(OUT_DIR, "gap_tests" + SUFFIX + ".csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    head = "| gap | n | " + " | ".join(n for _, n in METHODS) + " |"
    sep = "|---|---|" + "---|" * len(METHODS)
    out = ["Horizontal position error (cm), median / 90th percentile, vs the true sighting after the gap:", head, sep]
    out += table(rows, "h")
    out += table([r for r in rows if r["active"]], "h", ("hidden",), ", sticks active")
    out += table([r for r in rows if r["moved"] >= 15], "h", ("hidden",), ", moved >= 15 cm")
    out += ["", "Height error (cm), median / 90th percentile:", head, sep] + table(rows, "z")
    out += ["", "Tag bearing error (deg), median / 90th percentile (last seen = remembered position + VO heading, "
            "like tracker.py; const. velocity column repeats last seen):", head, sep]
    out += table(rows, "b")
    out += ["", "Range error along the tag normal (cm, |along|), median / 90th percentile (precise axis):", head, sep]
    for r in rows:
        for m, _ in METHODS:
            r[m + "_a"] = abs(r[m + "_along"])
    out += table(rows, "a")
    fl = [np.hypot(r["noise_side"], r["noise_along"]) for r in rows if r["kind"] == "hidden"]
    out += ["", f"Noise floor: the 'true' sighting itself is noisy; median expected horizontal error of one sighting "
                f"{np.median(fl):.1f} cm (sideways noise dominates)."]
    # bias along the tag normal (does the model know the backward drift?)
    hid = [r for r in rows if r["kind"] == "hidden" and r["gap"] == 2.0]
    if hid:
        out += ["", "Mean signed error along the normal at 2 s gaps (cm, + = predicted farther than true): " +
                ", ".join(f"{n} {np.mean([r[m + '_along'] for r in hid]):+.1f}" for m, n in METHODS)]
        cal = np.array([r["model_h"] / max(r["model_sig_h"], 1e-6) for r in hid])
        out += [f"KF + model: error / predicted sigma at 2 s, median {np.median(cal):.2f} (~0.8 = honest for a 2-D "
                f"error; lower = filter too cautious), q_h = {Q_H:g} (cm/s)^2/s"]
    txt = "\n".join(out)
    print(txt)
    with open(os.path.join(OUT_DIR, "eval_summary" + SUFFIX + ".md"), "w") as fh:
        fh.write(txt + "\n")
    plots(rows, traces, params, noise)
    return rows


def plots(rows, traces, params, noise):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    hid = [r for r in rows if r["kind"] == "hidden"]
    fig, ax = plt.subplots(1, 3, figsize=(14, 4.2))
    for a, key, lab in zip(ax, ("h", "z", "b"), ("horizontal error (cm)", "height error (cm)", "bearing error (deg)")):
        for off, (m, name) in zip((-0.06, -0.02, 0.02, 0.06), METHODS):
            if key == "b" and m == "const_vel":
                continue
            gs = [g for g in GAPS if sum(r["gap"] == g for r in hid) >= 3]
            if not gs:
                continue
            med = [np.median([r[f"{m}_{key}"] for r in hid if r["gap"] == g]) for g in gs]
            lo = [np.percentile([r[f"{m}_{key}"] for r in hid if r["gap"] == g], 25) for g in gs]
            hi = [np.percentile([r[f"{m}_{key}"] for r in hid if r["gap"] == g], 75) for g in gs]
            x = np.array(gs) + off * 3
            a.errorbar(x, med, yerr=[np.array(med) - lo, np.array(hi) - med], marker="o", capsize=3, label=name)
        a.set_xlabel("hidden gap (s)")
        a.set_ylabel(lab + ", median, IQR")
        a.grid(alpha=0.3)
    ax[0].legend(fontsize=8)
    fig.suptitle("Prediction error after hiding the tag (held-out folders)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "error_vs_gap" + SUFFIX + ".png"), dpi=90)
    plt.close(fig)
    # example flight: the segment with most sightings
    (s, tr) = max(traces.values(), key=lambda v: len(v[0]["st"]))
    fig, ax = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    tt = np.array([x[0] for x in tr])
    est = np.array([x[1] for x in tr])
    for i, name in enumerate(("sideways (cm)", "along normal (cm)", "height (cm)")):
        ax[i].plot(s["st"], s["z"][:, i], ".", ms=3, color="gray", label="tag sighting")
        ax[i].plot(tt, est[:, i], "-", lw=1, label="KF + model (all sightings)")
        ax[i].set_ylabel(name)
    # a few hidden-gap predictions drawn as lines from gap start to prediction
    p = params
    ev = run_events(s)
    for G, col in ((2.0, "tab:red"),):
        for t_start in np.arange(s["st"][min(10, len(s["st"]) - 1)], s["st"][-1] - G, 3.0):
            kf = MotionKF(p, noise)
            path = []
            for e in ev:
                apply(kf, e, s, use_tag=e[0] <= t_start)
                if kf.x is not None and t_start < e[0] <= t_start + G:
                    path.append((e[0], kf.x[:3].copy()))
                if e[0] > t_start + G:
                    break
            if path:
                pt = np.array([x[0] for x in path])
                px = np.array([x[1] for x in path])
                for i in range(3):
                    ax[i].plot(pt, px[:, i], "-", color=col, lw=2, alpha=0.8,
                               label="prediction, tag hidden 2 s" if t_start == s["st"][min(10, len(s["st"]) - 1)] and i == 0 else None)
    ax[0].legend(fontsize=8, loc="upper right")
    U = sticks_at(s, tt)
    for j, n in enumerate(("throttle", "yaw", "roll", "pitch")):
        ax[3].plot(tt, U[:, j], lw=1, label=n)
    ax[3].set_ylabel("stick sent (fraction)")
    ax[3].legend(fontsize=8, ncol=4)
    ax[3].set_xlabel("time (s, flight log clock)")
    fig.suptitle(f"{s['folder']} flight {s['k']}: estimate vs sightings (model fitted on all folders)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, "example_flight" + SUFFIX + ".png"), dpi=80)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=("build", "fit", "eval"))
    ap.add_argument("folders", nargs="+")
    ap.add_argument("--jobs", type=int, default=4)
    ap.add_argument("--force", action="store_true", help="rebuild caches")
    ap.add_argument("--q-h", type=float, default=Q_H, help="KF process noise, horizontal velocity (cm/s)^2/s")
    ap.add_argument("--setup", choices=("level", "tilt", "all"), default="level",
                    help="level = camera level (before 2026-11-10 22:39, default), tilt = camera tilted down after")
    a = ap.parse_args()
    folders = expand(a.folders)
    build_all(folders, a.jobs, a.force)
    global SUFFIX, Q_H
    Q_H = a.q_h
    if a.setup != "all":
        tilt = a.setup == "tilt"
        folders = [f for f in folders if (os.path.basename(os.path.normpath(f)) >= TILT_FROM) == tilt]
        SUFFIX = "_tilt" if tilt else ""
    else:
        SUFFIX = "_all"
    if a.cmd == "fit":
        cmd_fit(folders)
    elif a.cmd == "eval":
        cmd_eval(folders)


if __name__ == "__main__":
    main()
