"""How the HS420's own drift changes over time, and what battery-related trends the logs can show (OFFLINE ONLY).

Uses only the live logs of tools/live_flight.py (control.csv, events.csv per captures/flights/<time>/), no frame
processing (light on the CPU). control.csv holds, while in the air, the smoothed tag sighting (range, bearing,
elevation, "fresh" flag), the stick bytes sent BEFORE trim and the visual-odometry heading.

What it measures
  hover windows : after the auto climb (takeoff + 3 s) while the state is FLYING / ACQUIRE = pitch, roll, throttle
                  centred (only trim) and yaw = heading hold. Per window: drift forward / right / up (cm/s, body
                  frame, from fresh tag sightings; + = forward, right, up) and turn rate (deg/s, from VO heading).
  climb         : takeoff +1 s -> +3 s (one-key auto climb, after liftoff): drift and climb rate while climbing.
  approach      : APPROACH windows: forward speed vs pitch byte (incl. trim) -> forward authority over time.
  trims         : rebuilt per takeoff from every "tune trim_* = v" line in all events.csv files, carried forward in
                  time order (calib/tuning.json is not versioned). Before the first logged tune line trims = 0
                  (arrow-key trim era, not logged) -> marked "assumed".
  battery proxy : sessions = takeoffs with < SESSION_GAP_MIN minutes between them; per takeoff: index in the
                  session, motors-on air time of the session before it (takeoff -> "landing time over").
  battery byte  : flights from 2026-11-11 21:xx on log the `93 04` status (drone_link): "status at takeoff: ...
                  battery N" in events.csv + status.csv. When >= 8 takeoffs have it, the drifts are also fitted
                  against it (scale of the byte not verified).
Writes captures/drift_battery/: takeoffs.csv, windows.csv, *.png, summary.txt (also printed).

Usage:
  python tools/drift_trend.py                       # all folders in captures/flights
  python tools/drift_trend.py captures/flights/2026111*
  python tools/drift_trend.py --window 2.0 --session-gap 25
"""
import argparse
import csv
import glob
import math
import os
import re
from datetime import datetime, timedelta

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
OUT = os.path.join(ROOT, "captures", "drift_battery")
CLIMB_S = 3.0               # one-key takeoff climbs by itself ~3 s
CLIMB_FROM = 1.0            # ... but leaves the ground ~1 s after the command: earlier sightings are still on the
                            # ground and would dilute the climb drift toward 0 (sessions seeing the tag from the
                            # ground vs. not would not be comparable)
LAND_DESCENT_S = 8.0        # motors still on after the land command (fallback when "landing time over" is missing)
HOVER_STATES = ("FLYING", "ACQUIRE")
# yaw authority, deg/s per yaw byte: flight_review (2026-11-10, 21 takeoffs) ~12-18 deg/s per power unit
# (= 0.4 * 127 bytes) -> ~0.3; cross-check here: own turn vs trim_yaw across takeoffs. The heading hold's own
# command/rate pairs can NOT give it (feedback: the command follows the turn).
K_YAW = 0.30
# camera mount by time (from notes/findings.md): tilted down by the user 2026-11-10 22:39, level again 2026-11-11 20:33
TILT_FROM, TILT_TO = datetime(2026, 11, 10, 22, 39), datetime(2026, 11, 11, 20, 33)


# ------------------------------------------------------------------ loading
def folder_time(folder):
    """Start time from the folder name (captures/flights/<YYYYmmdd_HHMMSS>); else the folder's own time (tests)."""
    m = re.match(r"(\d{8}_\d{6})", os.path.basename(os.path.normpath(folder)))
    if m:
        return datetime.strptime(m.group(1), "%Y%m%d_%H%M%S")
    return datetime.fromtimestamp(os.path.getmtime(folder))


def load_events(folder):
    p = os.path.join(folder, "events.csv")
    if not os.path.exists(p):
        return []
    return [(float(r["t"]), r["event"]) for r in csv.DictReader(open(p, newline=""))]


def load_control(folder):
    p = os.path.join(folder, "control.csv")
    if not os.path.exists(p):
        return []
    out = []
    for r in csv.DictReader(open(p, newline="")):
        try:
            out.append(dict(t=float(r["t"]), state=r["state"], fresh=r["fresh"] == "1",
                            rng=float(r["range_cm"]) if r["range_cm"] else None,
                            bear=float(r["bearing_deg"]) if r["bearing_deg"] else None,
                            elev=float(r["elev_deg"]) if r["elev_deg"] else None,
                            thr=int(r["throttle"]), yaw=int(r["yaw"]), roll=int(r["roll"]), pitch=int(r["pitch"]),
                            vo=float(r["vo_heading_deg"]) if r.get("vo_heading_deg") not in (None, "") else None))
        except (KeyError, ValueError):
            continue
    return out


TUNE_RE = re.compile(r"tune (trim_(?:pitch|roll|yaw)) = (-?\d+)")


def trim_timeline(folders):
    """[(wall time, axis, value)] from all tune lines, time order."""
    tl = []
    for f in folders:
        t0 = folder_time(f)
        for t, ev in load_events(f):
            m = TUNE_RE.search(ev)
            if m:
                tl.append((t0 + timedelta(seconds=t), m.group(1)[5:], int(m.group(2))))
    return sorted(tl)


def trims_at(tl, wall):
    tr, src = {"pitch": 0, "roll": 0, "yaw": 0}, {"pitch": "assumed 0", "roll": "assumed 0", "yaw": "assumed 0"}
    for w, ax, v in tl:
        if w > wall:
            break
        tr[ax], src[ax] = v, "logged"
    return tr, src


# ------------------------------------------------------------------ math
def theil_sen(t, y):
    """Robust slope (median of pairwise slopes) + its rough spread; None if too few points."""
    t, y = np.asarray(t, float), np.asarray(y, float)
    if len(t) < 5 or t[-1] - t[0] < 1.0:
        return None, None
    i, j = np.triu_indices(len(t), 1)
    dt = t[j] - t[i]
    ok = dt > 0.15
    s = (y[j] - y[i])[ok] / dt[ok]
    if len(s) < 5:
        return None, None
    q1, q3 = np.percentile(s, [25, 75])
    return float(np.median(s)), float((q3 - q1) / 1.35 / math.sqrt(len(t)))


def positions(rows):
    """Fresh, changed tag sightings -> (t, X right, Y along initial heading, height, psi) of the drone relative to
    the tag, in the frame of the takeoff heading (VO heading = psi, 0 at takeoff)."""
    out, last = [], None
    for r in rows:
        if not r["fresh"] or r["rng"] is None or r["vo"] is None:
            continue
        key = (r["rng"], r["bear"], r["elev"])
        if key == last:
            continue            # control.csv repeats the smoothed value between new frames
        last = key
        h = r["rng"] * math.cos(math.radians(r["elev"]))
        a = math.radians(r["vo"] + r["bear"])              # direction to the tag from the drone
        out.append((r["t"], -h * math.sin(a), -h * math.cos(a), -r["rng"] * math.sin(math.radians(r["elev"])),
                    math.radians(r["vo"])))
    return np.array(out) if out else np.zeros((0, 5))


def body_drift(P):
    """Drift (forward, right, up) cm/s in the body frame + turn needs VO separately."""
    if len(P) < 6:
        return None
    vx, ex = theil_sen(P[:, 0], P[:, 1])
    vy, ey = theil_sen(P[:, 0], P[:, 2])
    vz, ez = theil_sen(P[:, 0], P[:, 3])
    if vx is None or vy is None:
        return None
    rr = np.hypot(P[:, 1], P[:, 2])
    # robust spread (one stray sighting must not hide it): a hovering drone moves several cm within 2 s
    if len(P) >= 8 and P[-1, 0] - P[0, 0] >= 2.0 and np.std(rr) < 1.2 and np.std(P[:, 3]) < 1.2:
        return dict(static=True, n=len(P))   # not moving at all for >= 2 s: on the ground / in a hand, not hovering
    psi = float(np.median(P[:, 4]))
    f = (math.sin(psi), math.cos(psi))                     # body forward in the (X right, Y along) frame
    r = (math.cos(psi), -math.sin(psi))
    # drone position relative to the tag: moving TOWARD the tag = forward
    fwd = vx * f[0] + vy * f[1]
    right = vx * r[0] + vy * r[1]
    return dict(fwd=fwd, right=right, up=vz, n=len(P), span=float(P[-1, 0] - P[0, 0]), static=False,
                se=float(math.hypot(ex or 0, ey or 0)))


def turn_rate(rows):
    """VO heading slope; control.csv repeats the heading between frames (and holds it while VO is blind), so only
    rows where the heading changed are used - repeats would pull the robust slope to exactly 0."""
    t, h, last = [], [], None
    for r in rows:
        if r["vo"] is not None and r["vo"] != last:
            t.append(r["t"])
            h.append(r["vo"])
            last = r["vo"]
    if len(t) < 6:
        return None
    s, _ = theil_sen(t, h)
    return s


# ------------------------------------------------------------------ per takeoff
def analyse(folders, window_s, gap_min):
    tl = trim_timeline(folders)
    takeoffs, windows = [], []
    for f in folders:
        name = os.path.basename(os.path.normpath(f))
        t0w = folder_time(f)
        ev = load_events(f)
        ctl = load_control(f)
        if not ev or not ctl:
            continue
        tk = [t for t, e in ev if e.startswith("TAKEOFF")]
        for i, ta in enumerate(tk):
            nxt = tk[i + 1] if i + 1 < len(tk) else 1e9
            land = next(((t, e) for t, e in ev if ta < t < nxt and e.startswith("LAND")), (None, ""))
            over = next((t for t, e in ev if ta < t < nxt and e.startswith("landing time over")), None)
            t_land = land[0] if land[0] is not None else max(r["t"] for r in ctl if r["t"] < nxt)
            t_off = over if over is not None else t_land + LAND_DESCENT_S
            wall = t0w + timedelta(seconds=ta)
            tr, src = trims_at(tl, wall)
            rows = [r for r in ctl if ta <= r["t"] <= t_land]
            climb = [r for r in rows if ta + CLIMB_FROM <= r["t"] <= ta + CLIMB_S]
            hover = []
            for r in rows:
                if r["t"] < ta + CLIMB_S:
                    continue
                if r["state"] not in HOVER_STATES:
                    break
                hover.append(r)
            Pc, Ph = positions(climb), positions(hover)
            dc, dh = body_drift(Pc), body_drift(Ph)
            static = bool(dh and dh["static"])
            if dc and dc["static"]:
                dc = None
            if static:
                dh = None
            turn = None if static else turn_rate(hover)
            ycmd = float(np.mean([r["yaw"] - 128 for r in hover])) if hover else None
            appr = [r for r in rows if r["state"] == "APPROACH"]
            bat = next((int(m.group(1)) for t, e in ev if ta - 1.0 <= t <= ta + 0.5
                        for m in [re.search(r"status at takeoff: .*battery (\d+)", e)] if m), None)
            rec = dict(folder=name, k=i + 1, wall=wall, air_s=round(t_land - ta, 1), motors_s=round(t_off - ta, 1),
                       reason=land[1].replace("LAND: ", "")[:60],
                       setup="tilted" if TILT_FROM <= wall < TILT_TO else "level",
                       trim_pitch=tr["pitch"], trim_roll=tr["roll"], trim_yaw=tr["yaw"],
                       trim_src=src["pitch"] if src["pitch"] == src["yaw"] else f"p {src['pitch']}, y {src['yaw']}",
                       hover_s=round(hover[-1]["t"] - hover[0]["t"], 1) if hover else 0.0,
                       climb_fwd=dc["fwd"] if dc else None, climb_n=dc["n"] if dc else 0,
                       climb_up=dc["up"] if dc else None,
                       fwd=dh["fwd"] if dh else None, right=dh["right"] if dh else None, up=dh["up"] if dh else None,
                       n_tag=dh["n"] if dh else 0, turn=turn, yaw_cmd=ycmd,
                       mode="approach" if appr else "hover", static=static, battery=bat)
            takeoffs.append(rec)
            # windows inside the hover stretch (within-flight trend)
            if hover:
                a = hover[0]["t"]
                while a + window_s <= hover[-1]["t"] + 1e-6:
                    wr = [r for r in hover if a <= r["t"] < a + window_s]
                    d = body_drift(positions(wr))
                    if static or (d and d["static"]):
                        a += window_s
                        continue
                    windows.append(dict(folder=name, k=i + 1, wall=wall, since=round(a - ta + window_s / 2, 2),
                                        setup=rec["setup"], trim_pitch=tr["pitch"], trim_roll=tr["roll"],
                                        trim_yaw=tr["yaw"], turn=turn_rate(wr),
                                        yaw_cmd=float(np.mean([r["yaw"] - 128 for r in wr])),
                                        fwd=d["fwd"] if d else None, right=d["right"] if d else None,
                                        up=d["up"] if d else None, n_tag=d["n"] if d else 0))
                    a += window_s
            # approach authority: forward speed per pitch byte (incl. trim)
            if appr:
                a = appr[0]["t"]
                while a + window_s <= appr[-1]["t"] + 1e-6:
                    wr = [r for r in appr if a <= r["t"] < a + window_s]
                    d = body_drift(positions(wr))
                    if d and not d["static"]:
                        windows.append(dict(folder=name, k=i + 1, wall=wall, since=round(a - ta + window_s / 2, 2),
                                            setup=rec["setup"], trim_pitch=tr["pitch"], trim_roll=tr["roll"],
                                            trim_yaw=tr["yaw"], turn=None, yaw_cmd=None, fwd=d["fwd"],
                                            right=d["right"], up=d["up"], n_tag=d["n"], approach=1,
                                            pitch_cmd=float(np.mean([r["pitch"] - 128 for r in wr]))))
                    a += window_s
    takeoffs.sort(key=lambda r: r["wall"])
    # sessions (battery proxy)
    sid, prev_end, cum, idx = 0, None, 0.0, 0
    for r in takeoffs:
        if prev_end is None or (r["wall"] - prev_end).total_seconds() > gap_min * 60:
            sid, cum, idx = sid + 1, 0.0, 0
        idx += 1
        r.update(session=sid, idx=idx, cum_motor_s=round(cum, 1))
        cum += r["motors_s"]
        prev_end = r["wall"] + timedelta(seconds=r["motors_s"])
    by = {(r["folder"], r["k"]): r for r in takeoffs}
    for w in windows:
        r = by[(w["folder"], w["k"])]
        w.update(session=r["session"], idx=r["idx"], cum_motor_s=round(r["cum_motor_s"] + w["since"], 1))
    return takeoffs, windows, tl


# ------------------------------------------------------------------ fits
def ols(X, y):
    X, y = np.asarray(X, float), np.asarray(y, float)
    b, *_ = np.linalg.lstsq(X, y, rcond=None)
    r = y - X @ b
    dof = max(1, len(y) - X.shape[1])
    cov = np.linalg.pinv(X.T @ X) * (r @ r / dof)
    return b, np.sqrt(np.diag(cov)), float(np.sqrt(r @ r / dof))


def fit_yaw_gain(windows):
    """turn rate = own(takeoff) + k * yaw byte command; k from within-takeoff variation (heading hold)."""
    ws = [w for w in windows if w.get("turn") is not None and w.get("yaw_cmd") is not None and not w.get("approach")]
    groups = {}
    for w in ws:
        groups.setdefault((w["folder"], w["k"]), []).append(w)
    xs, ys = [], []
    for g in groups.values():
        if len(g) < 2:
            continue
        mx = np.mean([w["yaw_cmd"] for w in g])
        my = np.mean([w["turn"] for w in g])
        xs += [w["yaw_cmd"] - mx for w in g]
        ys += [w["turn"] - my for w in g]
    if len(xs) < 8 or np.std(xs) < 0.5:
        return None, None, len(xs)
    b, se, _ = ols(np.column_stack([xs]), ys)
    return float(b[0]), float(se[0]), len(xs)


# ------------------------------------------------------------------ report
def fmt(v, n=1):
    return "-" if v is None else f"{v:+.{n}f}"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folders", nargs="*", default=[os.path.join(ROOT, "captures", "flights", "*")])
    ap.add_argument("--window", type=float, default=2.0, help="window length for within-flight trends, s")
    ap.add_argument("--session-gap", type=float, default=25.0, help="minutes between takeoffs that start a new session")
    a = ap.parse_args()
    folders = sorted({f for p in a.folders for f in glob.glob(p) if os.path.isdir(f)})
    takeoffs, windows, tl = analyse(folders, a.window, a.session_gap)
    os.makedirs(OUT, exist_ok=True)
    keys = ["session", "idx", "folder", "k", "wall", "setup", "mode", "static", "cum_motor_s", "air_s", "motors_s", "hover_s",
            "trim_pitch", "trim_roll", "trim_yaw", "trim_src", "climb_fwd", "climb_up", "climb_n", "fwd", "right", "up", "n_tag",
            "turn", "yaw_cmd", "battery", "reason"]
    with open(os.path.join(OUT, "takeoffs.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, keys, extrasaction="ignore")
        w.writeheader()
        for r in takeoffs:
            w.writerow({k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items()})
    wkeys = ["session", "idx", "folder", "k", "wall", "setup", "since", "cum_motor_s", "trim_pitch", "trim_roll",
             "trim_yaw", "fwd", "right", "up", "n_tag", "turn", "yaw_cmd", "approach", "pitch_cmd"]
    with open(os.path.join(OUT, "windows.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, wkeys, extrasaction="ignore")
        w.writeheader()
        for r in windows:
            w.writerow({k: (round(v, 2) if isinstance(v, float) else v) for k, v in r.items()})

    lines = []
    p = lines.append
    k_fit, k_se, k_n = fit_yaw_gain(windows)
    k_yaw = K_YAW
    p(f"{len(takeoffs)} takeoffs in {len({r['folder'] for r in takeoffs})} folders, "
      f"{len({r['session'] for r in takeoffs})} sessions (gap > {a.session_gap:g} min), "
      f"{len([w for w in windows if not w.get('approach')])} hover windows of {a.window:g} s")
    p(f"trim lines logged: {len(tl)}, first at {tl[0][0]:%m-%d %H:%M}" if tl else "no trim lines logged")
    p(f"yaw authority used: {K_YAW:.2f} deg/s per yaw byte (flight_review); from heading-hold windows it would be "
      f"{fmt(k_fit, 2)} (se {k_se:.2f}, n={k_n}) - unusable, the hold's command follows the turn"
      if k_fit is not None else f"yaw authority used: {K_YAW:.2f} deg/s per yaw byte (flight_review)")
    p(f"static takeoffs excluded (no movement >= 2 s while 'flying' = ground / hand): "
      f"{sum(1 for r in takeoffs if r['static'])}")
    p("")
    p("per session (medians over takeoffs; forward + = toward the tag, right +, turn + = right; trims in bytes):")
    p(f"{'ses':>3} {'start':>11} {'setup':>6} {'n':>3} {'motor min':>9} {'trim p/r/y':>12} {'climb fwd':>9} "
      f"{'climb up':>8} {'hover fwd':>9} {'right':>6} {'up':>6} {'turn':>6} {'own turn*':>9}")
    for s in sorted({r["session"] for r in takeoffs}):
        rs = [r for r in takeoffs if r["session"] == s]

        def med(key, rr=rs):
            v = [r[key] for r in rr if r[key] is not None]
            return float(np.median(v)) if v else None
        own = [r["turn"] - (k_yaw or 0) * r["yaw_cmd"] for r in rs if r["turn"] is not None and r["yaw_cmd"] is not None]
        tp = sorted({(r["trim_pitch"], r["trim_roll"], r["trim_yaw"]) for r in rs})
        tps = f"{tp[0][0]}/{tp[0][1]}/{tp[0][2]}" + ("+" if len(tp) > 1 else "")
        p(f"{s:>3} {rs[0]['wall']:%m-%d %H:%M} {rs[0]['setup']:>6} {len(rs):>3} "
          f"{(rs[-1]['cum_motor_s'] + rs[-1]['motors_s']) / 60:>9.1f} {tps:>12} {fmt(med('climb_fwd')):>9} "
          f"{fmt(med('climb_up')) if rs[0]['setup'] == 'level' else 'tilt':>8} "
          f"{fmt(med('fwd')):>9} {fmt(med('right')):>6} {fmt(med('up')):>6} {fmt(med('turn')):>6} "
          f"{fmt(float(np.median(own)) if own else None):>9}")
    p("  * own turn = hover turn rate minus the heading hold's share (yaw authority x mean hold command), at the trim")
    p("")
    # trim-corrected drift: drift = d0 + g * trim + c * cum_motor_min (pooled over takeoffs with tag data)
    for axis, tkey in (("fwd", "trim_pitch"), ("right", "trim_roll"), ("turn_own", "trim_yaw")):
        rs = []
        for r in takeoffs:
            if axis == "turn_own":
                if r["turn"] is None or r["yaw_cmd"] is None or k_yaw is None:
                    continue
                y = r["turn"] - k_yaw * r["yaw_cmd"]
            else:
                if r[axis] is None or r["n_tag"] < 8:
                    continue
                y = r[axis]
            rs.append((y, r[tkey], r["cum_motor_s"] / 60.0, r["idx"]))
        if len(rs) < 8:
            p(f"{axis}: too few takeoffs with data ({len(rs)})")
            continue
        Y = np.array([x[0] for x in rs])
        X = np.column_stack([np.ones(len(rs)), [x[1] for x in rs], [x[2] for x in rs]])
        b, se, sd = ols(X, Y)
        unit = "deg/s" if axis == "turn_own" else "cm/s"
        p(f"{axis:>8} = {b[0]:+.1f} {unit} untrimmed  {b[1]:+.2f} per trim byte (se {se[1]:.2f})  "
          f"{b[2]:+.2f} per minute of motor time in the session (se {se[2]:.2f})  [n={len(rs)}, scatter sd {sd:.1f}]")
    p("")
    # within flight: hover windows by time since takeoff
    p("within a flight (hover windows, median by time since takeoff):")
    p(f"{'since takeoff':>14} {'n':>4} {'fwd':>6} {'right':>6} {'up':>6} {'turn':>6}")
    hw = [w for w in windows if not w.get("approach")]
    for lo, hi in ((3, 6), (6, 10), (10, 15), (15, 22), (22, 31)):
        ws = [w for w in hw if lo <= w["since"] < hi]

        def m(key, ws=ws):
            v = [w[key] for w in ws if w[key] is not None and (key == "turn" or w["n_tag"] >= 5)]
            return (float(np.median(v)), len(v)) if v else (None, 0)
        p(f"{f'{lo}-{hi} s':>14} {len(ws):>4} {fmt(m('fwd')[0]):>6} {fmt(m('right')[0]):>6} {fmt(m('up')[0]):>6} "
          f"{fmt(m('turn')[0]):>6}")
    p("")
    # approach authority over the session
    aw = [w for w in windows if w.get("approach") and w["n_tag"] >= 6]
    if len(aw) >= 8:
        X = np.column_stack([np.ones(len(aw)), [w["pitch_cmd"] + w["trim_pitch"] for w in aw],
                             [w["cum_motor_s"] / 60 for w in aw]])
        b, se, sd = ols(X, [w["fwd"] for w in aw])
        p(f"approach forward speed = {b[0]:+.1f} cm/s {b[1]:+.2f} per pitch byte incl. trim (se {se[1]:.2f}) "
          f"{b[2]:+.2f} per motor-minute in the session (se {se[2]:.2f})  [n={len(aw)} windows, sd {sd:.1f}]")
    p("")
    p("per session vs motor-on minutes earlier in the session (battery proxy; slope per minute, se in brackets):")
    p(f"{'ses':>3} {'start':>11} {'n climb':>7} {'climb fwd':>14} {'climb up':>14} {'n appr win':>10} "
      f"{'appr fwd/byte':>13} {'appr fwd vs min':>15}")
    for s_ in sorted({r["session"] for r in takeoffs}):
        first = next(r for r in takeoffs if r["session"] == s_)
        rs = [r for r in takeoffs if r["session"] == s_ and r["climb_fwd"] is not None and r["climb_n"] >= 6]
        cf = cu = "-"
        if len(rs) >= 4:
            X = np.column_stack([np.ones(len(rs)), [r["cum_motor_s"] / 60 for r in rs]])
            b, se, _ = ols(X, [r["climb_fwd"] for r in rs])
            cf = f"{b[1]:+.1f} ({se[1]:.1f})"
            ru = [r for r in rs if r["climb_up"] is not None]
            if len(ru) >= 4 and first["setup"] == "level":
                Xu = np.column_stack([np.ones(len(ru)), [r["cum_motor_s"] / 60 for r in ru]])
                bu, seu, _ = ols(Xu, [r["climb_up"] for r in ru])
                cu = f"{bu[1]:+.1f} ({seu[1]:.1f})"
        ws = [w for w in aw if w["session"] == s_]
        a1 = a2 = "-"
        if len(ws) >= 8:
            X = np.column_stack([np.ones(len(ws)), [w["pitch_cmd"] + w["trim_pitch"] for w in ws],
                                 [w["cum_motor_s"] / 60 for w in ws]])
            b, se, _ = ols(X, [w["fwd"] for w in ws])
            a1, a2 = f"{b[1]:+.2f} ({se[1]:.2f})", f"{b[2]:+.1f} ({se[2]:.1f})"
        p(f"{s_:>3} {first['wall']:%m-%d %H:%M} {len(rs):>7} {cf:>14} {cu:>14} {len(ws):>10} {a1:>13} {a2:>15}")
    rb = [r for r in takeoffs if r.get("battery") is not None]
    p("")
    if len(rb) >= 8:
        p(f"vs the battery byte ({len(rb)} takeoffs, {min(r['battery'] for r in rb)}-"
          f"{max(r['battery'] for r in rb)}):")
        for key in ("climb_fwd", "climb_up", "fwd", "turn"):
            rr = [r for r in rb if r[key] is not None]
            if len(rr) >= 6 and np.std([r["battery"] for r in rr]) > 0:
                b, se, sd = ols(np.column_stack([np.ones(len(rr)), [r["battery"] for r in rr]]), [r[key] for r in rr])
                p(f"  {key:>9}: {b[1]:+.2f} per battery unit (se {se[1]:.2f}), at battery 90: {b[0] + 90 * b[1]:+.1f} "
                  f"[n={len(rr)}, sd {sd:.1f}]")
    else:
        p(f"battery byte logged at {len(rb)} takeoffs (need >= 8 for a fit; logging added 2026-11-11)")
    txt = "\n".join(lines)
    open(os.path.join(OUT, "summary.txt"), "w").write(txt + "\n")
    print(txt)
    plots(takeoffs, windows, k_yaw)
    print(f"\nwrote {OUT}: takeoffs.csv, windows.csv, summary.txt, *.png")


def plots(takeoffs, windows, k_yaw):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    sess = sorted({r["session"] for r in takeoffs})
    cmap = plt.get_cmap("tab10")
    # 1) drift per takeoff vs motor time in its session (battery proxy)
    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    for key, a_, lab in (("fwd", ax[0, 0], "hover forward drift cm/s (+ toward tag)"),
                         ("right", ax[0, 1], "hover sideways drift cm/s (+ right)"),
                         ("climb_fwd", ax[1, 0], "drift during the 3 s auto climb cm/s (+ toward tag)"),
                         ("turn", ax[1, 1], "hover turn rate deg/s (+ right, heading hold on)")):
        for s in sess:
            rs = [r for r in takeoffs if r["session"] == s and r[key] is not None
                  and (key in ("turn",) or r.get("n_tag", 0) >= 8 or key == "climb_fwd" and r["climb_n"] >= 6)]
            if rs:
                a_.plot([r["cum_motor_s"] / 60 for r in rs], [r[key] for r in rs], "o-", ms=4, lw=0.8,
                        color=cmap(s % 10), label=f"s{s} {rs[0]['wall']:%m-%d %H:%M}")
        a_.axhline(0, color="k", lw=0.5)
        a_.set_xlabel("motor-on minutes earlier in the same session (battery proxy)")
        a_.set_title(lab, fontsize=10)
    ax[0, 0].legend(fontsize=7, ncol=2)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "drift_vs_session_time.png"), dpi=90)
    plt.close(fig)
    # 2) within flight
    fig, ax = plt.subplots(1, 3, figsize=(13, 4))
    hw = [w for w in windows if not w.get("approach")]
    for key, a_, lab in (("fwd", ax[0], "forward drift cm/s"), ("right", ax[1], "sideways drift cm/s"),
                         ("turn", ax[2], "turn rate deg/s (hold on)")):
        ws = [w for w in hw if w[key] is not None and (key == "turn" or w["n_tag"] >= 5)]
        a_.scatter([w["since"] for w in ws], [w[key] for w in ws], s=8, alpha=0.5)
        a_.axhline(0, color="k", lw=0.5)
        a_.set_xlabel("seconds since takeoff")
        a_.set_title(lab, fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "drift_within_flight.png"), dpi=90)
    plt.close(fig)
    # 3) timeline over days with trims
    fig, ax = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    xs = list(range(len(takeoffs)))
    ax[0].plot(xs, [r["fwd"] if r["n_tag"] >= 8 else np.nan for r in takeoffs], "o", ms=4, label="hover fwd")
    ax[0].plot(xs, [r["climb_fwd"] if r["climb_n"] >= 6 else np.nan for r in takeoffs], "x", ms=4, label="climb fwd")
    ax[0].plot(xs, [r["right"] if r["n_tag"] >= 8 else np.nan for r in takeoffs], "s", ms=3, label="hover right")
    ax[0].axhline(0, color="k", lw=0.5)
    ax[0].set_ylabel("cm/s")
    ax[0].legend(fontsize=8)
    ax[1].plot(xs, [r["turn"] if r["turn"] is not None else np.nan for r in takeoffs], "o", ms=4, label="turn (hold on)")
    if k_yaw is not None:
        ax[1].plot(xs, [r["turn"] - k_yaw * r["yaw_cmd"] if r["turn"] is not None and r["yaw_cmd"] is not None
                        else np.nan for r in takeoffs], ".", ms=5, label="own turn (hold share removed)")
    ax[1].axhline(0, color="k", lw=0.5)
    ax[1].set_ylabel("deg/s")
    ax[1].legend(fontsize=8)
    for key, m in (("trim_pitch", "-"), ("trim_roll", "--"), ("trim_yaw", ":")):
        ax[2].step(xs, [r[key] for r in takeoffs], m, where="mid", label=key)
    ax[2].set_ylabel("trim bytes")
    ax[2].legend(fontsize=8)
    for i, r in enumerate(takeoffs):
        if i == 0 or r["session"] != takeoffs[i - 1]["session"]:
            for a_ in ax:
                a_.axvline(i - 0.5, color="grey", lw=0.6)
            ax[0].text(i, ax[0].get_ylim()[1], f"{r['wall']:%m-%d %H:%M}", fontsize=7, rotation=90, va="top")
    ax[2].set_xlabel("takeoff number (time order; grey lines = new session)")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "drift_timeline.png"), dpi=90)
    plt.close(fig)


if __name__ == "__main__":
    main()
