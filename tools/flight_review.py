"""Offline frame-by-frame review of a live_flight.py flight folder (no drone, no Wi-Fi).

For every takeoff in the folder it reconstructs, from the saved drone-camera frames + logs:
  - drone position relative to tag 2 (range, sideways from the tag's normal, height above the tag centre).
    The sideways value from one tag view has two candidate solutions (IPPE planar ambiguity: flips sign live at
    ~1 m); here both are computed and the time-consistent one is chosen over the whole flight (Viterbi over the 2
    candidates, cost = jump between frames + reprojection error), which live control cannot do.
  - heading from visual odometry (tracker.YawVO), picture sharpness (Laplacian variance).
  - drift with sticks centred (only trim applied): heading deg/s, range / sideways / height cm/s.
  - yaw response: heading rate vs. commanded yaw (least squares with 0.15 s lag) -> deg/s per power unit.
  - every tag loss in the air and its likely cause: left the view (edge + side), blur, or not detected in view.
Writes <folder>/review/: flight_<n>.png (timeline), frames_review.csv, summary.txt; prints the summary.
Trims: control.csv holds sticks BEFORE trim; trims are rebuilt from the "tune ..." lines in events.csv
(flights before 2026-11-10 had no trims).

Usage:
  python tools/flight_review.py captures/flights/20261110_010237
  python tools/flight_review.py captures/flights/2026111*        # several folders
"""
import argparse
import csv
import glob
import os
import re
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import tracker  # noqa: E402
from aruco_distance import load_focal  # noqa: E402
from flight_control import TAG_CM, find_tag  # noqa: E402

AIR = ("FLYING", "ACQUIRE", "APPROACH", "RECOVER", "HOP", "BRAKE", "DESCEND")
STICK_LIMIT, THROTTLE_LIMIT = 0.40, 0.55
EDGE = 0.08            # last sighting within 8 % of the frame border -> "left the view"
BLUR_VAR = 25.0        # Laplacian variance (320 px gray) below this = blurred: ground ~1900, air 40-100, VO/tag fail < ~25
PRE_S = 2.0            # also review this long before each takeoff (ground reference)


# ---------------------------------------------------------------- per-frame measurement
def pose_candidates(corners, f, w, h):
    """Both IPPE solutions as (vector [side, along, height] in cm, reprojection error).
    Gravity-aligned, independent of how the tag is turned on its box (camera assumed level):
      height = camera above the tag centre (= tag's downward offset in the camera, t_y)
      along  = distance along the tag's normal (horizontal), side = offset from that normal,
               + = camera to the RIGHT of the normal as seen by someone standing at the drone facing the tag.
    The two IPPE solutions differ in the tag normal -> they differ in side/along; range and height agree."""
    s = TAG_CM / 2
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)
    K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]], np.float64)
    n, rvecs, tvecs, errs = cv2.solvePnPGeneric(obj, corners.astype(np.float32), K, None,
                                                flags=cv2.SOLVEPNP_IPPE_SQUARE)
    out = []
    for rv, tv, e in zip(rvecs, tvecs, np.ravel(errs) if errs is not None else [0.0] * n):
        R = cv2.Rodrigues(rv)[0]
        t = tv.ravel()                      # tag centre in camera coords: x right, y down, z forward
        nrm = R[:, 2]                       # tag normal in camera coords
        nh = np.array([nrm[0], nrm[2]])
        if np.linalg.norm(nh) < 1e-6:
            continue
        nh /= np.linalg.norm(nh)
        c = -np.array([t[0], t[2]])         # camera relative to the tag, horizontal plane (x, z)
        if nh @ c < 0:
            nh = -nh                        # normal pointing from the tag toward the camera
        along = float(nh @ c)
        side = float(nh[0] * c[1] - nh[1] * c[0])   # sign checked with sim-rendered frames + real detector
        out.append((np.array([side, along, float(t[1])]), float(e)))
    return out


def measure(folder, rows, f):
    """One dict per frame (only frames near/in flights)."""
    keep = set()
    air_rows = [r for r in rows if r["state"] in AIR + ("LANDING",)]
    lo_hi = []
    if air_rows:   # time windows: PRE_S before each air block until its end
        ts = [float(r["t"]) for r in air_rows]
        start = prev = ts[0]
        for t in ts[1:]:
            if t - prev > 3.0:
                lo_hi.append((start - PRE_S, prev))
                start = t
            prev = t
        lo_hi.append((start - PRE_S, prev))
    for r in rows:
        t = float(r["t"])
        if any(a <= t <= b for a, b in lo_hi):
            keep.add(int(r["frame"]))
    vo, out, last_t = None, [], None
    for r in rows:
        n, t = int(r["frame"]), float(r["t"])
        if n not in keep:
            continue
        img = cv2.imread(os.path.join(folder, f"frame_{n:04d}.jpg"))
        if img is None:
            continue
        h, w = img.shape[:2]
        if vo is None or (last_t is not None and t - last_t > 3.0):
            vo = tracker.YawVO(f, w)          # new block: restart heading at 0
        last_t = t
        vo.update(img, t)
        small = cv2.cvtColor(cv2.resize(img, (320, 180)), cv2.COLOR_BGR2GRAY)
        d = dict(frame=n, t=t, state=r["state"], heading=vo.heading, vo_ok=vo.valid,
                 blur=float(cv2.Laplacian(small, cv2.CV_64F).var()), seen=False)
        s = find_tag(img, f)
        if s:
            c = s["corners"]
            d.update(seen=True, dist=s["dist"], bearing=s["bearing"], elev=s["elev"],
                     cx=float(c[:, 0].mean() / w), cy=float(c[:, 1].mean() / h),
                     edge=float(min(c[:, 0].min() / w, 1 - c[:, 0].max() / w, c[:, 1].min() / h,
                                    1 - c[:, 1].max() / h)),
                     cands=pose_candidates(c, f, w, h))
        out.append(d)
    return out


def resolve_sideways(frames):
    """Pick one IPPE candidate per sighting so the camera path is smooth (Viterbi, 2 states)."""
    seen = [d for d in frames if d["seen"] and len(d.get("cands", [])) >= 1]
    if not seen:
        return
    INF = 1e18
    cost = [[c[1] * 2.0 for c in seen[0]["cands"]]]
    back = [[0] * len(seen[0]["cands"])]
    for k in range(1, len(seen)):
        prev, cur = seen[k - 1], seen[k]
        dt = max(0.05, cur["t"] - prev["t"])
        row, brow = [], []
        for c in cur["cands"]:
            best, arg = INF, 0
            for j, p in enumerate(prev["cands"]):
                jump = float(np.linalg.norm(c[0] - p[0])) / min(dt / 0.06, 10.0)
                v = cost[k - 1][j] + jump + c[1] * 2.0
                if v < best:
                    best, arg = v, j
            row.append(best)
            brow.append(arg)
        cost.append(row)
        back.append(brow)
    j = int(np.argmin(cost[-1]))
    for k in range(len(seen) - 1, -1, -1):
        v = seen[k]["cands"][j][0]
        seen[k].update(side=float(v[0]), height=float(v[2]))
        j = back[k][j]


# ---------------------------------------------------------------- logs
def load_events(folder):
    ev = []
    p = os.path.join(folder, "events.csv")
    if os.path.exists(p):
        for r in csv.DictReader(open(p)):
            ev.append((float(r["t"]), r["event"]))
    return ev


def trims_at(events):
    """function t -> {yaw, roll, pitch} trim bytes, rebuilt from 'tune trim_x = v' events (start: 0)."""
    changes = []
    for t, e in events:
        m = re.match(r"tune (trim_\w+) = (-?\d+)", e)
        if m:
            changes.append((t, m.group(1)[5:], int(m.group(2))))

    def at(t):
        cur = {"yaw": 0, "roll": 0, "pitch": 0}
        for tc, k, v in changes:
            if tc <= t:
                cur[k] = v
        return cur
    return at


def load_control(folder):
    p = os.path.join(folder, "control.csv")
    return list(csv.DictReader(open(p))) if os.path.exists(p) else []


def power(byte, axis):
    return (int(byte) - 128) / ((THROTTLE_LIMIT if axis == "throttle" else STICK_LIMIT) * 127)


# ---------------------------------------------------------------- analysis per flight
def flights(events):
    """[(t_takeoff, t_land or None, land reason)] from events."""
    out, cur = [], None
    for t, e in events:
        if e.startswith("TAKEOFF"):
            cur = [t, None, ""]
            out.append(cur)
        elif cur and cur[1] is None and (e.startswith("LAND") or e.startswith("KILL") or e.startswith("CRASH")):
            cur[1], cur[2] = t, e
    return out


def fit_rate(ts, vs):
    if len(ts) < 4 or ts[-1] - ts[0] < 1.0:
        return None
    a = np.polyfit(np.array(ts), np.array(vs), 1)
    return float(a[0])


def analyse_flight(k, t0, t1, reason, frames, ctl, trim_at, out_dir):
    t1 = t1 if t1 is not None else max(d["t"] for d in frames)
    fr = [d for d in frames if t0 - PRE_S <= d["t"] <= t1 + 0.5]
    air = [d for d in fr if t0 + 1.0 <= d["t"] <= t1]
    cl = [r for r in ctl if t0 <= float(r["t"]) <= t1]
    lines = [f"flight {k}: takeoff {t0:.1f} s, {t1 - t0:.1f} s in the air, ended: {reason or 'end of log'}"]
    if not air:
        return lines + ["  no frames in the air"], None
    seen = [d for d in air if d["seen"]]
    blur = [d for d in air if d["blur"] < BLUR_VAR]
    lines.append(f"  frames in the air {len(air)}, tag seen {len(seen)} ({100 * len(seen) / len(air):.0f} %), "
                 f"blurred {len(blur)} ({100 * len(blur) / len(air):.0f} %), VO ok "
                 f"{100 * sum(d['vo_ok'] for d in air) / len(air):.0f} %")
    tr = trim_at(t0)
    lines.append(f"  trims at takeoff: yaw {tr['yaw']:+d}, pitch {tr['pitch']:+d}, roll {tr['roll']:+d}")

    # drift with centred sticks (only trim): stretches where the logged sticks were all 128 in FLYING
    centred_t = {round(float(r["t"]), 2) for r in cl if r["state"] == "FLYING" and
                 r["yaw"] == r["pitch"] == r["roll"] == r["throttle"] == "128"}
    ct = sorted(centred_t)
    if ct:
        segs, s0, prev = [], ct[0], ct[0]
        for t in ct[1:]:
            if t - prev > 0.2:
                segs.append((s0, prev))
                s0 = t
            prev = t
        segs.append((s0, prev))
        segs = [s for s in segs if s[1] - s[0] >= 1.5]
        for a, b in segs:
            seg = [d for d in air if a <= d["t"] <= b]
            hv = [(d["t"], d["heading"]) for d in seg if d["vo_ok"]]
            hr = fit_rate([x for x, _ in hv], [y for _, y in hv])
            sg = [d for d in seg if d["seen"] and "side" in d]
            parts = [f"  drift, sticks centred {a:.1f}-{b:.1f} s ({b - a:.1f} s):"]
            if hr is not None:
                parts.append(f"turn {hr:+.1f} deg/s ({'right' if hr > 0 else 'left'})")
            for key, name in (("dist", "range"), ("side", "sideways"), ("height", "height")):
                r_ = fit_rate([d["t"] for d in sg], [d[key] for d in sg])
                if r_ is not None:
                    parts.append(f"{name} {r_:+.0f} cm/s")
            if len(parts) > 1:
                lines.append(" ".join(parts))

    # yaw response: heading rate vs commanded yaw power (incl. trim), lag 0.15 s
    rows = [(float(r["t"]), power(int(r["yaw"]) + trim_at(float(r["t"]))["yaw"], "yaw")) for r in cl]
    hv = [(d["t"], d["heading"]) for d in air if d["vo_ok"]]
    if len(hv) > 20 and rows:
        X, Y = [], []
        for (ta, ha), (tb, hb) in zip(hv, hv[1:]):
            if 0.03 < tb - ta < 0.2:
                tm = (ta + tb) / 2 - 0.15
                cmd = min(rows, key=lambda r: abs(r[0] - tm))[1]
                X.append(cmd)
                Y.append((hb - ha) / (tb - ta))
        if len(X) > 20 and np.std(X) > 0.03:
            A = np.vstack([X, np.ones(len(X))]).T
            gain, bias = np.linalg.lstsq(A, np.array(Y), rcond=None)[0]
            lines.append(f"  yaw response: {gain:+.0f} deg/s per power unit (commanded yaw incl. trim), "
                         f"own turn {bias:+.1f} deg/s at zero command  [n={len(X)}, noisy: VO rate per frame]")
        else:
            lines.append("  yaw response: not enough yaw command variation in this flight")

    # tag losses in the air
    losses = []
    prev = None
    for d in air:
        if prev is not None and prev["seen"] and not d["seen"]:
            nxt = [x for x in air if d["t"] <= x["t"] <= d["t"] + 0.5]
            blurred = sum(x["blur"] < BLUR_VAR for x in nxt) > len(nxt) / 2 if nxt else False
            if prev["edge"] < EDGE:
                where = ("left" if prev["cx"] < 0.5 else "right") if min(prev["cx"], 1 - prev["cx"]) < \
                    min(prev["cy"], 1 - prev["cy"]) else ("top" if prev["cy"] < 0.5 else "bottom")
                cause = f"left the view at the {where} edge"
            elif blurred:
                cause = "picture blurred (swing / spin)"
            else:
                cause = "not detected while in view (glare / small / angle)"
            back = next((x for x in air if x["t"] > d["t"] and x["seen"]), None)
            gap = (back["t"] - d["t"]) if back else None
            losses.append((d["t"], cause, gap, prev.get("dist")))
        prev = d
    long_losses = [x for x in losses if x[2] is None or x[2] > 0.3]
    lines.append(f"  tag losses in the air: {len(losses)} (longer than 0.3 s: {len(long_losses)})")
    for t, cause, gap, dist in long_losses[:12]:
        lines.append(f"    {t:7.2f} s at {dist:.0f} cm: {cause}; back after "
                     + (f"{gap:.1f} s" if gap is not None else "never"))

    # position summary
    sg = [d for d in air if d["seen"] and "side" in d]
    if sg:
        lines.append(f"  position (tag frame, when seen): range {min(d['dist'] for d in sg):.0f}-"
                     f"{max(d['dist'] for d in sg):.0f} cm, sideways {min(d['side'] for d in sg):+.0f}.."
                     f"{max(d['side'] for d in sg):+.0f} cm, height {min(d['height'] for d in sg):+.0f}.."
                     f"{max(d['height'] for d in sg):+.0f} cm above the tag centre")
    png = plot(k, t0, t1, fr, cl, trim_at, losses, out_dir)
    return lines, png


def plot(k, t0, t1, fr, cl, trim_at, losses, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(5, 1, figsize=(12, 11), sharex=True)
    tt = [d["t"] for d in fr]
    sg = [d for d in fr if d["seen"]]
    ax[0].plot([d["t"] for d in sg], [d["dist"] for d in sg], ".", ms=3, label="range cm")
    ax[0].set_ylabel("range to tag (cm)")
    ax[1].plot([d["t"] for d in sg if "side" in d], [d["side"] for d in sg if "side" in d], ".", ms=3,
               label="sideways (+ right of tag normal)")
    ax[1].plot([d["t"] for d in sg if "height" in d], [d["height"] for d in sg if "height" in d], ".", ms=3,
               label="height above tag centre")
    ax[1].set_ylabel("cm")
    ax[1].legend(loc="upper right", fontsize=8)
    ax[2].plot(tt, [d["heading"] for d in fr], "-", lw=1)
    ax[2].plot([d["t"] for d in fr if not d["vo_ok"]], [d["heading"] for d in fr if not d["vo_ok"]], "x",
               ms=3, color="red", label="VO not valid")
    ax[2].set_ylabel("VO heading (deg, + right)")
    ax[2].legend(loc="upper right", fontsize=8)
    if cl:
        ct = [float(r["t"]) for r in cl]
        for name in ("yaw", "pitch", "roll", "throttle"):
            trim = [trim_at(t).get(name, 0) if name != "throttle" else 0 for t in ct]
            ax[3].plot(ct, [int(r[name]) + tr for r, tr in zip(cl, trim)], lw=1, label=name)
        ax[3].set_ylabel("stick byte sent (incl. trim)")
        ax[3].legend(loc="upper right", fontsize=8, ncol=4)
    ax[4].plot(tt, [d["blur"] for d in fr], lw=1, label="sharpness")
    ax[4].axhline(BLUR_VAR, color="red", lw=0.8, ls="--", label="blur threshold")
    ax[4].plot([d["t"] for d in sg], [BLUR_VAR * 0.5] * len(sg), "|", color="green", label="tag seen")
    ax[4].set_yscale("log")
    ax[4].set_ylabel("sharpness")
    ax[4].legend(loc="upper right", fontsize=8)
    for a in ax:
        a.axvline(t0, color="k", lw=0.8)
        a.axvline(t1, color="k", lw=0.8, ls="--")
        for t, *_ in losses:
            a.axvline(t, color="orange", lw=0.5, alpha=0.6)
    # state bands
    for d0, d1 in zip(fr, fr[1:]):
        col = {"FLYING": "#fde68a", "ACQUIRE": "#fde68a", "APPROACH": "#fed7aa", "RECOVER": "#fbcfe8",
               "LANDING": "#ddd6fe"}.get(d0["state"])
        if col:
            ax[0].axvspan(d0["t"], d1["t"], color=col, lw=0)
    ax[4].set_xlabel("time (s, flight log clock)   black = takeoff, dashed = land, orange = tag lost")
    fig.suptitle(f"flight {k}: yellow fly/acquire, orange approach, pink recover, purple landing")
    fig.tight_layout()
    path = os.path.join(out_dir, f"flight_{k}.png")
    fig.savefig(path, dpi=80)
    plt.close(fig)
    return path


def review(folder):
    rows = list(csv.DictReader(open(os.path.join(folder, "frames.csv"))))
    events = load_events(folder)
    fl = flights(events)
    out_dir = os.path.join(folder, "review")
    os.makedirs(out_dir, exist_ok=True)
    head = [f"=== {os.path.basename(os.path.normpath(folder))}: {len(rows)} frames, {len(fl)} takeoffs"]
    if not fl:
        return head + ["  no takeoff in this folder"]
    f = load_focal(1280)[0]
    frames = measure(folder, rows, f)
    resolve_sideways(frames)
    ctl, trim_at = load_control(folder), trims_at(events)
    lines = head
    for k, (t0, t1, reason) in enumerate(fl, 1):
        ls, png = analyse_flight(k, t0, t1, reason, frames, ctl, trim_at, out_dir)
        lines += ls + ([f"  timeline: {os.path.normpath(png)}"] if png else [])
    with open(os.path.join(out_dir, "frames_review.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["frame", "t", "state", "seen", "range_cm", "side_cm", "height_cm", "bearing_deg", "elev_deg",
                    "vo_heading_deg", "vo_ok", "sharpness"])
        for d in frames:
            w.writerow([d["frame"], d["t"], d["state"], int(d["seen"]),
                        *(round(d[k2], 1) if k2 in d else "" for k2 in ("dist", "side", "height", "bearing", "elev")),
                        round(d["heading"], 2), int(d["vo_ok"]), round(d["blur"])])
    with open(os.path.join(out_dir, "summary.txt"), "w") as fh:
        fh.write("\n".join(lines) + "\n")
    return lines


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folders", nargs="+")
    a = ap.parse_args()
    for pat in a.folders:
        for folder in sorted(glob.glob(pat)) or [pat]:
            if os.path.exists(os.path.join(folder, "frames.csv")):
                print("\n".join(review(folder)), flush=True)


if __name__ == "__main__":
    main()
