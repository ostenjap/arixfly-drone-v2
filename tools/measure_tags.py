"""Measure the tags on table B from saved drone-camera frames (offline, no drone, no Wi-Fi).

Uses the same two-pass detector as live_flight (flight_control.find_tag settings), but for every tag id.
Per tag: how often it is found, distance / bearing / elevation (median), apparent size in px, sharpness.
When tag 1 and tag 2 are both in a frame: their centre-to-centre gap and offset (cm), from the two poses.
Tilt per tag (as the drone's camera sees it, so put the drone flat and square to the board):
  turn = turned left/right about the vertical (from left vs right edge length), lean = top towards / away from
  the drone (top vs bottom edge length; includes any camera tilt), rotation = turned in its own plane (edge
  angles; includes any drone roll). Rotation is exact at any distance; turn/lean come from 1-2 px edge
  differences, so measure them at ~50 cm (~1-2 deg per px; at 120 cm ~7 deg per px). Writes <folder>/review/measure_tags.png (one frame with outlines + numbers).
With --true-cm (tape-measured distance from the drone's camera lens to tag 2's centre): checks the 10 cm size
assumption -> real black-square size = 10 * true / measured.

Ground run first (drone on the ground facing the board, props off or motors idle, nobody needs to fly):
  python tools/live_flight.py --test 10
Then:
  python tools/measure_tags.py captures/flights/<time>_test
  python tools/measure_tags.py captures/flights/<time>_test --true-cm 100
  python tools/measure_tags.py captures/reference/20261110_safe_baseline     # any folder of .jpg frames
"""
import argparse
import glob
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from aruco_distance import focal_from_hfov, load_focal, pose  # noqa: E402
from flight_control import TAG_CM  # noqa: E402

_DETS, _GAMMA2 = None, None


def detect_all(gray):
    """{id: corners (4x2)} with flight_control.find_tag's two passes; pass 2 only adds ids pass 1 missed."""
    global _DETS, _GAMMA2
    if _DETS is None:
        _DETS = []
        for const in (3, 1):
            prm = cv2.aruco.DetectorParameters()
            prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            prm.adaptiveThreshWinSizeMax, prm.adaptiveThreshConstant = 73, const
            _DETS.append(cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), prm))
        _GAMMA2 = np.array([255 * (i / 255.0) ** 2 for i in range(256)], np.uint8)
    found = {}
    for det, view in ((_DETS[0], gray), (_DETS[1], cv2.LUT(gray, _GAMMA2))):
        corners, ids, _ = det.detectMarkers(view)
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                found.setdefault(int(i), c[0])
    return found


def tilt(c, z, size_cm):
    """(turn, lean, rotation) in deg from corners (TL, TR, BR, BL). Edge length ~ 1/depth, so
    sin(turn) = (2 z / size) (L - R) / (L + R): + = the tag's left side is nearer the drone (it faces right).
    lean + = top nearer (leans towards the drone). rotation + = clockwise in the picture."""
    e = lambda a, b: float(np.linalg.norm(c[a] - c[b]))
    L, R, T, B = e(0, 3), e(1, 2), e(0, 1), e(3, 2)
    k = 2 * z / size_cm
    turn = np.degrees(np.arcsin(np.clip(k * (L - R) / (L + R), -1, 1)))
    lean = np.degrees(np.arcsin(np.clip(k * (T - B) / (T + B), -1, 1)))
    ang = lambda a, b, off: np.degrees(np.arctan2(c[b][1] - c[a][1], c[b][0] - c[a][0])) - off
    rot = float(np.mean([ang(0, 1, 0), ang(3, 2, 0), ang(0, 3, 90), ang(1, 2, 90)]))
    return float(turn), float(lean), rot


def verdict(turn, lean, rot):
    out = []
    for name, v, pos, neg in (("turn", turn, "faces to the right (its left side is nearer)",
                               "faces to the left (its right side is nearer)"),
                              ("lean", lean, "top leans towards the drone", "top leans away from the drone"),
                              ("rotation", rot, "rotated clockwise", "rotated anticlockwise")):
        if abs(v) >= 3:
            out.append(f"{name} {v:+.1f} deg: {pos if v > 0 else neg}")
    return "; ".join(out) or "straight (all within 3 deg)"


def med(v):
    return float(np.median(v)) if v else float("nan")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder", help="folder with frame_*.jpg (or any .jpg)")
    ap.add_argument("--size", type=float, default=TAG_CM, help="assumed black-square size, cm (default 10)")
    ap.add_argument("--true-cm", type=float, help="tape-measured lens -> tag 2 centre distance, cm")
    a = ap.parse_args()

    files = sorted(glob.glob(os.path.join(a.folder, "frame_*.jpg"))) or sorted(glob.glob(os.path.join(a.folder, "*.jpg")))
    if not files:
        sys.exit(f"no .jpg frames in {a.folder}")
    per, pairs, sharp, f = {}, [], [], None
    for fn in files:
        img = cv2.imread(fn)
        if img is None:
            continue
        h, w = img.shape[:2]
        if f is None:
            f = load_focal(w)[0] or focal_from_hfov(w, 58)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        sharp.append(cv2.Laplacian(gray, cv2.CV_64F).var())
        poses = {}
        for i, c in detect_all(gray).items():
            if i not in (1, 2):  # the board's tags only; other ids here were false hits (10 px specks)
                continue
            side = float(np.mean([np.linalg.norm(c[k] - c[(k + 1) % 4]) for k in range(4)]))
            r = pose({"corners": c, "side": side}, a.size, f, w, h)
            if r:
                r["side_px"] = side
                r["turn"], r["lean"], r["rot"] = tilt(c, r["z"], a.size)
                r["corners"], r["file"] = c, fn
                poses[i] = r
                per.setdefault(i, []).append(r)
        if 1 in poses and 2 in poses:
            d = np.array([poses[1][k] - poses[2][k] for k in ("x", "y", "z")])
            pairs.append(d)

    n = len(files)
    print(f"{n} frames in {a.folder}  (focal {f:.0f} px, assumed tag size {a.size:g} cm, "
          f"sharpness median {med(sharp):.0f}; ground ~1900, in air 40-100)")
    if not per:
        print("no tags found in any frame")
        return
    for i in sorted(per):
        rs = per[i]
        print(f"tag {i}: found in {len(rs)}/{n} frames ({100 * len(rs) / n:.0f} %)  "
              f"distance {med([r['dist'] for r in rs]):.1f} cm (spread {np.std([r['dist'] for r in rs]):.1f})  "
              f"bearing {med([r['bearing'] for r in rs]):+.1f} deg  elevation {med([r['elev'] for r in rs]):+.1f} deg  "
              f"size {med([r['side_px'] for r in rs]):.0f} px")
        t = [med([r[k] for r in rs]) for k in ("turn", "lean", "rot")]
        ts = [float(np.std([r[k] for r in rs])) for k in ("turn", "lean", "rot")]
        print(f"  tilt: turn {t[0]:+.1f} (spread {ts[0]:.1f})  lean {t[1]:+.1f} ({ts[1]:.1f})  "
              f"rotation {t[2]:+.1f} ({ts[2]:.1f}) deg -> {verdict(*t)}")
    snap(a.folder, per)
    if pairs:
        p = np.array(pairs)
        dx, dy, dz = np.median(p, axis=0)
        gap = float(np.median(np.linalg.norm(p, axis=1)))
        print(f"tag 1 vs tag 2 (both in {len(pairs)} frames): centre gap {gap:.1f} cm  "
              f"(tag 1 is {abs(dx):.1f} cm {'right' if dx > 0 else 'left'}, {abs(dy):.1f} cm "
              f"{'below' if dy > 0 else 'above'}, {abs(dz):.1f} cm {'further' if dz > 0 else 'nearer'})")
        if abs(dz) > 3:
            print("  note: the two tags are not in one flat plane facing the drone (depth differs > 3 cm)")
    elif 1 in per or 2 in per:
        print("tag 1 and tag 2 never in the same frame -> no gap measurement")
    if a.true_cm and 2 in per:
        m = med([r["dist"] for r in per[2]])
        print(f"tag 2 real size from the tape: {a.size * a.true_cm / m:.1f} cm "
              f"(measured {m:.1f} cm vs tape {a.true_cm:g} cm, {100 * (m - a.true_cm) / a.true_cm:+.1f} %)")


def snap(folder, per):
    """Save the frame with the most tags found, outlines + tilt numbers drawn, to <folder>/review/."""
    by_file = {}
    for i, rs in per.items():
        for r in rs:
            by_file.setdefault(r["file"], {})[i] = r
    fn = max(by_file, key=lambda k: (len(by_file[k]), k))
    img = cv2.imread(fn)
    for i, r in by_file[fn].items():
        pts = r["corners"].astype(np.int32)
        cv2.polylines(img, [pts], True, (0, 255, 0), 2)
        cv2.circle(img, tuple(pts[0]), 5, (0, 0, 255), -1)  # top-left corner
        x, y = int(pts[:, 0].min()), int(pts[:, 1].min()) - 10
        cv2.putText(img, f"tag {i}: {r['dist']:.0f} cm  turn {r['turn']:+.0f} lean {r['lean']:+.0f} "
                    f"rot {r['rot']:+.0f}", (max(x - 120, 5), max(y, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 0), 2)
    out = os.path.join(folder, "review")
    os.makedirs(out, exist_ok=True)
    cv2.imwrite(os.path.join(out, "measure_tags.png"), img)
    print(f"picture: {os.path.join(out, 'measure_tags.png')} (from {os.path.basename(fn)})")


if __name__ == "__main__":
    main()
