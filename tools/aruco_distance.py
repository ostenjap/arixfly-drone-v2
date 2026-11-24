"""Predict the distance from the drone camera to each ArUco tag in a JPEG (Day 4 groundwork).

Pinhole model: one focal length f (px), principal point = image centre, no lens distortion. Pose per tag
comes from cv2.solvePnP (IPPE_SQUARE) with the tag's printed size, so it also works when the tag is
tilted. Without a calibration f comes from an assumed horizontal field of view (--hfov) -> rough only.

Calibrate once: put tag 2 (table B) in the middle of the image at a ruler-measured distance from the
camera lens, grab a frame, then
  python tools/aruco_distance.py frame.jpg --calibrate 50        # tag was 50 cm away -> calib/drone_cam.json
Predict (and test against a ruler):
  python tools/aruco_distance.py frames/*.jpg                    # distance per tag, annotated copies in out/
  python tools/aruco_distance.py frame.jpg --truth 80            # also print the error vs. 80 cm
  python tools/aruco_distance.py --selftest                      # synthetic tags at known distances
Tag sizes (black square side, cm) default to the A4 sheet: ID 0 = 4, ID 1 = 10, ID 2 = 10.
Override after measuring the print:  --size 2=9.8 --size 0=4.0
"""
import argparse
import glob
import json
import math
import os
import statistics

import cv2
import numpy as np

from aruco_track import NAMES, detect

HERE = os.path.dirname(os.path.abspath(__file__))
CALIB = os.path.join(HERE, "..", "calib", "drone_cam.json")
SIZES_CM = {0: 4.0, 1: 10.0, 2: 10.0}
DEFAULT_HFOV = 80.0  # guess for a small FPV camera; replaced by --calibrate


def focal_from_hfov(width, hfov_deg):
    return (width / 2) / math.tan(math.radians(hfov_deg) / 2)


def load_focal(width):
    """(f_px, source) for an image of this width; calibration is stored per width and rescaled."""
    try:
        c = json.load(open(CALIB))
        return c["f_px"] * width / c["width"], f"calibrated ({os.path.basename(CALIB)})"
    except (OSError, KeyError, ValueError):
        return None, None


def pose(tag, size_cm, f, w, h):
    """Tag position in camera coordinates: x right, y down, z forward (cm)."""
    s = size_cm / 2
    obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float32)  # IPPE_SQUARE order
    K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]], np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, tag["corners"].astype(np.float32), K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    x, y, z = tvec.ravel()
    R = cv2.Rodrigues(rvec)[0]
    cam = -R.T @ tvec.ravel()  # camera in tag coordinates: x = sideways from the tag's normal (autopilot roll)
    return {"x": x, "y": y, "z": z, "dist": float(np.linalg.norm(tvec)), "lat": float(cam[0]),
            "off_deg": math.degrees(math.atan2(cam[0], cam[2])),
            "bearing": math.degrees(math.atan2(x, z)), "elev": math.degrees(math.atan2(-y, z)),
            "simple": f * size_cm / tag["side"]}  # size-only estimate, valid when the tag faces the camera


def analyse(img, sizes, f):
    h, w = img.shape[:2]
    out = {}
    for i, t in detect(img).items():
        if i in sizes:
            p = pose(t, sizes[i], f, w, h)
            if p:
                out[i] = dict(p, tag=t)
    return out


def annotate(img, res, f_src):
    vis = img.copy()
    for i, r in res.items():
        pts = r["tag"]["corners"].astype(int)
        cv2.polylines(vis, [pts], True, (0, 255, 0), 2)
        x, y = pts[:, 0].min(), pts[:, 1].min() - 8
        cv2.putText(vis, f"{NAMES.get(i, i)} {r['dist']:.0f} cm  {r['bearing']:+.0f} deg", (int(x), int(max(y, 15))),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    cv2.putText(vis, f"f: {f_src}", (8, vis.shape[0] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return vis


def calibrate(files, true_cm, sizes, hfov):
    """f such that the PnP distance of the tags equals the ruler distance (median over images and tags)."""
    fs, width = [], None
    for path in files:
        img = cv2.imread(path)
        if img is None:
            continue
        width = img.shape[1]
        f0 = focal_from_hfov(width, hfov)
        for i, r in analyse(img, sizes, f0).items():
            fs.append(f0 * true_cm / r["dist"])  # PnP distance scales linearly with f
            print(f"  {os.path.basename(path)}: tag {i} side {r['tag']['side']:.1f} px -> f {fs[-1]:.0f} px")
    if not fs:
        raise SystemExit("no known tag found in the calibration image(s)")
    f = statistics.median(fs)
    os.makedirs(os.path.dirname(CALIB), exist_ok=True)
    json.dump({"f_px": f, "width": width, "hfov_deg": round(math.degrees(2 * math.atan(width / 2 / f)), 1),
               "true_cm": true_cm, "n": len(fs), "images": [os.path.basename(p) for p in files]},
              open(CALIB, "w"), indent=2)
    print(f"f = {f:.0f} px for {width} px wide images (hfov {math.degrees(2 * math.atan(width / 2 / f)):.1f} deg)"
          f" -> {os.path.normpath(CALIB)}")


def predict(files, sizes, hfov, truth, outdir):
    errs = []
    for path in files:
        img = cv2.imread(path)
        if img is None:
            print(f"{path}: not an image")
            continue
        h, w = img.shape[:2]
        f, src = load_focal(w)
        if f is None:
            f, src = focal_from_hfov(w, hfov), f"UNCALIBRATED, assumed hfov {hfov:g} deg"
        res = analyse(img, sizes, f)
        print(f"{os.path.basename(path)} ({w}x{h}, f {f:.0f} px, {src})")
        if not res:
            print("  no known tag found")
        for i, r in sorted(res.items()):
            line = (f"  tag {i} ({NAMES.get(i, i)}, {sizes[i]:g} cm): {r['dist']:6.1f} cm  "
                    f"(forward {r['z']:.1f}, right {r['x']:+.1f}, up {-r['y']:+.1f} cm; bearing {r['bearing']:+.1f}"
                    f" deg, elev {r['elev']:+.1f} deg; size-only {r['simple']:.1f} cm; side {r['tag']['side']:.0f} px)")
            if truth:
                errs.append(r["dist"] - truth)
                line += f"  error {errs[-1]:+.1f} cm ({100 * errs[-1] / truth:+.1f} %)"
            print(line)
        if outdir:
            os.makedirs(outdir, exist_ok=True)
            cv2.imwrite(os.path.join(outdir, os.path.basename(path)), annotate(img, res, src))
    if errs:
        print(f"mean abs error {statistics.mean(abs(e) for e in errs):.1f} cm over {len(errs)} detections")


def render_tag_scene(id_, size_cm, dist_cm, f, w=640, h=480, yaw_deg=0.0, x_cm=0.0):
    """Synthetic camera image of one tag at a known pose (for --selftest)."""
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    n = 400
    tag = cv2.copyMakeBorder(cv2.aruco.generateImageMarker(d, id_, n), 80, 80, 80, 80, cv2.BORDER_CONSTANT, value=255)
    m = n / size_cm  # tag px per cm
    a = math.radians(yaw_deg)
    R = np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])
    src, dst = [], []
    for u, v in ((0, 0), (n, 0), (n, n), (0, n)):
        p = R @ np.array([(u - n / 2) / m, (v - n / 2) / m, 0]) + [x_cm, 0, dist_cm]
        src.append((u + 80, v + 80))
        dst.append((f * p[0] / p[2] + w / 2, f * p[1] / p[2] + h / 2))
    H = cv2.getPerspectiveTransform(np.float32(src), np.float32(dst))
    img = cv2.warpPerspective(tag, H, (w, h), borderValue=200)
    img = cv2.GaussianBlur(img, (3, 3), 0)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def selftest():
    w, f = 640, focal_from_hfov(640, DEFAULT_HFOV)
    print(f"synthetic 640x480 camera, f {f:.0f} px, tag 2 = 10 cm")
    worst = 0
    for dist, yaw, x in ((30, 0, 0), (60, 0, 5), (100, 25, -10), (150, 0, 0), (200, 40, 15), (300, 0, 0)):
        img = render_tag_scene(2, 10.0, dist, f, yaw_deg=yaw, x_cm=x)
        r = analyse(img, {2: 10.0}, f).get(2)
        true = math.hypot(dist, x)
        if r is None:
            print(f"  true {true:6.1f} cm (yaw {yaw}): NOT DETECTED")
            continue
        e = 100 * (r["dist"] - true) / true
        worst = max(worst, abs(e))
        print(f"  true {true:6.1f} cm, yaw {yaw:2} deg, x {x:+3} cm -> {r['dist']:6.1f} cm ({e:+.1f} %), "
              f"side {r['tag']['side']:.0f} px")
    print(f"worst error {worst:.1f} %")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("images", nargs="*")
    ap.add_argument("--calibrate", type=float, metavar="CM", help="images show tags at this ruler distance")
    ap.add_argument("--truth", type=float, metavar="CM", help="ruler distance, prints the prediction error")
    ap.add_argument("--size", action="append", default=[], metavar="ID=CM")
    ap.add_argument("--hfov", type=float, default=DEFAULT_HFOV, help="assumed hfov when not calibrated")
    ap.add_argument("--out", default=os.path.join(HERE, "..", "captures", "aruco_out"))
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        return selftest()
    sizes = dict(SIZES_CM)
    for s in a.size:
        k, v = s.split("=")
        sizes[int(k)] = float(v)
    files = [p for g in a.images for p in (glob.glob(g) or [g])]
    if not files:
        ap.error("give one or more JPEG files (or --selftest)")
    if a.calibrate:
        calibrate(files, a.calibrate, sizes, a.hfov)
    else:
        predict(files, sizes, a.hfov, a.truth, a.out)


if __name__ == "__main__":
    main()
