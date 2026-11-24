"""Track the drone tag (ID 0) relative to the landing tag on table B (ID 2) with a fixed webcam.

Gives the error drone -> table B in cm and the heading difference in degrees. The cm scale comes from
the table-B tag's size in the image, so it is accurate when the camera looks roughly straight down and
the drone sits at about the same height as table B (good enough for the Day 4 P controller).

Usage:
  python tools/aruco_track.py                         # live window, q = quit
  python tools/aruco_track.py --cam 1 --table-cm 10   # other camera / measured tag size
  python tools/aruco_track.py --headless --frames 30  # no window, print detections (for tests)
"""
import argparse
import math
import time

import cv2
import numpy as np

DRONE_ID, TABLE_A_ID, TABLE_B_ID = 0, 1, 2
NAMES = {DRONE_ID: "drone", TABLE_A_ID: "A", TABLE_B_ID: "B"}
_params = cv2.aruco.DetectorParameters()
_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX  # selftest: far-tag distance error 5.9 % -> 4.3 %
_detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), _params)


def detect(frame):
    """Return {id: {"c": (x, y) px, "side": px, "deg": heading}} for all visible tags."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    corners, ids, _ = _detector.detectMarkers(gray)
    out = {}
    if ids is None:
        return out
    for c, i in zip(corners, ids.flatten()):
        p = c[0]
        side = float(np.mean([np.linalg.norm(p[k] - p[(k + 1) % 4]) for k in range(4)]))
        top = p[1] - p[0]
        out[int(i)] = {"c": tuple(p.mean(axis=0)), "side": side,
                       "deg": math.degrees(math.atan2(top[1], top[0])), "corners": p}
    return out


def error_to_b(tags, table_cm):
    """(dx_cm, dy_cm, dist_cm, dyaw_deg) from drone to table B in image axes, or None."""
    if DRONE_ID not in tags or TABLE_B_ID not in tags:
        return None
    d, b = tags[DRONE_ID], tags[TABLE_B_ID]
    cm_per_px = table_cm / b["side"]
    dx = (b["c"][0] - d["c"][0]) * cm_per_px
    dy = (b["c"][1] - d["c"][1]) * cm_per_px
    dyaw = (b["deg"] - d["deg"] + 180) % 360 - 180
    return dx, dy, math.hypot(dx, dy), dyaw


def draw(frame, tags, err):
    for i, t in tags.items():
        pts = t["corners"].astype(int)
        cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
        cx, cy = map(int, t["c"])
        cv2.putText(frame, NAMES.get(i, str(i)), (cx - 15, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    if err:
        d, b = tags[DRONE_ID]["c"], tags[TABLE_B_ID]["c"]
        cv2.arrowedLine(frame, tuple(map(int, d)), tuple(map(int, b)), (0, 0, 255), 2)
        cv2.putText(frame, f"to B: dx {err[0]:+.1f} dy {err[1]:+.1f} = {err[2]:.1f} cm  yaw {err[3]:+.0f} deg",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--table-cm", type=float, default=10.0, help="measured side of the printed table-B tag")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--frames", type=int, default=0, help="stop after N frames (0 = run until q)")
    a = ap.parse_args()

    cap = cv2.VideoCapture(a.cam, cv2.CAP_DSHOW)
    if not cap.isOpened():
        raise SystemExit(f"camera {a.cam} not available")
    screen = None
    if not a.headless:  # installed OpenCV is headless (no imshow), so show frames with pygame
        import pygame
        pygame.init()
        pygame.display.set_caption("aruco_track (q = quit)")
    n, t0, running = 0, time.time(), True
    while running:
        ok, frame = cap.read()
        if not ok:
            break
        tags = detect(frame)
        err = error_to_b(tags, a.table_cm)
        n += 1
        if a.headless:
            seen = ", ".join(f"{NAMES.get(i, i)}@({t['c'][0]:.0f},{t['c'][1]:.0f})" for i, t in sorted(tags.items()))
            msg = f" | to B {err[2]:.1f} cm, yaw {err[3]:+.0f}" if err else ""
            print(f"frame {n:3}: {seen or 'no tags'}{msg}")
        else:
            draw(frame, tags, err)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            if screen is None:
                screen = pygame.display.set_mode((rgb.shape[1], rgb.shape[0]))
            screen.blit(pygame.surfarray.make_surface(rgb.swapaxes(0, 1)), (0, 0))
            pygame.display.flip()
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_q):
                    running = False
        if a.frames and n >= a.frames:
            break
    print(f"{n} frames, {n / max(time.time() - t0, 1e-6):.1f} fps")
    cap.release()
    if screen is not None:
        pygame.quit()


if __name__ == "__main__":
    main()
