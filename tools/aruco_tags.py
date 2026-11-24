"""Make a printable A4 sheet of ArUco tags (DICT_4X4_50) for the drone and the tables.

  ID 0 = drone (small, goes on top of the HS420)
  ID 1 = table A (start)
  ID 2 = table B (landing spot)

Print at 100 % / "actual size" (no fit-to-page), then measure the black square of each tag with a
ruler and pass the real size to aruco_track.py.

Usage:
  python tools/aruco_tags.py                      # -> markers/aruco_sheet_A4.png (drone 4 cm, tables 10 cm)
  python tools/aruco_tags.py --drone-cm 3 --table-cm 12
"""
import argparse
import os

import cv2
import numpy as np

DPI = 300
A4_MM = (210, 297)
DICT = cv2.aruco.DICT_4X4_50


def px(cm):
    return round(cm / 2.54 * DPI)


def tag(id_, cm):
    d = cv2.aruco.getPredefinedDictionary(DICT)
    img = cv2.aruco.generateImageMarker(d, id_, px(cm))
    border = px(0.6)  # white quiet zone the detector needs
    return cv2.copyMakeBorder(img, border, border, border, border, cv2.BORDER_CONSTANT, value=255)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--drone-cm", type=float, default=4.0)
    ap.add_argument("--table-cm", type=float, default=10.0)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(__file__), "..", "markers", "aruco_sheet_A4.png"))
    a = ap.parse_args()

    w, h = round(A4_MM[0] / 25.4 * DPI), round(A4_MM[1] / 25.4 * DPI)
    sheet = np.full((h, w), 255, np.uint8)
    items = [(0, a.drone_cm, "ID 0  DRONE"), (1, a.table_cm, "ID 1  TABLE A (start)"), (2, a.table_cm, "ID 2  TABLE B (land)")]
    x, y = px(1.5), px(3.0)
    for id_, cm, label in items:
        t = tag(id_, cm)
        if x + t.shape[1] > w - px(1):
            x, y = px(1.5), y + row_h + px(1.5)
        sheet[y:y + t.shape[0], x:x + t.shape[1]] = t
        cv2.putText(sheet, f"{label}  {cm:g} cm", (x, y + t.shape[0] + px(0.6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.6, 0, 3)
        row_h = t.shape[0] + px(1)
        x += t.shape[1] + px(1.5)
    # 10 cm scale bar at the top to check the print size
    sy = px(1.6)
    cv2.rectangle(sheet, (px(1.5), sy), (px(1.5) + px(10), sy + px(0.3)), 0, -1)
    cv2.putText(sheet, "this bar must measure 10.0 cm when printed", (px(1.5), sy - px(0.3)),
                cv2.FONT_HERSHEY_SIMPLEX, 1.4, 0, 3)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    cv2.imwrite(a.out, sheet, [cv2.IMWRITE_PNG_COMPRESSION, 9])
    print(f"wrote {os.path.normpath(a.out)}  ({w}x{h} px @ {DPI} dpi = A4)")


if __name__ == "__main__":
    main()
