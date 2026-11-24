"""Laptop-webcam drone body tracker (no marker on the drone): difference to a background picture.

  1. Background: median of a few frames taken with the drone NOT in view (hold it out of the picture, press B),
     then put the drone on table A. Light changes are followed slowly, never under the drone.
  2. Foreground = colour difference > thresh, minus shadow pixels (darker but same colour: ratio 0.55-0.95,
     chroma change < 0.035), then open/close to merge body + blurred props into one blob.
  3. The blob nearest to the estimator's predicted pixel (within gate_px) wins; without a prediction the one
     nearest the hint (table A's pixel) or the largest.
Output per frame: {u, v, size (sqrt area), area, bbox, blur, n_blobs} at the processing width, or None.

Usage (try it on recorded webcam frames):  python flight/bodytrack.py <folder with bg.jpg + frame_*.jpg>
"""
import glob
import os
import sys

import cv2
import numpy as np


class BodyTracker:
    def __init__(self, cfg, width=640):
        self.c, self.w = cfg, width
        self.bg = None
        self.k_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self.k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        self.mask = None

    def prep(self, img):
        h, w = img.shape[:2]
        if w != self.w:
            img = cv2.resize(img, (self.w, round(h * self.w / w)), interpolation=cv2.INTER_AREA)
        return img

    def set_background(self, frames):
        stack = np.stack([self.prep(f).astype(np.float32) for f in frames])
        self.bg = np.median(stack, axis=0)

    def foreground(self, img):
        f = img.astype(np.float32)
        diff = np.abs(f - self.bg).max(axis=2)
        fg = diff > self.c["thresh"]
        if self.c["shadow"]:
            gi, gb = f.sum(axis=2) + 1.0, self.bg.sum(axis=2) + 1.0
            ratio = gi / gb
            chroma = np.abs(f / gi[..., None] - self.bg / gb[..., None]).max(axis=2)
            lo, hi = self.c["shadow_ratio"]
            shadow = (ratio > lo) & (ratio < hi) & (chroma < self.c["shadow_chroma"])
            fg &= ~shadow
        m = fg.astype(np.uint8) * 255
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self.k_open)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, self.k_close)
        return m

    def detect(self, img, predict=None, hint=None, gate=None):
        """img: BGR frame (any width). predict: (u, v, size) from the estimator at this width, or None.
        hint: (u, v) where to look when there is no prediction (table A)."""
        img = self.prep(img)
        if self.bg is None or self.bg.shape[:2] != img.shape[:2]:
            return None
        m = self.foreground(img)
        self.mask = m
        n, lab, stats, cent = cv2.connectedComponentsWithStats(m, 8)
        H, W = m.shape
        cands = []
        for i in range(1, n):
            area = stats[i, cv2.CC_STAT_AREA]
            if area < self.c["min_area_px"] or area > self.c["max_area_frac"] * H * W:
                continue
            x, y, bw, bh = stats[i, :4]
            cands.append({"u": float(cent[i][0]), "v": float(cent[i][1]), "size": float(np.sqrt(area)),
                          "area": int(area), "bbox": (int(x), int(y), int(bw), int(bh)),
                          "blur": float(max(bw, bh) - np.sqrt(area))})
        best = None
        gate = gate or self.c["gate_px"]
        if predict is not None:
            pu, pv, ps = predict
            scored = []
            for c in cands:
                d = np.hypot(c["u"] - pu, c["v"] - pv)
                if d > gate + 0.5 * ps:
                    continue
                scored.append((d / gate + abs(np.log(max(c["size"], 1) / max(ps, 1))), c))
            if scored:
                best = min(scored, key=lambda s: s[0])[1]
        elif hint is not None and cands:
            best = min(cands, key=lambda c: np.hypot(c["u"] - hint[0], c["v"] - hint[1]))
        elif cands:
            best = max(cands, key=lambda c: c["area"])
        if best:
            best["n_blobs"] = len(cands)
        self._adapt(img, m, best)
        return best

    def _adapt(self, img, m, best):
        a = self.c["adapt"]
        if not a:
            return
        keep = cv2.dilate(m, self.k_close, iterations=2) == 0
        if best:
            x, y, bw, bh = best["bbox"]
            pad = max(bw, bh)
            keep[max(0, y - pad):y + bh + pad, max(0, x - pad):x + bw + pad] = False
        f = img.astype(np.float32)
        self.bg[keep] = (1 - a) * self.bg[keep] + a * f[keep]


def main():
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from flight.config import load_settings
    folder = sys.argv[1]
    cfg = load_settings()["bodytrack"]
    tr = BodyTracker(cfg)
    tr.set_background([cv2.imread(os.path.join(folder, "bg.jpg"))])
    for p in sorted(glob.glob(os.path.join(folder, "frame_*.jpg"))):
        d = tr.detect(cv2.imread(p))
        print(os.path.basename(p), "-" if d is None else f"u {d['u']:.0f} v {d['v']:.0f} size {d['size']:.1f} "
                                                         f"blobs {d['n_blobs']}")


if __name__ == "__main__":
    main()
