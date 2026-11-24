"""Fighter-jet style HUD for the HS420 camera: live video plus ArUco range and bearing to the target tag.

Only receives video (sends the `ef 00 04 00` video request, never a stick packet), so it cannot start
the motors. OpenCV does the image work and draws the HUD; pygame only shows the finished picture
(the installed OpenCV is the headless build without windows).

HUD: conformal bearing tape on top and pitch ladder (from the calibrated focal length, so the ticks match
real angles in the picture), range tape (right), closure-rate tape (left), target designator box with
LOCK after 3 frames in a row, big range readout with closure and time-to-target, 15 s range history.
Target = tag 2 (table B) if visible, else the nearest known tag.

Usage:
  python tools/hud.py                                      # live: joins drone Wi-Fi, home Wi-Fi again on quit
  python tools/hud.py --seconds 60                         # live, quits by itself after 60 s
  python tools/hud.py --replay captures/frames/test_100cm  # replay a folder of frames (or a video_probe .bin)
  python tools/hud.py --replay X --headless --seconds 3 --shot hud.png   # no window, save the last HUD frame
Keys: Q / Esc quit, F full screen, S screenshot (captures/hud/), Space pause (replay)
Live needs Mullvad off (or its local network sharing on); the laptop has no internet while it runs.
"""
import argparse
import glob
import math
import os
import subprocess
import threading
import time
from collections import deque

import cv2
import numpy as np

import drone_video as dv
from aruco_distance import DEFAULT_HFOV, SIZES_CM, analyse, focal_from_hfov, load_focal
from aruco_track import NAMES, TABLE_B_ID
from replay_test import DRONE_SSID, HOME_SSID, wait_for_ip, wifi_connect

HERE = os.path.dirname(os.path.abspath(__file__))
SHOTS = os.path.join(HERE, "..", "captures", "hud")
MULLVAD = r"C:\Program Files\Mullvad VPN\resources\mullvad.exe"
GREEN, DIM, AMBER, RED = (110, 255, 130), (60, 165, 85), (40, 190, 255), (80, 80, 255)
FONT = cv2.FONT_HERSHEY_SIMPLEX
CLOSE_CM = 25  # range below this turns the readout amber


# ---------------------------------------------------------------- frame sources (background threads)
class Source:
    mode = "?"

    def __init__(self):
        self.lock = threading.Lock()
        self.jpg, self.seq, self.t_frame = None, 0, None
        self.times = deque(maxlen=20)
        self.status, self.alive, self.paused = "STARTING", True, False

    def _push(self, jpg):
        now = time.monotonic()
        with self.lock:
            self.jpg, self.t_frame = jpg, now
            self.seq += 1
            self.times.append(now)

    def latest(self):
        with self.lock:
            return self.seq, self.jpg

    def fps(self):
        with self.lock:
            ts = list(self.times)
        if len(ts) < 2 or time.monotonic() - ts[-1] > 2 or ts[-1] <= ts[0]:
            return 0.0  # last check: frames in one clock tick (Windows monotonic = 15.6 ms) -> no division by 0
        return (len(ts) - 1) / (ts[-1] - ts[0])

    def age(self):
        return None if self.t_frame is None else time.monotonic() - self.t_frame

    def close(self):
        self.alive = False


def mullvad_blocks_lan():
    if not os.path.exists(MULLVAD):
        return False
    try:
        st = subprocess.run([MULLVAD, "status"], capture_output=True, text=True, timeout=5).stdout
        lan = subprocess.run([MULLVAD, "lan", "get"], capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return False
    return st.strip().lower().startswith("connected") and "block" in lan.lower()


class LiveSource(Source):
    mode = "LIVE"

    def __init__(self):
        super().__init__()
        self.joined, self.pkts, self.asm = False, 0, None
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def _run(self):
        if mullvad_blocks_lan():
            self.status = "MULLVAD BLOCKS LAN - DISCONNECT IT"
            return
        ip = None
        for attempt in range(1, 4):  # Windows sometimes needs >20 s or a second connect to get a drone IP
            if not self.alive:
                return
            self.status = f"JOINING DRONE WI-FI ({attempt}/3)"
            wifi_connect(DRONE_SSID)
            self.joined = True
            ip = wait_for_ip("192.168.169.")
            if ip:
                break
        if not ip:
            self.status = "NO DRONE WI-FI - DRONE ON?"
            return
        self.status = f"LINK {ip}"
        self.asm = asm = dv.Assembler()
        for _, d in dv.live_packets(1e9, cap=None, alive=lambda: self.alive):
            self.pkts += 1
            jpg = asm.feed(d)
            if jpg:
                if self.seq == 0:
                    self.status = f"LINK {ip}"
                self._push(jpg)
            elif self.seq == 0:
                self.status = f"LINK {ip}  {self.pkts} PKTS, NO FRAME YET"

    def close(self):
        self.alive = False
        self.th.join(timeout=2)
        if self.joined:
            wifi_connect(HOME_SSID)


class ReplaySource(Source):
    mode = "REPLAY"

    def __init__(self, path, fps=7.2):
        super().__init__()
        if os.path.isdir(path):
            files = sorted(f for f in glob.glob(os.path.join(path, "frame_*.jpg")) if not f.endswith("_aruco.jpg"))
            self.frames = [(i / fps, open(f, "rb").read()) for i, f in enumerate(files)]
        else:
            from video_probe import load
            asm, self.frames = dv.Assembler(), []
            for t, _, _, d in load(path):
                j = asm.feed(d)
                if j:
                    self.frames.append((t, j))
        if not self.frames:
            raise SystemExit(f"no frames in {path}")
        t0 = self.frames[0][0]
        self.frames = [(t - t0, j) for t, j in self.frames]
        self.status = f"REPLAY {len(self.frames)} FRAMES"
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def _run(self):
        i, start = 0, time.monotonic()
        while self.alive:
            if self.paused:
                time.sleep(0.02)
                start += 0.02
                continue
            t, j = self.frames[i]
            if time.monotonic() - start < t:
                time.sleep(0.005)
                continue
            self._push(j)
            i += 1
            if i == len(self.frames):
                i, start = 0, time.monotonic() + 1 / 7.2


# ---------------------------------------------------------------- target track (smoothed range, closure)
class Track:
    def __init__(self):
        self.d, self.vc, self.t_seen, self.streak, self.cur = None, 0.0, None, 0, None
        self.hist = deque()

    def update(self, tgt, now):
        self.cur = tgt
        if tgt is None:
            self.streak = 0
            return
        m = tgt["dist"]
        if self.d is None or now - self.t_seen > 1.0:
            self.d, self.vc = m, 0.0
        else:
            d_new = self.d + 0.5 * (m - self.d)
            rate = (self.d - d_new) / max(now - self.t_seen, 1e-3)  # + = closing
            self.vc += 0.3 * (rate - self.vc)
            self.d = d_new
        self.t_seen, self.streak = now, self.streak + 1
        self.hist.append((now, self.d))

    def expire(self, now):
        while self.hist and now - self.hist[0][0] > 15:
            self.hist.popleft()
        if self.t_seen and now - self.t_seen > 1.0:
            self.d, self.vc, self.cur, self.streak = None, 0.0, None, 0

    @property
    def locked(self):
        return self.streak >= 3


def pick(tags):
    if TABLE_B_ID in tags:
        return dict(tags[TABLE_B_ID], id=TABLE_B_ID)
    if tags:
        i = min(tags, key=lambda k: tags[k]["dist"])
        return dict(tags[i], id=i)
    return None


# ---------------------------------------------------------------- drawing helpers
def put(img, s, x, y, sc, col, th=1, anchor="l"):
    (w, h), _ = cv2.getTextSize(s, FONT, sc, th)
    x = x - w / 2 if anchor == "c" else x - w if anchor == "r" else x
    cv2.putText(img, s, (int(x), int(y)), FONT, sc, col, th, cv2.LINE_AA)
    return w, h


def line(img, p, q, col, th):
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), col, th, cv2.LINE_AA)


def dashed(img, p, q, col, th, dash, gap):
    p, q = np.array(p, float), np.array(q, float)
    length = np.linalg.norm(q - p)
    if length < 1:
        return
    d, s = (q - p) / length, 0.0
    while s < length:
        e = min(s + dash, length)
        line(img, p + d * s, p + d * e, col, th)
        s = e + gap


def tri(img, pts, col):
    cv2.fillConvexPoly(img, np.array(pts, np.int32), col, cv2.LINE_AA)


def box(img, x0, y0, x1, y1, col, th):
    cv2.rectangle(img, (int(x0), int(y0)), (int(x1), int(y1)), col, th, cv2.LINE_AA)


def panel(img, x0, y0, x1, y1, k=0.35):
    """Darken a region of the camera picture so HUD text stays readable over bright spots."""
    h, w = img.shape[:2]
    x0, y0, x1, y1 = max(0, int(x0)), max(0, int(y0)), min(w, int(x1)), min(h, int(y1))
    if x1 > x0 and y1 > y0:
        img[y0:y1, x0:x1] = (img[y0:y1, x0:x1] * k).astype(np.uint8)


def sgn0(v):
    return 0.0 if abs(v) < 0.05 else v  # no "-0.0"


# ---------------------------------------------------------------- the HUD
def render(frame, st):
    H, W = (720, 1280) if frame is None else frame.shape[:2]
    u = H / 720
    T = lambda v: max(1, int(round(v * u)))  # noqa: E731  (line widths scale with the picture)
    base = np.zeros((H, W, 3), np.uint8) if frame is None else cv2.convertScaleAbs(frame, alpha=0.72)
    hud = np.zeros_like(base)
    cx, cy, f, now = W / 2, H / 2, st["f"] * W / st["f_w"], st["now"]
    blink = int(now * 2.5) % 2 == 0
    tr = st["track"]
    d = tr.d
    tgt = tr.cur
    sm, md = 0.45 * u, 0.6 * u  # font scales
    vc = sgn0(tr.vc)
    rx, yt, yb, lx = W - 150 * u, 140 * u, H - 200 * u, 150 * u
    for p in ((0, 0, 260 * u, 70 * u), (W - 270 * u, 0, W, 70 * u), (cx - 70 * u, 0, cx + 70 * u, 36 * u),
              (rx - 105 * u, yt - 40 * u, rx + 60 * u, yb + 15 * u), (lx - 60 * u, yt - 10 * u, lx + 105 * u, cy + (yb - yt) / 2 + 15 * u),
              (0, H - 175 * u, 260 * u, H), (W - 375 * u, H - 175 * u, W, H),
              (cx - 180 * u, H - 138 * u, cx + 180 * u, H)):
        panel(base, *p)

    # waterline / boresight
    pts = [(-46, 0), (-22, 0), (-11, 12), (0, 0), (11, 12), (22, 0), (46, 0)]
    for a, b in zip(pts, pts[1:]):
        line(hud, (cx + a[0] * u, cy + a[1] * u), (cx + b[0] * u, cy + b[1] * u), GREEN, T(2))

    # pitch ladder (conformal: y = f * tan(elevation))
    for e in (-15, -10, -5, 0, 5, 10, 15):
        y = cy - f * math.tan(math.radians(e))
        if not 120 * u < y < H - 190 * u:
            continue
        inner, outer = (70 * u, 300 * u) if e == 0 else (70 * u, 175 * u)
        for sgn in (-1, 1):
            p, q = (cx + sgn * inner, y), (cx + sgn * outer, y)
            if e < 0:
                dashed(hud, p, q, DIM, T(1), 9 * u, 7 * u)
            else:
                line(hud, p, q, DIM, T(1))
            if e:
                line(hud, (cx + sgn * outer, y), (cx + sgn * outer, y + (10 if e > 0 else -10) * u), DIM, T(1))
                put(hud, f"{abs(e)}", cx + sgn * (outer + 10 * u), y + 5 * u, sm, DIM, 1, "l" if sgn > 0 else "r")

    # bearing tape (conformal across the top)
    ty = 62 * u
    line(hud, (cx - W * 0.36, ty), (cx + W * 0.36, ty), DIM, T(1))
    for b in range(-30, 31, 5):
        x = cx + f * math.tan(math.radians(b))
        if abs(x - cx) > W * 0.36:
            continue
        big = b % 10 == 0
        line(hud, (x, ty), (x, ty - (14 if big else 7) * u), GREEN if big else DIM, T(1))
        if big:
            put(hud, f"{b:+d}" if b else "0", x, ty - 20 * u, sm, GREEN, 1, "c")
    line(hud, (cx, ty + 2 * u), (cx, ty + 12 * u), GREEN, T(2))
    box(hud, cx - 58 * u, 4 * u, cx + 58 * u, 30 * u, GREEN, T(1))
    brg = tgt["bearing"] if tgt else None
    put(hud, f"BRG {brg:+05.1f}" if brg is not None else "BRG ---", cx, 24 * u, sm, GREEN, 1, "c")
    if brg is not None:
        x = cx + f * math.tan(math.radians(brg))
        tri(hud, [(x, ty + 3 * u), (x - 8 * u, ty + 17 * u), (x + 8 * u, ty + 17 * u)], GREEN)

    # range tape (right)
    rmax = 300 if (d or 0) < 270 else 600
    ry = lambda r: yb - min(max(r, 0), rmax) / rmax * (yb - yt)  # noqa: E731
    line(hud, (rx, yt), (rx, yb), DIM, T(1))
    for r in range(0, rmax + 1, 10 if rmax == 300 else 20):
        big = r % (50 if rmax == 300 else 100) == 0
        line(hud, (rx, ry(r)), (rx + (14 if big else 6) * u, ry(r)), GREEN if big else DIM, T(1))
        if big:
            put(hud, f"{r}", rx + 20 * u, ry(r) + 5 * u, sm, GREEN)
    line(hud, (rx - 4 * u, ry(0)), (rx - 4 * u, ry(CLOSE_CM)), AMBER, T(3))
    put(hud, "RNG CM", rx + 7 * u, yt - 18 * u, sm, GREEN, 1, "c")
    if d is not None:
        y = ry(d)
        col = AMBER if d < CLOSE_CM else GREEN
        tri(hud, [(rx - 2 * u, y), (rx - 14 * u, y - 8 * u), (rx - 14 * u, y + 8 * u)], col)
        box(hud, rx - 98 * u, y - 15 * u, rx - 16 * u, y + 15 * u, col, T(1))
        put(hud, f"{d:5.1f}", rx - 57 * u, y + 7 * u, md, col, T(1), "c")

    # closure-rate tape (left), + = closing
    vmax = 40
    vy = lambda v: cy - max(-vmax, min(vmax, v)) / vmax * (yb - yt) / 2  # noqa: E731
    line(hud, (lx, vy(vmax)), (lx, vy(-vmax)), DIM, T(1))
    for v in range(-vmax, vmax + 1, 5):
        big = v % 20 == 0
        line(hud, (lx, vy(v)), (lx - (14 if big else 6) * u, vy(v)), GREEN if big else DIM, T(1))
        if big:
            put(hud, f"{v:+d}" if v else "0", lx - 20 * u, vy(v) + 5 * u, sm, GREEN, 1, "r")
    put(hud, "Vc CM/S", lx - 7 * u, vy(vmax) - 18 * u, sm, GREEN, 1, "c")
    if d is not None:
        y = vy(vc)
        tri(hud, [(lx + 2 * u, y), (lx + 14 * u, y - 8 * u), (lx + 14 * u, y + 8 * u)], GREEN)
        box(hud, lx + 16 * u, y - 15 * u, lx + 96 * u, y + 15 * u, GREEN, T(1))
        put(hud, f"{vc:+5.1f}", lx + 56 * u, y + 7 * u, md, GREEN, T(1), "c")

    # other tags: small diamonds
    for i, r in st["tags"].items():
        if tgt and i == tgt["id"]:
            continue
        x, y = r["tag"]["c"]
        s = 9 * u
        cv2.polylines(hud, [np.int32([(x, y - s), (x + s, y), (x, y + s), (x - s, y)])], True, DIM, T(1), cv2.LINE_AA)
        put(hud, f"{NAMES.get(i, i)} {r['dist']:.0f}", x + 12 * u, y + 4 * u, sm, DIM)

    # target designator
    if tgt:
        c = tgt["tag"]["corners"]
        x, y = c.mean(axis=0)
        h = max(float(np.ptp(c[:, 0])), float(np.ptp(c[:, 1]))) / 2 + 12 * u
        col = GREEN
        if tr.locked:
            box(hud, x - h, y - h, x + h, y + h, col, T(2))
            for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):  # outer corner ticks
                line(hud, (x + sx * h, y + sy * h), (x + sx * (h + 10 * u), y + sy * (h + 10 * u)), col, T(2))
        elif blink:
            k = h * 0.35
            for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                line(hud, (x + sx * h, y + sy * h), (x + sx * (h - k), y + sy * h), col, T(2))
                line(hud, (x + sx * h, y + sy * h), (x + sx * h, y + sy * (h - k)), col, T(2))
        s = 6 * u
        cv2.polylines(hud, [np.int32([(x, y - s), (x + s, y), (x, y + s), (x - s, y)])], True, col, T(1), cv2.LINE_AA)
        put(hud, f"TGT {NAMES.get(tgt['id'], tgt['id'])}  ID{tgt['id']}", x - h, y - h - 10 * u, sm, col)
        put(hud, f"{tgt['dist']:.1f} CM", x - h, y + h + 22 * u, md, col, T(1))
        put(hud, "LOCK" if tr.locked else "ACQ", x + h, y - h - 10 * u, sm, col, 1, "r")
        v = np.array([x - cx, y - cy])
        n = np.linalg.norm(v)
        if n > h * 1.6:  # steering line from boresight to the box
            dashed(hud, (cx, cy), (cx + v[0] * (1 - h * 1.3 / n), cy + v[1] * (1 - h * 1.3 / n)), DIM, T(1), 8 * u, 8 * u)

    # big range readout (bottom centre)
    by0, by1 = H - 128 * u, H - 58 * u
    col = AMBER if d is not None and d < CLOSE_CM else GREEN
    box(hud, cx - 170 * u, by0, cx + 170 * u, by1, col, T(2))
    put(hud, "RNG", cx - 160 * u, by0 + 22 * u, sm, col)
    put(hud, "CM", cx + 160 * u, by1 - 10 * u, sm, col, 1, "r")
    put(hud, f"{d:5.1f}" if d is not None else "---.-", cx, by1 - 14 * u, 1.45 * u, col, T(3), "c")
    tti = f"{d / vc:4.1f} S" if d is not None and vc > 1 else "--"
    put(hud, f"Vc {vc:+.1f} CM/S    TTI {tti}" if d is not None else "Vc --    TTI --",
        cx, by1 + 26 * u, sm, col, 1, "c")
    if d is not None and tr.locked:
        put(hud, "LOCK", cx - 160 * u, by1 - 10 * u, sm, col)

    # status block (bottom left)
    age = st["age"]
    lost = st["has_frames"] and age is not None and age > 1.0
    rows = [("MODE", st["mode"], GREEN),
            ("LINK", (f"LOST {age:.1f}S" if lost else f"OK {age:.1f}S") if age is not None else "WAIT",
             RED if lost else GREEN),
            ("FPS", f"{st['fps']:.1f}", GREEN),
            ("FRM", f"{st['seq']:05d}", GREEN),
            ("CAL", st["f_src"], GREEN if st["calibrated"] else AMBER)]
    for k, (a, b, c) in enumerate(rows):
        y = H - 150 * u + k * 24 * u
        put(hud, a, 24 * u, y, sm, DIM)
        put(hud, b, 84 * u, y, sm, c)

    # target data + range history (bottom right)
    x0 = W - 360 * u
    if tgt:
        put(hud, f"TGT  ID {tgt['id']}  {NAMES.get(tgt['id'], '')}   SIDE {tgt['tag']['side']:.0f}PX",
            x0, H - 150 * u, sm, GREEN)
        put(hud, f"X {tgt['x']:+6.1f}  Y {-tgt['y']:+6.1f}  Z {tgt['z']:6.1f} CM", x0, H - 126 * u, sm, GREEN)
        put(hud, f"BRG {tgt['bearing']:+5.1f}   ELEV {tgt['elev']:+5.1f} DEG", x0, H - 102 * u, sm, GREEN)
    else:
        put(hud, "TGT  NONE", x0, H - 150 * u, sm, DIM)
    gx0, gy0, gw, gh = x0, H - 84 * u, 336 * u, 60 * u
    box(hud, gx0, gy0, gx0 + gw, gy0 + gh, DIM, T(1))
    put(hud, "RNG 15S", gx0 + 6 * u, gy0 + 14 * u, 0.38 * u, DIM)
    if len(tr.hist) > 1:
        vals = [v for _, v in tr.hist]
        lo, hi = min(vals), max(vals)
        pad = max(5.0, (hi - lo) * 0.2)
        lo, hi = lo - pad, hi + pad
        pts = [(gx0 + gw - (now - t) / 15 * gw, gy0 + gh - (v - lo) / (hi - lo) * gh) for t, v in tr.hist]
        cv2.polylines(hud, [np.int32(pts)], False, GREEN, T(1), cv2.LINE_AA)
        put(hud, f"{hi:.0f}", gx0 + gw - 4 * u, gy0 + 14 * u, 0.38 * u, DIM, 1, "r")
        put(hud, f"{lo:.0f}", gx0 + gw - 4 * u, gy0 + gh - 4 * u, 0.38 * u, DIM, 1, "r")

    # corners: title, clock, keys
    put(hud, "ARIXFLY  HS420", 24 * u, 30 * u, md, GREEN, T(1))
    put(hud, f"A/G   {'TGT ' + NAMES.get(tgt['id'], str(tgt['id'])) if tgt else 'SRCH'}", 24 * u, 56 * u, sm, GREEN)
    put(hud, time.strftime("%H:%M:%S"), W - 24 * u, 30 * u, md, GREEN, T(1), "r")
    put(hud, "Q QUIT  F FULL  S SHOT", W - 24 * u, 56 * u, 0.4 * u, DIM, 1, "r")

    # warnings
    if not st["has_frames"]:
        if blink:
            put(hud, st["status"], cx, cy - 70 * u, 0.9 * u, AMBER, T(2), "c")
    elif lost:
        if blink:
            put(hud, "LINK LOST", cx, cy - 70 * u, 1.1 * u, RED, T(2), "c")
    elif tgt is None and blink:
        put(hud, "NO TGT", cx, cy + 70 * u, 0.8 * u, AMBER, T(2), "c")
    if d is not None and d < CLOSE_CM and blink:
        put(hud, "CLOSE", cx, cy + 70 * u, 0.9 * u, AMBER, T(2), "c")
    if st["paused"]:
        put(hud, "PAUSED", cx, cy - 100 * u, 0.8 * u, AMBER, T(2), "c")

    glow = cv2.GaussianBlur(hud, (0, 0), 3.5 * u)
    return cv2.add(base, cv2.addWeighted(hud, 1.0, glow, 1.1, 0))


# ---------------------------------------------------------------- main loop
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--replay", help="folder of frame_*.jpg or a video_probe .bin")
    ap.add_argument("--seconds", type=float, default=0, help="quit after N s (0 = until Q)")
    ap.add_argument("--headless", action="store_true", help="no window (tests)")
    ap.add_argument("--shot", help="save the last HUD frame to this PNG on exit")
    a = ap.parse_args()

    src = ReplaySource(a.replay) if a.replay else LiveSource()
    track, focal = Track(), {}
    frame, tags, last_seq, out = None, {}, -1, None
    f_info = (focal_from_hfov(1280, DEFAULT_HFOV), 1280, f"GUESS HFOV {DEFAULT_HFOV:.0f}", False)
    screen = None
    if not a.headless:
        import pygame
        pygame.init()
        pygame.display.set_caption("ARIXFLY HUD - HS420")
        screen = pygame.display.set_mode((1280, 720), pygame.RESIZABLE)
        clock = pygame.time.Clock()
    t_start, running, full = time.monotonic(), True, False
    try:
        while running:
            now = time.monotonic()
            seq, jpg = src.latest()
            if jpg is not None and seq != last_seq:
                last_seq = seq
                img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    frame, w = img, img.shape[1]
                    if w not in focal:
                        fl, srcname = load_focal(w)
                        focal[w] = (fl, w, f"F{fl:.0f}  HFOV {math.degrees(2 * math.atan(w / 2 / fl)):.0f}", True) \
                            if fl else (focal_from_hfov(w, DEFAULT_HFOV), w, f"GUESS HFOV {DEFAULT_HFOV:.0f}", False)
                    f_info = focal[w]
                    tags = analyse(img, SIZES_CM, f_info[0])
                    track.update(pick(tags), now)
            track.expire(now)
            st = {"now": now, "track": track, "tags": tags, "f": f_info[0], "f_w": f_info[1], "f_src": f_info[2],
                  "calibrated": f_info[3], "mode": src.mode, "status": src.status, "age": src.age(),
                  "fps": src.fps(), "seq": src.seq, "has_frames": frame is not None, "paused": src.paused}
            out = render(frame, st)

            if screen is not None:
                for ev in pygame.event.get():
                    if ev.type == pygame.QUIT:
                        running = False
                    elif ev.type == pygame.KEYDOWN:
                        if ev.key in (pygame.K_q, pygame.K_ESCAPE):
                            running = False
                        elif ev.key == pygame.K_f:
                            full = not full
                            screen = pygame.display.set_mode((0, 0) if full else (1280, 720),
                                                             pygame.FULLSCREEN if full else pygame.RESIZABLE)
                        elif ev.key == pygame.K_s:
                            os.makedirs(SHOTS, exist_ok=True)
                            p = os.path.join(SHOTS, f"hud_{time.strftime('%Y%m%d_%H%M%S')}.png")
                            cv2.imwrite(p, out)
                            print("screenshot", os.path.normpath(p))
                        elif ev.key == pygame.K_SPACE and src.mode == "REPLAY":
                            src.paused = not src.paused
                ws = screen.get_size()
                sc = min(ws[0] / out.shape[1], ws[1] / out.shape[0])
                size = (max(1, int(out.shape[1] * sc)), max(1, int(out.shape[0] * sc)))
                rgb = cv2.cvtColor(cv2.resize(out, size, interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
                screen.fill((0, 0, 0))
                screen.blit(pygame.image.frombuffer(rgb.tobytes(), size, "RGB"),
                            ((ws[0] - size[0]) // 2, (ws[1] - size[1]) // 2))
                pygame.display.flip()
                clock.tick(30)
            else:
                time.sleep(1 / 30)
            if a.seconds and now - t_start > a.seconds:
                running = False
    finally:
        src.close()
        if screen is not None:
            pygame.quit()
        if a.shot and out is not None:
            os.makedirs(os.path.dirname(os.path.abspath(a.shot)), exist_ok=True)
            print("saved" if cv2.imwrite(a.shot, out) else "COULD NOT SAVE", a.shot)
        if track.hist:
            print(f"last range {track.hist[-1][1]:.1f} cm, frames {src.seq}")
        if src.mode == "LIVE":
            asm = getattr(src, "asm", None)
            print(f"live: status '{src.status}', packets {src.pkts}, frames {src.seq}"
                  + (f", dropped {asm.dropped}" if asm else ""))


if __name__ == "__main__":
    main()
