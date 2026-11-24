"""Day 4 closed loop: steer the HS420 onto table B from its own camera (OpenCV ArUco, tag 2 upright on B).

Setup: tag 2 (10 cm) standing UPRIGHT at the far edge of table B, facing table A (a flat tag drops out of
the forward camera's view up close). Drone on table A, nose pointing roughly at the tag.
Flight: takeoff 0x01 -> ACQUIRE (hover until 3 frames in a row see the tag) -> APPROACH (P control) ->
land 0x02 (still steering pitch/yaw/roll during the descent) when the range is within --target + 8 cm for 3 frames (with roll on: and within 12 cm of the
tag's normal).
  pitch  = KP_RANGE * (range - target), forward <= MAX_FWD, back <= MAX_BACK, less forward when off-centre
  yaw    = KP_YAW * bearing   (tag right of centre -> yaw right)          [yaw sign only as flown, D key]
  roll   = -KP_ROLL * sideways offset from the tag's normal (from the tag's tilt, median of 3 frames),
           only closer than 70 cm and only when the last 3 estimates agree in sign: farther away the tilt
           is ambiguous and flips sign (real frames: 22 flips in 71 at 100 cm, none at 50 cm)  [roll sign as flown]
  alt    = KP_ALT * elevation (tag above centre -> climb)
  power units as in hop.py: 1.0 = a full key press at the tethered defaults (sticks 40 %, throttle 55 %).
--axes picks which channels the controller drives; the rest stay centred. Default "none" = OBSERVE ONLY:
take off, hover, log what the controller WOULD send, land after --observe s. Add one axis per test.
Lands on: tag lost > 1.5 s, tag not found within 5 s, approach > 15 s, overshoot (range < target - 20 cm),
total cap (--cap, default 30, max 60), L key, window focus lost. Space = KILL (0x04, then packets stop).
Video and control share ONE UDP socket, as the app does: `ef 02` stick packets at 39 Hz that also carry the
app's frame-ack records, plus `ef 00 04 00` until frames flow. Without the acks the drone streamed nothing
while sticks were sent (2026-11-08 22:19: status replies only). Every live run records its video (see Logs).

Keys: C connect   G calibrate   hold Enter 1 s = go   Space = KILL   L = land now   Esc = quit
After C the window waits in VIDEO: only the video request goes out (no stick packets) until the first frame arrives -
stick packets sent before the drone's video has started keep it from ever starting. Then SAFE (G, Enter).
Takeoff also stays LOCKED whenever video stops (5 frames, newest < 1 s old). Video needs the frame acks in the stick packets (probe 2026-11-09: 4 frames then a stall
without them, 24 frames / 5 s with them).
Usage:
  python tools/autopilot.py --sim                         # closed-loop simulator (synthetic camera), no Wi-Fi
  python tools/autopilot.py --sim --axes pitch,yaw --auto # same, headless, keys pressed by a timer
  python tools/autopilot.py --replay captures/frames/test_100cm   # recorded frames, nothing sent
  python tools/autopilot.py                               # LIVE observe only (tethered, soft ground)
  python tools/autopilot.py --axes pitch                  # LIVE, controller drives pitch only
  python tools/autopilot.py --axes pitch,yaw --target 35
Logs: captures/autopilot/<run>.csv per run (39 Hz rows: state, sighting, sticks, video fps / frame counts,
landing reason), captures/autopilot/runs.csv one row per flight, and for live runs (sim/replay: --record) the
folder captures/autopilot/<run>/ with every decoded frame as frame_NNNN.jpg plus frames.csv (one row per frame:
state, tag seen, range, sharpness). Replay a flight's video:  python tools/autopilot.py --replay captures/autopilot/<run>
"""
import argparse
import csv
import math
import os
import select
import socket
import sys
import threading
import time
import traceback

import cv2
import numpy as np
import pygame

import drone_video as dv
import hud
from aruco_distance import DEFAULT_HFOV, SIZES_CM, analyse, focal_from_hfov, load_focal, render_tag_scene
from aruco_track import TABLE_B_ID
from hop import MAX_CAP_S, RAMP_S, to_bytes
from joystick_fly import CALIBRATE, CAL_S, ESTOP, KILL_S, LAND, LAND_S, PULSE_S, TAKEOFF, Link
from replay_test import DRONE_SSID, RATE_HZ, build

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "..", "captures", "autopilot")

TAG_ID = TABLE_B_ID
TARGET_CM = 35        # stop range from the upright tag
ARRIVE_TOL_CM = 8
ARRIVE_FRAMES = 3
SIDE_TOL_CM = 12      # with roll on: also this close to the tag's normal before landing
OVERSHOOT_CM = 20
LOCK_FRAMES = 3
LOST_S = 1.5          # no fresh sighting for this long while approaching -> land
STALE_S = 0.7         # older sightings are not used for control (sticks centred)
TAKEOFF_S = 3.0
ACQUIRE_S = 5.0
APPROACH_S = 15.0
ARM_HOLD_S = 1.0
VIDEO_MIN_FRAMES = 5  # takeoff locked until this many frames arrived and the newest is < VIDEO_FRESH_S old:
VIDEO_FRESH_S = 1.0   # after the Wi-Fi join nothing comes back for 5-10 s (probe 2026-11-09, phase A)
RANGE_OK = (10, 400)  # cm; detections outside are ignored
SMOOTH = 0.5          # range/bearing low-pass per frame

KP_RANGE, MAX_FWD, MAX_BACK = 0.006, 0.5, 0.3   # power per cm
KP_YAW, MAX_YAW, YAW_DEAD = 0.02, 0.4, 2.0      # power per degree
KP_ROLL, MAX_ROLL, ROLL_DEAD, ROLL_RANGE_CM = 0.008, 0.25, 5.0, 70  # real frames: stable at 50 cm, flips at 100   # power per cm sideways
KP_ALT, MAX_ALT, ALT_DEAD = 0.03, 0.4, 3.0
LAT_MEDIAN = 3
SIM_DESCENT_S = 3.0
LAND_STEER_S = 3.0    # keep steering this long into the descent, then sticks centred
OFF_CENTRE_DEG = 20   # forward power fades to 0 at this bearing
AXES = ("pitch", "yaw", "roll", "alt")
CHANNEL_AXIS = ("alt", "yaw", "roll", "pitch")  # stick order: bytes 20..23


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def control(m, axes, target):
    """Smoothed sighting {dist, bearing, elev} -> sticks [throttle, yaw, roll, pitch] in power units."""
    s = [0.0] * 4
    if "pitch" in axes:
        fwd = clamp(KP_RANGE * (m["dist"] - target), -MAX_BACK, MAX_FWD)
        if fwd > 0:
            fwd *= max(0.0, 1 - abs(m["bearing"]) / OFF_CENTRE_DEG)
        s[3] = fwd
    if "yaw" in axes and abs(m["bearing"]) > YAW_DEAD:
        s[1] = clamp(KP_YAW * m["bearing"], -MAX_YAW, MAX_YAW)
    if "roll" in axes and m.get("lat_ok") and m["dist"] < ROLL_RANGE_CM and abs(m["lat"]) > ROLL_DEAD:
        s[2] = clamp(-KP_ROLL * m["lat"], -MAX_ROLL, MAX_ROLL)
    if "alt" in axes and abs(m["elev"]) > ALT_DEAD:
        s[0] = clamp(KP_ALT * m["elev"], -MAX_ALT, MAX_ALT)
    return s


# ---------------------------------------------------------------- camera sources
class LiveVideo(hud.Source):
    """Drone video on the CONTROL socket (link.sock), like the app: one socket sends sticks + frame acks.
    While `ef 02` packets flow, the drone streams only if they ACK its frames (22:19 run: sticks without ack
    records -> only `93 04` status replies, zero `93 01` fragments). So every finished frame is queued here
    and the main loop appends its ack record to the next stick packets (take_acks -> dv.add_acks), each frame
    in up to ACK_REPEAT packets, max 3 records per packet (app: 0-3). `ef 00 04 00` goes out at 10 Hz from the
    same socket until frames flow, and again whenever no frame came for HELLO_GAP_S. No timeout is set on the
    shared socket (a send timeout would look like a lost link); reads wait with select instead."""
    mode = "LIVE"
    pkts = frags = 0
    HELLO_GAP_S = 1.0

    def __init__(self):
        super().__init__()
        self.acks, self.ack_lock = [], threading.Lock()  # [[frame id, fragment count, times sent]]

    def take_acks(self):
        """Ack records for the next stick packet (called by the main loop at 39 Hz)."""
        with self.ack_lock:
            out = self.acks[:3]
            for a in out:
                a[2] += 1
            self.acks = [a for a in self.acks if a[2] < dv.ACK_REPEAT]
        return [(f, n) for f, n, _ in out]

    def start(self, link):
        self.link = link
        if getattr(self, "th", None):
            return  # reconnect: the running thread picks up the new link.sock
        self.status = "VIDEO STARTING"
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def _run(self):
        asm, next_hello, prepared = dv.Assembler(), 0.0, None
        while self.alive:
            s = self.link.sock
            if s in (None, "dry"):
                time.sleep(0.05)
                continue
            try:
                if s is not prepared:
                    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
                    if hasattr(socket, "SIO_UDP_CONNRESET"):
                        s.ioctl(socket.SIO_UDP_CONNRESET, False)  # Windows: ignore ICMP resets on recv
                    prepared = s
                now = time.monotonic()
                if now >= next_hello and (self.t_frame is None or now - self.t_frame > self.HELLO_GAP_S):
                    s.sendto(dv.HELLO, dv.DRONE)  # also binds the socket before the first read
                    next_hello = now + 0.1
                if not select.select([s], [], [], 0.02)[0]:
                    continue
                d = s.recvfrom(65535)[0]
            except (OSError, ValueError):  # socket closed / replaced by the link
                time.sleep(0.02)
                continue
            self.pkts += 1
            self.frags += d[:2] == b"\x93\x01"
            jpg = asm.feed(d)
            if jpg:
                with self.ack_lock:
                    self.acks.append([*asm.last, 0])
                self._push(jpg)
            if self.seq == 0:
                self.status = f"{self.pkts} pkts ({self.frags} video), no frame yet"


class Sim(hud.Source):
    """Point-mass drone in front of an upright tag; renders the camera at ~7 fps with latency.
    World: tag at (0, 0) facing the drone, drone at (x right, -d), heading psi deg (+ = yawed right)."""
    mode = "SIM"
    V, W, TAU, FPS, LATENCY = 100.0, 60.0, 0.4, 7.0, 0.15   # cm/s and deg/s per power unit, lag s

    def __init__(self, dist, lateral, heading, drift):
        super().__init__()
        self.x, self.y, self.psi = lateral, -dist, heading
        self.vx = self.vy = 0.0
        self.drift, self.airborne = drift, False
        self.w, self.h = 1280, 720  # same as the drone camera
        self.f = load_focal(self.w)[0] or focal_from_hfov(self.w, DEFAULT_HFOV)
        self.status = "SIM"
        threading.Thread(target=self._run, daemon=True).start()

    def step(self, sticks, dt):
        """sticks [thr, yaw, roll, pitch] in power units."""
        if not self.airborne:
            return
        a = math.radians(self.psi)
        fwd, lat = self.V * sticks[3], self.V * sticks[2] + self.drift
        tvx = fwd * math.sin(a) + lat * math.cos(a)
        tvy = fwd * math.cos(a) - lat * math.sin(a)
        k = min(1.0, dt / self.TAU)
        self.vx += k * (tvx - self.vx)
        self.vy += k * (tvy - self.vy)
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.psi += self.W * sticks[1] * dt

    def camera(self):
        """Tag position in camera coordinates (x right, z forward), cm."""
        a = math.radians(self.psi)
        vx, vy = -self.x, -self.y
        return vx * math.cos(a) - vy * math.sin(a), vx * math.sin(a) + vy * math.cos(a)

    def _run(self):
        while self.alive:
            cx, cz = self.camera()
            psi = self.psi
            time.sleep(self.LATENCY)
            if cz > 8:
                img = render_tag_scene(TAG_ID, SIZES_CM[TAG_ID], cz, self.f, self.w, self.h, yaw_deg=-psi, x_cm=cx)
            else:
                img = np.full((self.h, self.w, 3), 200, np.uint8)
            self._push(cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes())
            time.sleep(max(0.0, 1 / self.FPS - self.LATENCY))


class Eye:
    """Decodes the newest frame and finds the tag in a background thread (keeps the 39 Hz loop smooth)."""

    def __init__(self, source, t0, rec_dir=None):
        self.src, self.lock, self.alive = source, threading.Lock(), True
        self.m, self.thumb, self.n = None, None, 0
        self.t0, self.rec_dir, self.state, self.rows = t0, rec_dir, "OFFLINE", []
        if rec_dir:
            os.makedirs(rec_dir, exist_ok=True)
        self.th = threading.Thread(target=self._run, daemon=True)
        self.th.start()

    def _run(self):
        last = 0
        while self.alive:
            seq, jpg = self.src.latest()
            if not jpg or seq == last:
                time.sleep(0.005)
                continue
            last, t = seq, self.src.t_frame or time.monotonic()
            img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            h, w = img.shape[:2]
            f = load_focal(w)[0] or focal_from_hfov(w, DEFAULT_HFOV)
            r = analyse(img, {TAG_ID: SIZES_CM[TAG_ID]}, f).get(TAG_ID)
            m = {"t": t, "seen": r is not None}
            if r:
                m.update(dist=r["dist"], bearing=r["bearing"], elev=r["elev"], lat=r["lat"],
                         c=(r["tag"]["c"][0] / w, r["tag"]["c"][1] / h))
            if self.rec_dir:  # raw JPEG as received, no re-encode; frame_NNNN = this Eye's frame number
                with open(os.path.join(self.rec_dir, f"frame_{self.n + 1:04d}.jpg"), "wb") as fh:
                    fh.write(jpg)
                sharp = cv2.Laplacian(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
                self.rows.append([self.n + 1, seq, round(t - self.t0, 3), self.state, int(m["seen"]),
                                  *(round(m[k], 1) if r else "" for k in ("dist", "bearing", "elev", "lat")),
                                  round(sharp)])
            tw = 360
            th = round(h * tw / w)
            thumb = cv2.cvtColor(cv2.resize(img, (tw, th)), cv2.COLOR_BGR2RGB)
            with self.lock:
                self.n += 1
                self.m, self.thumb = dict(m, n=self.n), thumb

    def get(self):
        with self.lock:
            return self.m, self.thumb

    def close(self):
        self.alive = False
        self.src.close()
        self.th.join(timeout=1.0)  # let the last frame finish writing
        if self.rec_dir:
            with open(os.path.join(self.rec_dir, "frames.csv"), "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["frame", "src_seq", "t", "state", "seen", "range_cm", "bearing_deg", "elev_deg",
                            "side_cm", "sharpness"])
                w.writerows(self.rows)


def video_summary(rows):
    """Frames, fps, gaps and tag sightings per flight phase (answers 'did video survive takeoff?')."""
    out = []
    for name, states in (("ground", ("OFFLINE", "CONNECTING", "VIDEO", "SAFE", "READY")),
                         ("airborne", ("TAKEOFF", "ACQUIRE", "APPROACH")), ("landing", ("LANDING", "KILLING"))):
        rr = [r for r in rows if r[3] in states]
        if not rr:
            out.append(f"  {name:8}: no frames")
            continue
        span = rr[-1][2] - rr[0][2]
        skipped = sum(b[1] - a[1] - 1 for a, b in zip(rr, rr[1:]) if b[1] - a[1] > 1)
        gap = max((b[2] - a[2] for a, b in zip(rr, rr[1:])), default=0)
        fps = (len(rr) - 1) / span if span else 0
        sharp = sorted(r[9] for r in rr)[len(rr) // 2]
        out.append(f"  {name:8}: {len(rr)} frames in {span:.1f} s ({fps:.1f} fps, longest gap {gap:.2f} s, "
                   f"{skipped} not analysed), tag seen in {sum(r[4] for r in rr)}, median sharpness {sharp}")
    return "\n".join(out)


# ---------------------------------------------------------------- window
def draw(screen, fonts, state, lines, thumb, m):
    font, big = fonts
    color = {"TAKEOFF": (251, 191, 36), "ACQUIRE": (251, 191, 36), "APPROACH": (251, 191, 36),
             "LANDING": (167, 139, 250), "KILLING": (248, 113, 113), "READY": (45, 212, 191),
             "SAFE": (74, 222, 128), "VIDEO": (96, 165, 250)}.get(state, (244, 244, 245))
    screen.fill((9, 9, 11))
    screen.blit(big.render(state, True, color), (30, 20))
    for i, (txt, col) in enumerate(lines):
        screen.blit(font.render(txt, True, col), (30, 110 + i * 32))
    if thumb is not None:
        th, tw = thumb.shape[:2]
        x0, y0 = screen.get_width() - tw - 30, screen.get_height() - th - 30  # below the long status lines
        screen.blit(pygame.image.frombuffer(thumb.tobytes(), (tw, th), "RGB"), (x0, y0))
        pygame.draw.line(screen, (80, 80, 90), (x0 + tw // 2, y0), (x0 + tw // 2, y0 + th))
        pygame.draw.line(screen, (80, 80, 90), (x0, y0 + th // 2), (x0 + tw, y0 + th // 2))
        if m and m.get("seen"):
            pygame.draw.circle(screen, (110, 255, 130), (x0 + m["c"][0] * tw, y0 + m["c"][1] * th), 14, 2)
    pygame.display.flip()


def ask_result(mode, axes, outcome, final):
    print(f"\nrun ended: {outcome}" + (f" | sim final: {final}" if final else ""))
    row = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "mode": mode, "axes": "+".join(axes) or "none",
           "outcome": outcome, "on_b": "", "tag_cm": "", "note": final or ""}
    if mode != "live":
        return  # runs.csv is for real flights only
    if True:
        try:
            row["on_b"] = input("landed ON table B (y/n, Enter = skip)? ").strip().lower()[:1]
            row["tag_cm"] = input("distance tag -> drone centre, cm (Enter = skip)? ").strip()
            row["note"] = input("note (Enter = none)? ").strip()
        except EOFError:
            pass
    path = os.path.join(OUT_DIR, "runs.csv")
    new = not os.path.exists(path)
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(row))
        if new:
            w.writeheader()
        w.writerow(row)
    print("logged to", os.path.normpath(path))


# ---------------------------------------------------------------- main loop
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--axes", default="none", help="comma list of pitch,yaw,roll,alt the controller drives; none = observe")
    ap.add_argument("--target", type=float, default=TARGET_CM, help="stop range from the tag, cm")
    ap.add_argument("--observe", type=float, default=6.0, help="observe-only: seconds of hover after the tag lock")
    ap.add_argument("--cap", type=float, default=30, help=f"airborne seconds before auto land (max {MAX_CAP_S})")
    ap.add_argument("--sim", action="store_true", help="simulated drone + synthetic camera, nothing sent")
    ap.add_argument("--sim-start", default="150,15,8", help="sim start: distance cm, lateral cm (right +), heading deg")
    ap.add_argument("--sim-drift", type=float, default=5.0, help="sim sideways drift, cm/s")
    ap.add_argument("--replay", help="frames folder or video_probe .bin, nothing sent")
    ap.add_argument("--auto", action="store_true", help="sim/replay only: headless, keys pressed by a timer")
    ap.add_argument("--record", action="store_true", help="sim/replay: also save frames (live always records)")
    a = ap.parse_args()
    axes = [] if a.axes == "none" else a.axes.split(",")
    if any(x not in AXES for x in axes):
        ap.error(f"--axes takes {', '.join(AXES)} or none")
    dry = bool(a.sim or a.replay)
    if a.auto and not dry:
        ap.error("--auto only with --sim or --replay")
    mode = "sim" if a.sim else "replay" if a.replay else "live"
    cap = min(a.cap, MAX_CAP_S)

    if a.sim:
        d, x, psi = (float(v) for v in a.sim_start.split(","))
        src = Sim(d, x, psi, a.sim_drift)
    elif a.replay:
        src = hud.ReplaySource(a.replay)
    else:
        if hud.mullvad_blocks_lan():
            sys.exit("Mullvad is connected and blocks the LAN - disconnect it first")
        src = LiveVideo()
    run_id = f"{time.strftime('%Y%m%d_%H%M%S')}_{mode}_{a.axes.replace(',', '+')}"
    rec_dir = os.path.join(OUT_DIR, run_id) if mode == "live" or a.record else None
    t0 = time.monotonic()
    eye, link = Eye(src, t0, rec_dir), Link(dry)

    if a.auto:
        os.environ["SDL_VIDEODRIVER"] = "dummy"
    pygame.init()
    screen = pygame.display.set_mode((1100, 560))
    pygame.display.set_caption(f"HS420 autopilot ({mode}, axes {a.axes})")
    fonts = pygame.font.SysFont("consolas", 22), pygame.font.SysFont("consolas", 60, bold=True)
    clock = pygame.time.Clock()

    state, note, outcome = "OFFLINE", "press C to connect", "not flown"
    seq, pulse_bit, pulse_until, hold_t, kill_until = 0, 0, 0.0, None, 0.0
    start_t = phase_t = land_t = None
    sticks, cmd = [0.0] * 4, [0.0] * 4
    sm, last_n, streak, t_seen, arrive, lats = None, 0, 0, None, 0, []
    rows, logged, land_steer, reason = [], None, False, ""
    script_keys = {0.3: "c", 1.5: "g", 3.0: "enter"} if a.auto else {}
    try:
        while state != "DONE":
            now = time.monotonic()
            dt = 1.0 / RATE_HZ
            keys_down = set()
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    keys_down.add("esc")
                elif ev.type == pygame.KEYDOWN:
                    keys_down.add({pygame.K_SPACE: "space", pygame.K_ESCAPE: "esc", pygame.K_c: "c",
                                   pygame.K_g: "g", pygame.K_l: "l"}.get(ev.key, ""))
            for at in [k for k in script_keys if now - t0 >= k]:
                keys_down.add(script_keys.pop(at))
            enter = "enter" in keys_down or (not a.auto and pygame.key.get_pressed()[pygame.K_RETURN])
            focused = a.auto or pygame.key.get_focused()

            # --- perception: new frame -> smoothed sighting
            eye.state = state
            m, thumb = eye.get()
            new_frame = m is not None and m["n"] != last_n
            if new_frame:
                last_n = m["n"]
                if m["seen"] and RANGE_OK[0] < m["dist"] < RANGE_OK[1]:
                    if sm is None or t_seen is None or m["t"] - t_seen > 1.0:
                        sm, lats = {k: m[k] for k in ("dist", "bearing", "elev")}, []
                    else:
                        sm = {k: sm[k] + SMOOTH * (m[k] - sm[k]) for k in ("dist", "bearing", "elev")}
                    lats = (lats + [m["lat"]])[-LAT_MEDIAN:]
                    sm["lat"] = sorted(lats)[len(lats) // 2]
                    sm["lat_ok"] = len(lats) == LAT_MEDIAN and (min(lats) > 0 or max(lats) < 0)  # no sign flip
                    streak, t_seen = streak + 1, m["t"]
                else:
                    streak = 0
            fresh = t_seen is not None and now - t_seen < STALE_S
            locked = fresh and streak >= LOCK_FRAMES
            lost_for = now - t_seen if t_seen is not None else 1e9

            # --- state machine
            airborne = state in ("TAKEOFF", "ACQUIRE", "APPROACH")
            land_reason = None
            if "space" in keys_down and state in ("SAFE", "READY", "TAKEOFF", "ACQUIRE", "APPROACH", "LANDING"):
                pulse_bit, pulse_until, kill_until = ESTOP, now + KILL_S, now + KILL_S
                state, note, outcome = "KILLING", "EMERGENCY STOP 0x04, then packets stop", "killed"
                reason = "Space kill"
                airborne = False
            if state == "KILLING" and now >= kill_until:
                state = "DONE"
            if "esc" in keys_down:
                if airborne:
                    land_reason = "Esc"
                elif state != "LANDING":
                    state = "DONE"
            if state == "OFFLINE" and "c" in keys_down:
                link.connect()
                state, note = "CONNECTING", f"joining {DRONE_SSID} ..."
            if state == "CONNECTING" and not link.busy:
                if link.sock:
                    if isinstance(src, LiveVideo):
                        # video request ONLY until frames flow: stick packets from the first moment keep the
                        # drone from ever starting its video (every autopilot run 11-08/09); hello-only works
                        src.start(link)
                        state, note = "VIDEO", "starting drone video - no control packets yet (5-15 s, Esc = quit)"
                    else:
                        state, note = "SAFE", "drone level on table A, nose at the tag - press G to calibrate"
                else:
                    state, note = "OFFLINE", (link.error or "join failed") + " - C to retry"
            if state in ("SAFE", "READY") and "g" in keys_down:
                pulse_bit, pulse_until = CALIBRATE, now + CAL_S
                state, note = "SAFE", "calibrating gyro ..."
            if state == "SAFE" and pulse_bit == CALIBRATE and now >= pulse_until:
                state, note = "READY", "hold Enter 1 s to fly   (Space = KILL)"
            age = src.age()  # 0.0 is a valid age (frame in this clock tick), None = no frame yet
            video_ok = src.seq >= VIDEO_MIN_FRAMES and age is not None and age < VIDEO_FRESH_S
            if state == "VIDEO" and src.seq >= 1:  # first frame: start sticks + acks (its ack is queued)
                state, note = "SAFE", "video OK - drone level on table A, nose at the tag - press G to calibrate"
            if state == "READY" and not video_ok:
                hold_t = None  # never fly blind
                note = "waiting for video - takeoff locked (5-10 s after joining is normal)"  # counts: line 2
            elif state == "READY" and note.startswith("waiting for video"):
                note = "video OK - hold Enter 1 s to fly   (Space = KILL)"
            if state == "READY" and enter and video_ok:
                hold_t = hold_t or now
                if now - hold_t >= ARM_HOLD_S or a.auto:
                    state, start_t, phase_t, note = "TAKEOFF", now, now, "takeoff 0x01"
                    pulse_bit, pulse_until = TAKEOFF, now + PULSE_S
                    if a.sim:
                        src.airborne = True
                    airborne = True
            elif not enter:
                hold_t = None
            if airborne:
                if "l" in keys_down:
                    land_reason = "L pressed"
                elif not focused:
                    land_reason = "window lost focus"
                elif now - start_t >= cap:
                    land_reason = f"time cap {cap:.0f} s"
            if state == "TAKEOFF" and not land_reason and now - phase_t >= TAKEOFF_S:
                state, phase_t, note = "ACQUIRE", now, "looking for tag 2 ..."
            if state == "ACQUIRE" and not land_reason:
                if locked:
                    state, phase_t, arrive = "APPROACH", now, 0
                elif now - phase_t >= ACQUIRE_S:
                    land_reason = f"tag not found in {ACQUIRE_S:.0f} s"
            cmd = [0.0] * 4
            if state == "APPROACH" and not land_reason:
                if lost_for > LOST_S:
                    land_reason = f"tag lost {lost_for:.1f} s"
                elif now - phase_t >= APPROACH_S:
                    land_reason = f"approach longer than {APPROACH_S:.0f} s"
                elif not axes and now - phase_t >= a.observe:
                    land_reason = "observe done"
                elif sm and axes and sm["dist"] < a.target - OVERSHOOT_CM:
                    land_reason = f"overshoot ({sm['dist']:.0f} cm)"
                else:
                    if new_frame and m["seen"] and sm and axes:
                        centred = "roll" not in axes or abs(sm["lat"]) <= SIDE_TOL_CM
                        arrive = arrive + 1 if sm["dist"] <= a.target + ARRIVE_TOL_CM and centred else 0
                        if arrive >= ARRIVE_FRAMES:
                            land_reason = "arrived"
                    if locked and not land_reason:
                        would = control(sm, AXES, a.target)  # what full control would do (logged)
                        cmd = [v if ax in axes else 0.0 for v, ax in zip(would, CHANNEL_AXIS)]
                        note = (f"range {sm['dist']:5.1f}  bearing {sm['bearing']:+5.1f}  side {sm['lat']:+5.1f}"
                                f"  elev {sm['elev']:+5.1f} -> pitch {would[3]:+.2f} yaw {would[1]:+.2f}"
                                f" roll {would[2]:+.2f} alt {would[0]:+.2f}"
                                + ("" if axes else "  (observe only)"))
                    elif not locked:
                        note = f"tag not fresh ({lost_for:.1f} s) - sticks centred"
            if land_reason:
                outcome = "landed (arrived)" if land_reason == "arrived" else f"landed ({land_reason})"
                land_steer = land_reason == "arrived"  # steer the descent only after a normal arrival
                reason = land_reason
                pulse_bit, pulse_until, land_t = LAND, now + PULSE_S, now
                state, note, cmd = "LANDING", land_reason + " - landing 0x02, Space = KILL", [0.0] * 4
            if a.sim and state == "LANDING" and now - land_t >= SIM_DESCENT_S:
                src.airborne = False  # sim: on the ground after the descent (findings: slow descent, drifts)
            if state == "LANDING" and not land_reason and axes and locked and land_steer and                     now - land_t < LAND_STEER_S:
                # keep steering while the drone descends (the app allows sticks while landing), no throttle
                would = control(sm, AXES, a.target)
                cmd = [v if ax in axes and ax != "alt" else 0.0 for v, ax in zip(would, CHANNEL_AXIS)]
            if state == "LANDING" and now - land_t >= LAND_S:
                state, note = "DONE", "landed - packets stopped"
                if a.sim:
                    src.airborne = False
            if now >= pulse_until:
                pulse_bit = 0

            # --- sticks (slew-limited like a key press) -> packet
            step = dt / RAMP_S
            sticks = [s + clamp(c - s, -step, step) for s, c in zip(sticks, cmd)]
            flying = state in ("TAKEOFF", "ACQUIRE", "APPROACH", "LANDING")
            out = to_bytes(sticks, {}) if flying else [0x80] * 4
            button = pulse_bit if now < pulse_until else 0
            if a.sim:
                src.step(sticks if flying else [0.0] * 4, dt)
            if state in ("SAFE", "READY", "TAKEOFF", "ACQUIRE", "APPROACH", "LANDING", "KILLING") and link.sock:
                pkt = build(seq, axes=out, button=button)
                if isinstance(src, LiveVideo):
                    pkt = dv.add_acks(pkt, src.take_acks())  # frame acks ride on the stick packet, like the app
                if link.send(pkt):
                    seq += 1
                elif flying:
                    state, outcome = "DONE", f"link lost ({link.error}) - drone failsafe stops motors"
                else:
                    state, note = "OFFLINE", f"{link.error} - C to reconnect"
            if flying or state == "KILLING":
                rows.append([round(now - t0, 3), state, int(locked),
                             *(round(sm[k], 1) if sm else "" for k in ("dist", "bearing", "lat", "elev")),
                             round(lost_for, 2) if t_seen else "", *out, button,
                             round(src.x, 1) if a.sim else "", round(-src.y, 1) if a.sim else "",
                             round(src.fps(), 1), src.seq, getattr(src, "pkts", ""), getattr(src, "frags", ""), eye.n, reason])
            if (state, note) != logged and (state != (logged or ("",))[0] or not flying or new_frame):
                print(f"{now - t0:7.2f}s  {state:9} {note}", flush=True)
                logged = (state, note)

            eye_txt = (f"TAG LOCK  range {sm['dist']:.0f} cm  bearing {sm['bearing']:+.1f}  side {sm['lat']:+.0f} cm"
                   f"  elev {sm['elev']:+.1f}"
                       if locked else f"no lock ({src.status}, {src.fps():.1f} fps)")
            draw(screen, fonts, state, [
                (note, (244, 244, 245)),
                (eye_txt, (110, 255, 130) if locked else (113, 113, 122)),
                (f"axes {a.axes}  target {a.target:.0f} cm  cap {cap:.0f} s"
                 + (f"  {cap - (now - start_t):.1f} s left" if start_t and flying else ""), (113, 113, 122)),
                (f"thr {out[0]} yaw {out[1]} roll {out[2]} pitch {out[3]}  0x{button:02x}  seq {seq}", (113, 113, 122)),
                ("", (0, 0, 0)),
                ("SPACE = KILL    L = land now    Esc = quit", (248, 113, 113)),
                ("C connect   G calibrate   hold Enter 1 s = fly", (113, 113, 122)),
                ("tag 2 UPRIGHT on B facing A", (113, 113, 122)),
                ("tethered, soft ground, nobody within 2 m", (251, 191, 36)),
            ], thumb, m)
            clock.tick(RATE_HZ)
    except BaseException:  # crash or Ctrl+C: never just go silent in the air (silence = motors off, drop)
        traceback.print_exc()
        if state in ("TAKEOFF", "ACQUIRE", "APPROACH", "LANDING") and link.sock:
            first = state != "LANDING"  # already landing: keep it going, no new 0x02 pulse
            print(f"SOFTWARE CRASH while {state} - sending {'land 0x02 + ' if first else ''}centred sticks "
                  f"for {LAND_S:.0f} s, then stopping (Ctrl+C again = stop now, failsafe)", flush=True)
            t_crash = time.monotonic()
            while time.monotonic() - t_crash < LAND_S:
                bit = LAND if first and time.monotonic() - t_crash < PULSE_S else 0
                if not link.send(build(seq, axes=[0x80] * 4, button=bit)):
                    break
                seq += 1
                time.sleep(1 / RATE_HZ)
            outcome, reason = f"software crash in {state} - landed with 0x02", "software crash"
        elif state != "DONE":
            outcome = f"software crash in {state}"
    finally:
        link.home()
        eye.close()
        pygame.quit()
    print(f"closed ({seq} packets sent)")
    os.makedirs(OUT_DIR, exist_ok=True)
    if rows:
        path = os.path.join(OUT_DIR, run_id + ".csv")
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "state", "lock", "range_cm", "bearing_deg", "side_cm", "elev_deg", "since_seen_s",
                        "throttle", "yaw", "roll", "pitch", "button", "sim_side_cm", "sim_front_cm",
                        "video_fps", "video_frames", "video_pkts", "video_frags", "eye_frames",
                        "reason"])
            w.writerows(rows)
        print("flight log", os.path.normpath(path))
    if rec_dir:
        print(f"video: {len(eye.rows)} frames saved to {os.path.normpath(rec_dir)}")
        print(video_summary(eye.rows))
    final = None
    if a.sim:
        final = (f"{-src.y:.0f} cm in front of the tag, {src.x:+.0f} cm sideways of its normal (right +), "
                 f"heading {src.psi:+.0f} deg")
    if start_t:
        ask_result(mode, axes, outcome, final)


if __name__ == "__main__":
    main()
