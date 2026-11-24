"""Closed-loop fake HS420 with height, textured room and disturbances (for recovery tests).
World (cm): tag 2 at origin, upright, facing the drone (-y side). Drone at (x right, y = -distance, z = height of
the camera above the tag centre), heading psi (+ right). Camera: 1280x720, f from calib. Background: random-texture
walls 300 cm away (cylindrical panorama) so optical flow sees the turn. Takeoff (0x01): climbs to CLIMB_Z in 2 s.
Real-flight-like drift: backward vy, slow yaw. jerk=(t_after_takeoff, dpsi, dz): sudden swing (tether)."""
import math
import os
import socket
import struct
import sys
import threading
import time

import cv2
import numpy as np

TOOLS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools")
sys.path.insert(0, TOOLS)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from aruco_distance import load_focal  # noqa: E402
from sim_drone import HELLO, STATUS, header_template, scan_data  # noqa: E402

W, H = 1280, 720
STICK_LIMIT, THROTTLE_LIMIT = 0.40, 0.55
rng = np.random.default_rng(3)
PANO = np.full((1800, 9000, 3), 150, np.uint8)
for _ in range(2500):   # random rectangles = trackable corners
    x, y = int(rng.integers(0, 9000)), int(rng.integers(0, 1800))
    c = tuple(int(v) for v in rng.integers(30, 230, 3))
    cv2.rectangle(PANO, (x, y), (x + int(rng.integers(8, 60)), y + int(rng.integers(8, 60))), c, -1)
PANO = cv2.GaussianBlur(PANO, (3, 3), 0)
TAG = cv2.copyMakeBorder(cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                                       2, 400), 80, 80, 80, 80, cv2.BORDER_CONSTANT, value=255)


class SimDrone2(threading.Thread):
    V, VZ, WY, TAU, CLIMB_Z = 100.0, 40.0, 60.0, 0.4, 55.0
    TILT = 0.0   # camera pitched down, deg (2026-11-10 22:39: the user tilted the drone camera down)

    def __init__(self, dist=140, lateral=0, heading=0, drift_back=20.0, yaw_drift=4.0, jerk=None, yaw_sign=1,
                 fps=12.0, dead=1.0):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.002)
        self.addr = self.sock.getsockname()
        self.x, self.y, self.z, self.psi = float(lateral), -float(dist), 0.0, float(heading)
        self.vx = self.vy = 0.0
        self.drift_back, self.yaw_drift, self.jerk, self.yaw_sign = drift_back, yaw_drift, jerk, yaw_sign
        self.fps, self.dead = fps, dead
        self.f = load_focal(W)[0]
        self.hdr = header_template()
        self.stop_ev = threading.Event()
        self.air, self.t_air, self.land_t, self.power = False, None, None, [0.0] * 4
        self.buttons, self.track, self.t0 = [], [], time.monotonic()

    def render(self):
        a = math.radians(self.psi)
        # background: panorama crop shifted by heading and height
        ox = int((self.f * a) % (PANO.shape[1] - W)) if a >= 0 else int((PANO.shape[1] - W) + self.f * a) % (PANO.shape[1] - W)
        oy = int(clamp(600 - self.f * self.z / 300.0, 0, PANO.shape[0] - H))
        img = PANO[oy:oy + H, ox:ox + W].copy()
        # tag corners (10 cm black square inside a white border) -> camera -> pixels
        s = 10.0 / 400   # cm per pixel of the 400 px marker (560 px with its white border)
        src, dst = [], []
        for u, v in ((0, 0), (560, 0), (560, 560), (0, 560)):
            X, Z = (u - 280) * s, -(v - 280) * s           # tag plane: x right, z up (cm)
            dx, dy, dz = X - self.x, 0.0 - self.y, Z - self.z
            cx = dx * math.cos(a) - dy * math.sin(a)        # camera right
            cz = dx * math.sin(a) + dy * math.cos(a)        # camera forward
            tl = math.radians(self.TILT)                    # camera looks down by TILT
            cz, dz = cz * math.cos(tl) - dz * math.sin(tl), dz * math.cos(tl) + cz * math.sin(tl)
            if cz < 5:
                return img
            src.append((u, v))
            dst.append((W / 2 + self.f * cx / cz, H / 2 - self.f * dz / cz))
        M = cv2.getPerspectiveTransform(np.float32(src), np.float32(dst))
        tag = cv2.warpPerspective(cv2.cvtColor(TAG, cv2.COLOR_GRAY2BGR), M, (W, H))
        mask = cv2.warpPerspective(np.full(TAG.shape, 255, np.uint8), M, (W, H)) > 0
        img[mask] = tag[mask]
        return img

    def send_frame(self, fid, peer):
        scan = scan_data(self.render())
        n = (len(scan) + 1023) // 1024
        for i in range(n):
            part = scan[i * 1024:(i + 1) * 1024]
            h = bytearray(self.hdr)
            h[2:4] = (56 + len(part)).to_bytes(2, "little")
            h[16:20] = struct.pack("<I", fid)
            h[32:44] = struct.pack("<III", i, n, len(scan))
            h[44:48] = struct.pack("<HH", W, H)
            h[48] = 50
            self.sock.sendto(bytes(h) + part, peer)

    def run(self):
        first, peer, next_id, acked, last_send, last_btn, t_prev = None, None, 1, 0, 0.0, None, time.monotonic()
        jerked = False
        while not self.stop_ev.is_set():
            now = time.monotonic()
            try:
                d, addr = self.sock.recvfrom(4096)
                first = first or now
                if now - first >= self.dead:
                    if d == HELLO:
                        peer = addr
                        self.sock.sendto(STATUS, addr)
                    elif d[:2] == b"\xef\x02":
                        b = d[25]
                        if b != last_btn:
                            last_btn = b
                            self.buttons.append((round(now - self.t0, 2), b))
                            if b == 0x01 and not self.air:
                                self.air, self.t_air = True, now
                            elif b == 0x02:
                                self.land_t = now
                            elif b == 0x04:
                                self.air = False
                        self.power = [(d[20 + i] - 128) / ((THROTTLE_LIMIT if i == 0 else STICK_LIMIT) * 127)
                                      for i in range(4)]
                        for k in range(d[8]):
                            acked = max(acked, struct.unpack_from("<I", d, 88 + 24 * k)[0])
            except (socket.timeout, ConnectionResetError):
                pass
            dt, t_prev = now - t_prev, now
            if self.land_t and now - self.land_t > 3.0:
                self.air, self.land_t = False, None
            if self.air:
                ta = now - self.t_air
                if ta < 2.0:
                    self.z = self.CLIMB_Z * ta / 2.0
                elif not self.land_t:
                    self.z += self.VZ * self.power[0] * dt
                else:
                    self.z = max(-30.0, self.z - 25 * dt)
                if self.jerk and not jerked and ta >= self.jerk[0]:
                    self.psi += self.jerk[1]
                    self.z += self.jerk[2]
                    jerked = True
                a = math.radians(self.psi)
                fwd = self.V * self.power[3] - (self.drift_back if not self.land_t else 0.0)
                lat = self.V * self.power[2]
                tvx, tvy = fwd * math.sin(a) + lat * math.cos(a), fwd * math.cos(a) - lat * math.sin(a)
                k = min(1.0, dt / self.TAU)
                self.vx += k * (tvx - self.vx)
                self.vy += k * (tvy - self.vy)
                self.x += self.vx * dt
                self.y += self.vy * dt
                self.psi += (self.WY * self.power[1] * self.yaw_sign + self.yaw_drift) * dt
            if peer and now - last_send >= 1 / self.fps and next_id - acked <= 4:
                self.send_frame(next_id, peer)
                next_id += 1
                last_send = now
                self.track.append((round(now - self.t0, 2), self.air, round(-self.y, 1), round(self.x, 1),
                                   round(self.z, 1), round(self.psi, 1)))

    def stop(self):
        self.stop_ev.set()
        self.join(timeout=1)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))
