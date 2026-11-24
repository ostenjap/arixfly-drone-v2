"""Closed-loop fake HS420: renders tag 2 from the simulated drone pose, sends it as real `93 01` fragments
(JPEG q50 4:4:4 scan data, header stripped like the drone), strict app-mode acks, and flies from the stick
bytes in the `ef 02` packets. World: tag at (0,0) facing the drone; drone at (x right, y = -distance), heading
psi (+ = yawed right). Takeoff on 0x01, on the ground 3 s after 0x02, motors off on 0x04."""
import os
import socket
import struct
import sys
import threading
import time

import cv2

TOOLS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools")
sys.path.insert(0, TOOLS)
from aruco_distance import load_focal, render_tag_scene  # noqa: E402
from video_probe import load  # noqa: E402
import math  # noqa: E402

HELLO = bytes.fromhex("ef000400")
STATUS = bytes.fromhex("93041400a5a5a5a50800a5a5008800005a005a55")
W, H = 1280, 720
STICK_LIMIT, THROTTLE_LIMIT = 0.40, 0.55


def header_template():
    for _, _, _, d in load(os.path.join(TOOLS, "..", "captures", "video_probe_20261107_191124.bin")):
        if d[:2] == b"\x93\x01":
            return bytearray(d[:56])


def scan_data(img):
    ok, j = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 50,
                                       cv2.IMWRITE_JPEG_SAMPLING_FACTOR, cv2.IMWRITE_JPEG_SAMPLING_FACTOR_444])
    j = j.tobytes()
    i = j.index(b"\xff\xda")
    n = struct.unpack(">H", j[i + 2:i + 4])[0]
    return j[i + 2 + n:-2]


class SimDrone(threading.Thread):
    V, Wy, TAU = 100.0, 60.0, 0.4   # cm/s and deg/s per power unit, response time

    def __init__(self, dist=150, lateral=10, heading=5, drift=3.0, pitch_sign=+1, fps=12.0, dead=1.0):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.002)
        self.addr = self.sock.getsockname()
        self.x, self.y, self.psi, self.vx, self.vy = lateral, -dist, heading, 0.0, 0.0
        self.drift, self.pitch_sign, self.fps, self.dead = drift, pitch_sign, fps, dead
        self.f = load_focal(W)[0]
        self.hdr = header_template()
        self.stop_ev = threading.Event()
        self.air, self.land_t, self.power = False, None, [0.0] * 4
        self.buttons, self.track = [], []
        self.t0 = time.monotonic()

    def render(self):
        a = math.radians(self.psi)
        vx, vy = -self.x, -self.y
        cx, cz = vx * math.cos(a) - vy * math.sin(a), vx * math.sin(a) + vy * math.cos(a)
        if cz > 8:
            return render_tag_scene(2, 10.0, cz, self.f, W, H, yaw_deg=-self.psi, x_cm=cx)
        return cv2.cvtColor(cv2.UMat(H, W, cv2.CV_8UC1, 200).get(), cv2.COLOR_GRAY2BGR)

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
                            if b == 0x01:
                                self.air = True
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
                a = math.radians(self.psi)
                fwd = self.V * self.pitch_sign * self.power[3]
                lat = self.V * self.power[2] + self.drift
                tvx, tvy = fwd * math.sin(a) + lat * math.cos(a), fwd * math.cos(a) - lat * math.sin(a)
                k = min(1.0, dt / self.TAU)
                self.vx += k * (tvx - self.vx)
                self.vy += k * (tvy - self.vy)
                self.x += self.vx * dt
                self.y += self.vy * dt
                self.psi += self.Wy * self.power[1] * dt
            if peer and now - last_send >= 1 / self.fps and next_id - acked <= 4:
                self.send_frame(next_id, peer)
                next_id += 1
                last_send = now
                self.track.append((round(now - self.t0, 2), self.air, round(-self.y, 1), round(self.x, 1),
                                   round(self.psi, 1)))

    def stop(self):
        self.stop_ev.set()
        self.join(timeout=1)
