"""Strict fake HS420 for loopback tests. Model (from real-drone evidence 2026-11-08/09):
- silent for DEAD s after the first packet (Wi-Fi join dead time)
- hello -> one 93 04 status reply; video goes to the hello sender
- hello-only mode: 4-frame window, frame ids cycle 1..4, resumes after 0.35 s without acks
- once ef 02 packets arrive (app mode): running frame ids, 4-frame window, advances ONLY on ack records, no timeout
- LOSS fraction of fragments dropped; ~FPS frames/s
- logs every change of the ef 02 button byte with time"""
import os
import random
import socket
import struct
import sys
import threading
import time

TOOLS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools")
sys.path.insert(0, TOOLS)
from video_probe import load  # noqa: E402

HELLO = bytes.fromhex("ef000400")
STATUS = bytes.fromhex("93041400a5a5a5a50800a5a5008800005a005a55")


def load_frames():
    frames, cur = [], {}
    for _, _, _, d in load(os.path.join(TOOLS, "..", "captures", "video_probe_20261107_191124.bin")):
        if d[:2] != b"\x93\x01":
            continue
        idx = struct.unpack_from("<I", d, 32)[0]
        if idx == 0 and cur:
            frames.append([cur[i] for i in sorted(cur)])
            cur = {}
        cur[idx] = d
    return [f for f in frames if len(f) == struct.unpack_from("<I", f[0], 36)[0]]


class FakeDrone(threading.Thread):
    def __init__(self, dead=2.0, loss=0.03, fps=12.0, seed=1):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.002)
        self.addr = self.sock.getsockname()
        self.dead, self.loss, self.fps = dead, loss, fps
        self.frames, self.rnd = load_frames(), random.Random(seed)
        self.stop_ev = threading.Event()
        self.t0 = time.monotonic()
        self.buttons = []          # (t, button) on change
        self.st = dict(hello=0, ef02=0, acks=0, sent_frames=0, app_mode=False, timeouts=0)

    def run(self):
        first, peer, app = None, None, False
        next_id, acked, cyc, last_send, last_ack_t = 1, 0, 0, 0.0, time.monotonic()
        last_button = None
        while not self.stop_ev.is_set():
            now = time.monotonic()
            try:
                d, addr = self.sock.recvfrom(4096)
                first = first or now
                if now - first >= self.dead:
                    if d == HELLO:
                        peer = addr
                        self.sock.sendto(STATUS, addr)
                        self.st["hello"] += 1
                    elif d[:2] == b"\xef\x02":
                        self.st["ef02"] += 1
                        if not app:
                            app, self.st["app_mode"] = True, True
                            next_id = acked + 1
                        if d[25] != last_button:
                            last_button = d[25]
                            self.buttons.append((round(now - self.t0, 2), d[25]))
                        for k in range(d[8]):
                            fid = struct.unpack_from("<I", d, 88 + 24 * k)[0]
                            acked = max(acked, fid)
                            last_ack_t = now
                            self.st["acks"] += 1
            except (socket.timeout, ConnectionResetError):
                pass
            if not peer or now - last_send < 1 / self.fps:
                continue
            if app:
                if next_id - acked > 4:
                    continue  # strict: wait for acks forever
                fid = next_id
            else:
                if next_id - acked > 4:
                    if now - last_ack_t < 0.35:
                        continue
                    acked, last_ack_t = next_id - 1, now  # hello mode: time out, move on
                    self.st["timeouts"] += 1
                cyc = cyc % 4 + 1
                fid = cyc
            for frag in self.frames[next_id % len(self.frames)]:
                if self.rnd.random() < self.loss:
                    continue
                p = bytearray(frag)
                p[16:20] = struct.pack("<I", fid)
                self.sock.sendto(bytes(p), peer)
            next_id += 1
            self.st["sent_frames"] += 1
            last_send = now
            if not app:
                pass

    def stop(self):
        self.stop_ev.set()
        self.join(timeout=1)
