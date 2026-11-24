"""Strict-per-id fake drone: app mode keeps a set of sent-but-unacked frame ids; sends a new frame only while
next_id < oldest unacked id + 4 (sliding window, matches 22:22: got 3,4, acked them, froze); an ack removes exactly that id. LOSE = ids whose fragments
all get lost (the laptop never sees them). Video starts after `dead` s; runs ~fps."""
import socket
import struct
import threading
import time

from fake_drone import HELLO, STATUS, load_frames


class StrictDrone(threading.Thread):
    def __init__(self, lose=(), dead=1.0, fps=15.0):
        super().__init__(daemon=True)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.settimeout(0.002)
        self.addr = self.sock.getsockname()
        self.lose, self.dead, self.fps = set(lose), dead, fps
        self.frames = load_frames()
        self.stop_ev = threading.Event()
        self.st = dict(sent=0, delivered=0, acks=0, stuck_on=None)

    def run(self):
        first, peer, next_id, out, last = None, None, 1, set(), 0.0
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
                        for k in range(d[8]):
                            fid = struct.unpack_from("<I", d, 88 + 24 * k)[0]
                            out.discard(fid)
                            self.st["acks"] += 1
            except (socket.timeout, ConnectionResetError):
                pass
            if peer and now - last >= 1 / self.fps:
                base = min(out) if out else next_id   # sliding window: oldest unacked id
                if next_id >= base + 4:
                    self.st["stuck_on"] = sorted(out)
                    continue
                if next_id not in self.lose:
                    for frag in self.frames[next_id % len(self.frames)]:
                        p = bytearray(frag)
                        p[16:20] = struct.pack("<I", next_id)
                        self.sock.sendto(bytes(p), peer)
                    self.st["delivered"] += 1
                out.add(next_id)
                next_id += 1
                self.st["sent"] += 1
                last = now
                self.st["stuck_on"] = None

    def stop(self):
        self.stop_ev.set()
        self.join(timeout=1)
