"""Ground probe: which way of talking to the drone gets video while stick packets run? Drone on the table.

Every `ef 02` packet sent here has sticks centred (0x80) and button byte 0 = the packets the autopilot sends
while waiting in SAFE: nothing arms, nothing spins. Five phases, 5 s each (hard cap), 2 s pause between,
each on fresh socket(s), then back to the home Wi-Fi:
  A  hello only (`ef 00 04 00` 10 Hz)                      = drone_video.py --live (works)
  B  hello + centred `ef 02` 39 Hz, same socket, NO acks   = autopilot 22:19 build
  C  hello + centred `ef 02` 39 Hz + frame acks, same sock = app-like
  D  hello on socket V, centred `ef 02` on socket C        = autopilot 21:29-22:13 build
  E  autopilot.LiveVideo itself + a 39 Hz stick loop with take_acks (the exact code that showed no video)
Prints per phase and socket: packets by type (`93 01` video, `93 04` status, other), complete frames,
time to the first packet. Saves the first frame of each phase to captures/frames/probe_<time>/.

Usage:
  python tools/video_link_probe.py            # ~35 s, laptop off the home Wi-Fi meanwhile
  python tools/video_link_probe.py --phases AE
"""
import argparse
import collections
import os
import select
import socket
import sys
import threading
import time

import drone_video as dv
from replay_test import DRONE, DRONE_SSID, HOME_SSID, RATE_HZ, build, wait_for_ip, wifi_connect

PHASE_S, PAUSE_S = 5.0, 2.0
CENTRED = [0x80] * 4
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "captures", "frames",
                   "probe_" + time.strftime("%Y%m%d_%H%M%S"))


def new_sock():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", 0))
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    if hasattr(socket, "SIO_UDP_CONNRESET"):
        s.ioctl(socket.SIO_UDP_CONNRESET, False)
    return s


class Rx:
    """Counts what one socket receives."""

    def __init__(self, name):
        self.name, self.types, self.frames, self.first, self.asm = name, collections.Counter(), 0, None, dv.Assembler()
        self.jpg, self.other = None, None

    def feed(self, d, t):
        self.first = self.first if self.first is not None else t
        k = d[:2].hex()
        self.types[k] += 1
        if k not in ("9301", "9304") and self.other is None:
            self.other = d[:32].hex()
        j = self.asm.feed(d)
        if j:
            self.frames += 1
            self.jpg = self.jpg or j
            return self.asm.last
        return None

    def line(self):
        t = self.types
        first = f"{self.first:.2f} s" if self.first is not None else "-"
        other = sum(v for k, v in t.items() if k not in ("9301", "9304"))
        return (f"{self.name:8} video 93 01: {t['9301']:5}  status 93 04: {t['9304']:4}  other: {other:3}  "
                f"frames: {self.frames:3}  first pkt: {first}" + (f"  other e.g. {self.other}" if self.other else ""))


def phase_raw(sticks, acks, split):
    """A-D. sticks: send ef 02; acks: ack finished frames in them; split: ef 02 from a second socket."""
    v = new_sock()
    c = new_sock() if split else v
    rx = {v: Rx("video" if split else "socket")}
    if split:
        rx[c] = Rx("control")
    pending, seq, t0 = [], 0, time.perf_counter()
    next_hello = next_stick = 0.0
    while (t := time.perf_counter() - t0) < PHASE_S:
        if t >= next_hello:
            v.sendto(dv.HELLO, DRONE)
            next_hello += 0.1
        if sticks and t >= next_stick:
            p = build(seq, axes=CENTRED, button=0)
            if acks:
                p = dv.add_acks(p, [(f, n) for f, n, _ in pending[:3]])
                for a in pending[:3]:
                    a[2] += 1
                pending = [a for a in pending if a[2] < dv.ACK_REPEAT]
            c.sendto(p, DRONE)
            seq += 1
            next_stick += 1 / RATE_HZ
        for s in select.select(list(rx), [], [], 0.003)[0]:
            try:
                d = s.recvfrom(65535)[0]
            except OSError:
                continue
            done = rx[s].feed(d, t)
            if done and s is v:
                pending.append([*done, 0])
    for s in rx:
        s.close()
    return list(rx.values())


def phase_autopilot():
    """E: the autopilot's own LiveVideo on a shared socket + a 39 Hz centred stick loop with take_acks."""
    import autopilot  # noqa: E402  (pygame import is harmless here, no window)

    class ProbeLink:
        sock = new_sock()

    link, src = ProbeLink(), autopilot.LiveVideo()
    src.start(link)
    seq, t0 = 0, time.perf_counter()
    while time.perf_counter() - t0 < PHASE_S:
        p = dv.add_acks(build(seq, axes=CENTRED, button=0), src.take_acks())
        link.sock.sendto(p, DRONE)
        seq += 1
        time.sleep(1 / RATE_HZ)
    src.alive = False
    time.sleep(0.1)
    link.sock.close()
    r = Rx("LiveVideo")
    r.types.update({"9301": src.frags, "9304": src.pkts - src.frags})
    r.frames, r.jpg = src.seq, src.latest()[1]
    r.first = None
    return [r]


PHASES = {
    "A": ("hello only", lambda: phase_raw(False, False, False)),
    "B": ("hello + ef02, one socket, no acks", lambda: phase_raw(True, False, False)),
    "C": ("hello + ef02 + acks, one socket", lambda: phase_raw(True, True, False)),
    "D": ("hello / ef02 on two sockets, no acks", lambda: phase_raw(True, False, True)),
    "E": ("autopilot LiveVideo + take_acks", phase_autopilot),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--phases", default="ABCDE")
    a = ap.parse_args()
    results = []
    try:
        ip = None
        for attempt in range(3):
            wifi_connect(DRONE_SSID)
            ip = wait_for_ip("192.168.169.")
            if ip:
                break
            print(f"join attempt {attempt + 1}: no 192.168.169.x address")
        if not ip:
            sys.exit("ABORT: drone Wi-Fi not joined (drone on? Mullvad off? phone off the drone Wi-Fi?)")
        print(f"on drone Wi-Fi as {ip}; button byte 0 and centred sticks in every ef 02 packet")
        for k in a.phases.upper():
            name, fn = PHASES[k]
            print(f"phase {k}: {name} ...", flush=True)
            results.append((k, name, fn()))
            time.sleep(PAUSE_S)
    finally:
        wifi_connect(HOME_SSID)
    os.makedirs(OUT, exist_ok=True)
    print("\nRESULT (5 s per phase)")
    for k, name, rxs in results:
        print(f"{k}  {name}")
        for r in rxs:
            print("   " + r.line())
            if r.jpg:
                with open(os.path.join(OUT, f"phase_{k}_{r.name}.jpg"), "wb") as f:
                    f.write(r.jpg)
    print(f"first frame per phase -> {os.path.normpath(OUT)}")


if __name__ == "__main__":
    main()
