"""Ask the HS420 for its camera stream and record every UDP packet it sends back (no flight commands).

Sends ONLY the app's 4-byte `ef 00 04 00` packet (captures/idle.pcap: the app repeats it at ~10 Hz before
video starts), never an `ef 02` stick packet, so the motors cannot be unlocked by this tool.
Joins the drone Wi-Fi, records for at most --seconds (hard cap 15 s), then reconnects to the home Wi-Fi.

Usage:
  python tools/video_probe.py                    # 8 s probe -> captures/video_probe_<stamp>.bin + summary
  python tools/video_probe.py --seconds 12
  python tools/video_probe.py --summary captures/video_probe_X.bin   # re-print the summary of a recording

File format (.bin): repeated [<d t> <H src_port> <H local_port> <I len> payload], little-endian.
"""
import argparse
import os
import socket
import struct
import time
from collections import Counter, defaultdict

from replay_test import DRONE, DRONE_SSID, HOME_SSID, wait_for_ip, wifi_connect

HELLO = bytes.fromhex("ef000400")  # byte-identical to the app's packet in captures/idle.pcap
HELLO_HZ = 10
HARD_CAP_S = 15
REC = struct.Struct("<dHHI")


def record(seconds):
    socks = []
    main = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    main.bind(("0.0.0.0", 0))
    main.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    socks.append(main)
    try:  # in case the drone streams to a fixed port instead of our source port
        extra = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        extra.bind(("0.0.0.0", 8800))
        socks.append(extra)
    except OSError:
        pass
    for s in socks:
        s.setblocking(False)
    pkts, t0, next_hello = [], time.perf_counter(), 0.0
    while (t := time.perf_counter() - t0) < min(seconds, HARD_CAP_S):
        if t >= next_hello:
            main.sendto(HELLO, DRONE)
            next_hello += 1 / HELLO_HZ
        got = False
        for s in socks:
            try:
                data, src = s.recvfrom(65535)
                pkts.append((time.perf_counter() - t0, src[1], s.getsockname()[1], data))
                got = True
            except (BlockingIOError, OSError):
                pass
        if not got:
            time.sleep(0.0005)
    for s in socks:
        s.close()
    return pkts


def save(pkts, path):
    with open(path, "wb") as f:
        for t, sp, lp, d in pkts:
            f.write(REC.pack(t, sp, lp, len(d)) + d)


def load(path):
    buf, pos, out = open(path, "rb").read(), 0, []
    while pos + REC.size <= len(buf):
        t, sp, lp, n = REC.unpack_from(buf, pos)
        pos += REC.size
        out.append((t, sp, lp, buf[pos:pos + n]))
        pos += n
    return out


def summary(pkts):
    if not pkts:
        print("no packets received")
        return
    flows = defaultdict(list)
    for p in pkts:
        flows[(p[1], p[2])].append(p)
    for (sp, lp), ps in flows.items():
        sizes = Counter(len(p[3]) for p in ps)
        dur = ps[-1][0] - ps[0][0]
        print(f"drone:{sp} -> laptop:{lp}  {len(ps)} pkts, {sum(len(p[3]) for p in ps)} B in {dur:.1f} s, "
              f"first at {ps[0][0]:.2f} s, sizes {sizes.most_common(5)}")
        heads = Counter(p[3][:4].hex(" ") for p in ps)
        print(f"  first-4-byte headers: {heads.most_common(6)}")
        soi = [(i, p[3].find(b"\xff\xd8\xff")) for i, p in enumerate(ps) if b"\xff\xd8\xff" in p[3]]
        eoi = sum(1 for p in ps if b"\xff\xd9" in p[3])
        print(f"  packets containing JPEG start ff d8 ff: {len(soi)} (offsets {Counter(o for _, o in soi).most_common(3)}),"
              f" containing ff d9: {eoi}")
        for t, _, _, d in ps[:6]:
            print(f"    {t:6.3f}s {len(d):5} B  {d[:48].hex(' ')}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=8)
    ap.add_argument("--summary", help="print the summary of an existing .bin and exit")
    a = ap.parse_args()
    if a.summary:
        summary(load(a.summary))
        return
    pkts = []
    try:
        wifi_connect(DRONE_SSID)
        ip = wait_for_ip("192.168.169.")
        if not ip:
            print("ABORT: laptop did not get a 192.168.169.x address (drone on? Wi-Fi up after ~10 s)")
            return
        print(f"on drone Wi-Fi as {ip}, sending ef 00 04 00 at {HELLO_HZ} Hz for {min(a.seconds, HARD_CAP_S):g} s")
        pkts = record(a.seconds)
    finally:
        wifi_connect(HOME_SSID)
    out = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "captures",
                                        f"video_probe_{time.strftime('%Y%m%d_%H%M%S')}.bin"))
    save(pkts, out)
    print(f"saved {len(pkts)} packets to {out}")
    summary(pkts)


if __name__ == "__main__":
    main()
