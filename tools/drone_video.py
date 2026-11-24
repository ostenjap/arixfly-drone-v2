"""Receive the HS420 camera as JPEG frames (no flight commands) and optionally predict ArUco distances.

Protocol (verified 2026-11-04, captures/video_probe_20261104_054103.bin, see notes/findings.md):
  laptop -> 192.168.169.1:8800  `ef 00 04 00` at ~10 Hz  (only packet we send; no `ef 02`, no motors)
  drone:1234 -> laptop          `93 01` fragments, 56-byte header + up to 1024 B of JPEG scan data:
     2-3 packet length, 16 frame slot id (cycles 1..4), 32 fragment index, 36 fragment count,
     40 frame bytes, 44 width, 46 height, 48 JPEG quality (0x32 = 50)
  The scan data has no JPEG header: we prepend SOI/DQT/SOF0/DHT/SOS built from the standard JPEG
  (Annex K) tables at that quality, 4:4:4 sampling (byte-identical to the app's q50 template), plus EOI.
  `93 04` 20-byte packets also arrive (~10/s), not decoded yet.

Usage:
  python tools/drone_video.py --from captures/video_probe_X.bin       # decode a recording -> captures/frames/
  python tools/drone_video.py --live --seconds 15                     # join drone Wi-Fi, save frames, return
  python tools/drone_video.py --live --seconds 15 --aruco             # + distance per tag per frame
  python tools/drone_video.py --live --aruco --truth 60               # + error vs. a ruler distance
The laptop is off the home Wi-Fi (no internet) while --live runs; hard cap 60 s.
"""
import argparse
import os
import socket
import struct
import time

from replay_test import DRONE, DRONE_SSID, HOME_SSID, wait_for_ip, wifi_connect

HELLO = bytes.fromhex("ef000400")
HARD_CAP_S = 60
HDR = 56
HERE = os.path.dirname(os.path.abspath(__file__))

# JPEG Annex K tables (zigzag order for DQT, as stored in the file)
_LUMA_Q = bytes([16, 11, 12, 14, 12, 10, 16, 14, 13, 14, 18, 17, 16, 19, 24, 40, 26, 24, 22, 22, 24, 49, 35, 37, 29, 40,
                 58, 51, 61, 60, 57, 51, 56, 55, 64, 72, 92, 78, 64, 68, 87, 69, 55, 56, 80, 109, 81, 87, 95, 98, 103,
                 104, 103, 62, 77, 113, 121, 112, 100, 120, 92, 101, 103, 99])
_CHROMA_Q = bytes([17, 18, 18, 24, 21, 24, 47, 26, 26, 47, 99, 66, 56, 66, 99] + [99] * 49)
_DC_BITS = [0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0]
_DC_C_BITS = [0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0]
_DC_VALS = list(range(12))
_AC_L_BITS = [0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d]
_AC_L_VALS = bytes.fromhex(
    "01020300041105122131410613516107227114328191a1082342b1c11552d1f02433627282090a161718191a25262728292a3435363738393a"
    "434445464748494a535455565758595a636465666768696a737475767778797a838485868788898a92939495969798999aa2a3a4a5a6a7a8a9"
    "aab2b3b4b5b6b7b8b9bac2c3c4c5c6c7c8c9cad2d3d4d5d6d7d8d9dae1e2e3e4e5e6e7e8e9eaf1f2f3f4f5f6f7f8f9fa")
_AC_C_BITS = [0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77]
_AC_C_VALS = bytes.fromhex(
    "00 01 02 03 11 04 05 21 31 06 12 41 51 07 61 71 13 22 32 81 08 14 42 91 a1 b1 c1 09 23 33 52 f0 15 62 72 d1 0a 16"
    " 24 34 e1 25 f1 17 18 19 1a 26 27 28 29 2a 35 36 37 38 39 3a 43 44 45 46 47 48 49 4a 53 54 55 56 57 58 59 5a 63 64"
    " 65 66 67 68 69 6a 73 74 75 76 77 78 79 7a 82 83 84 85 86 87 88 89 8a 92 93 94 95 96 97 98 99 9a a2 a3 a4 a5 a6 a7"
    " a8 a9 aa b2 b3 b4 b5 b6 b7 b8 b9 ba c2 c3 c4 c5 c6 c7 c8 c9 ca d2 d3 d4 d5 d6 d7 d8 d9 da e2 e3 e4 e5 e6 e7 e8 e9"
    " ea f2 f3 f4 f5 f6 f7 f8 f9 fa")


def _scale(table, quality):
    s = 5000 // quality if quality < 50 else 200 - 2 * quality  # IJG quality scaling
    return bytes(min(255, max(1, (q * s + 50) // 100)) for q in table)


def jpeg_header(width, height, quality=50):
    def seg(marker, body):
        return b"\xff" + bytes([marker]) + struct.pack(">H", len(body) + 2) + body

    def dht(cls_id, bits, vals):
        return seg(0xC4, bytes([cls_id]) + bytes(bits) + bytes(vals))

    return (b"\xff\xd8"
            + seg(0xDB, b"\x00" + _scale(_LUMA_Q, quality))
            + seg(0xDB, b"\x01" + _scale(_CHROMA_Q, quality))
            + seg(0xC0, struct.pack(">BHHB", 8, height, width, 3) + bytes([1, 0x11, 0, 2, 0x11, 1, 3, 0x11, 1]))
            + dht(0x00, _DC_BITS, _DC_VALS) + dht(0x10, _AC_L_BITS, _AC_L_VALS)
            + dht(0x01, _DC_C_BITS, _DC_VALS) + dht(0x11, _AC_C_BITS, _AC_C_VALS)
            + seg(0xDA, bytes([3, 1, 0x00, 2, 0x11, 3, 0x11, 0, 0x3F, 0])))


class Assembler:
    """Feed raw UDP payloads; returns a complete JPEG (bytes) whenever a frame finishes."""

    def __init__(self):
        self.slot, self.frags, self.meta = None, {}, None
        self.complete = self.dropped = 0

    def feed(self, d):
        if len(d) < HDR or d[:2] != b"\x93\x01":
            return None
        slot, idx, count, size = struct.unpack_from("<I", d, 16)[0], *struct.unpack_from("<III", d, 32)
        w, h, q = struct.unpack_from("<HHB", d, 44)
        if idx == 0 or slot != self.slot:
            if self.frags:
                self.dropped += 1
            self.slot, self.frags, self.meta = slot, {}, (count, size, w, h, q)
        if idx < count:
            self.frags[idx] = d[HDR:]
        count, size, w, h, q = self.meta
        if len(self.frags) == count:
            scan = b"".join(self.frags[i] for i in range(count))[:size]
            self.frags = {}
            self.complete += 1
            self.last = (slot, count)  # for the ack
            return jpeg_header(w, h, q or 50) + scan + b"\xff\xd9"
        return None


# Video-only `ef 02` packet, byte-identical to the app's when its on-screen sticks are OFF
# (captures/idle2.pcap, 3182 of 3223 record-free packets): control block length 8, bytes 18-28 all zero,
# so it carries NO stick values and NO button (byte 25 = 0 = no motor unlock).
VIDEO_BASE = bytes.fromhex(
    "ef025800020200010000000021000000080000000000000000000000000000000000000000000000000000000000"
    "000000000000000000000000000000000000000000000000000000000000000000000000324b142d0000")
assert len(VIDEO_BASE) == 88 and not any(VIDEO_BASE[18:29])
ACK_HZ, ACK_REPEAT = 39, 3  # app: ~39 packets/s, each finished frame acked in 2-3 packets


def add_acks(base, frames):
    """88-byte `ef 02` packet + one 'complete' record per (frame id, fragment count), as the app sends them:
    <u32 id> <u32 0> <u32 1=complete> <u32 record length> <bitmap, padded to 4 bytes, unused bits = 1>.
    Works on the video-only packet and on stick packets alike: the app appends the same records to its stick
    packets (captures/sticks.pcap: 945 packets with 1 record, 153 with 2, 828 with none); the stick bytes and
    the XOR checksum (bytes 20..27) are untouched."""
    body = b""
    for fid, n in frames:
        nb = (n + 31) // 32 * 4
        body += struct.pack("<IIII", fid, 0, 1, 16 + nb) + b"\xff" * nb
    p = bytearray(base[:88])
    p[2:4] = (88 + len(body)).to_bytes(2, "little")
    p[8] = len(frames)
    return bytes(p) + body


def ack_packet(frames):
    return add_acks(VIDEO_BASE, frames)


def live_frames(seconds, cap=HARD_CAP_S, alive=lambda: True, acks=True, hello="until_video"):
    """Yield (t, jpeg, frame id) from the drone. acks=True sends the app's video-only `ef 02` ack packets
    (no stick values); hello = 'always' | 'until_video' (stop `ef 00 04 00` once frames arrive)."""
    limit = seconds if cap is None else min(seconds, cap)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", 0))
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    s.settimeout(0.005)
    asm, pending, got_video = Assembler(), [], False  # pending = [[fid, nfrag, times_sent]]
    t0, next_hello, next_ack = time.perf_counter(), 0.0, 0.0
    try:
        while (t := time.perf_counter() - t0) < limit and alive():
            if t >= next_hello:
                if hello == "always" or not got_video:
                    s.sendto(HELLO, DRONE)
                next_hello += 0.1
            if acks and got_video and t >= next_ack:
                s.sendto(ack_packet([(f, n) for f, n, _ in pending]), DRONE)
                for p in pending:
                    p[2] += 1
                pending = [p for p in pending if p[2] < ACK_REPEAT]
                next_ack = max(next_ack + 1 / ACK_HZ, t)
            try:
                d = s.recvfrom(65535)[0]
            except (socket.timeout, OSError):
                continue
            jpg = asm.feed(d)
            if jpg:
                got_video = True
                pending.append([*asm.last, 0])
                yield t, jpg, asm.last[0]
    finally:
        s.close()


def live_packets(seconds, cap=HARD_CAP_S, alive=lambda: True):
    """Yield (t, payload) from the drone for at most `seconds` (and `cap`, None = no cap) while alive(),
    sending only the video request."""
    limit = seconds if cap is None else min(seconds, cap)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", 0))
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
    s.settimeout(0.02)
    t0, next_hello = time.perf_counter(), 0.0
    try:
        while (t := time.perf_counter() - t0) < limit and alive():
            if t >= next_hello:
                s.sendto(HELLO, DRONE)
                next_hello += 0.1
            try:
                yield t, s.recvfrom(65535)[0]
            except (socket.timeout, OSError):
                pass
    finally:
        s.close()


def run(packets, outdir, aruco, truth, every):
    import cv2
    import numpy as np
    if aruco:
        from aruco_distance import SIZES_CM, analyse, annotate, focal_from_hfov, load_focal, DEFAULT_HFOV
    os.makedirs(outdir, exist_ok=True)
    asm, n, errs, t_first = Assembler(), 0, [], None
    for t, d in packets:
        jpg = asm.feed(d)
        if jpg is None:
            continue
        n += 1
        t_first = t if t_first is None else t_first
        if (n - 1) % every:
            continue
        path = os.path.join(outdir, f"frame_{n:04d}.jpg")
        open(path, "wb").write(jpg)
        if not aruco:
            continue
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            print(f"{t:6.2f}s frame {n}: decode failed")
            continue
        f, src = load_focal(img.shape[1])
        if f is None:
            f, src = focal_from_hfov(img.shape[1], DEFAULT_HFOV), "UNCALIBRATED"
        res = analyse(img, SIZES_CM, f)
        cv2.imwrite(path.replace(".jpg", "_aruco.jpg"), annotate(img, res, src))
        msg = ", ".join(f"tag {i} {r['dist']:.1f} cm bearing {r['bearing']:+.0f}" for i, r in sorted(res.items()))
        if truth and res:
            errs += [r["dist"] - truth for r in res.values()]
            msg += f"  (error {errs[-1]:+.1f} cm)"
        print(f"{t:6.2f}s frame {n}: {msg or 'no tag'}  [{src}]")
    dur = (t - t_first) if n and t_first is not None else 0
    print(f"{n} frames ({asm.dropped} incomplete dropped), {n / dur if dur else 0:.1f} fps -> {os.path.normpath(outdir)}")
    if errs:
        print(f"mean abs error {sum(abs(e) for e in errs) / len(errs):.1f} cm over {len(errs)} detections "
              f"(truth {truth:g} cm)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--from", dest="src", help="decode a video_probe .bin recording")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--seconds", type=float, default=15)
    ap.add_argument("--aruco", action="store_true", help="predict tag distances per frame")
    ap.add_argument("--truth", type=float, help="ruler distance in cm (prints the error)")
    ap.add_argument("--every", type=int, default=1, help="keep every Nth frame")
    ap.add_argument("--out", default=os.path.join(HERE, "..", "captures", "frames", time.strftime("%Y%m%d_%H%M%S")))
    a = ap.parse_args()
    if a.src:
        from video_probe import load
        run(((t, d) for t, _, _, d in load(a.src)), a.out, a.aruco, a.truth, a.every)
    elif a.live:
        try:
            wifi_connect(DRONE_SSID)
            ip = wait_for_ip("192.168.169.")
            if not ip:
                print("ABORT: laptop did not get a 192.168.169.x address (drone on? Mullvad off/LAN sharing on?)")
                return
            print(f"on drone Wi-Fi as {ip}, receiving for {min(a.seconds, HARD_CAP_S):g} s")
            run(live_packets(a.seconds), a.out, a.aruco, a.truth, a.every)
        finally:
            wifi_connect(HOME_SSID)
    else:
        ap.error("use --from FILE or --live")


if __name__ == "__main__":
    main()
