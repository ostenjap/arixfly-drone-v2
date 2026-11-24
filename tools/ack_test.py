"""Frame-rate test: does acking frames (like the app) remove the 4-frame stall? PROPS OFF.

Three phases on one Wi-Fi connection, --seconds each (hard cap 10 s per phase):
  A  no acks, hello always          (today's behaviour, ~7 fps)
  B  acks, hello always
  C  acks, hello only until video arrives
Acks use the app's video-only `ef 02` packet: no stick values, button byte 0 (no motor unlock).
Prints fps, frame-id progression and the longest gap per phase, then returns to the home Wi-Fi.

Usage:
  python tools/ack_test.py              # 8 s per phase
"""
import argparse
import statistics
import time

import drone_video as dv
from replay_test import DRONE_SSID, HOME_SSID, wait_for_ip, wifi_connect

PHASES = (("A no acks, hello always", False, "always"),
          ("B acks, hello always", True, "always"),
          ("C acks, hello until video", True, "until_video"))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=8)
    a = ap.parse_args()
    results = []
    try:
        wifi_connect(DRONE_SSID)
        ip = wait_for_ip("192.168.169.")
        if not ip:
            print("ABORT: no 192.168.169.x address (drone on? Mullvad off?)")
            return
        print(f"on drone Wi-Fi as {ip}")
        for name, acks, hello in PHASES:
            got = [(t, fid) for t, _, fid in dv.live_frames(a.seconds, cap=10, acks=acks, hello=hello)]
            results.append((name, got))
            time.sleep(2)
    finally:
        wifi_connect(HOME_SSID)
    for name, got in results:
        if len(got) < 2:
            print(f"{name}: {len(got)} frames")
            continue
        ts, fids = [t for t, _ in got], [f for _, f in got]
        gaps = [b - a for a, b in zip(ts, ts[1:])]
        print(f"{name}: {len(ts)} frames, {(len(ts) - 1) / (ts[-1] - ts[0]):.1f} fps, median gap "
              f"{statistics.median(gaps) * 1000:.0f} ms, longest {max(gaps) * 1000:.0f} ms, "
              f"frame ids {fids[:6]} ... {fids[-3:]}")


if __name__ == "__main__":
    main()
