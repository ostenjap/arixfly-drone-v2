"""Props-OFF preflight: verify calibrate (0x80), takeoff (0x01), emergency stop (0x04) with the laptop mic.

Command bits in byte 25 decoded from the HS420 app (ba2.a(), 2026-11-05): 0x01 takeoff, 0x02 land,
0x04 emergency stop, 0x20 flip, 0x80 gyro calibration. Bytes 20..23 = throttle, yaw, roll, pitch.
(The 2026-11-04 runs used turbodrone's meanings: their "calibrate" 0x04 was e-stop, "kill" 0x02 was land.)
The laptop mic times the motors (numbers only, no audio saved).

  kill run (default), props OFF, timeline in seconds:
    0-2 centred (room noise) | 2-3 CALIBRATE 0x80 (motors must stay off; watch the LEDs) |
    3.5-4.5 TAKEOFF | 4.5-7.5 centred (motors run) | 7.5-8.5 E-STOP 0x04, packets continue to 11.0
    -> motors must stop at ~7.5 (e-stop works), not at ~12 (that would only be the no-packet failsafe)
  --axes run: takeoff, then bytes 20, 21, 22, 23 each set to 200 for 2 s with 1 s centred between.

Usage:
  python tools/preflight_test.py              # dry run: print the timeline, send nothing
  python tools/preflight_test.py --send       # kill run, props OFF
  python tools/preflight_test.py --send --axes
  python tools/preflight_test.py --send --land   # props ON, tethered: takeoff, hover 3 s, land 0x02
  python tools/preflight_test.py --send --throttle      # props ON, tethered: hover, byte 20 +40 then -40, land
  python tools/preflight_test.py --send --throttle 60   # bigger step if +40 shows no height change

Verified: 0x01 = one-key takeoff (lifted off 2/2); the no-packet failsafe stops the motors ~1.2 s after the
last packet. Not verified: 0x80, 0x04 in flight, 0x02 land, stick directions.
"""
import argparse
import socket
import time

import numpy as np

from motor_listen import Listener, save, timeline
from replay_test import DRONE, DRONE_SSID, HOME_SSID, RATE_HZ, build, wait_for_ip, wifi_connect

TAKEOFF, LAND, ESTOP, CALIBRATE = 0x01, 0x02, 0x04, 0x80  # byte 25, from the app's ba2.a()
THROTTLE = 0  # index of byte 20 in the axes tuple (bytes 20..23 = throttle, yaw, roll, pitch)
CENTRE = (0x80,) * 4


def kill_plan():
    """[(t_start, t_end, axes, button, label)]"""
    return [(0.0, 2.0, CENTRE, 0, "centre"), (2.0, 3.0, CENTRE, CALIBRATE, "CALIBRATE 0x80"),
            (3.0, 3.5, CENTRE, 0, "centre"), (3.5, 4.5, CENTRE, TAKEOFF, "TAKEOFF"),
            (4.5, 7.5, CENTRE, 0, "centre (motors on)"), (7.5, 8.5, CENTRE, ESTOP, "E-STOP 0x04"),
            (8.5, 11.0, CENTRE, 0, "centre (packets continue)")]


def land_plan():
    """Props ON, tethered: takeoff, short hover, then LAND 0x02. Ends by stopping packets."""
    return [(0.0, 2.0, CENTRE, 0, "centre"), (2.0, 3.0, CENTRE, TAKEOFF, "TAKEOFF"),
            (3.0, 6.0, CENTRE, 0, "hover (centred)"), (6.0, 7.0, CENTRE, LAND, "LAND 0x02"),
            (7.0, 11.0, CENTRE, 0, "centre (should descend + stop)")]


def throttle_plan(step=40):
    """Props ON, tethered: takeoff, hover, throttle byte 20 up, hover, down, hover, land. One axis only."""
    up, down = [0x80] * 4, [0x80] * 4
    up[THROTTLE], down[THROTTLE] = 0x80 + step, 0x80 - step
    return [(0.0, 2.0, CENTRE, 0, "centre"), (2.0, 3.0, CENTRE, TAKEOFF, "TAKEOFF"),
            (3.0, 6.0, CENTRE, 0, "hover"), (6.0, 7.5, tuple(up), 0, f"THROTTLE UP (byte 20 = {0x80 + step})"),
            (7.5, 10.0, CENTRE, 0, "hover"), (10.0, 11.5, tuple(down), 0, f"THROTTLE DOWN (byte 20 = {0x80 - step})"),
            (11.5, 13.0, CENTRE, 0, "hover"), (13.0, 14.0, CENTRE, LAND, "LAND 0x02"),
            (14.0, 20.0, CENTRE, 0, "landing")]


def axes_plan():
    plan = [(0.0, 2.0, CENTRE, 0, "centre"), (2.0, 3.0, CENTRE, TAKEOFF, "TAKEOFF"), (3.0, 5.0, CENTRE, 0, "centre")]
    t = 5.0
    for i in range(4):
        ax = [0x80] * 4
        ax[i] = 200
        plan += [(t, t + 2, tuple(ax), 0, f"byte {20 + i} = 200"), (t + 2, t + 3, CENTRE, 0, "centre")]
        t += 3
    return plan + [(t, t + 1.0, CENTRE, ESTOP, "E-STOP 0x04")]


def run(plan, sock, t_start):
    seq, n = 0, int(plan[-1][1] * RATE_HZ)
    for i in range(n):
        t = i / RATE_HZ
        while time.perf_counter() - t_start < t:
            time.sleep(0.001)
        seg = next(s for s in plan if s[0] <= t < s[1])
        sock.sendto(build(seq, axes=seg[2], button=seg[3]), DRONE)
        seq += 1
    return seq


def analyse(fr, plan, off):
    """off = mic time of packet t=0."""
    floor, runs = timeline(fr)
    print(f"\nmic noise floor {floor:.1f} dB")
    for s, e, hz, db in runs:
        print(f"  motors ON {s - off:6.2f}s -> {e - off:6.2f}s (packet time)  ~{hz:.0f} Hz")
    print("\nper segment (packet time):")
    for s, e, _, _, label in plan:
        seg = [(db, hz) for t, db, hz in fr if s + 0.3 <= t - off < e - 0.1]
        if seg:
            on = np.mean([db > floor + 12 for db, _ in seg])
            print(f"  {s:5.1f}-{e:5.1f}s  {label:26} motors on {on:4.0%}  median {np.median([d for d, _ in seg]):6.1f} dB"
                  f"  pitch {np.median([h for _, h in seg]):5.0f} Hz")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--send", action="store_true")
    ap.add_argument("--axes", action="store_true")
    ap.add_argument("--land", action="store_true", help="props ON + tethered: takeoff, hover 3 s, land")
    ap.add_argument("--throttle", type=int, nargs="?", const=40, metavar="STEP",
                    help="props ON + tethered: hover, throttle +STEP 1.5 s, hover, -STEP 1.5 s, land (default 40)")
    a = ap.parse_args()
    plan = (axes_plan() if a.axes else land_plan() if a.land else
            throttle_plan(a.throttle) if a.throttle else kill_plan())
    for s, e, ax, btn, label in plan:
        print(f"{s:5.1f}-{e:5.1f}s  byte25=0x{btn:02x}  sticks {ax}  {label}")
    if not a.send:
        print("dry run: nothing sent")
        return

    lst = Listener()
    lst.start()
    off = None
    try:
        wifi_connect(DRONE_SSID)
        ip = wait_for_ip("192.168.169.")
        if not ip:
            print("ABORT: no 192.168.169.x address (drone on? Mullvad off?)")
            return
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        time.sleep(1.0)  # extra quiet second for the mic baseline
        t_start = time.perf_counter()
        off = t_start - lst.t0
        n = run(plan, sock, t_start)
        sock.close()
        print(f"\nsent {n} packets on {ip}")
    finally:
        time.sleep(1.5)
        fr = lst.stop()
        wifi_connect(HOME_SSID)
        print("mic data:", save(fr, tag="preflight_audio") + ".csv")
        if off is not None:
            analyse(fr, plan, off)


if __name__ == "__main__":
    main()
