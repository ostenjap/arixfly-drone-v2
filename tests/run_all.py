"""Regression tests for tools/live_flight.py + tools/tracker.py against fake drones on this laptop (loopback).
Nothing is sent to the real drone, the Wi-Fi is not touched, calib/ is not touched (tuning goes to a temp file).

Fakes (tests/): fake_drone.py (recorded video, 4-frame window, packet loss), strict_drone.py (sliding window
that waits for the OLDEST unacked frame, whole frames lost), sim_drone.py (closed loop: renders tag 2 from the
simulated pose), sim_drone2.py (adds height, textured room, drift, tether jerk, reversed yaw).

Usage:
  python tests/run_all.py            # everything (~8 min)
  python tests/run_all.py --quick    # video + controls only (~2 min)
  python tests/run_all.py approach recover_jerk      # chosen tests (names below)
Prints PASS/FAIL per test; exit code 1 if any fails. Logs of each test: the temp folder printed at the start.
"""
import argparse
import contextlib
import io
import os
import sys
import tempfile
import time
import traceback
import types

os.environ["SDL_VIDEODRIVER"] = "dummy"   # no window
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "tools"))

import flight_control as fc  # noqa: E402
import live_flight as lf  # noqa: E402
from fake_drone import FakeDrone  # noqa: E402
from strict_drone import StrictDrone  # noqa: E402

TMP = tempfile.mkdtemp(prefix="drone_tests_")
fc.TRIM_FILE = os.path.join(TMP, "tuning.json")   # never the real calib/tuning.json
fc.OLD_TRIM_FILE = os.path.join(TMP, "no_old_trim.json")   # and no carry-over from the real calib/trim.json


def fresh_tuning():
    if os.path.exists(fc.TRIM_FILE):
        os.remove(fc.TRIM_FILE)


def events(out):
    p = os.path.join(out, "events.csv")
    return open(p).read() if os.path.exists(p) else ""


def window(name, addr, script, hold=lambda t: False, cap=30, approach=False, axes="pitch", target=60,
           recover=True, hold_on=True, hop=False):
    fresh_tuning()
    out = os.path.join(TMP, name)
    a = types.SimpleNamespace(cap=cap, no_wifi=True, approach=approach, axes=axes, target=target,
                              recover=recover, hold=hold_on, script=script, script_hold_enter=hold, hop=hop)
    lf.window(a, addr, out)
    return events(out)


def ground(name, drone, secs):
    out = os.path.join(TMP, name)
    net_holder = {}
    orig = lf.Net

    class Spy(orig):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            net_holder["net"] = self
    lf.Net = Spy
    try:
        lf.ground_test(types.SimpleNamespace(test=secs, no_wifi=True), drone.addr, out)
    finally:
        lf.Net = orig
    return net_holder["net"]


def held(*ranges):
    return lambda t: any(a <= t < b for a, b in ranges)


# ------------------------------------------------------------------ tests: each returns (ok, detail)
def t_feed_clean():
    fd = FakeDrone(dead=1.0, loss=0.0)
    fd.start()
    net = ground("feed_clean", fd, 6)
    fd.stop()
    return net.frames.complete >= 50 and net.fps() >= 0, f"{net.frames.complete} frames, 0 dropped expected " \
        f"({net.frames.dropped})"


def t_feed_loss():
    fd = FakeDrone(dead=1.0, loss=0.03)
    fd.start()
    ground("feed_loss", fd, 6)
    fd.stop()
    return fd.st["sent_frames"] >= 40, f"fake kept sending: {fd.st['sent_frames']} frames (3 % fragment loss)"


def t_lost_first_frames():
    sd = StrictDrone(lose={1, 2})
    sd.start()
    net = ground("lost_first", sd, 6)
    sd.stop()
    return net.frames.complete >= 50 and sd.st["stuck_on"] is None, \
        f"{net.frames.complete} frames, drone stuck on {sd.st['stuck_on']} (22:22 freeze case)"


def t_lost_four():
    sd = StrictDrone(lose={30, 31, 32, 33})
    sd.start()
    net = ground("lost_four", sd, 7)
    sd.stop()
    return net.frames.complete >= 50 and net.frames.stalls >= 1, \
        f"{net.frames.complete} frames, {net.frames.stalls} stall recoveries"


def t_controls():
    fd = FakeDrone(dead=1.0, loss=0.0)
    fd.start()
    ev = window("controls", fd.addr, {0.5: "c", 3.0: "g", 21.0: "l", 22.0: "space", 25.0: "esc"},
                held((4.0, 5.3), (8.0, 9.3), (18.0, 19.3)))
    fd.stop()
    b = [x for _, x in fd.buttons if x]
    want = [0x80, 0x01, 0x02, 0x01, 0x02, 0x04]
    ok = b == want and ev.count("TAKEOFF") == 2 and "LAND: Enter held" in ev and "LAND: L pressed" in ev
    return ok, f"drone saw buttons {[hex(x) for x in b]} (want {[hex(x) for x in want]})"


def t_enter_release():
    fd = FakeDrone(dead=1.0, loss=0.0)
    fd.start()
    ev = window("enter_release", fd.addr, {0.5: "c", 10.0: "l", 19.0: "esc"}, held((4.0, 6.5)))
    fd.stop()
    return "LAND: L pressed" in ev and "Enter held" not in ev, "Enter held 2.5 s through takeoff must not land"


def t_cap():
    fd = FakeDrone(dead=1.0, loss=0.0)
    fd.start()
    ev = window("cap", fd.addr, {0.5: "c", 18.0: "esc"}, held((4.0, 5.3)), cap=3)
    fd.stop()
    return "LAND: time cap 3 s" in ev, "time cap 3 s"


def t_feed_lost_air():
    fd = FakeDrone(dead=1.0, loss=0.0)
    fd.start()

    def hold(t):
        if t > 7:
            fd.fps = 0.001
        return 4.0 <= t < 5.3
    ev = window("feed_lost", fd.addr, {0.5: "c", 18.0: "esc"}, hold)
    fd.stop()
    return "camera feed lost" in ev, "feed stops in the air -> land"


def t_crash():
    fd = FakeDrone(dead=1.0, loss=0.0)
    fd.start()

    def hold(t):
        if t >= 7.0:
            raise RuntimeError("injected crash")
        return 4.0 <= t < 5.3
    with contextlib.redirect_stderr(io.StringIO()):
        ev = window("crash", fd.addr, {0.5: "c"}, hold)
    fd.stop()
    lands = [x for _, x in fd.buttons if x == 0x02]
    return "CRASH LANDING" in ev and lands, "program error in the air -> 0x02 + packets"


def t_no_live_no_takeoff():
    fd = FakeDrone(dead=1.0, loss=0.0)
    fd.start()

    def hold(t):
        if t > 2.5:
            fd.fps = 0.001
        return 4.0 <= t < 6.0
    ev = window("no_live", fd.addr, {0.5: "c", 9.0: "esc"}, hold)
    fd.stop()
    return "TAKEOFF" not in ev, "no live feed -> takeoff refused"


def t_watchdog():
    fd = FakeDrone(dead=0.0, loss=0.0)
    fd.start()
    rec = lf.Recorder(os.path.join(TMP, "watchdog"), time.monotonic())
    net = lf.Net(fd.addr, "0.0.0.0", rec)
    net.start()
    time.sleep(1.0)
    net.hb, net.flying = time.monotonic(), True
    time.sleep(2.0)
    net.stop()
    rec.close()
    fd.stop()
    return (not net.flying) and any(b == 0x02 for _, b in fd.buttons), "window silent 1 s in the air -> 0x02"


def t_tuning_keys():
    fd = FakeDrone(dead=1.0, loss=0.0)
    got = []

    class Spy:
        def __init__(s, sock):
            s.s = sock

        def recvfrom(s, n):
            d, a = s.s.recvfrom(n)
            if d[:2] == b"\xef\x02":
                got.append(d[20:28])
            return d, a

        def __getattr__(s, k):
            return getattr(s.s, k)
    fd.sock = Spy(fd.sock)
    fd.start()
    keys = ["[", "[", "[", "up", "tab", "tab", "tab", "tab", "+", "+"]   # 0.25 s apart: one per window frame
    script = {2.5 + 0.25 * i: k for i, k in enumerate(keys)}
    window("tuning", fd.addr, {**script, 0.5: "c", 6.5: "esc"})
    fd.stop()
    import json
    t = json.load(open(fc.TRIM_FILE))
    last = got[-1]
    x = 0
    for v in last[:7]:
        x ^= v
    ok = (t["trim_yaw"], t["trim_pitch"], t["hold_gain"]) == (-6, 2, 0.04) and list(last[1:4]) == [122, 128, 130] \
        and x == last[7]
    return ok, f"file yaw {t['trim_yaw']} pitch {t['trim_pitch']} gain {t['hold_gain']}; bytes {list(last[:4])}"


def sim_run(name, sim, axes="pitch", target=60, end=40, **kw):
    sim.start()
    ev = window(name, sim.addr, {0.5: "c", 2.0: "g", float(end): "esc"}, held((3.0, 4.3)), approach=True,
                axes=axes, target=target, **kw)
    sim.stop()
    return ev, sim.track[-1]


def t_approach():
    from sim_drone import SimDrone
    ev, fin = sim_run("approach", SimDrone(dist=150, lateral=5, heading=2, drift=0.0))
    return "LAND: arrived" in ev and 40 <= fin[2] <= 75, f"ended {fin[2]} cm in front of the tag (target 60)"


def t_pitch_sign_guard():
    from sim_drone import SimDrone
    ev, fin = sim_run("pitch_sign", SimDrone(dist=150, lateral=5, heading=2, drift=0.0, pitch_sign=-1))
    return "pitch sign?" in ev, "reversed pitch -> 'moving AWAY while pushing forward'"


def t_no_tag():
    from sim_drone import SimDrone
    ev, fin = sim_run("no_tag", SimDrone(dist=150, lateral=5, heading=60, drift=0.0))
    return "tag not found" in ev, "tag never in view -> land"


def t_recover_drift():
    from sim_drone2 import SimDrone2
    ev, fin = sim_run("recover_drift", SimDrone2(drift_back=13.0, yaw_drift=6.7), axes="pitch,yaw,alt", end=45)
    return "LAND: arrived" in ev, f"measured drifts (13 cm/s, 6.7 deg/s): ended {fin[2]} cm in front"


def t_recover_fastyaw():
    from sim_drone2 import SimDrone2
    ev, fin = sim_run("recover_fastyaw", SimDrone2(drift_back=13.0, yaw_drift=15.0), axes="pitch,yaw,alt", end=45)
    return "LAND: arrived" in ev and "yaw sign" not in ev, f"15 deg/s yaw drift: ended {fin[2]} cm in front"


def t_recover_jerk():
    from sim_drone2 import SimDrone2
    ev, fin = sim_run("recover_jerk", SimDrone2(drift_back=8.0, yaw_drift=0.0, jerk=(6.0, 45.0, 25.0)),
                      axes="pitch,yaw,alt", end=45)
    return "RECOVERED" in ev and "LAND: arrived" in ev, f"+45 deg / +25 cm jerk: ended {fin[2]} cm in front"


def t_recover_realyaw_trimmed():
    """Measured yaw strength (~15 deg/s per power unit, flight_review of the 01:02 flights) and a trimmed drone
    (3 deg/s own turn left over): must arrive. Untrimmed (15 deg/s) it spins away - trim is required."""
    from sim_drone2 import SimDrone2
    sim = SimDrone2(drift_back=13.0, yaw_drift=3.0)
    sim.WY = 15.0
    ev, fin = sim_run("recover_realyaw", sim, axes="pitch,yaw,alt", end=45)
    return "LAND: arrived" in ev, f"real yaw strength, trimmed: ended {fin[2]} cm in front, heading {fin[5]:+.0f}"


def _render_tags(tags, f=1152.0, w=1280, h=720):
    """White frame with flat-on 10 cm tags: tags = [(id, x_cm, y_cm, z_cm)] in camera axes (x right, y down)."""
    import cv2
    import numpy as np
    img = np.full((h, w, 3), 255, np.uint8)
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for i, x, y, z in tags:
        n = int(round(fc.TAG_CM * f / z))
        cx, cy = w / 2 + f * x / z, h / 2 + f * y / z
        x0, y0 = int(round(cx - n / 2)), int(round(cy - n / 2))
        img[y0:y0 + n, x0:x0 + n] = cv2.cvtColor(cv2.aruco.generateImageMarker(dic, i, n), cv2.COLOR_GRAY2BGR)
    return img


def t_top_tag():
    """Bottom tag hidden, top tag seen -> find_tag(use_top) returns where the bottom tag is; bottom wins if both."""
    dx, dy, dz = fc.TOP_TO_BOTTOM_CM
    bottom = (10.0, 15.0, 150.0)
    top = (bottom[0] - dx, bottom[1] - dy, bottom[2] - dz)
    only_top = _render_tags([(fc.TOP_ID, *top)])
    both = _render_tags([(fc.TOP_ID, *top), (fc.TAG_ID, *bottom)])
    r_top, r_off = fc.find_tag(only_top, 1152.0, use_top=True), fc.find_tag(only_top, 1152.0)
    r_both = fc.find_tag(both, 1152.0, use_top=True)
    want = (bottom[0] ** 2 + bottom[1] ** 2 + bottom[2] ** 2) ** 0.5
    ok = (r_top is not None and r_top["src"] == "top" and abs(r_top["dist"] - want) < 3 and r_off is None
          and r_both is not None and r_both["src"] == "bottom" and abs(r_both["dist"] - want) < 3
          and abs(r_top["elev"] - r_both["elev"]) < 1.0 and abs(r_top["bearing"] - r_both["bearing"]) < 1.0)
    d = lambda r: "none" if r is None else f"{r['src']} {r['dist']:.1f} cm b {r['bearing']:+.1f} e {r['elev']:+.1f}"
    return ok, f"top only -> {d(r_top)}; both -> {d(r_both)}; want {want:.1f} cm; top w/o use_top -> {d(r_off)}"


def t_blind_hover():
    """VO blind the whole search (plain curtain/wall, 21:55 flights): centred sticks, no landing until RECOVER_S."""
    import tracker
    vo = types.SimpleNamespace(t_valid=-3.0, heading=0.0, blind_for=lambda t: t + 3.0)   # blind since the climb
    rec = tracker.Recover(0.0, 0.0, check_on=False)
    steered, abort_t, t = False, None, 0.0
    while t < tracker.RECOVER_S + 1.0 and abort_t is None:
        out, txt = rec.step(t, (-5.0, -10.0, 2.0), vo)
        steered |= any(abs(v) > 1e-9 for v in out)
        if rec.abort:
            abort_t = t
        t += 0.05
    ok = not steered and abort_t is not None and abort_t >= tracker.RECOVER_S - 0.1 and "not recovered" in rec.abort
    return ok, f"steered {steered}; landed at {abort_t} s ({rec.abort}); last phase: {txt}"


def t_hop():
    """Demo hop in the simulator (true speed 50 cm/s at hop_power 0.5, start 200 cm, target 60): hop 1 with the
    default guess (60 cm/s) stops short, measures the landing spot and sets hop_speed; hop 2 then lands near 60."""
    from sim_drone import SimDrone
    import json
    sim = SimDrone(dist=200, lateral=0, heading=0, drift=0.0)
    sim.start()
    ev = window("hop", sim.addr, {0.5: "c", 2.0: "g", 38.0: "esc"}, held((3.0, 4.3), (20.0, 21.3)), hop=True,
                target=60)
    sim.stop()
    res = [line for line in ev.splitlines() if "HOP RESULT" in line]
    speed = json.load(open(fc.TRIM_FILE))["hop_speed"]
    ends = [float(x.split("landed ")[1].split(" cm")[0]) for x in res]
    ok = len(ends) == 2 and 35 <= speed <= 60 and abs(ends[1] - 60) <= 15
    return ok, f"landings {ends} cm (target 60), hop_speed now {speed} cm/s (sim true 50)"


def t_tilted_close():
    """Camera tilted 25 deg down, drone 55 cm above the tag, axes pitch,yaw (22:39 overshoot): the tag leaves the
    picture at the bottom when close -> BRAKE -> land in front, not glide over the box."""
    from sim_drone2 import SimDrone2
    sim = SimDrone2(dist=220, drift_back=0.0, yaw_drift=0.0)
    sim.TILT = 25.0
    ev, fin = sim_run("tilted_close", sim, axes="pitch,yaw", end=45)
    air = [p for p in sim.track if p[1]]
    closest = min(p[2] for p in air) if air else None
    ok = "BRAKE" in ev and "LAND:" in ev and closest is not None and 15 <= closest and 25 <= fin[2] <= 110
    reason = [line for line in ev.splitlines() if "LAND:" in line][:1]
    return ok, f"closest {closest} cm, ended {fin[2]} cm in front (box at 0); {reason}"


def t_status_log():
    """`93 04` status -> status.csv (battery 90 from the fake drone's real captured packet); trims + battery logged
    at takeoff as 'tune trim_x = v (at takeoff ...)' lines that flight_review / drift_trend read."""
    import csv as _csv
    import json
    fd = FakeDrone(dead=1.0, loss=0.0)
    fd.start()
    fresh_tuning()
    t = json.load(open(fc.TRIM_FILE)) if os.path.exists(fc.TRIM_FILE) else dict(fc.TUNE_DEFAULT)
    t.update(trim_pitch=6, trim_roll=-4, trim_yaw=-2)
    fc.save_tuning(t)
    out = os.path.join(TMP, "status_log")
    a = types.SimpleNamespace(cap=30, no_wifi=True, approach=False, axes="pitch", target=60, recover=True, hold=True,
                              script={0.5: "c", 7.0: "l", 17.0: "esc"}, script_hold_enter=held((3.0, 4.3)), hop=False)
    lf.window(a, fd.addr, out)
    fd.stop()
    ev = events(out)
    rows = list(_csv.DictReader(open(os.path.join(out, "status.csv")))) if os.path.exists(
        os.path.join(out, "status.csv")) else []
    want = ["tune trim_pitch = 6 (at takeoff", "tune trim_roll = -4 (at takeoff", "tune trim_yaw = -2 (at takeoff",
            "status at takeoff: fly_state 0 battery 90"]
    ok = rows and rows[0]["battery"] == "90" and rows[0]["xor_ok"] == "1" and all(w in ev for w in want)
    return bool(ok), f"status.csv rows {len(rows)} battery {rows[0]['battery'] if rows else '-'}; " \
                     f"takeoff lines {sum(w in ev for w in want)}/4"


QUICK = ["feed_clean", "feed_loss", "lost_first_frames", "lost_four", "controls", "enter_release", "cap",
         "feed_lost_air", "crash", "no_live_no_takeoff", "watchdog", "tuning_keys", "top_tag", "blind_hover", "hop", "status_log"]
ALL = QUICK + ["tilted_close", "approach", "pitch_sign_guard", "no_tag", "recover_drift", "recover_fastyaw", "recover_jerk",
               "recover_realyaw_trimmed"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("names", nargs="*")
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args()
    names = a.names or (QUICK if a.quick else ALL)
    print(f"logs: {TMP}")
    results = []
    for n in names:
        t0 = time.monotonic()
        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                ok, detail = globals()["t_" + n]()
        except Exception as e:
            ok, detail = False, f"{type(e).__name__}: {e}"
            traceback.print_exc(file=buf)
        open(os.path.join(TMP, n + ".log"), "w").write(buf.getvalue())
        results.append((n, ok))
        print(f"{'PASS' if ok else 'FAIL'}  {n:20} {time.monotonic() - t0:5.1f} s  {detail}", flush=True)
    bad = [n for n, ok in results if not ok]
    print(f"\n{len(results) - len(bad)}/{len(results)} passed" + (f"; FAILED: {', '.join(bad)}" if bad else ""))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
