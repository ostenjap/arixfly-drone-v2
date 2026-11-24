"""Live drone camera + safe takeoff/land window for the HS420, written from scratch (2026-11-09).

A LIVE camera feed that stays live while the control link runs (verified 18.8 fps), every frame saved for
analysis later, takeoff / land / kill, and with --approach: fly to tag 2 and land in front of it.

Approach (--approach): after takeoff CLIMB 3 s -> ACQUIRE (tag 2 seen in 3 frames within 6 s) -> APPROACH:
  pitch = 0.006/cm * (range - target) + 0.002 * integral(range - target) dt (max 0.3), max 0.5 forward /
          0.3 back; forward fades to 0 at 20 deg bearing (pitch only) or to 30 % at 45 deg (with yaw)
  Heading hold (all flights, --no-hold to switch off): visual odometry holds the heading whenever the tag is
  not steering yaw (the drone turned right 6.7 deg/s by itself); lands if the hold turns the wrong way.
  yaw   = 0.02/deg * bearing (deadband 2 deg, max 0.4)          only with --axes pitch,yaw
  power 1.0 = a full key press at the tethered limits (sticks 40 %, throttle 55 %), sticks slew in 0.25 s.
  Lands (0x02) when the range is within target + 8 cm for 3 sightings ("arrived"), or on: tag lost 1.5 s,
  tag not found 6 s, approach > 15 s, overshoot (range < target - 20 cm), range GROWING 30 cm over its start
  (pitch sign wrong?), plus all the normal reasons. Sightings older than 0.5 s are not used (sticks centred).
  alt   = 0.03/deg * elevation (tag below centre -> descend, deadband 3 deg)  only with --axes ...,alt
  Tag lost while approaching (0.5 s) or not seen 1.5 s after takeoff but known from earlier (e.g. on the
  ground) -> RECOVER (tools/tracker.py): a map of where the tag was seen, kept up to date by visual odometry,
  steers yaw/throttle back toward it, then backs off, then sweeps +-30 deg; max 9 s, max 3 per flight, then
  land. --no-recover = land instead. The window draws an orange arrow toward the remembered tag.
  Gains are simulator values (tools/autopilot.py), not yet tuned on the drone. Signs as flown with the keyboard
  (pitch forward = byte 23 up, yaw right = byte 21 up), not yet checked by the camera.

Network (one thread, one socket bound to the drone network, as proven by tools/video_link_probe.py):
  - `ef 00 04 00` video request at 10 Hz, always.
  - `ef 02` stick packets at 39 Hz (centred sticks, button bits for takeoff/land/kill/calibrate), started as soon
    as the drone answers anything, each carrying the app's frame-ack records.
  - EVERY video frame gets acked, also frames we only got partly (given up after 0.2 s without a new fragment):
    with `ef 02` running the drone sends 4 frames and then waits for their acks - one unacked frame froze the
    old autopilot's feed after 2 frames (2026-11-09 17:11).
Safety: takeoff only with a live feed; cap 30 s (max 60) -> land; window loses focus -> land; feed lost 2 s in
the air -> land; the window stops responding 1 s in the air -> the network thread lands by itself;
crash / Ctrl+C in the air -> land 0x02 + 8 s of packets; Space = KILL (0x04 e-stop 1 s, then no stick packets).

Keys: C connect   G calibrate (on the ground)   hold Enter 1 s = takeoff / land   L = land   Space = KILL
      Esc = quit (in the air: land first)
LIVE TUNING (ground or air, saved at once to calib/tuning.json, logged in events.csv, shown under the picture):
      arrows = trim: Up forward, Down back, Right right, Left left   ] turn right, [ turn left
      Tab / Shift+Tab = pick a setting, = / - = change it: hold_on, hold_gain, hold_max, hold_delay, yaw_check,
      approach_gain, approach_max, target_cm. Trims are byte counts added to bytes 22 (roll) / 23 (pitch) /
      21 (yaw) of every stick packet. Old calib/trim.json values are taken over on the first start.
Saved per run in captures/flights/<time>/: frame_NNNN.jpg (every frame as received), frames.csv (time, frame id,
state, button), events.csv (connect, first frame, takeoff, land reasons, kill ...). Summary printed at the end.

Usage:
  python tools/live_flight.py                 # window (drone on table A, tethered, props as you choose)
  python tools/live_flight.py --test 10       # headless ground test: join, live feed 10 s (centred sticks,
                                              # button 0 = nothing arms), print fps, back to home Wi-Fi
  python tools/live_flight.py --cap 20
  python tools/live_flight.py --approach                     # fly to tag 2, pitch only, land 60 cm in front
  python tools/live_flight.py --approach --axes pitch,yaw    # + yaw to keep the tag centred
  python tools/live_flight.py --approach --target 45
  python tools/live_flight.py --approach --axes pitch,yaw,alt   # also keep the tag vertically centred
  python tools/live_flight.py --hop      # DEMO: measure the tag distance on the ground, take off, hover 2.5 s,
      # fly straight forward (heading hold) for (distance - target) / hop_speed seconds, land. Lands early if
      # the tag reads <= target + 8 cm. After landing it measures the distance again on the ground and sets
      # hop_speed (calib/tuning.json) from the real result, so each hop corrects the next one.
Testing without the drone: --addr 127.0.0.1:PORT --no-wifi (a fake drone on this laptop); full regression:
  python tests/run_all.py   (loopback fakes + simulators, never touches the drone, Wi-Fi or calib/)
Code: this file = window + state machine + ground test; tools/drone_link.py = protocol, video acks, network
thread, recording, Wi-Fi; tools/flight_control.py = control law, tag detection, tuning store; tools/tracker.py =
visual odometry, tag memory, recovery search.
"""

import argparse
import math
import os
import sys
import textwrap
import threading
import time
import traceback

import tracker
from drone_link import (CALIBRATE, CAL_S, CENTRED, LAND, LAND_S, LIVE_S, OUT_ROOT, PULSE_S, TAKEOFF, Net, Recorder,
                        join_wifi, summary)
from flight_control import (ACQUIRE_S, ACQUIRE_SEARCH_S, AIR, APPROACH_S, ARRIVE_FRAMES, ARRIVE_TOL_CM, AWAY_CM,
                            BRAKE_POWER, BRAKE_S, CLOSE_ELEV_DEG,
                            CLIMB_S, FRESH_S, HOP_CLIMB_S, HOP_MAX_S, KI_RANGE, LOCK_FRAMES, LOST_RECOVER_S, LOST_S, MAX_I, MAX_RECOVERIES,
                            OVERSHOOT_CM, RAMP_S, REACQUIRE_FRAMES, SMOOTH, TUNE_SPEC, clamp, control, find_tag,
                            load_tuning, reload_tuning, save_tuning, to_bytes, tune_step, tuning_mtime)
from replay_test import DRONE, DRONE_SSID, HOME_SSID, wifi_connect

HERE = os.path.dirname(os.path.abspath(__file__))

HOLD_S = 1.0               # hold Enter this long
STREAK_S = 1.0             # lock / re-lock = LOCK_FRAMES / REACQUIRE_FRAMES sightings within this time

CAP_S, MAX_CAP_S = 30.0, 60.0

FEED_LOST_S = 2.0          # in the air: no frame this long -> land

# ------------------------------------------------------------------ headless ground test
def ground_test(a, addr, out_dir):
    t0 = time.monotonic()
    ip = "0.0.0.0"
    if not a.no_wifi:
        ip, err = join_wifi(print)
        if not ip:
            sys.exit("ABORT: " + err)
        print(f"on drone Wi-Fi as {ip}; centred sticks, button 0 in every packet (nothing arms)")
    rec = Recorder(out_dir, t0)
    net = Net(addr, ip, rec)
    net.state = "GROUND_TEST"
    net.start()
    try:
        end_wait = time.monotonic() + 25
        while net.t_first_frame is None and time.monotonic() < end_wait and net.running:
            time.sleep(0.1)
        if net.t_first_frame is None:
            print("NO VIDEO within 25 s")
        else:
            end = time.monotonic() + a.test
            while time.monotonic() < end and net.running:
                time.sleep(0.5)
                print(f"  live {net.live()}  {net.fps():4.1f} fps  frames {net.frames.complete}"
                      f"  dropped {net.frames.dropped}", flush=True)
    finally:
        net.stop()
        rec.close()
        if not a.no_wifi:
            wifi_connect(HOME_SSID)
    print(summary(net, rec))

# ------------------------------------------------------------------ window
def window(a, addr, out_dir):
    import cv2
    import numpy as np
    import pygame
    from aruco_distance import focal_from_hfov, load_focal

    cap = min(a.cap, MAX_CAP_S)
    pygame.init()
    screen = pygame.display.set_mode((1280, 720))
    pygame.display.set_caption("HS420 live flight")
    font, big = pygame.font.SysFont("consolas", 20), pygame.font.SysFont("consolas", 44, bold=True)
    clock = pygame.time.Clock()
    t0 = time.monotonic()
    rec = Recorder(out_dir, t0)
    net = None
    state, msg = "OFFLINE", "press C to connect"
    joiner = {"busy": False, "ip": None, "err": ""}
    hold_t = fly_t = land_t = None
    released = False
    shown_n, surf, tag = 0, None, None
    tune = load_tuning()
    tune_m, tune_check = tuning_mtime(), 0.0   # live reload of calib/tuning.json
    sel = 0                          # selected tuning row (Tab)
    cli_target = a.target            # --target given on the command line wins over the tuning file
    sm, t_seen, streak, arrive, sticks, phase_t, start_range = None, None, 0, 0, [0.0] * 4, None, None
    seen_ts, src_count = [], {"bottom": 0, "top": 0}
    ground_obs = []              # (t, cm, sideways cm) tag readings on the ground (start / landing point)
    hop = {"start": None, "T": 0.0, "fwd_s": 0.0, "measure_at": None, "result": ""}
    brake_until, brake_reason, brake_pow = 0.0, "", BRAKE_POWER

    def ground_range(now):
        v = sorted(d for t2, d, _ in ground_obs if now - t2 < 1.0)
        return v[len(v) // 2] if len(v) >= 5 else None

    def ground_side(now):
        v = sorted(s for t2, _, s in ground_obs if now - t2 < 1.0)
        return v[len(v) // 2] if len(v) >= 5 else 0.0
    axes = [x for x in a.axes.split(",") if x]
    vo = tmap = recov = None     # tracker.YawVO / TagMap / Recover, made at the first frame
    recoveries, rec_phase, pred = 0, "", None
    integ, fwd_avg = 0.0, 0.0   # fwd_avg: running mean of the commanded pitch (pitch-sign guard)
    hold_ref, hold_check = None, tracker.YawSignCheck()   # VO heading hold against the drone's own yaw drift
    script = dict(a.script or {})  # test hook: {seconds: key}

    def log(text):
        rec.event(time.monotonic(), text)

    def do_join():
        ip, err = ("0.0.0.0", "") if a.no_wifi else join_wifi(lambda s: None)
        joiner.update(busy=False, ip=ip, err=err)

    def land(reason):
        nonlocal state, land_t, msg, sticks
        sticks = [0.0] * 4
        net.set_axes(CENTRED)
        net.pulse(LAND, PULSE_S)
        net.flying = False
        if state == "HOP":
            hop["fwd_s"] = time.monotonic() - phase_t
        if (a.hop and hop["start"] is not None) or a.approach:
            hop["measure_at"] = "after landing"   # scorecard: measure the landing spot from the ground
        state, land_t, msg = "LANDING", time.monotonic(), f"landing (0x02): {reason}"
        log(f"LAND: {reason}")
        log(f"tag sightings in the air: bottom {src_count['bottom']}, top {src_count['top']}")

    try:
        while state != "DONE":
            now = time.monotonic()
            keys = set()
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    keys.add("esc")
                elif ev.type == pygame.KEYDOWN:
                    keys.add({pygame.K_SPACE: "space", pygame.K_ESCAPE: "esc", pygame.K_c: "c", pygame.K_g: "g",
                              pygame.K_l: "l", pygame.K_UP: "up", pygame.K_DOWN: "down", pygame.K_LEFT: "left",
                              pygame.K_RIGHT: "right", pygame.K_LEFTBRACKET: "[", pygame.K_RIGHTBRACKET: "]",
                              pygame.K_EQUALS: "+", pygame.K_PLUS: "+", pygame.K_KP_PLUS: "+", pygame.K_MINUS: "-",
                              pygame.K_KP_MINUS: "-",
                              pygame.K_TAB: "shift-tab" if ev.mod & pygame.KMOD_SHIFT else "tab"}.get(ev.key, ""))
            for at in [k for k in script if now - t0 >= k]:
                keys.add(script.pop(at))
            enter = bool(pygame.key.get_pressed()[pygame.K_RETURN]) or a.script_hold_enter(now - t0)
            focused = pygame.key.get_focused() or a.script is not None
            if net:
                net.hb = now
                net.state = state
            # --- trim (arrows, ground or air): drone drifts BACK -> Up (forward), drifts LEFT -> Right
            # --- live tuning: arrows/[ ] = trims, Tab picks a row, = / - change it; saved + logged at once
            changes = [(k2, name, sgn) for k2, name, sgn in (
                ("up", "trim_pitch", 1), ("down", "trim_pitch", -1), ("right", "trim_roll", 1),
                ("left", "trim_roll", -1), ("]", "trim_yaw", 1), ("[", "trim_yaw", -1),
                ("+", TUNE_SPEC[sel][0], 1), ("-", TUNE_SPEC[sel][0], -1)) if k2 in keys]
            if "tab" in keys:
                sel = (sel + 1) % len(TUNE_SPEC)
            if "shift-tab" in keys:
                sel = (sel - 1) % len(TUNE_SPEC)
            for _, name, sgn in changes:
                v = tune_step(tune, name, sgn)
                save_tuning(tune)
                tune_m = tuning_mtime()
                log(f"tune {name} = {v} (in {state})")
            if now - tune_check > 0.5:   # file changed outside the window (edited between flights) -> use it
                tune_check = now
                m = tuning_mtime()
                if m is not None and m != tune_m:
                    tune_m = m
                    for name, old, new in reload_tuning(tune):
                        log(f"tune {name} = {new} (from file, was {old}, in {state})")
                        msg = f"tuning file: {name} {old} -> {new}"
            if net:
                net.trim = {"pitch": int(tune["trim_pitch"]), "roll": int(tune["trim_roll"]),
                            "yaw": int(tune["trim_yaw"])}
            a.target = cli_target if cli_target is not None else float(tune["target_cm"])

            # --- connect
            if state == "OFFLINE" and "c" in keys and not joiner["busy"]:
                joiner.update(busy=True, ip=None, err="")
                threading.Thread(target=do_join, daemon=True).start()
                state, msg = "JOINING", f"joining {DRONE_SSID} ..."
            if state == "JOINING" and not joiner["busy"]:
                if joiner["ip"]:
                    net = Net(addr, joiner["ip"], rec)
                    net.start()
                    log(f"link up as {joiner['ip']}")
                    state, msg = "WAITING", "waiting for the drone's first packets (5-15 s after joining is normal)"
                else:
                    state, msg = "OFFLINE", joiner["err"] + " - C to retry"
            if net and not net.running and state not in ("DONE",):
                msg, state = net.error or "network stopped", "DONE"
            if state == "WAITING" and net.t_first_frame is not None:
                state, msg = "GROUND", "live - G calibrate, hold Enter 1 s = takeoff"

            # --- kill (any time with a link)
            if "space" in keys and net and state in ("WAITING", "GROUND", "LANDING") + AIR:
                net.kill()
                state, msg = "KILLED", "KILLED - 0x04 sent, stick packets stopped. Esc = quit"

            # --- ground
            if state == "GROUND":
                if "g" in keys:
                    net.pulse(CALIBRATE, CAL_S)
                    log("calibrate 0x80")
                if enter:
                    hold_t = hold_t or now
                    gr = ground_range(now) if a.hop else None
                    if not net.live():
                        msg = "TAKEOFF LOCKED - camera feed is not live"
                    elif a.hop and gr is None:
                        msg = "TAKEOFF LOCKED - hop: the tag must be in view on the ground"
                    elif now - hold_t >= HOLD_S:
                        net.pulse(TAKEOFF, PULSE_S)
                        net.flying, fly_t, hold_t, released = True, now, None, False
                        recoveries = 0
                        hold_ref, hold_check = (vo.heading if vo else None), tracker.YawSignCheck()
                        state, msg = "FLYING", "flying - hold Enter 1 s or L = land, Space = KILL"
                        src_count.update(bottom=0, top=0)
                        if a.hop:
                            travel = max(0.0, gr - a.target)
                            hop.update(start=gr, T=min(HOP_MAX_S, travel / tune["hop_speed"]), fwd_s=0.0,
                                       measure_at=None, result="")
                            log(f"HOP plan: tag at {gr:.0f} cm on the ground, target {a.target:.0f} cm -> "
                                f"forward {hop['T']:.2f} s at power {tune['hop_power']:g} "
                                f"(hop_speed {tune['hop_speed']:g} cm/s)")
                        log("TAKEOFF 0x01")  # Enter is still down: the air block waits for its release
                        for k in ("trim_yaw", "trim_pitch", "trim_roll"):   # for flight_review / scorecard
                            log(f"tune {k} = {int(tune[k])} (at takeoff)")
                    else:
                        msg = f"keep holding Enter ... {HOLD_S - (now - hold_t):.1f} s"
                else:
                    hold_t = None
                    if msg.startswith(("keep holding", "TAKEOFF LOCKED")):
                        msg = "live - G calibrate, hold Enter 1 s = takeoff"
                if "esc" in keys:
                    state = "DONE"

            # --- tag sightings (newest frame; the picture block below decodes it)
            if net:
                n, jpg, t_frame = net.latest()
                if jpg and n != shown_n:
                    shown_n = n
                    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                    if img is not None:
                        h, w = img.shape[:2]
                        fpx = load_focal(w)[0] or focal_from_hfov(w, 58)
                        if vo is None:
                            vo, tmap = tracker.YawVO(fpx, w), tracker.TagMap()
                        vo.update(img, t_frame)
                        r = find_tag(img, fpx, use_top=True)
                        tag = (r["dist"], r["bearing"], r["corners"] * (960 / w)) if r else None
                        if r and 10 < r["dist"] < 400 and state == "GROUND":
                            ground_obs = ([o for o in ground_obs if t_frame - o[0] < 3.0]
                                          + [(t_frame, r["dist"], r.get("lat", 0.0))])
                        if r and 10 < r["dist"] < 400:
                            if sm is None or t_seen is None or t_frame - t_seen > 1.0:
                                sm = {k: r[k] for k in ("dist", "bearing", "elev")}
                            else:
                                sm = {k: sm[k] + SMOOTH * (r[k] - sm[k]) for k in ("dist", "bearing", "elev")}
                            t_seen = t_frame
                            seen_ts = [x for x in seen_ts if t_frame - x < STREAK_S] + [t_frame]
                            streak = len(seen_ts)  # sightings in the last STREAK_S, not in a row: in the air only
                            #                        9-19 % of frames find a tag (21:55 flight)
                            if state in AIR:
                                src_count[r["src"]] += 1
                            tmap.see(t_frame, vo.heading, vo.tilt, r["bearing"], r["elev"], r["dist"])
                            if state == "APPROACH":
                                arrive = arrive + 1 if sm["dist"] <= a.target + ARRIVE_TOL_CM else 0
                        else:
                            seen_ts = [x for x in seen_ts if t_frame - x < STREAK_S]
                            streak = len(seen_ts)
                        small = cv2.cvtColor(cv2.resize(img, (960, 540)), cv2.COLOR_BGR2RGB)
                        surf = pygame.image.frombuffer(small.tobytes(), (960, 540), "RGB").copy()
            fresh = t_seen is not None and now - t_seen < FRESH_S
            pred = tmap.predict(now, vo.heading, vo.tilt) if tmap else None

            # --- air
            if state in AIR:
                if not enter:
                    released, hold_t = True, None  # Enter must be let go after takeoff before it can land
                elif released:
                    hold_t = hold_t or now
                    if now - hold_t >= HOLD_S:
                        land("Enter held")
                if state in AIR and not net.flying:
                    sticks = [0.0] * 4
                    state, land_t, msg = "LANDING", now, "landing (0x02): watchdog"
                if state in AIR:
                    if "l" in keys:
                        land("L pressed")
                    elif "esc" in keys:
                        land("Esc")
                    elif not focused:
                        land("window lost focus")
                    elif now - fly_t >= cap:
                        land(f"time cap {cap:.0f} s")
                    elif net.t_frame is None or now - net.t_frame > FEED_LOST_S:
                        land(f"camera feed lost {FEED_LOST_S:.0f} s")
                # hop phases: hover HOP_CLIMB_S, then straight forward for the planned time (heading hold on)
                if state == "FLYING" and a.hop and now - fly_t >= HOP_CLIMB_S:
                    state, phase_t, msg = "HOP", now, f"HOP: forward {hop['T']:.1f} s"
                    log(f"HOP: forward {hop['T']:.2f} s")
                if state == "HOP":
                    if fresh and sm["dist"] <= a.target + ARRIVE_TOL_CM:
                        land(f"hop: tag reads {sm['dist']:.0f} cm - arrived early")
                    elif now - phase_t >= hop["T"]:
                        land(f"hop done ({hop['T']:.1f} s forward)")
                # approach phases
                if state == "FLYING" and a.approach and now - fly_t >= CLIMB_S:
                    state, phase_t, streak, seen_ts, msg = "ACQUIRE", now, 0, [], "looking for the tags ..."
                    log("ACQUIRE: looking for the bottom / top tag")
                if state == "ACQUIRE":
                    if fresh and streak >= LOCK_FRAMES:
                        state, phase_t, arrive, start_range = "APPROACH", now, 0, sm["dist"]
                        msg = f"approaching tag 2 ({'+'.join(axes)}), target {a.target:.0f} cm"
                        log(f"APPROACH: tag at {sm['dist']:.0f} cm, bearing {sm['bearing']:+.1f} deg")
                    elif a.recover and pred and now - phase_t >= ACQUIRE_SEARCH_S and recoveries < MAX_RECOVERIES:
                        recoveries += 1
                        state, recov = "RECOVER", tracker.Recover(now, vo.heading, bool(tune["yaw_check"]), tune["recover_s"])
                        log(f"RECOVER {recoveries} (search after takeoff): tag last seen {pred[2]:.1f} s ago, "
                            f"predicted bearing {pred[0]:+.0f} deg, elev {pred[1]:+.0f} deg")
                    elif now - phase_t >= ACQUIRE_S:
                        land(f"tag not found in {ACQUIRE_S:.0f} s")
                if state == "APPROACH" and now - t_seen > LOST_RECOVER_S \
                        and sm["dist"] < a.target + tune["close_cm"] and sm["elev"] < CLOSE_ELEV_DEG:
                    brake_reason = (f"tag left the picture at the bottom {sm['dist']:.0f} cm away "
                                    f"(elev {sm['elev']:+.0f} deg) - close enough")
                    brake_pow = -clamp(1.4 * fwd_avg, 0.0, tune["brake_max"])   # as hard as it pushed forward
                    state, brake_until, msg = "BRAKE", now + BRAKE_S, "BRAKE, then land"
                    log(f"BRAKE {brake_pow:+.2f}: {brake_reason}")
                if state == "APPROACH" and a.recover and now - t_seen > LOST_RECOVER_S:
                    if recoveries < MAX_RECOVERIES:
                        recoveries += 1
                        state, recov = "RECOVER", tracker.Recover(now, vo.heading, bool(tune["yaw_check"]), tune["recover_s"])
                        brake_until = now + BRAKE_S   # stop the forward glide before searching
                        brake_pow = -clamp(1.4 * fwd_avg, 0.0, tune["brake_max"])
                        log(f"RECOVER {recoveries}: tag lost {now - t_seen:.1f} s, last at {sm['dist']:.0f} cm; "
                            f"predicted bearing {pred[0]:+.0f} deg, elev {pred[1]:+.0f} deg")
                    else:
                        land(f"tag lost again after {MAX_RECOVERIES} recoveries")
                if state == "RECOVER":
                    if streak >= REACQUIRE_FRAMES and fresh:
                        log(f"RECOVERED after {now - recov.t0:.1f} s: tag at {sm['dist']:.0f} cm, "
                            f"bearing {sm['bearing']:+.0f} deg")
                        state, phase_t, arrive = "APPROACH", now, 0
                        start_range = max(start_range or 0, sm["dist"])
                        rec_phase, recov = "", None
                    elif recov.abort:
                        land(recov.abort)
                if state == "APPROACH":
                    if now - t_seen > LOST_S:
                        land(f"tag lost {now - t_seen:.1f} s")
                    elif now - phase_t >= APPROACH_S:
                        land(f"approach longer than {APPROACH_S:.0f} s")
                    elif sm["dist"] < a.target - OVERSHOOT_CM:
                        land(f"overshoot ({sm['dist']:.0f} cm)")
                    elif sm["dist"] > start_range + AWAY_CM and fwd_avg > 0.15:
                        land(f"moving AWAY while pushing forward ({start_range:.0f} -> {sm['dist']:.0f} cm)"
                             f" - pitch sign?")
                    elif sm["dist"] > start_range + 2 * AWAY_CM:
                        land(f"drifting away from the tag ({start_range:.0f} -> {sm['dist']:.0f} cm)")
                    elif arrive >= ARRIVE_FRAMES:
                        brake_reason = f"arrived ({sm['dist']:.0f} cm from the tag)"
                        brake_pow = -clamp(1.4 * fwd_avg, 0.0, tune["brake_max"])
                        state, brake_until, msg = "BRAKE", now + BRAKE_S, "BRAKE, then land"
                        log(f"BRAKE {brake_pow:+.2f}: {brake_reason}")
                if state == "BRAKE" and now >= brake_until:
                    land(brake_reason)
                # sticks: controller while approaching on a fresh sighting, else centred; slew like a key press
                if state == "APPROACH" and fresh:
                    integ = clamp(integ + (sm["dist"] - a.target) * clock.get_time() / 1000,
                                  -MAX_I / KI_RANGE, MAX_I / KI_RANGE)
                elif state not in ("APPROACH", "RECOVER"):
                    integ = 0.0
                ax_now = [x for x in axes if x != "alt" or tune["alt_on"]]
                cmd = (control(sm, ax_now, a.target, integ, tune["approach_gain"], tune["approach_max"])
                       if state == "APPROACH" and fresh else [0.0] * 4)
                fwd_avg = fwd_avg + 0.05 * (cmd[3] - fwd_avg) if state == "APPROACH" else 0.0
                if state == "HOP":
                    cmd = [0.0, 0.0, 0.0, float(tune["hop_power"])]
                # heading hold: whenever the tag is not steering yaw, hold the VO heading (takeoff heading, or the
                # last heading the tag was centred at); the drone turns right ~6.7 deg/s by itself (23:42)
                if state == "APPROACH" and fresh and "yaw" in axes and vo:
                    hold_ref = vo.heading
                tag_yaw = state == "APPROACH" and fresh and "yaw" in axes
                if a.hold and tune["hold_on"] and vo and state in ("FLYING", "ACQUIRE", "APPROACH", "HOP", "BRAKE") \
                        and not tag_yaw and now - fly_t > max(PULSE_S, tune["hold_delay"]):
                    cmd[1] = tracker.heading_hold(hold_ref, vo, tune["hold_gain"], tune["hold_max"])
                    hold_check.update(now, cmd[1], vo)
                    if hold_check.bad and tune["yaw_check"]:
                        land("heading hold turns the wrong way (VO) - yaw sign?")
                        cmd = [0.0] * 4
                if state == "RECOVER":
                    cmd, txt = recov.step(now, pred, vo)
                    if txt.split(" ")[0] != rec_phase.split(" ")[0]:
                        log(f"recover: {txt}")
                    rec_phase, msg = txt, f"RECOVER: {txt}"
                if state in ("BRAKE", "RECOVER") and now < brake_until:
                    cmd[0], cmd[3] = 0.0, brake_pow
                if state in AIR:
                    step = clock.get_time() / 1000 / RAMP_S
                    sticks = [v + clamp(c - v, -step, step) for v, c in zip(sticks, cmd)]
                    net.set_axes(to_bytes(sticks))
                    rec.control(now, state, sm, fresh, to_bytes(sticks), net.button,
                                vo.heading if vo else "", pred, rec_phase if state == "RECOVER" else "")
            if state == "LANDING" and now - land_t >= LAND_S:
                state, msg = "GROUND", "landed - live, hold Enter 1 s = takeoff again, Esc = quit"
                log("landing time over -> ground")
                if hop["measure_at"]:
                    hop["measure_at"], ground_obs = now + 0.5, []   # fresh readings from the landing spot
            if state == "GROUND" and isinstance(hop["measure_at"], float) and now >= hop["measure_at"]:
                end = ground_range(now)
                if end is not None:
                    side = ground_side(now)
                    along = math.sqrt(max(0.0, end * end - side * side))
                    err = math.hypot(along - a.target, side)
                    log(f"LANDING RESULT: {along:.0f} cm in front of the tag, sideways {side:+.0f} cm, "
                        f"target {a.target:.0f} -> error {err:.0f} cm")
                    msg = f"landed {along:.0f} cm out, {side:+.0f} sideways: error {err:.0f} cm"
                if end is not None and hop["start"] is None:
                    hop.update(measure_at=None, result=msg)
                elif end is not None:
                    moved = hop["start"] - end
                    txt = (f"HOP RESULT: landed {end:.0f} cm from the tag (target {a.target:.0f}, "
                           f"error {end - a.target:+.0f} cm), moved {moved:.0f} cm")
                    if hop["fwd_s"] >= 0.5 and moved > 5:
                        old, new = tune["hop_speed"], clamp(moved / hop["fwd_s"], 5.0, 150.0)
                        tune["hop_speed"] = round(new, 1)
                        save_tuning(tune)
                        txt += f" in {hop['fwd_s']:.1f} s -> hop_speed {old:g} -> {tune['hop_speed']:g} cm/s"
                    else:
                        txt += " - hop_speed not changed (too short / no forward move)"
                    log(txt)
                    hop.update(measure_at=None, result=txt)
                    msg = txt
                elif now - hop["measure_at"] > 5.0:
                    log("LANDING RESULT: tag not seen after landing - no measurement")
                    hop.update(measure_at=None, result="no landing measurement (tag not in view)")
            if state == "KILLED" and "esc" in keys:
                state = "DONE"
            if state in ("OFFLINE", "JOINING", "WAITING") and "esc" in keys:
                state = "DONE"

            # --- draw
            screen.fill((9, 9, 11))
            if surf:
                screen.blit(surf, (0, 0))
                if tag:
                    pygame.draw.polygon(screen, (110, 255, 130), [(float(x), float(y)) for x, y in tag[2]], 3)
            age = max(0.0, now - net.t_frame) if net and net.t_frame else None
            live = age is not None and age < LIVE_S
            badge = ("LIVE", (74, 222, 128)) if live else (("FROZEN", (248, 113, 113)) if age else ("NO VIDEO", (113, 113, 122)))
            screen.blit(big.render(badge[0], True, badge[1]), (20, 560))
            # tuning panel (two columns right of the badge); selected row highlighted
            for i2, (name, *_rest) in enumerate(TUNE_SPEC):
                col, row = divmod(i2, 7)
                on = i2 == sel
                val = tune[name]
                shown = (("on" if val else "off") if name in ("hold_on", "yaw_check", "alt_on")
                         else f"{val:+d}" if name.startswith("trim") else f"{val:g}")
                txt = f"{'>' if on else ' '}{name} {shown}"
                screen.blit(font.render(txt, True, (250, 204, 21) if on else (161, 161, 170)),
                            (330 + 215 * col, 548 + 22 * row))
            screen.blit(font.render(TUNE_SPEC[sel][5][:60], True, (250, 204, 21)), (330, 548 + 22 * 7 + 2))
            if age is not None:
                screen.blit(font.render(f"{net.fps():4.1f} fps  age {age:4.2f}", True,
                                        (244, 244, 245)), (20, 620))
            if tag:
                screen.blit(font.render(f"tag {tag[0]:.0f} cm {tag[1]:+.0f} deg", True,
                                        (110, 255, 130)), (20, 650))
            if pred and not fresh and pred[2] < 15:
                # where the map says the tag is: arrow from the picture centre toward it (clipped to the picture)
                px = 480 + 960 / 2 * math.tan(math.radians(clamp(pred[0], -80, 80))) / math.tan(math.radians(29))
                py = 270 - 540 / 2 * math.tan(math.radians(clamp(pred[1], -80, 80))) / math.tan(math.radians(17))
                px, py = clamp(px, 12, 948), clamp(py, 12, 528)
                pygame.draw.line(screen, (251, 146, 60), (480, 270), (px, py), 3)
                pygame.draw.circle(screen, (251, 146, 60), (int(px), int(py)), 10, 3)
                screen.blit(font.render(f"memory {pred[0]:+.0f}/{pred[1]:+.0f} deg",
                                        True, (251, 146, 60)), (20, 680))
            color = {"FLYING": (251, 191, 36), "ACQUIRE": (251, 191, 36), "APPROACH": (251, 146, 60),
                     "RECOVER": (244, 114, 182), "HOP": (251, 146, 60), "BRAKE": (248, 113, 113), "LANDING": (167, 139, 250), "KILLED": (248, 113, 113),
                     "GROUND": (74, 222, 128)}.get(state, (96, 165, 250))
            x = 980
            screen.blit(big.render(state, True, color), (x, 10))
            lines = textwrap.wrap(msg, 24) + [""]
            if net:
                lines += [f"button 0x{net.button:02x}  frames {net.frames.complete}",
                          f"dropped {net.frames.dropped}  acks {net.ack_records}"]
                if vo:
                    lines += [f"VO heading {vo.heading:+.0f} deg{'' if vo.valid else ' (blur)'}"]
            if state in AIR:
                lines += ["", f"cap: {cap - (now - fly_t):4.1f} s left"]
            if net and a.approach:
                ax = net.axes
                lines += ["", f"approach {'+'.join(axes)} -> {a.target:.0f} cm",
                          f"thr {ax[0]} yaw {ax[1]} pitch {ax[3]}", f"sighting {'fresh' if fresh else 'stale'}"
                          f"  rec {recoveries}"]
            if net and a.hop:
                gr2 = ground_range(now)
                lines += ["", f"hop -> {a.target:.0f} cm, {tune['hop_speed']:g} cm/s",
                          f"ground tag {gr2:.0f} cm" if gr2 else "ground tag: not seen"]
                lines += textwrap.wrap(hop["result"], 24)
            lines += ["", "Tab pick  = / - change", "arrows pitch/roll trim", "[ ] yaw trim",
                      "", "C connect  G calibrate", "hold Enter 1 s", " = takeoff / land", "L land  Esc quit"]
            for i, line in enumerate(lines):
                screen.blit(font.render(line, True, (200, 200, 205)), (x, 70 + 24 * i))
            screen.blit(font.render("SPACE = KILL", True, (248, 113, 113)), (x, 680))
            pygame.display.flip()
            clock.tick(30)
    except BaseException as e:
        traceback.print_exc()
        rec.event(time.monotonic(), f"PROGRAM ERROR in {state}: {type(e).__name__}: {e}")
        if net and net.running and state in AIR + ("LANDING",):
            print(f"CRASH in the air - landing 0x02, packets for {LAND_S:.0f} s, then stop", flush=True)
            rec.event(time.monotonic(), f"CRASH LANDING: 0x02 + {LAND_S:.0f} s of packets")
            net.set_axes(CENTRED)
            if state in AIR:
                net.pulse(LAND, PULSE_S)
            net.flying = False
            end = time.monotonic() + LAND_S
            while time.monotonic() < end and net.running:
                net.hb = time.monotonic()
                time.sleep(0.05)
    finally:
        if net:
            net.stop()
        rec.close()
        pygame.quit()
        if joiner["ip"] and not a.no_wifi:
            wifi_connect(HOME_SSID)
    if net:
        print(summary(net, rec))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--test", type=float, metavar="S", help="headless ground test: live feed for S seconds")
    ap.add_argument("--cap", type=float, default=CAP_S, help=f"airborne seconds before auto land (max {MAX_CAP_S:.0f})")
    ap.add_argument("--addr", default=f"{DRONE[0]}:{DRONE[1]}", help="drone address (tests: a fake drone)")
    ap.add_argument("--no-wifi", action="store_true", help="tests: do not switch Wi-Fi")
    ap.add_argument("--approach", action="store_true", help="after takeoff fly to tag 2 and land in front of it")
    ap.add_argument("--hop", action="store_true",
                    help="demo hop: ground distance -> timed straight flight -> land, self-calibrating")
    ap.add_argument("--axes", default="pitch", help="approach channels, any of pitch,yaw,alt")
    ap.add_argument("--no-hold", dest="hold", action="store_false",
                    help="no VO heading hold (by default the camera holds the heading against yaw drift)")
    ap.add_argument("--no-recover", dest="recover", action="store_false",
                    help="approach: land when the tag is lost instead of searching for it")
    ap.add_argument("--target", type=float, default=None,
                    help="approach: land at this range from the tag, cm (default: target_cm in calib/tuning.json)")
    a = ap.parse_args()
    if a.hop and a.approach:
        ap.error("--hop and --approach are separate modes")
    if any(x not in ("pitch", "yaw", "alt") for x in a.axes.split(",")):
        ap.error("--axes takes any of pitch,yaw,alt")
    a.script, a.script_hold_enter = None, (lambda t: False)  # test hooks (set by test harnesses)
    host, port = a.addr.split(":")
    out_dir = os.path.join(OUT_ROOT, time.strftime("%Y%m%d_%H%M%S") + ("_test" if a.test else ""))
    if a.test:
        ground_test(a, (host, int(port)), out_dir)
    else:
        window(a, (host, int(port)), out_dir)

if __name__ == "__main__":
    main()

