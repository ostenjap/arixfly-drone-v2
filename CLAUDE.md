# Hollystone autonomous show drone

## Goal
Make my own Holystone drone fly shows autonomously. Milestone for this week: from one sentence of text, the drone lifts off table A and lands on table B, safely.

## Hardware and tools
- Drone: Holy Stone HS420 (HVIN HS420A), FCC ID 2AJ55HOLYSTONEGW, app com.mjx.holystone 1.0.23
- Remote: HS420 / RC211A, FCC ID 2AJ55HOLYSTONEGT, own 2.4 GHz radio (2451-2481 MHz), not needed
- Control link: Wi-Fi (verified Day 1). Drone is an open AP; UDP 192.168.169.1:8800, `ef 02` packets
- ESP32 dev board = "antenna": joins the drone Wi-Fi, gets stick/button commands from the laptop over
  USB serial, builds and sends the `ef 02` packets. Own watchdog (no serial command for ~200 ms ->
  stop sending) and physical kill button. Laptop stays on home Wi-Fi (internet, LLM, camera).
- Xbox controller on the laptop for manual flight and as dead-man / kill switch
- Android phone (Bluetooth HCI snoop log capture), laptop with Wireshark, Python, OpenCV
- Fixed camera (phone or webcam) and printed ArUco tags for position feedback

## Architecture (software and data flow)
1. Plan (offline): show brief -> LLM choreographer -> show script (JSON, timed moves) -> simulator / bounds check -> approved show
2. Run (live): show runner (or Xbox controller) -> safety supervisor -> controller (pose error to stick commands) -> USB serial -> ESP32 antenna -> Wi-Fi -> drone
3. Sense (feedback): fixed webcam -> ArUco detector (tag on drone, tag on table B; tools/aruco_track.py) -> state estimator -> controller. Optional VLA policy sends small nudges, and they pass through the safety supervisor first.

## 5-day plan
- Day 1: Recon. Identify the control link, capture the app's traffic for takeoff, hover, forward, yaw, land. Done when the link type is known and each basic move is captured.
- Day 2: Decode and replay. Diff bytes per stick, find header, checksum and send rate, replay from the laptop (done), Xbox controller app (tools/joystick_fly.py), then ESP32 antenna with serial commands (arm, takeoff, hover, land, stop). In parallel: print ArUco tags and get the webcam tracker running. Done when takeoff then land works 10 times in a row.
- Day 3: Open-loop hop. Move library with hard time caps, measure drift, script A to B, add a physical kill button. Done when it lands on B in 3 of 5 runs. [SET LANDING TOLERANCE]
- Day 4: Closed loop. ArUco tags on table B and on the drone, fixed camera, OpenCV error, simple P controller. Done when it steers onto B from three start spots.
- Day 5: AI on top. JSON show script, LLM turns one sentence into a script, validate against bounds, safety supervisor (time cap, geofence, land on low battery). Record the demo.

## Current status
2026-11-01: Day 1 done, Day 2 started early. Control link = Wi-Fi (open AP HolyStoneFPV_2BA841),
UDP 192.168.169.1:8800, `ef 02` packets at ~39 Hz, 4 stick bytes (20-23), button byte 25, XOR checksum 27.
Laptop (tools/replay_test.py) spun the motors, props off. Next: find the stop/land code and the
no-packet failsafe, map the axes one at a time, then port to ESP32. Details in notes/findings.md.
2026-11-02: project moved into drone-software/ of the arixfly repo (website lives at the repo root).
Added Xbox controller app (tools/joystick_fly.py, untested with a controller yet) and ArUco tools
(tools/aruco_tags.py -> markers/, tools/aruco_track.py). Failsafe verified: motors stop when packets
stop. joystick_fly.py is now a full-screen flight window (Y connect, hold Start arm, B/Space stop,
survives Wi-Fi drops); controller_test.py and motor_listen.py (mic timing) added.
Next: props-off axis test in the flight window, land code, then tethered takeoff/land.
2026-11-04: Drone camera on the laptop: tools/drone_video.py (sends only `ef 00 04 00`, gets 1280x720
JPEG frames at ~7 fps from UDP port 1234; Mullvad must be off). ArUco distance from the drone camera:
calibrated at 50 cm (hfov 58 deg), test at 100 cm read 103.7 cm (+3.7 %), very stable.
Next: third distance + offset fit, then use tag distance/bearing as feedback for the autopilot.
Live view: tools/hud.py (fighter-jet style HUD: range, bearing, closure, lock), tested live at 8.6 fps.
2026-11-04 evening: first props-on flights (tethered). Verified: byte 25 = 0x01 one-key takeoff, 0x01 again =
land (slow descent, drifts sideways), 0x04 calibrate sends fine, 0x02 does NOT stop motors; kill = stop
sending (~1.2 s). tools/preflight_test.py (mic-timed props-off/tethered runs). Flight window: Y connect,
X calibrate, hold A takeoff/land, B/Space kill, cap -> auto land. Pad buttons standard 0/1/2/3 (confirmed),
input works with the window in the background. Axis bytes 20..23 = roll, pitch, throttle, yaw adopted from
turbodrone + capture pairing, not visually verified. OPEN: sticks had no visible effect in flight at 30 %.
Next: one tethered flight at --limit 0.6 with the stick-byte log, then a throttle-only test.
2026-11-05: free flight with the window -> drone flew sideways, no control, crashed into water (board still
boots). Decompiled the HS420 app (androguard): real layout is bytes 20..23 = throttle, yaw, roll(+trim),
pitch(+trim); byte 25: 0x01 takeoff, 0x02 land, 0x04 e-stop, 0x20 flip, 0x80 gyro calibration. Our earlier
map and command meanings (from turbodrone) were wrong. joystick_fly.py, flight_ui.py, preflight_test.py now use
the decoded layout (simulated in dry mode only). NO FLIGHTS until: stick directions confirmed from an app
capture with announced stick moves, 0x80 and 0x04 checked props off, then tethered over soft ground only.
Rule learned: never take a protocol detail from another app family; check it in this drone's own app/captures.
2026-11-07: post-crash check. Drone Wi-Fi AP and camera survived (sharp 1280x720 video, no fog). Phone app shows
a grey screen (data off, no VPN, Location on) - cause on the phone/app side, not found. Laptop replaces the phone
for video: tools/hud.py live ran 120 s at 8.1 fps once the phone was off the drone Wi-Fi; the join is flaky,
so hud.py now retries it 3x and shows a packet count. Motors, gyro, battery still unchecked after the water.
Next: props-off check of 0x01 / 0x04 e-stop / 0x80 calibration, then confirm stick directions from an app capture.
2026-11-08: Xbox pad unusable (laptop AMD USB 3.1 controllers code 43, no Bluetooth) -> joystick_fly.py --keys
(C connect, G calibrate, hold Enter takeoff/land, WASD + arrows, Space kill, focus lost -> auto land).
Online cross-check: turbodrone's research on a sibling lx-family app matches our bits (0x01/0x02/0x04/0x80) and
throttle sign (up = higher byte). Props-off keyboard run: VERIFIED 0x04 = instant emergency stop. Yaw changes the
motors clearly, pitch/roll slightly, throttle not at all (likely props-off behaviour: altitude hold).
Next: tethered hop over soft ground away from water to check throttle in the air (rules: tethered, cap, kill).
2026-11-08 evening: speed button found in the app (byte 24, 2 levels, we send 0) -> key V / --speed. Tethered (rope)
flights with the keyboard window: "best ever flight"; throttle works in the air. Throttle alone raised to 55 %
(--throttle-limit, PgUp/PgDn) = "perfect setting", now the default (sticks 40 %, throttle 55 %, speed 1).
Note: after the window closes, Windows sometimes stays on the drone Wi-Fi; reconnect home by hand.
Next: speed 2 comparison (tethered), then open-loop hop A -> B with the camera/ArUco feedback (Day 3/4).
2026-11-08 later: Day 3 runner tools/hop.py + shows/hover_test.json, shows/forward_test.json (JSON timed moves,
validator with per-move caps, 30 s cap, Space kill, L land, focus lost -> land; logs landing offsets to
captures/hops/hops.csv; --fit -> calib/moves.json; --to-b CM writes the A->B script). Dry-tested only.
0x02 land still unverified. Next: hover_test tethered x3, forward_test x3, --fit, --to-b, 5 hops onto B.
2026-11-08 night: closed loop from the DRONE camera: tools/autopilot.py (tag 2 upright on B, P control on
range/bearing/sideways, --axes none = observe only, --sim simulator, --replay recorded frames). Sim: 4/4 starts
land 34-35 cm from the tag, within 10 cm sideways. Sideways estimate only trusted < 70 cm (flips at 100 cm on
real frames). Not flown. Next: hover_test (0x02), then autopilot --axes none -> pitch -> pitch,yaw -> +roll, tethered.
2026-11-08 late: 7 tethered flights: 0x02 LAND VERIFIED (slow descent, motors off). autopilot --axes none never locked
in the air (tag seen once, on the ground); cause unknown (video in flight vs detection), logs can't tell. Decided:
one tag (B) only, room lit, next op = hold 100 cm (pitch only, pass +-15 cm for 10 s x3, <60 cm -> land) after one
pitch-pulse run to measure real speed/lag. autopilot.py now records every live flight's frames + fps + land reason
(captures/autopilot/<run>/, summary printed at the end). 22:13 rerun: 0 video frames with control running ->
the drone sends video to the socket that sends `ef 02` (app uses one socket); autopilot now reads video from the
control socket (loopback-tested only). 22:19 run: still 0 video (only `93 04` status replies) -> stick packets must
ACK frames like the app's; autopilot now appends the app's ack records to every stick packet (fake-drone tested:
156 frames / 4 s vs 4 without). Also fixed: fps() divide-by-zero crash, crash/Ctrl+C in the air now lands with 0x02.
Next: ground check that the thumbnail moves in SAFE (don't press Enter), then observe flight.
2026-11-09: tools/video_link_probe.py on the drone: VERIFIED video with sticks running needs the app's frame acks
(no acks: 4 frames then stall; acks: 22-24 frames / 5 s; autopilot's own LiveVideo code works). Nothing comes back
for ~5 s after the Wi-Fi join -> autopilot takeoff now locked until video flows. Footage stays as JPEG frames per
run in captures/autopilot/<run>/ (no MP4), analysed together after flights.
Ground check still 0 packets: stick packets before the video starts seem to block it -> autopilot now waits in
VIDEO (hello only, no sticks) until the first frame, then SAFE (fake-drone tested only).
17:11: that build froze after 2 frames (unacked partial frame stalls the drone's window) -> user: rewrite.
NEW tools/live_flight.py from scratch: live feed first (one net thread, every frame acked incl. partial ones),
frames saved to captures/flights/<time>/, takeoff/land/kill + cap/watchdog; fake-drone tested only.
17:23 VERIFIED on the drone: live_flight.py --test 10 -> 18.8 fps live feed with stick packets running, 190 frames,
1 dropped and recovered. Tag 2 (taped) found in only 58/190 static frames at 138 cm -> tag without tape / larger.
Window works on the ground (user). live_flight.py --approach added: climb, find tag 2, P control (pitch, optional
yaw), land at --target (60 cm) in front of the tag; pitch-sign guard; tuned detector 106/190 on real frames.
Closed-loop sim tests pass; fixed a tag-box drawing crash. Not flown yet.
22:22: feed froze after 2 frames (ids 1, 2 never arrived -> unacked -> drone's sliding window stuck). Fixed: ack
skipped ids + stall breaker (re-ack after 0.4 s); fake drone reproduces the freeze with the old code, new recovers.
22:31/22:32 approach flights: feed live throughout; approach never started (tag 0/120 on the ground: tape glare),
backward motion = own drift with centred sticks (calibrated on the soft bed?). Fixed: Enter held through takeoff
landed it after 1 s. Then: second detector pass (157/190 on real frames, glare still 0 -> new tag), arrow-key trim
(calib/trim.json), crash events logged.
22:58 flights: tag lost after takeoff = drift back ~20 cm/s + climb (tag at the bottom edge) + swing. NEW
tools/tracker.py: visual-odometry heading (0.02 deg median error on real frames), tag map with prediction,
RECOVER search (turn/climb toward the remembered tag, back off, sweep, VO yaw-sign check); approach gets alt axis
+ integral term. Sim: drift case arrives at 65 cm with pitch,yaw,alt; tether jerk recovered. Not flown.
23:42 flight: controls NOT flipped - pitch stayed 128 (forward faded to 0, tag 27 deg off), drone drifts back
13 cm/s and turns right 6.7 deg/s by itself; yaw sign VERIFIED by VO. Fixed: VO heading hold, forward kept with
yaw on, search never steers blind (it spun 6 s). Sim with the measured drifts arrives at 63 cm.
2026-11-10: 00:02 flights all landed on a FALSE "yaw sign?" alarm (drone turns right ~15 deg/s after takeoff) ->
yaw_check off by default. NEW live tuning in the window, saved to calib/tuning.json: arrows pitch/roll trim, [ ]
yaw trim (new), Tab + = / - for hold_on/gain/max/delay, yaw_check, approach_gain/max, target_cm.
01:02 hover session: trims set (yaw -10, pitch +16, roll -4), holds height and heading (+-10 deg) but wanders
left/right - no position sensor. SAFE BASELINE frozen: calib/tuning_safe_20261110.json + reference frames in
captures/reference/20261110_safe_baseline/. Plan in notes/next_steps.md: Phase 0 offline flight_review.py +
tests/ + split live_flight.py; Phase 1 measure axes with pulses; Phase 2 tune; Phase 3 overhead webcam for B.
Phase 0 DONE (offline): tools/flight_review.py (frame-by-frame review, timelines in <flight>/review/), tests/run_all.py
(19 loopback tests, all pass), live_flight split into drone_link.py + flight_control.py. Footage over 21 takeoffs:
own right turn 6.6-18 deg/s untrimmed (trim brings it to -1..+3), yaw only ~15 deg/s per stick unit (hold max 0.4
adds ~6 deg/s -> try hold_max 0.6-0.8), in-air picture ~20x softer, tag found in 1-21 % of air frames -> bigger
matte tag first. Next session: new tag, hold_max test, then Phase 1 pulses.
21:55 approach flights (start 240 cm): all landed ~5 s after takeoff - climb pushes the bottom tag out of view,
black curtain + plain wall blind the VO -> search gave up. Fixed (sim/replay only): approach also uses the TOP tag
(37 cm above, 10 cm nearer the drone), lock = 3 sightings within 1 s, blind search hovers instead of landing.
Tools: tools/measure_tags.py (tag detection %, distance, gap, tilt from a --test run). Not flown yet.
Update this section at the end of every session.

## Rules
- Only analyse my own drone and my own captures. Never break encryption or bypass access protection. If pairing or encryption blocks us, stop and tell me.
- Do not assume the protocol. Ask me for captures and verify bytes against them.
- Safety first: any new flight command is tested with props off or the drone tethered. Every flight script needs a hard time cap and a kill path. Never fly near people.
- Change one thing per test and log the result in notes/findings.md.

## Conventions
- Python for tooling (tools/), ESP32 firmware in firmware/, captures in captures/ named by move (for example takeoff.btsnoop.log), findings in notes/findings.md.
- Flight software: tools/live_flight.py (window + state machine) uses tools/drone_link.py (protocol, video acks,
  network thread, recording), tools/flight_control.py (control law, tag detection, tuning store), tools/tracker.py
  (visual odometry, tag memory, recovery). After any change run `python tests/run_all.py` (loopback fakes and
  simulators; never touches the drone, Wi-Fi or calib/). Review a flight: `python tools/flight_review.py <folder>`.
- Keep scripts small and runnable from the command line with a short usage note at the top.
