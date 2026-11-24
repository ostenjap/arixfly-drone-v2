# Findings

One test per entry. Format: date, what changed, result, next step.

## 2026-11-08 21:28-21:34 First tethered runs of hop.py and autopilot.py (user flew, logs in captures/)
- hop.py shows/hover_test.json x2: 0x01 at 0 s, hover, 0x02 at 6.0 s. autopilot.py --axes none x5: 0x02 at
  3.3-5.0 s. User: in all 7 the drone came down slowly and the motors stopped. => 0x02 = LAND VERIFIED in the air.
- autopilot observe-only: NO LOCK in any of the 5 runs. Two landed on "tag not found in 5 s", three at ~3.5 s
  after ACQUIRE (reason not logged). Only sighting: run 5, tag 2 at 106 cm / +13 deg / elev +11 deg, seen 9 s
  BEFORE takeoff (on the ground), nothing in the air. User: drone hovered just above tag height, ~120 cm away
  -> inside the camera's +-17 deg vertical view. User did not watch the live thumbnail.
  Open: does video stop once control + motors run, or does the detector fail in flight (blur, dark room)?
  The log can't tell: no frame count, no saved frames, no landing reason.
- Footage review: test_100cm 142/142 frames with tag (side ~111 px, sharp); calib_50cm 0/142, calib_50cm_b
  98/138. Room dark with one lamp blowing out the right of the frame.
- Decisions (grilling session): one tag (B) only for navigation; room lights on, lamp lighting the tag, never
  into the lens; next camera op = HOLD 100 cm from the tag (pitch only), pass = range 85-115 cm for 10 s,
  3 tethered runs in a row; under 60 cm -> land 0x02; before the hold loop, one pitch-pulse run (lock, then
  back/forward/back 0.5 s pulses) to measure real speed and video lag from the footage, gains set from that.
- Next: autopilot logs fps + landing reason and saves every flight frame; observe-only rerun with the new light.
- 22:04 BUILT (sim/replay tested only): every live autopilot run now saves each decoded frame (raw JPEG) to
  captures/autopilot/<run>/ + frames.csv (state, tag seen, range, sharpness per frame); the flight CSV gained
  video_fps, video_frames, video_pkts, eye_frames, reason; at the end it prints frames / fps / longest gap /
  tag sightings for ground, airborne and landing. A recorded run replays with --replay captures/autopilot/<run>.
  Sim check: 105 frames saved, airborne 5.7 fps, tag in 50/50; replay of that folder locks and approaches.
- 22:13 live observe rerun (Terminal 2): link joined, SAFE at 15 s, 655 control packets sent, 0 VIDEO FRAMES in
  17 s on the ground (user quit before takeoff; captures/autopilot/20261108_221358_live_none/ is empty).
  Not the cause: Mullvad disconnected; nothing else on UDP 1234/8800; a leftover autopilot.py from 21:34 was stuck
  at the runs.csv prompt in Terminal 1 (sockets already closed) - stopped it.
  Cause (strong, not yet flight-verified): two sockets. hud.py (video socket only, no `ef 02`) gets 8 fps; autopilot
  sends `ef 02` from Link's socket and asked for video from a second socket. The app uses ONE socket: idle2.pcap and
  sticks.pcap = only `ef 02` from a single source port, with frame acks inside the `ef 02` records. So the drone
  streams video to the port that sends `ef 02` -> it went to the control socket, which never read it.
  Fix: autopilot LiveVideo now reads video from link.sock and sends `ef 00 04 00` from it (10 Hz); no timeout on
  the shared socket (select instead). Loopback test with a fake drone replaying the 19:11 recording to the `ef 02`
  sender: 32 frames / 3 s, video follows a socket swap on reconnect. Next: observe rerun on the ground, then fly.
- 22:19 live observe run (one-socket build): joined (2nd try), calibrate, takeoff 0x01 1.0 s, ACQUIRE, "tag not
  found in 5 s" -> 0x02 at 8.0 s, user Space at 12.1 s (4 s into the descent) -> 0x04 1 s, stop. Loop 39 Hz (max
  gap 32 ms), sticks 128 throughout (observe only). VIDEO: video_pkts rose ~10/s = exactly the `93 04` status reply
  per `ef 00 04 00` (recordings: 9.8/s); ZERO `93 01` fragments -> drone hears the request but does not stream.
  => One socket alone is not enough. Most likely: once `ef 02` packets arrive the drone streams only if they
  ACK its frames (app stick packets carry ack records: sticks.pcap 945 pkts with 1 record, 153 with 2, 828 none;
  11-04: without acks it sends a 4-frame window then waits). The single ground sighting in the 21:34 run fits.
  User note: did not expect observe mode to take off by itself (it does: Enter 1 s = takeoff, it only doesn't steer).
- 22:30 BUILT (loopback-tested, not on the drone): autopilot appends the app's ack record for every finished frame
  to its stick packets (dv.add_acks, same records as dv.ack_packet; each frame in up to 3 packets, max 3 per
  packet); packet = byte-identical to an app sticks.pcap packet except seq and byte 26/27. `ef 00 04 00` only until
  frames flow, again after 1 s without a frame. New log column video_frags (`93 01` count).
  Fake drone (4-frame window, advances only on acks): no acks 4 frames / 4 s, with acks 156 frames / 4 s, hello
  sent once. Full autopilot window LIVE path vs the fake drone (only the Wi-Fi join faked): video on the ground
  during SAFE/READY, 110 frames saved, 534 ack records received by the fake.
  Bugs found by that test and fixed: hud.Source.fps() divided by zero when frames share a clock tick (Windows
  monotonic = 15.6 ms) -> would have crashed autopilot mid-flight; any crash / Ctrl+C while airborne now sends
  0x02 + centred sticks for 8 s before stopping instead of going silent (tested with an injected crash in the sim).
  Regression: sim pitch,yaw,roll arrives 35 cm / +8 cm; replay test_100cm locks; aruco selftest 4.3 %; hop --check OK.
  Next: ground check (props off or just don't press Enter): thumbnail must move in SAFE; then observe flight.
- 22:49 / 22:50 / 22:53 autopilot ground checks (ack build): 0 packets of ANY kind (not even `93 04`, which the
  22:19 build got at 10/s). 22:53:34, 3 s after the last one: drone_video.py --live -> 25 frames (user: "video OK").
  => drone + laptop Wi-Fi fine; the autopilot path is the problem. Stick packets with no frames to ack are
  byte-identical to the 22:19 build; the video thread differs only in the hello gate. Cause NOT found by reading.
  Built tools/video_link_probe.py (ground, button 0, centred sticks only): phases A hello only, B + ef02 one
  socket, C + acks, D two sockets, E autopilot.LiveVideo itself; prints packets by type per socket. Loopback-run
  OK (mechanics). Not run on the drone yet.

## 2026-11-09 video_link_probe.py on the drone (user ran it, drone on the table, button 0 / centred sticks only)
| phase | what | `93 01` video | `93 04` status | frames / 5 s |
|---|---|---|---|---|
| A | hello only, first 5 s after the Wi-Fi join | 0 | 0 | 0 |
| B | hello + `ef 02`, one socket, no acks | 258 | 50 | 4, then stall |
| C | hello + `ef 02` + frame acks, one socket | 1506 | 51 | 22 |
| D | hello on socket V, `ef 02` on socket C, no acks | 258 on V, 0 on C | 51 on V | 4, then stall |
| E | autopilot.LiveVideo + take_acks (the real autopilot code) | 1618 | 53 | 24 |
=> VERIFIED: with `ef 02` packets running, the drone sends a 4-frame window and then waits for ack records in
   them; with the app's acks it streams (~4.5-5 fps here). Video goes to the socket that sends the HELLO (D), not
   to the `ef 02` socket - the one-socket theory of 22:16 was wrong; one socket is still needed for the acks to match.
=> NEW: for the first ~5 s after Windows joins the drone Wi-Fi NOTHING comes back (A, also the 22:53 video-only run:
   25 frames in a 10 s run). The 22:49/22:50 ground checks (quit 8-10 s after SAFE, 0 packets) most likely died
   in that dead time; the 22:19 flight even took off with no video.
- Change: autopilot takeoff is LOCKED until video flows (5 frames, newest < 1 s old); window says "waiting for
  video". Bug found in the first version (age 0.0 counted as no frame -> flicker), fixed. Fake drone with 5 s of
  silence: locked 3.0-5.1 s, then "video OK" once, 130 frames; sim regression arrives 35 cm / +8 cm.
- Ground check right after (user screenshot): SAFE, seq 430 (~11 s of stick packets), status "VIDEO STARTING" =
  not one packet back. Pattern over all runs: every autopilot run sent `ef 02` from the first moment -> 0 video,
  ever; drone_video / probe A send hello ONLY first -> video starts after a few s; probe B-E worked because A had
  already started the video. Hypothesis: the drone must start its video from hello-only; `ef 02` before that
  stops it from ever starting.
  Change: new state VIDEO after C: hello only, NO stick packets, until the first frame -> then SAFE with sticks +
  acks. Fake drone with that rule: video at 4.0 s, SAFE at 4.03 s, 249 frames in 10 s, 1154 ack records.
  First version waited for 5 frames in VIDEO -> deadlock on a fake that wants acks even in hello mode; fixed.
- 17:11 user run of that build: 2 frames on hello-only, then the moment sticks + acks started the feed froze;
  takeoff "not working" = the video gate (no fresh frames). User: rewrite from scratch, live feed first.
  Likely freeze cause: in app mode (sticks running) the drone waits for an ack of EVERY frame (4-frame window,
  no timeout); the old code acked only frames it got complete, so one lost fragment froze the stream forever.
  (The app sends partial records with missing-fragment bits; not copied yet.)
- NEW tools/live_flight.py (from scratch): one network thread + one socket bound to the drone-net IP; hello 10 Hz
  always; sticks 39 Hz (centred) with acks start at the drone's FIRST packet (any type, = inbound works), so video
  starts in app mode like probe C instead of switching mid-stream; every frame id acked, partial frames acked as
  complete after 0.2 s without a new fragment (drone moves on, we drop the frame). Recording thread saves every
  frame to captures/flights/<time>/ + frames.csv + events.csv. Window: big LIVE / FROZEN / NO VIDEO badge with fps
  and frame age, tag 2 box + range. Keys C, G, hold Enter 1 s takeoff / land (Enter must be released after
  takeoff), L, Space kill, Esc. Safety: takeoff only with a live feed, cap 30 s, focus lost, feed lost 2 s,
  window-silent watchdog 1 s (network thread lands by itself), crash -> land + 8 s. --test S = headless ground test.
  Loopback tests vs a strict fake drone (2 s dead time, 93 04 replies, app-mode window without timeout, ~12 fps):
  0 % loss 10.7 fps live, every frame acked; 3 % fragment loss (harsh): no freeze, stream keeps moving, 1 in 5
  frames complete. Scripted window runs: calibrate 0x80, takeoff 0x01 after 1 s hold, land 0x02 on 2nd hold,
  ground after 8 s, takeoff again, L land, Space 0x04 then sticks stop; cap 3 s; feed lost 2 s; crash in the air;
  no live feed -> takeoff refused; watchdog -> 0x02. NOT run on the real drone yet. autopilot.py left as is.
- 17:23 FIRST REAL RUN: live_flight.py --test 10 on the drone (table, centred sticks, button 0). Join on try 2,
  first packet 18.5 s after start (incl. join), first frame 0.17 s later. LIVE FEED VERIFIED with stick packets
  running: 190 frames in 10 s = 18.8 fps steady (app: 12 fps, hello-only: 7 fps), longest gap 0.12 s, 1 frame
  dropped + acked and the stream went on, 572 ack records, 404 stick packets. All 190 JPEGs decode, 1280x720,
  sharp (captures/flights/20261109_172319_test/). => the ack-every-frame design works on the real drone.
- Same frames, tag 2 upright on a box on the bed at 138 cm (sd 1.7 cm), bearing +1.9, elev +5.2: detected in only
  58/190 frames although nothing moved; misses scattered (marginal, not blocked). Clear tape over the tag shows
  glare streaks across the black cells. Detector tuning on these frames: adaptiveThreshWinSizeMax 73 +
  adaptiveThreshConstant 3 -> 106/190, 0 false hits on 56 tag-free frames; blur/CLAHE worse. NOT applied yet.
  Physical fix first: tag without tape (matte), larger print (15-20 cm) for 1-1.5 m range.
- User: window works on the ground. -> approach added to live_flight.py (--approach, --axes pitch|pitch,yaw,
  --target, default 60 cm): CLIMB 3 s -> ACQUIRE (3 sightings in 6 s) -> APPROACH (P control from autopilot.py:
  pitch 0.006/cm, max 0.5 fwd / 0.3 back, fade with bearing; yaw 0.02/deg) -> land 0x02 when arrived (range <=
  target + 8 cm, 3 sightings). Lands on tag lost 1.5 s, not found 6 s, approach > 15 s, overshoot, and range
  GROWING 30 cm (pitch-sign guard). Only sightings < 0.5 s old steer; sticks slew 0.25 s; watchdog/kill/crash
  centre the sticks. Tuned detector in live_flight (win 73, const 3, subpix): real taped frames 106/190 (was 58),
  range sd 0.58 cm, 0 false hits. control.csv per flight (range, bearing, stick bytes, button at ~30 Hz).
  BUG found + fixed: drawing the tag box crashed (numpy floats in pygame.draw.polygon) - first test with a
  visible tag; the ground-test window could have crashed whenever the tag was detected.
  Closed-loop loopback tests (sim drone renders tag 2 from its pose, real `93 01` fragments, strict acks, flies
  from the stick bytes): pitch from 150 cm -> arrived, landed 65 cm from the tag; pitch+yaw from 180 cm / 8 deg
  -> 60 cm, 21 cm sideways (no roll, drift 2 cm/s); pitch sign inverted -> "moving AWAY" land after 1.3 s
  (ended 203 cm); tag out of view -> "tag not found in 6 s". NOT flown on the drone yet.
- 17:27 / 17:29 window flights (pre-approach build): 450 / 398 frames, live throughout (longest gap 0.33 s), but
  events.csv ends at TAKEOFF with no LAND -> most likely the tag-box drawing crash (fixed since) ended both, landed
  by the crash path (which logs no event).
- 22:22 --approach run: picture FROZE after 2 frames on the ground ("packets but no live feed"), no takeoff.
  frames.csv: the first frames received were ids 3 and 4 (good runs start at 1) -> ids 1, 2 never arrived, never
  acked. The drone's window slides from the OLDEST unacked id: with 1 unacked it never sends beyond 4, so acking
  3, 4 does not help -> frozen forever. Same mechanism for any whole frame lost mid-stream.
  Fix: (1) frame id N arrives -> also ack unseen ids below it (up to 8 back); (2) no fragment for 0.4 s -> re-ack
  the last 4 ids and ack the next 4 (once per s), logged as "video stall". Summary shows "never seen+acked" + stalls.
  Fake drone with a sliding window (reproduces 22:22 exactly with the old code: 2 frames, stuck on [1, 2]):
  ids 1,2 lost: old 2 frames / new 102; frame 50 lost: old froze at 52 / new 101; 40-43 lost: old froze at 39 /
  new recovered after one 0.48 s stall, 97 frames; nothing lost: equal. Approach sim regression unchanged (65 cm).
- 22:31 / 22:32 tethered --approach (pitch), drone on the bed 130 cm from tag 2. Feed LIVE whole flight (620 / 513
  frames, no freeze). User: "the drone kept moving back".
  22:31: takeoff 31.0 s, ACQUIRE 34.0 s, "tag not found in 6 s" -> land, then Space. control.csv: sticks 128 on
  ALL axes the whole flight -> the approach never started, the backward motion is the drone's own drift, not the
  controller. Frames: in the air it turned right (wall fills the frame) and swung/blurred (tether?); tag seen only
  2x at 161 and 165 cm (moving away). On the GROUND at 130 cm the tag was found in 0 of ~120 frames: strong glare
  from the tape washes out the inner cells (17:23 had less glare).
  Likely drift causes: gyro calibration (G) done on the soft, uneven bed = tilted level reference; tether pull.
  22:32: LAND "Enter held" 1.05 s after takeoff = BUG: takeoff cleared the Enter state, so a held Enter counted as
  "released" and then as a land hold. Fixed (Enter must really be let go); tested: held 2.5 s through takeoff ->
  no land; release + hold 1.2 s -> lands.
  Next: tag without tape (matte, larger), calibrate + take off from a firm flat board, check drift with sticks
  centred (hover 5 s) before the approach.
- Fixes after the 22:31/22:32 flights (loopback-tested):
  * Tag glare: every detector/preprocessing setting tried gets 0/115 on the 22:31 glare frames (gray/red/min/
    gamma/CLAHE x thresholds/error correction) -> only a matte, untaped tag fixes that. Found a second pass that
    helps normal frames: if pass 1 finds nothing, darken (gamma 2) + adaptiveThreshConstant 1 -> 17:23 frames
    157/190 (pass 1 alone 106, defaults 58), 0 false hits on 56 tag-free frames.
  * Trim keys (arrows, ground or air, 2 counts per press, max +-40): added to bytes 22 (roll) / 23 (pitch) of
    every stick packet, saved in calib/trim.json, logged as events. Drone drifts back -> Up. Test: Up, Up, Right ->
    the drone got bytes 128,128,130,132, checksum correct.
  * Crash logging: a program error now writes "PROGRAM ERROR in <state>" and "CRASH LANDING" to events.csv.
  Regression: approach sim 65 cm, wrong-sign guard lands.
- 22:58 session (captures/flights/20261109_225836, 5 tethered --approach flights, pitch only): 2x tag locked
  (205 cm / -20.5 deg, 233 cm / -10.3 deg) and lost within 2 s; 2x "tag not found"; 1x L. On the ground the tag
  was seen at 138 cm, bearing ~0, elev ~0 (new tag works on the ground). After the 3 s climb: range +70..95 cm
  (drift back ~20 cm/s), bearing -10..-20 deg (turned right), elev -15.5 deg = bottom edge of the +-17 deg view.
  Frame 1394: tag leaving at the bottom-left; next frames heavily blurred (swing/spin, tether?). Controller sent
  almost nothing (pitch 129-140, yaw off) -> losses = drift + climb + swing, not the controller.
- Lost-tag memory + recovery (NEW tools/tracker.py, used by live_flight --approach), proven methods:
  * Visual odometry yaw: Shi-Tomasi + pyramidal LK flow at 320x180, median shift = turn. On the 22:58 frames:
    VO yaw vs the tag's own bearing change over 42 in-flight pairs: median error 0.02 deg, max 0.13; valid on
    78 % of in-flight pairs (rest: blur). Full-run prediction check: in-air re-sightings after gaps up to 6.5 s
    predicted within 0-3 deg while VO tracked; 25-52 deg off after long blur / hand handling.
  * Tag map: sightings stored in a world frame (VO heading + camera bearing, elevation + VO tilt) with an
    alpha-beta rate filter (SORT-style prediction while occluded); the window draws an arrow to the predicted tag.
  * RECOVER state (follow-me style last-known-position search): settle while blurred -> turn + climb/descend
    toward the prediction -> back off -> +-30 deg sweep -> land after 9 s; max 3 per flight. Also starts from
    ACQUIRE when the tag was seen on the ground but not 1.5 s after takeoff. VO yaw-sign check aborts if a turn
    command moves the heading the other way.
  * Approach: new --axes alt (throttle from elevation), integral term on range (KI 0.002, max 0.3) - P alone
    stalled at ~100 cm against 20 cm/s drift in the sim. Pitch-sign guard now needs forward push (avg > 0.15);
    plain drift of 60 cm lands as "drifting away" (was a false "pitch sign?").
  Sim (sim drone with height, textured walls, 20 cm/s back drift, 4 deg/s yaw drift, real-like: tag first seen
  207 cm / -11.6 deg): pitch,yaw,alt -> arrived 65 cm; pitch only -> "drifting away" (can't hold vs drift);
  tether jerk +45 deg/+25 cm -> search after takeoff found the tag in 0.7 s (descend), jerk too fast for VO
  (prediction 0 instead of -45) -> sweep found it after 7.1 s -> arrived 59 cm; yaw reversed -> "yaw sign?" land.
  Regression: basic approach 63 cm, wrong pitch sign guard, no tag, all window controls/kill/cap/crash/watchdog OK.
  NOT flown yet. Recovery always uses yaw + throttle (even with --axes pitch); yaw sign only as flown by keyboard.
- 23:42 tethered --approach --axes pitch,yaw,alt (captures/flights/20261109_234246). User: "kept moving back -
  flipped control?". NO: control.csv shows pitch byte 128 (centre) during the whole approach - never backward,
  never forward: the forward fade (0 at 20 deg bearing) zeroed it because the tag sat at -25..-27 deg. Range grew
  249 -> 283 cm in 2.6 s = 13 cm/s backward drift with centred pitch; no trim set (calib/trim.json absent).
  VO heading with centred yaw: 0 -> +30.5 deg in 4.5 s = drone turns RIGHT 6.7 deg/s by itself -> tag ends up
  left. VERIFIED by the camera for the first time: yaw sign (search yaw left, byte 110 -> heading 27.8 -> 21.0 deg)
  and yaw authority ~18 deg/s at 0.35 power (~50 deg/s per unit). Search: found the tag once (descend + turn,
  1.0 s), lost it again, then the picture blurred, VO froze at 21 deg and the search kept yawing left + descending
  for 6 s while blind -> frame 329: drone spinning (horizontal smear) -> gave up after 9 s.
  Fixes: (1) VO heading hold (tracker.heading_hold, KP 0.03/deg) whenever the tag is not steering yaw, incl. the
  climb and plain flights, with a VO yaw-sign check (--no-hold to switch off); (2) with yaw on, forward fades only
  to 30 % at 45 deg (pitch-only keeps 0 at 20 deg); (3) search never steers while VO is blind > 0.5 s, gives up
  after 2.5 s blind, climb/descend at most 2 s per search.
  Sim with the measured drifts (13 cm/s back, 6.7 deg/s right): hold -> tag at -3 deg after the climb, arrived
  63 cm; no hold -> tag at -29 deg, found by turning, arrived 64 cm. Full regression (controls, kill, cap, crash,
  watchdog, approach, pitch-sign guard, no tag, jerk, reversed yaw) passes.
  Still to do on the drone: pitch trim against the 13 cm/s backward drift (Up arrow).
- 2026-11-10 00:02 plain flights (captures/flights/20261110_000232): all 4 landed ~2 s after takeoff with
  "heading hold turns the wrong way (VO) - yaw sign?" = FALSE ALARM: ~1 s after takeoff the drone turns right
  ~15 deg/s by itself (heading 66 -> 80 deg in < 1 s), faster than the ramping hold correction; the check only
  looked at the heading. A rate-based rewrite fixed these 4 logs but in the sim still false-alarmed at 15 deg/s
  (rate swings back as the hold settles) AND missed a reversed yaw. -> yaw_check now OFF by default (sign verified
  by VO on 23:42); spins stay bounded by the search limits (9 s, blind pause 0.5 s, blind give-up 2.5 s).
- Live tuning (live_flight.py, calib/tuning.json, replaces calib/trim.json, takes its values over): trim_yaw
  (NEW, bytes added to b21), trim_pitch, trim_roll, hold_on, hold_gain, hold_max, hold_delay, yaw_check,
  approach_gain, approach_max, target_cm. Keys: arrows pitch/roll trim, [ ] yaw trim, Tab/Shift+Tab pick, = / -
  change; saved at once, logged as "tune <name> = <value> (in <state>)" in events.csv, shown under the picture with
  a help line. --target on the command line overrides target_cm. Test: [ x3, Up, Tab x4, = x2 -> file trim_yaw -6,
  trim_pitch 2, hold_gain 0.035; drone got yaw 122 / roll 128 / pitch 130 on the ground, checksum ok.
  Sim with 15 deg/s yaw drift + 13 cm/s back drift, check off: arrived 65 cm. Full regression passes.
  Starting estimates for trims (sim/VO numbers, not measured as trims): yaw ~50 deg/s per power unit, 1 unit =
  51 bytes -> 15 deg/s needs trim_yaw about -15; back drift 13 cm/s -> trim_pitch about +6 (pitch authority unknown).
- 2026-11-10 Phase 0 (offline, while the user slept; no drone, no Wi-Fi, calib/ untouched):
  * NEW tools/flight_review.py <folder>: per takeoff, from the saved frames + logs: tag pose with the IPPE
    sideways ambiguity resolved over time (Viterbi), gravity-aligned side/height (independent of how the tag is
    turned on its box; checked on sim-rendered frames: side within 4 cm, height within 0.2 cm; a first version
    measured along the tag's printed axes and swapped side/height, and had the side sign flipped - both fixed),
    VO heading, sharpness, drift with centred sticks, yaw response, every tag loss with its cause; writes
    review/flight_N.png timelines, frames_review.csv, summary.txt. NOTE: live autopilot.py "lat" (cam[0] along
    the tag's printed axis) has the same rotation dependence.
  * Results over 21 takeoffs (22:58, 23:42, 00:02, 01:02 sessions):
    - own turn untrimmed: +6.6..+18 deg/s right (22:58: 10.1, 16.0, 15.3, 6.6, 18.2; 23:42: 10.6); with trim
      yaw -10..-14: -1..+3 deg/s. Not constant (one untrimmed 01:02 flight: -1 deg/s) -> calibration/battery.
    - yaw response: ~12-18 deg/s per power unit in most flights (23:42: 42, noisy, few samples) -> heading hold
      at hold_max 0.4 corrects only ~6 deg/s, less than the own turn: trim must do most of it. Sim with this
      strength: untrimmed (15 deg/s) spins 183 deg away, search fails; trimmed (3 deg/s) arrives 66 cm.
    - sharpness: ground median ~1900, in the air 40-100 (vibration/motion); <25 = VO/tag fail.
    - tag seen in only 1-21 % of in-air frames, mostly "in view but not detected" (ground at the same range
      works) -> bigger matte tag (15-20 cm) is the main fix for flight.
    - drone hovers 30-60 cm off the tag's normal, either side (the left/right wander).
  * tests/: fake_drone, strict_drone, sim_drone, sim_drone2 + run_all.py (19 tests, PASS/FAIL, temp folder,
    never the real calib/). All 19 pass, incl. new recover_realyaw_trimmed (measured yaw strength).
  * live_flight.py split by moving top-level blocks verbatim: drone_link.py (protocol, acks, Net, Recorder,
    Wi-Fi), flight_control.py (control, find_tag, tuning store, constants), live_flight.py (window, state
    machine, ground test, CLI). Every old code line found in the new files; all tests pass after the split.
- 2026-11-10 `93 04` status packet (online reference + our captures): turbodrone docs/research/wifi_uav.md (sibling
  "lx" app family): native receiver `93 01` = JPEG fragment, `93 04` = "MCU/control data"; the app parses a serial
  status frame `88 00 <fly-state> <battery> <flags> ... <xor> 55` (flags: speed, RC support, PTZ, headless/photo/
  video/light). Our `93 04` (both recordings, 153 packets, all identical, ground, motors off):
  `93 04 14 00 a5 a5 a5 a5 08 00 a5 a5 00 | 88 00 | 00 | 5a | 00 | 5a | 55` -> fly-state 0, battery 0x5a = 90,
  flags 0, XOR(00 00 5a 00) = 5a matches, terminator 55. Our own OUTGOING control block (ef 02 bytes 18-28) has
  the same frame shape (88 00 <sticks, speed, buttons, flags> <xor> 55) -> 93 04 = the flight controller's serial
  frame coming back. NO accelerometer/attitude data in it. Meanings of fly-state values and battery scale
  (percent? level?) NOT verified on our drone yet -> log 93 04 during flights: fly-state should change at
  takeoff/land, battery should fall over a session.
  Same reference: ack status values 0 receiving, 1 complete, 2 dropped, 3 future; we send 1 for given-up/never-seen
  frames (works, 18.8 fps), the native app sends 2 for dropped - possible refinement, not needed now.
- User decision: no MP4/video files; keep the live footage as frames (already: captures/autopilot/<run>/frame_NNNN.jpg
  + frames.csv per live run) and analyse them together after flights.

## 2026-11-07 Post-crash health check (user: phone app no longer seems to work with the drone)
- wifi_scan.py: AP HolyStoneFPV_2BA841 up, same BSSID b4:c2:e0:2b:a8:41, ch 4, 100 % -> drone board + Wi-Fi radio alive.
- video_probe.py 8 s (only `ef 00 04 00`, no stick packets): joined as 192.168.169.2, 0 packets back.
  INCONCLUSIVE: Mullvad was connected (known to drop 192.168.169.x traffic, see 2026-11-04 05:38). Repeat with Mullvad off.
- 19:11 same probe, Mullvad disconnected by the user: VIDEO OK. 2867 pkts / 3.0 MB in 7.8 s from drone UDP 1234,
  same `93 01` / `93 04` format, 56 frames 1280x720 at 7.2 fps (2 incomplete dropped), sharp colour picture of the
  room, no fog/water marks (captures/video_probe_20261107_191124.bin, captures/frames/postcrash_20261107/).
  => Drone Wi-Fi + camera + video protocol survived the crash. Phone problem is most likely on the phone side
  (not joined to the drone AP, Android dropping a no-internet network, or a VPN app). Motors/IMU NOT checked yet.
- User after that: phone joined to HolyStoneFPV_2BA841, mobile data off, no VPN/PCAPdroid, Location on ->
  app control screen stays GREY/BLACK (no picture). Laptop got video from the same drone minutes before.
  Open: phone IP on the drone net, app version (auto-update since 1.0.23?), result after a drone power cycle.
- Laptop live HUD (tools/hud.py): run 1 "NO DRONE WI-FI" (no 192.168.169.x IP within 20 s); run 2 LINK but 0 frames
  (not explained); headless run still joining after 25 s. Standalone receive test of the same code: IP in 1.1 s,
  69 frames / 10 s. Changed hud.py: Wi-Fi join retried up to 3x, packet counter in status + exit summary.
  Run 3 (phone off the drone Wi-Fi): LIVE VIDEO OK for 120 s, 31976 pkts, 774 frames, 8.1 fps, 13 dropped,
  calibration loaded (F1152, hfov 58) (captures/hud/laptop_view_20261107.png).
  => Laptop replaces the phone for video. Join is flaky (cause unproven: Windows preferring home Wi-Fi,
  a second client on the drone AP, or water damage to the radio).
- 21:49 PROPS-OFF hand-held test, joystick_fly.py live --cap 45 --limit 0.4 (user confirmed props off, pad on).
  Log: SAFE 3.4 s, X calibrate 0x80 twice (6.6 s, 11.2 s), takeoff 0x01 -> FLYING 17.4 s, sticks moved one axis at
  a time (bytes ranged 77..179 on each of throttle/yaw/roll/pitch), B NOT pressed -> 0x04 e-stop still untested,
  time cap -> auto land 0x02 at 62.5 s, packets stopped 70.5 s, 2617 packets sent.
  Throttle mostly at 77 (code: stick DOWN). User: "the throttle and calibration is clearly not working".
  Code check: packets match the app-decompile layout byte for byte (throttle b20, 0x80 calibration in b25, XOR
  b20..26 = app's block[1..8] since b19 = 0). Open: is this the protocol, or props-off behaviour (altitude hold
  may ignore throttle on the ground; 0x80 may have no visible sign)? Motor spin + per-stick motor response not reported.
- Re-read of the decompile (scratchpad src from the 2026-11-05 session, still on disk): mode 2 (eCtrlModeKey default 2)
  -> throttle = left slider Q.t(), yaw = Q.p(), roll = R.p() (+ tilt u0.x), pitch = R.t() (+ u0.y). Same order as our code.
  Byte 25 bit 0x80 = F.getSel() (calibration button's selected state, sent every packet while selected); 0x01/0x02 from
  the take/land button state 1/2. Byte 26: 0x04 = G.getSel() (always 0x04 in captures, we send 0x04), 0x10 = p0.
  Throttle t() = 0.5 +/- offset from the knob's y; if that y is a screen coordinate (grows downward), stick DOWN
  gives > 128 and our sign would be reversed. NOT provable from static code -> needs the announced-stick capture.
- Online cross-check: turbodrone docs/research/wifi_uav.md decompiles a SIBLING app of the same "lx" code family
  (lxCtrlView/lxBasicView, same h(f, trim, speed, centered) formula, trim (t-0.5)*63, status packet `88 00 .. 55`).
  Its short RC packet bit field = ours: 0x01 takeoff/up, 0x02 land/down, 0x04 stop, 0x80 gyro calibration.
  Turbodrone (flight-tested on that family) sends stick UP -> higher byte (towards 255) for throttle, the same sign
  as our joystick_fly.py. => Online evidence supports our layout and throttle sign; still a sibling, not our app.
  Likely reason "throttle/calibration not working" props off: altitude hold ignores throttle on the ground, 0x80
  gives no visible feedback, and in the 21:49 run the throttle byte was mostly 77 = stick DOWN in our convention.

## 2026-11-08 Day 4 early: closed-loop autopilot from the drone camera (tools/autopilot.py) - NOT FLOWN YET
- Decision (user): steer with OpenCV ArUco from the drone's own camera instead of only timed open-loop moves.
  Tag 2 (10 cm) UPRIGHT at the far edge of table B facing A. Video = `ef 00 04 00` requests only (no ack packets,
  ~7 fps), control = `ef 02` from a second socket in the same process. That pairing is untested.
- Flow: takeoff 0x01 -> ACQUIRE (3 frames in a row see tag 2, else land after 5 s) -> APPROACH (P control) ->
  land 0x02 at range <= target(35) + 8 cm for 3 frames, keeps steering 3 s into the descent.
  pitch = 0.006/cm * (range - 35), max 0.5 forward / 0.3 back, fades to 0 at 20 deg bearing; yaw = 0.02/deg *
  bearing (max 0.4); roll = -0.008/cm * sideways offset (max 0.25); alt (throttle from elevation) available, off.
  --axes none (default) = observe only: hover, log what it WOULD send, land. Add one axis per test.
  Lands on: tag lost 1.5 s, approach > 15 s, overshoot (< 15 cm), 30 s cap, L, focus lost. Space = 0x04 kill.
- Sideways offset comes from the tag's tilt (solvePnP, new `lat` in aruco_distance.pose). On REAL frames it is
  stable at 50 cm (calib_50cm_b: +5.2..+7.0 cm, 0 sign flips in 64) but ambiguous at 100 cm (test_100cm: -13.6..+11.5,
  22 sign flips in 71 = planar pose ambiguity). Range/bearing at 100 cm stay rock solid (103.6..103.8, 2.2 deg).
  => roll only acts closer than 70 cm and only when the last 3 estimates agree in sign.
- Simulator (--sim: point-mass drone, 100 cm/s and 60 deg/s per full power, 0.4 s lag, synthetic 1280x720 camera at
  7 fps with 150 ms latency, side drift, 3 s descent). pitch,yaw,roll from 4 starts (150/+15/8 deg drift +5;
  180/-30/-10 drift -8; 100/+20/-5 drift +3; 140/+10/0 drift -4): all "arrived", landed 34-35 cm from the tag,
  -10..+8 cm sideways. pitch,yaw only: lands 15-20 cm from the tag but 34-38 cm sideways (pure pursuit + drift).
  Tag out of view at start (heading 45 deg) -> "tag not found" landing. Space mid-approach -> 0x04 then DONE.
- Replay of real frames (test_100cm): locks in 3 frames, would command pitch +0.37, yaw +0.04, roll 0 (100 cm).
- Signs used: forward = pitch up (arrow up), yaw right = D, roll right = arrow right; only throttle up is verified
  in the air. A wrong yaw/roll sign shows as the tag drifting OUT of the centre -> tag lost -> land.
- Test order: (0) hop.py hover_test (0x02 lands?) (1) autopilot --axes none: video + control together, does the tag
  stay in view at hover height? (2) --axes pitch (3) --axes pitch,yaw (4) --axes pitch,yaw,roll. All tethered.

## 2026-11-08 Day 3 start: open-loop hop runner (tools/hop.py, shows/*.json) - NOT FLOWN YET
- New: JSON show script (timed moves) -> validator -> runner window. Moves: takeoff, hover, forward/back/right/left,
  up/down, yaw_right/yaw_left, land. Hard per-move caps (translate <= 3 s, up/down <= 2 s, hover <= 10 s, takeoff
  2..6 s), total airborne cap --cap 30 (max 60), power 0..1 = fraction of a full key press at the tethered defaults
  (sticks 40 %, throttle 55 %), so a script can never push harder than what was flown. Any script error = refuse.
  Kill path same as the flight window: Space = 0x04 then stop, L = land now, focus lost -> land, cap -> land.
- Dry tests (nothing sent): forward_test timeline correct (takeoff 0x01 1 s, pitch ramps 128 -> 153 in 0.13 s and back,
  land 0x02 1 s, 8 s centred, stop); injected Space during takeoff -> 0x04 for 1 s then DONE; bad script (forward 5 s)
  refused; --fit / --to-b on fake numbers gave the expected forward time; a 400 cm hop at power 0.5 is refused.
- Uses 0x02 land, which is still NOT verified to land the drone (only sent, tethered). hover_test checks it first.
- Plan: (1) shows/hover_test.json tethered x3 -> hover drift + 0x02 behaviour, log offsets (terminal asks after
  landing -> captures/hops/hops.csv); (2) shows/forward_test.json x3 -> cm/s at power 0.5; (3) --fit, then
  --to-b <A-B distance> -> shows/hop_to_b_*.json, fly 5x, goal 3/5 on B. Landing tolerance still to be set.

## 2026-11-08 Keyboard flight window (pad unusable: laptop AMD USB 3.1 host controllers code 43, no Bluetooth)
- joystick_fly.py --keys: C connect, G calibrate, hold Enter takeoff/land, WASD + arrows sticks (ramped), Space kill,
  window focus lost while flying -> auto land. Dry run with scripted keys: every key hits its byte (up keys -> 179).
- 18:47 LIVE, PROPS OFF (user confirmed), --cap 45 --limit 0.4, Mullvad off. Three Space kills while READY (motors
  off, no test value). Takeoff 0x01 at 44.3 s, W/S throttle 179/77, A/D yaw 77/179, up/down pitch 179/77; time cap
  -> land 89.3 s, Space e-stop 0x04 at 95.9 s during LANDING, packets stopped 96.9 s. 3221 packets.
  User: "throttle still not working, yaw is working, pitch/roll works but very minimal".
  Reading (unverified): fits a working layout with props off. Yaw = rate control -> big motor split; pitch/roll =
  angle control on a drone that cannot tilt -> small split; throttle = climb-rate request in altitude hold -> little
  change on the ground. Throttle can only be checked airborne (tethered). After the run the laptop stayed on the
  drone Wi-Fi (Windows did not switch back); reconnected by hand.
  User: motors STOPPED INSTANTLY on Space at 95.9 s (packets continued 1 s after it).
  => VERIFIED (1 run, props off): byte 25 = 0x04 is an EMERGENCY STOP on the HS420 - faster than the ~1.2 s
     no-packet failsafe. 0x80 calibration: sent many times, motors stay off, no visible confirmation from the drone.
  Status of the no-flight conditions: layout in code = app decompile (yes); 0x04 props off (yes); 0x80 props off
  (sent, no feedback possible); stick directions from an announced app capture (NO - only the sibling-app reference
  and props-off motor sounds). Throttle can only be checked tethered in the air.
- Speed level: app has a 2-level speed button (ctrl_speed1/2, eCtrlSpeedKey default 0) sent in byte 24 bits 0-1
  (E.j = n % 2). The app's own stick scaling is always full (R[1] = 1.0), so the drone applies the speed level.
  We always sent 0 (slow). Added: build(speed=), joystick_fly.py --speed and key V / pad Back to toggle.
  Dry run: V switches byte 24 0 -> 1, all checksums OK.
- 19:11 LIVE props off (user confirmed), --keys --cap 60 --limit 0.4: two takeoffs, all axes 77/179, time cap
  landed both. V was NOT pressed -> both runs at speed 0, so the speed comparison is still open.
  User observation: after takeoff the motors go to MAX; moving the drone by hand changes how the motors
  accelerate (IMU/MEMS reacting). Reading: one-key takeoff on the ground can't detect a climb, so the motors run
  at max (saturated) -> throttle W/S has nothing left to change; the IMU response to hand movement dominates the
  small stick commands. Supports "props-off cannot test throttle". After the run the laptop again stayed on
  the drone Wi-Fi (home reconnect in Link.home() did not take); reconnected by hand.
  SAFETY INCIDENT: user later said the PROPS WERE ON during this run (confirmed "props off" beforehand), drone held
  in the hand. "Motors go to max" = one-key takeoff pushing against the hand, not a props-off effect. The run's
  throttle/IMU observations are therefore hand-restrained flight, not ground behaviour. Same failure as 2026-11-04
  20:53 (props on when planned off). Rule: no hand-held runs with props on, ever; props-on only tethered.
  User decision afterwards: hand-held props-on is OK for them (gloves, tiny drone, move fast). Rule now: say the
  setup out loud per run (props off / hand-held with gloves / tethered), face away, finger on Space.
- 19:18 HAND-HELD, PROPS ON, gloves (user "Go"), --keys --cap 30 --limit 0.4, speed 0 (V not pressed, S not pressed).
  Takeoff 17.9 s; roll 77, pitch 179/77, yaw 179/77, throttle W = 179 at 34 s and 40.2-46.4 s; cap -> land 47.9 s.
  User: "best ever flight", flown on a ROPE (tether), wants throttle "just slightly" stronger.
- Change (one thing): joystick_fly.py --throttle-limit (throttle only, PgUp/PgDn +/-5 % live). Next run 0.55 ->
  full W = 198 (was 179); roll/pitch/yaw stay at 0.4. Dry run: byte 20 = 198, others 128, checksums OK.
- 19:21 TETHERED (rope), props on, gloves, --keys --cap 30 --limit 0.4 --throttle-limit 0.55, speed 0.
  Flight 1: takeoff 78.2 s, throttle reached 198 (W) and 58 (S) once each (short taps), Space e-stop 106.2 s.
  Flight 2: takeoff 121.8 s, mostly other axes, time cap -> land 151.8 s, Space e-stop 157.0 s.
  Throttle samples logged over both: mostly 128, extremes 58..198. User: "this is perfect setting".
  => DEFAULTS now: --limit 0.4 (roll/pitch/yaw, full = 77/179), --throttle-limit 0.55 (full = 58/198), speed 0.
     First good controlled (tethered) flights after the crash with the decoded layout. Throttle works in the air.

## 2026-11-05 CRASH + app decompile: the real control layout (supersedes earlier guesses below)
Free flight with the joystick window: drone flew sideways the whole time, "no control", went into water.
Board still blinks blue at power-on (survived so far); battery treated as damaged.
Root cause found by decompiling OUR app (com.mjx.holystone 1.0.23, androguard 4.1.2; classes
com.activity.ctrl.fragment.FrgmCtrl.b0(), ba2 "lxMjxCtrlPro" .a(), w92.e()/f()). The 11-byte control block
at packet offset 18 is built as:
  [0] 0x88  [1] 0x00
  [2] = byte 20 = a = THROTTLE  (mode 2: left stick vertical)
  [3] = byte 21 = b = YAW       (left stick horizontal)
  [4] = byte 22 = c = ROLL      (right stick horizontal) + aileron trim  (eTrimAileKey)
  [5] = byte 23 = d = PITCH     (right stick vertical)   + elevator trim (eTrimElevKey)
  [6] = byte 24 = s | B<<2 | C<<4 | r<<6 ; s = speed button state (ctrl_speed1), others 0 in the app
  [7] = byte 25 = 0x01 TAKEOFF, 0x02 LAND (same ctrl_takeland button, state 1/2), 0x04 EMERGENCY STOP
                  (ctrl_stop), 0x08 photo, 0x10 record, 0x20 flip (ctrl_roll), 0x40 (D, unknown),
                  0x80 GYRO CALIBRATION (ctrl_gyochk)
  [8] = byte 26 = 0x01 H!=0 | 0x04 ctrl_udrlsw selected | 0x08 arg | 0x10 p0   (app sent 0x04 / 0x14)
  [9] = byte 27 = XOR of block[1..8]      [10] = 0x55
Stick value -> byte (w92.e, ba2(255, 63, 32, 2)):
  byte = round((stick - 0.5) * 255 + 127.5 + (trim - 0.5) * 3 * 63), clamped 0..255; within 1 of 127.5 -> 128
  stick 0..1 from lxHraeSlider.t() (vertical) / .p() (horizontal); direction of "up" NOT yet confirmed.
What this means for everything flown so far:
  - Our axis map (20 roll, 21 pitch, 22 throttle, 23 yaw, from turbodrone) was WRONG on every axis:
    our "throttle" went to the roll byte -> sideways flight; "yaw" went to the pitch byte.
  - Our "calibrate" 0x04 was really EMERGENCY STOP (no effect on the ground). The drone was never calibrated.
  - Our "kill" 0x02 was really LAND (motors keep running until on the ground) -> explains the 2026-11-04 result.
  - Real calibration (0x80) and real emergency stop (0x04 in flight) are UNTESTED.
NO FLIGHTS until: code uses this layout, stick directions are confirmed from an app capture with known
stick moves, 0x80 and 0x04 are checked props off, and the next flight is tethered over soft ground away from water.
Code fixed the same day (joystick_fly.py, flight_ui.py, preflight_test.py): new AXIS_MAP, X = 0x80 calibration,
A = 0x01 takeoff / 0x02 land, B/Space = 0x04 e-stop for 1 s then packets stop, controller lost in flight ->
auto land 0x02, roll/pitch trim on D-pad left/right and LB/RB (+/-2 counts). Dry-run simulation with a fake pad:
every stick lands on its decoded byte, every button sends its decoded bit. Nothing sent to a drone yet.

## Drone identity
- Model: Holy Stone HS420 (HVIN HS420A, batch HS420-R...-EU-250801), 3.7 V
- FCC ID: 2AJ55HOLYSTONEGW (IC 22332-HOLYSTONEGW), grantee Xiamen Huoshiquan Import & Export
- FCC grant (2022-07-20): one radio, DTS, Part 15C, 2412-2462 MHz, 0.09 W conducted.
  2412-2462 MHz = exactly Wi-Fi channels 1-11 -> strong hint the link is Wi-Fi. NOT confirmed yet.
  Test report PDF not read (fccid.io consent wall, fcc.report Cloudflare check).
  Product code "HOLYSTONEGW" is generic, so this grant may cover several Holy Stone models.
- Companion app: Android package com.mjx.holystone (already installed on the phone; display name and version TBD)
- Capture phone: Xiaomi 11 Lite 5G NE, Android 14, wireless adb (USB data not working on laptop),
  PCAPdroid 2.0.2 started via CaptureCtrl intent, app_filter=com.mjx.holystone, pcap_file -> /sdcard/Download/PCAPdroid/
- Controller in box? (physical RC transmitter yes/no): [TODO]

## IMPORTANT (2026-11-01, user): the drone is normally flown with the physical REMOTE CONTROL, not the phone.
- The phone app's control screen showed no on-screen sticks (only a gamepad icon in the toolbar) and the
  idle `ef 02` packets carried no varying stick bytes -> the Wi-Fi link may be video only, with flight
  control on the remote's own radio. UNVERIFIED. To check:
  1. Remote's FCC ID label -> its radio type/frequency.
     DONE: remote = Model HS420, HVIN RC211A, FCC ID 2AJ55HOLYSTONEGT (IC 22332-HOLYSTONEGT), 3x AAA.
     FCC grant 2022-06-10: Part 15C, 2451-2481 MHz. This is NOT the Wi-Fi channel plan of the drone's
     grant (2412-2462), so the remote very likely uses its own proprietary 2.4 GHz radio (chip unknown;
     internal photos exist on fccid.io but not viewed yet). Conclusion so far: two links -
     drone Wi-Fi AP = video (+ app control?), remote = separate 2.4 GHz flight-control radio.
  2. Does the app have on-screen sticks (gamepad icon) that fly the drone WITHOUT the remote?

## 2026-11-01 VERIFIED: app controls the drone over Wi-Fi WITHOUT the remote
Props off, remote OFF, PCAPdroid off, phone on HolyStoneFPV_2BA841, app -> 🎮 -> 🔒 once: MOTORS SPIN (user).
-> Flight control over Wi-Fi works. Plan stays: laptop/ESP32 -> UDP 192.168.169.1:8800 -> drone.

## Laptop replay tests (tools/replay_test.py, packets byte-identical to captured app packets)
- 2026-11-01 23:05 Test A: laptop joined drone AP (got 192.168.169.2), sent 195 x `ef 02` centred sticks
  at 39 Hz for 5 s, props off, remote off, phone off drone Wi-Fi. Replies received: 0
  (captures/replay_replies_20261101_230511.txt). Laptop auto-returned to home Wi-Fi.
  0 replies is inconclusive: control may be un-acked, drone may need a session start first (app sent
  `ef 00 04 00` before), or Windows firewall may drop replies from another source port.
- 2026-11-01 23:06 Test B: same as A, plus byte 25 = 0x01 from t=1.0 to 2.0 s (app's 🔒 packet).
  MOTORS SPUN (user, props off). Replies: 0 (captures/replay_replies_20261101_230621.txt).
  => VERIFIED: laptop controls the HS420 over Wi-Fi with no app, no hello/handshake needed.
     byte 25 = 0x01 (held ~1 s) = motor unlock/start. Open: what stops motors (0x01 again? 0x02?),
     and do they stop on their own when packets stop (failsafe)? Must know before any flight.
  User: motors STOPPED BY THEMSELVES after the run. Not yet clear WHY: (a) failsafe on packet loss, or
  (b) idle auto-disarm a few seconds after unlock with throttle centred. Next test separates them:
  unlock, then keep sending centred sticks for 8 s - if motors run until ~8 s then stop -> (a).
- 2026-11-01 23:10 "Jarvis test mode" (= Test B again, recorded on video): motors spun again (user had taped
  small flags to the motors to make the spin visible; drone shook/moved from the imbalance). Repeatable: 2/2.
  Replies: 0 (replies file lost when the drone files were removed from the shared folder on 2026-11-02).
- 2026-11-02 22:33 Failsafe test: unlock (byte 25 = 0x01 at t=1-2 s), then centred sticks until t=8 s,
  props off, drone weighted down. Motors ran until the END of sending (~7 s of spin) and stopped then (user).
  => VERIFIED (1 run): motors stop when packets stop - this is (a), a no-packet failsafe, not an idle
     timeout. Kill path for all tools = stop sending (joystick_fly: B, Start, or releasing LB).
     Not measured yet: how long after the last packet the motors stop (looked immediate to the eye).
  (22:29 attempt aborted before sending: drone Wi-Fi not up yet, laptop got no 192.168.169.x address.)
  Replies: 0 (captures/replay_replies_20261102_223350.txt).
- 2026-11-02 ~23:30 Flight window (joystick_fly.py, live, --limit 0.5 --cap 30), props off, Xbox controller:
  Y -> SAFE in 1.8 s, hold A -> ARMED, two armed runs of 30 s each, both ended by the time cap -> STOPPED.
  No link loss, no crash. User: "impressive". Axis mapping per stick NOT yet reported -> still a hypothesis.
- 2026-11-04 20:53 Preflight test (tools/preflight_test.py --send), Mullvad disconnected. PROPS WERE ON (test was
  planned props off): the drone TOOK OFF and flew, badly unbalanced (user). Mic timeline (packet time):
    CALIBRATE 0x04 at 2.0-2.5 s: motors stayed off (expected).
    TAKEOFF 0x01 at 3.5-4.5 s: motors on at 3.34 s -> VERIFIED: 0x01 = one-key TAKEOFF (it really lifts off).
    KILL 0x02 at 7.5-8.0 s: motors stayed ON (100 %), also 8-10 s while centred packets continued.
      -> 0x02 is NOT an emergency stop on the HS420 (or needs a different form). turbodrone hint wrong here.
    Last packet 10.0 s, motors off 11.24 s -> no-packet failsafe takes ~1.2 s. This is the ONLY verified kill.
  Motor pitch ~1750 Hz on the mic. Data: captures/preflight_audio_20261104_205320.csv.
  Lessons: (1) no command may be sent until "props off / tethered" is confirmed out loud for THIS run;
  (2) kill = stop sending is ~1.2 s slow -> find a faster stop (land 0x01 again? throttle min?) props off;
  (3) unbalanced flight: check props A/B (CW/CCW) placement before any props-on run.
- 2026-11-04 20:59 Land run (preflight_test.py --send --land), props ON, safety rig, user confirmed props
  checked / rig on / nobody near / battery. Mic (packet time): motors on 1.77 s (takeoff 2.0 s; ~0.2 s clock
  offset mic vs packets) -> off 12.22 s; last packet 11.0 s. 0x01 a second time at 6-7 s did NOT stop the
  motors. Motor pitch hover 1540 Hz, during land cmd 1560 Hz, after it 1480 Hz (slightly lower: possibly a
  slow descent - needs the user's eyes). Takeoff 0x01 verified 2/2. Failsafe delay again ~1.0-1.2 s.
  Data: captures/preflight_audio_20261104_205929.csv.
  User: after the second 0x01 the drone DESCENDED (landed), but not on its take-off spot (drifted sideways).
  => VERIFIED: 0x01 toggles takeoff / land. Landing = slow descent, motors keep running until on the ground
     (or until packets stop). Horizontal drift: no position hold (altitude hold only); no gyro calibration
     was sent before this run. Next: calibrate (0x04) on a level surface before takeoff, correct with sticks,
     later close the loop with camera + ArUco.
- 2026-11-04 21:04 Axis run (preflight_test.py --send --axes), PROPS OFF (user confirmed). Takeoff, then bytes
  20..23 each = 200 for 2 s. Mic (dominant peak / dB): centre 1020 Hz -41.8 dB; b20 820 Hz -44.1 dB;
  b21 3020 Hz; then 3000 Hz for everything after (peak jumped to a harmonic and stayed). Loudness flat.
  => INCONCLUSIVE by mic: props off + altitude hold means "throttle" does not simply speed all motors up,
     and the single-peak estimator is too fragile (needs a spectrum per motor, or saved audio).
  Visual observation from the user pending. Data: captures/preflight_audio_20261104_210359.csv.
- 2026-11-04 Axis mapping ADOPTED (user decision: "use the pitch and online stuff", no visual test):
  byte 20 = roll, 21 = pitch, 22 = throttle, 23 = yaw. Basis: turbodrone (fld_compat order R,P,T,Y),
  reversing-wifi-uav (roll, pitch, throttle, yaw), our sticks.pcap pairing (20+21 move together, 22+23 together).
  Directions (sign) unverified -> first tethered flight at --limit 0.3; flip a sign in AXIS_MAP if reversed.
  joystick_fly.py now: X = calibrate 0x04 (0.5 s), hold A = takeoff 0x01 (1 s), hold A again = land 0x01,
  A must be released between them, B/Space = KILL (stop sending), time cap -> auto land, then stop after 8 s.
  Simulated in dry mode with a fake pad: all transitions and pulses as intended.
- 2026-11-04 ~21:30 First props-on flight in the window (--limit 0.3 --cap 15), tethered: Y -> SAFE 26.5 s,
  X calibrate 28.8 s, READY 29.3 s, takeoff (FLYING) 33.9 s, time cap -> auto LANDING 48.9 s, window closed
  during landing (packets stop -> failsafe). 951 packets. User then reported the pad's buttons are swapped:
  physical A = index 1, B = 0, X = 3, Y = 2 (was coded as standard XInput 0/1/2/3). Swapped map tried, but the
  controller test panel (same label map) then showed A/B and X/Y swapped -> reverted to standard 0/1/2/3.
  Also found: SDL ignores pad input while the pygame window is not in front (button probe got 0 presses) ->
  joystick_fly.py sets SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS=1 so B/KILL works whatever window has focus.
  Standard map confirmed by the user in the test panel: A, B, X, Y each light up their own label.
- 2026-11-04 ~22:00 Two more tethered flights (--limit 0.3): takeoff at 43.1 s and 79.2 s, both ended with B/KILL.
  User: "control doesn't seem to work at all" (sticks had no visible effect). Pad dropped out twice (USB).
  Bug found + fixed: after a reconnect the window reused the old calibration flag and went straight to READY;
  now X is required again. Added a terminal log of the stick bytes sent every 0.5 s while flying.
  Suspect 30 % stick range (bytes 90..166) is too weak; app sends 0..255. Next flight at --limit 0.6.
- 2026-11-05 Flight at --limit 0.6 (tethered): user "the flight controls are too aggressive" -> the stick bytes
  DO steer the drone (first evidence the axis bytes work). Which axis/direction matches is not reported yet.
  Changed: default --limit 0.4, new --expo 0.6 (soft centre: half stick -> 138 instead of 158 at 60 % linear,
  full stick -> 179), D-pad up/down changes the limit live in 5 % steps (10..80 %), logged in the terminal.
  Added a MOTORS - EXPECTED SPEED card (standard X-quad mixer, props-in layout) for fast visual feedback.
  Flight behaviour itself not yet reported.

## Video (drone -> laptop), for the ArUco distance test
- Capture analysis (sticks.pcap): the `ef 02` records after offset 88 are frame acks:
  `<u32 frame id> <u32 0> <u32 1=complete/0=partial> <u32 0x18> <8-byte fragment bitmap>`, frame ids
  rise ~10/s -> video = numbered frames in fragments, app acks each with a received-fragment bitmap.
  Before video runs (idle.pcap, VPN broken) the app repeats `ef 00 04 00` at ~10 Hz -> likely "start video".
- 2026-11-04 05:38 tools/video_probe.py (only `ef 00 04 00`, 10 Hz, 8 s, no stick packets): 0 packets back.
  Cause found: Mullvad VPN connected with "Local network sharing" = block -> its firewall drops LAN
  traffic to/from 192.168.169.x. Windows Firewall is fine (Python allowed inbound UDP, Public).
  Next: repeat with Mullvad local network sharing on (or Mullvad disconnected).
- 2026-11-04 05:41 same probe, Mullvad disconnected: VIDEO RECEIVED (captures/video_probe_20261104_054103.bin).
  3297 pkts / 3.4 MB in 7.9 s from drone UDP port 1234 to our source port. No `ef 02` needed.
  `93 01` fragment = 56-byte header + <=1024 B data: 2-3 len, 16 frame slot (cycles 1..4, new image each
  time even without acks), 32 frag index, 36 frag count (57), 40 frame bytes (~58 KB), 44/46 = 1280 x 720,
  48 = 0x32 (JPEG quality 50). Data = JPEG scan data WITHOUT header (every ff is ff 00, no markers).
  The app (libuav_lib.so) holds six 605-byte JPEG header templates (q levels); the q50 one is byte-identical
  to a header built from the standard Annex K tables, 4:4:4 (tools/drone_video.py builds it itself).
  Result: 56/56 frames decode, ~7.3 fps, sharp 1280x720 colour image of the desk. `93 04` 20-byte
  packets (~10/s, `93 04 14 00 a5 a5 a5 a5 08 00 ...`) not decoded yet (telemetry/status?).
  Laptop needs Mullvad off (or LAN sharing on) for any drone traffic.

## ArUco distance from the drone camera (tools/drone_video.py --live --aruco, tools/aruco_distance.py)
- 2026-11-04 tag 2 (10 cm) upright facing the drone, 50 cm from the lens (ruler). First try: tag cut off
  at the top of the image (camera sits a few cm above the table) -> 0 detections; lowered the tag ->
  detected in 64/69 frames. Uncalibrated (assumed hfov 80 deg) read 33.1 cm.
  Calibrated on these frames: f = 1152 px at 1280 px width = hfov 58.1 deg (calib/drone_cam.json).
- 2026-11-04 TEST at 100 cm (ruler), calibrated: predicted 103.7 cm median (103.5..103.9, sd 0.09 cm),
  71/71 frames detected, bearing +2 deg -> error +3.7 cm (+3.7 %).
  Two points cannot separate a scale error from an offset in where the distance is measured from: the
  data fit exactly if the camera's optical centre is ~3.5 cm in front of the ruler's zero point (a
  ruler read slightly long fits too). Next: a third distance (e.g. 30 or 150 cm) and fit
  f + offset from all points. Drone switched itself off twice during these tests (battery?).

## Frame rate budget (2026-11-04, from video_probe_20261104_054103.bin and sticks.pcap)
- Without acks the drone sends frames in groups of 4 (slot 1..4) back to back, then stalls 270-450 ms
  -> looks like a 4-frame send window that waits for acks and times out. Average 7.3 fps.
- One 58 KB frame takes ~48 ms on air = ~10 Mbit/s burst rate -> link ceiling ~20 fps at 1280x720 q50.
- Phone app (acks inside its `ef 02` packets): 607/607 frames complete in 50.7 s = 12.0 fps.
- Laptop per frame: JPEG decode 5 ms, ArUco 13 ms, HUD render 20 ms = 39 ms (~26 fps max).
- 60 fps x 58 KB = 28 Mbit/s: ~3x what this link delivers; not reachable at this resolution/quality.
- App acks (idle2.pcap, video-only mode, 15.9 fps): record-free base `ef 02` (88 B, byte 12 = 0x21, control
  block length 8, bytes 18-28 zero = no sticks, no unlock) + records `<u32 id> <u32 0> <u32 1> <u32 len>
  <bitmap padded to 4 B, unused bits 1>`, each frame acked in 2-3 packets at ~39 Hz; partial frames get
  done=0 records with the missing fragments as 0 bits (retransmit request, not copied yet).
  Implemented in drone_video.live_frames (acks=True); tools/ack_test.py compares no-acks / acks+hello /
  acks without hello. NOT RUN YET (needs props off).

## On-screen sticks (🎮) capture - 2026-11-01 (captures/sticks.pcap)
Source: user's PCAPdroid capture Downloads/withDroneControls.pcap, filtered to drone IPs (2280 of 2459 pkts).
Props off, remote off, user enabled 🎮 on-screen sticks and moved them. Note: PCAPdroid VPN disturbs the
drone video on this phone (video ok as soon as capture stops); outbound packets are still recorded.
All UDP -> 192.168.169.1:8800, type `ef 02`, ~39 pkt/s, 50.7 s. Per-packet layout (offsets in UDP payload):
| off   | value seen            | meaning (status)                                                  |
|-------|-----------------------|-------------------------------------------------------------------|
| 0     | ef                    | magic (seen in every packet)                                      |
| 1     | 02                    | packet type                                                       |
| 2-3   | 58 00 / 70 00 / ...   | total length, little-endian (verified vs. actual length)          |
| 8     | 0..3                  | number N of 24-byte records appended after offset 88              |
| 12-13 | increments by 1       | 16-bit LE sequence counter                                        |
| 16    | 0b (08 in idle2)      | probably length of control block (11 B) - hypothesis              |
| 18-19 | 88 00                 | constant                                                          |
| 20-23 | 00..ff, centre 0x80   | FOUR STICK AXES (b20 b21 b22 b23) - verified they move with sticks |
| 24    | 00                    | constant so far                                                   |
| 25    | 00 / 01 / 02          | button command: 01 for ~1 s at 7.5 s, 02 for ~1 s at 42.1 s        |
| 26    | 14 / 04               | 14 at start/end, 04 in between (bit 0x10 = 🎮 off?) - hypothesis   |
| 27    | xor                   | CHECKSUM = XOR of bytes 20..26 - verified on 1962/1962 packets     |
| 28    | 55                    | constant (end marker?)                                            |
Records after offset 88: `<u16 counter> 00.. 01 00 00 00 18 00 00 00 ff*8`, counter rising -> likely video
frame acks, not flight control.
Axis mapping HYPOTHESIS (from wobble when grabbing a stick: b20+b21 move together, b22+b23 move together):
left stick = b20 (throttle, moved first) + b21 (yaw); right stick = b22/b23 (pitch/roll, order unclear).
Must be verified with one-axis-at-a-time captures before any replay.

## Day 1: Recon

### Step 1: Identify the control link (no flying needed)
Check each and write what you saw:
- [x] FCC ID lookup: 2412-2462 MHz, single DTS radio (Wi-Fi channel range). Points to Wi-Fi.
- [x] Wi-Fi scan from laptop (tools/wifi_scan.py, baseline in captures/wifi_baseline_drone_off.txt):
      drone ON -> new AP `HolyStoneFPV_2BA841`, BSSID b4:c2:e0:2b:a8:41, 2.4 GHz ch 4, 802.11n,
      Authentication Open, Encryption None. AP took >10 s after power-on to appear (first scan missed it).
- [ ] Phone Bluetooth / nRF Connect scan with drone on: new advertiser appears?
- [ ] Does the app require joining the drone's Wi-Fi, or pairing over Bluetooth?
- [ ] Does the drone fly without the phone, from the RC transmitter only?

Result: link type = Wi-Fi (drone is an open access point). Still to confirm: that the app's
flight commands go over this Wi-Fi and not over a separate RC radio (next: join AP, capture app traffic).

### Step 2: Capture each basic move (props OFF)
Capture method depends on step 1:
- BLE: Android Developer options -> Enable Bluetooth HCI snoop log -> toggle BT off/on -> do one move -> pull log (`adb bugreport`) -> save as `captures/<move>.btsnoop.log`
- Wi-Fi: capture phone traffic (PCAPdroid on Android, no root) -> save as `captures/<move>.pcap`
- 2.4 GHz RC only: stop and discuss; needs a different approach.

One move per capture, with ~3 s idle before and after:
- [ ] idle (connected, sticks centred)
- [ ] takeoff
- [ ] hover
- [ ] forward (pitch stick full forward, 2 s)
- [ ] yaw (yaw stick full right, 2 s)
- [ ] land
- [ ] emergency stop (if the app has one)

Done when: link type known and every move above has a capture.

## App static hints (com.mjx.holystone 1.0.23, versionCode 151) - HINTS ONLY, verify against captures
- Multi-drone app: native libs per drone family, each with a hardcoded drone IP:
  libuav_lib.so -> 192.168.169.1, liblxPro.so/liblxAVPlayer.so -> 192.168.100.1,
  liblgPro.so -> 172.16.11.1, Java RTSP video -> rtsp://192.168.99.1:554/
- No Java/NDK network-binding calls found (bindProcessToNetwork, bindSocket, android_setsocknetwork),
  so the app does not obviously bypass a VPN. None of these IPs overlap home LAN 192.168.0.0/24.

## Log
- 2026-11-01 21:35: drone-only PCAPdroid capture (hs420_idle.pcap) = 0 B. Usage stats show no live
  drone session during it (app opened at 21:37 while phone was still on home Wi-Fi: grey screen, red warning).
- 2026-11-01 21:52: user restarted PCAPdroid with defaults (ALL apps), 32 s, 247 KB; drone app opened at
  21:52:53 inside that window with live camera (video froze at times, likely VPN overhead).
- 2026-11-01 22:10: with user's OK, filtered that file to the 4 drone IPs (tools/pcap_filter.py) ->
  captures/idle.pcap (276 of 851 packets), original deleted unread on phone and laptop.
  Result (tools/pcap_summary.py), 20 s, ONLY outgoing packets, ZERO replies from any drone IP:
    - UDP 10.215.173.1:49906 -> 192.168.169.1:8800, payload `ef 00 04 00` (4 B, constant), every ~102 ms (~9.8 Hz)
    - TCP SYN -> 192.168.99.1:8080 and -> 172.16.11.1:18000, alternating every ~0.5 s, never answered
      (app probing other drone families)
    (10.215.173.1 = PCAPdroid's virtual VPN address, not the phone's real Wi-Fi IP)
  Reading: app talks UDP to 192.168.169.1:8800 (matches libuav_lib.so hint), but the drone never answered
  while the VPN was on. Likely cause (unverified): PCAPdroid forwards the app's packets over mobile data
  (validated network) instead of the drone Wi-Fi (no internet). Fits "camera froze under capture".
  Next: repeat with mobile data OFF so drone Wi-Fi is the only network. Not yet known: drone replies,
  video port, whether `ef 00 04 00` is a heartbeat/hello or a video request.
- 2026-11-01 22:12: user's own PCAPdroid capture (Downloads/PCAPdroid_01_Nov._22_12_50.pcap, 250 s), filtered
  to drone IPs -> captures/idle2.pcap (11637 of 12261 packets; other 624 not looked at).
  Result: STILL ZERO UDP packets FROM the drone. App -> 192.168.169.1:8800 only, ~39 pkt/s, all type `ef 02`:
    - byte 0 = 0xef (magic), byte 1 = 0x02 (type), bytes 2-3 = total length, little-endian (0x0058 = 88)
    - 88 B base packet: `ef 02 58 00 02 02 00 01 | 00 00 00 00 | 16 00 00 00 | 08 00 ...zeros... | 32 4b 14 2d 00 00` (tail at offset 82)
      only offset 12 varies (0x16 or 0x21) across 3223 packets -> no stick values visible in idle
    - byte 8 = record count N; length = 88 + 24*N (112, 136, 160 seen); record e.g.
      `18 01 00 00 00 00 00 00 01 00 00 00 18 00 00 00 ff ff ff ff ff ff ff ff`
    - other sizes (108, 116, 124, 128, 132, 140, 144, 152) not yet decoded
  No `ef 00 04 00` this time. Interpretation (UNVERIFIED): these look like video/session requests the app
  repeats because it gets no answer, not stick commands. TCP to 172.16.11.1/192.168.99.1 only got RSTs
  (probably synthesized by PCAPdroid). Open question: did the drone answer at all while the VPN was on?
  ANSWER (user): live camera picture during this capture (saw own face). So the drone DID send video, but
  PCAPdroid recorded none of it -> PCAPdroid on this phone only sees app->drone. Fine for decoding commands
  (that's the direction we replay); drone->app (video, telemetry, acks) needs another vantage point later
  (ESP32 promiscuous sniffer on ch 4, or monitor-mode USB Wi-Fi adapter; network is open/unencrypted).
- 2026-11-01: FCC lookup (2412-2462 MHz). Laptop Wi-Fi scan off vs on: drone exposes open AP
  HolyStoneFPV_2BA841 on ch 4. Link = Wi-Fi. Next: phone joins AP, PCAPdroid capture per move.

## 2026-11-10 - tools/measure_tags.py (offline tag check for the two-tag board)
- New offline tool: reads saved frames (e.g. a `live_flight.py --test 10` folder), finds tags 1 and 2 with the
  flight detector, prints per tag: found %, distance, bearing, elevation, size in px; with both tags in view the
  centre-to-centre gap and offset; with `--true-cm` the real black-square size from a tape-measured distance.
- Checked: old ground test 20261109_172319_test (tag 2 at 138 cm by tape) -> 137.9 cm, size 10.0 cm, 157/190.
  Rendered board (tag 1 37 cm above tag 2, 120 cm away) -> gap 37.1 cm, 0.0 cm sideways / depth.
- Other ids are ignored (pass 2 gave a 10 px false "tag 17" in 5/190 frames).
- Next: ground run in front of the new board, then measure_tags on it.

## 2026-11-10 21:55 - approach flights all land ~5 s after takeoff (flight 20261110_215518, 4 takeoffs)
- Start 240-250 cm from the box. Every takeoff: climb -> bottom tag slides off the bottom edge -> ACQUIRE ->
  RECOVER -> "LAND: camera blind 2.5 s during the search". Frames: the view is black curtain + bare white wall,
  nothing for the visual-odometry heading to track (VO heading frozen from 49.3 s) -> blind -> safety landing.
  The blind time already builds up during the climb, so the search gives up the moment it starts.
- Top tag (id 1) IS in the picture after the climb, but the flight code only looks for the bottom tag (id 2).
  Detector on air frames: top 42/444 (9 %), bottom 84/444 (19 %); both ~45-48 px at 240 cm, sharpness median 55.
  On the ground at 250 cm: bottom 92/92, top 1/92.
- Board from one ground frame: top tag 37 cm above the bottom one, ~10 cm nearer the drone (not one plane).
- FIX (3 changes, approach only; hover unchanged):
  1. find_tag(use_top=True) in live_flight: bottom tag not found but top tag found -> the bottom tag's position
     from the top tag + TOP_TO_BOTTOM_CM (0, 37, 10) (flight_control.py). Controller unchanged. flight_review
     still uses the bottom tag only. events.csv logs "tag sightings in the air: bottom N, top M" at each landing.
  2. Lock / re-lock = 3 / 2 sightings within 1 s (was: in a row; with 9-19 % detection it never locked).
  3. tracker.Recover: camera blind > 2.5 s -> hover with centred sticks and keep looking until the 9 s search
     limit (was: land at once). Still never steers blind; time cap and Space kill unchanged.
- Replay of the 21:55 air frames: sightings bottom only 17/16/28/23 -> bottom+top 17/27/45/37 per takeoff; new
  lock rule locks 3.0-3.4 s after takeoff in takeoffs 2-4 (old rule: never). Tests: top_tag, blind_hover added.

## 2026-11-10 22:39 - camera tilted down: approach overshoots over the box -> brake, close-landing, live tuning
- User tilted the drone camera down: tag stays in view longer (245 -> 134 cm in 4 s, ~30 cm/s), but with the alt
  axis the drone centres the tag = climbs to look DOWN on it, and when the tag left the bottom of the picture the
  sticks centred with no braking -> it glided over the box.
- Changes: BRAKE state (pitch back 1.4 x recent forward push, max tuning brake_max, 0.5 s) before every landing
  from approach and before a search; tag lost below centre (< -3 deg) within target + close_cm -> brake + land
  ("close enough"); alt_on tuning switch; --hop demo mode (ground distance -> timed forward -> land, hop_speed
  self-calibrates from the landing distance; sim: 84 cm then 62 cm for target 60).
- calib/tuning.json is now reloaded live by the window (changed values logged as "from file") -> values can be
  tuned between flights without restarting. Saves are atomic (tmp + replace).
- Sims under CPU load fail (window loop 4 Hz -> held Enter missed, watchdog). Cause: motion-model session's 6
  worker processes + OneDrive syncing the flight frames. Pause both while flying.
- 23:27 flights (alt off, start ~290 cm): approach closes to 150-165 cm, then the tag leaves the picture at the
  BOTTOM (elev -18..-20) -> 9 s search -> user lands with L. Change: close_cm 80 -> 150 in calib/tuning.json (tag
  lost below centre within 210 cm -> brake + land), new tuning recover_s (search time, default 4 s, was 9 s).
- 23:36 phone video (flight 20261110_232726, takeoff 534 s): drone climbs near the ceiling after the desk takeoff,
  box leaves the bottom of the picture ~1 m out (165 cm straight-line), L -> slow 0x02 descent lands left of the box.
- NEW tools/live_flight_v2.py (live_flight.py kept as is): descend-only height hold while approaching (tag below
  -8 deg -> sink) and, when the tag leaves the bottom close by: BRAKE -> DESCEND (throttle -0.5, max 2.5 s) until
  the tag is back -> approach again; max 2 descents, then brake + land. tests/test_v2.py (tilt 20, 110 cm high,
  start 230 cm): v1 brake-lands at 117 cm; v2 descends twice, re-finds the tag, arrives at 60 cm, lands at 55 cm.

## 2026-11-11 00:09 flights + v2 yaw learning
- Approaches (flight 20261111_000909): tag sat 10-14 deg LEFT of centre the whole approach, yaw command -10..-14
  bytes all the time, VO heading still creeping right: while pitching forward the drone turns right more than in
  a hover; trim_yaw -14 (hover value) is not enough and P-only yaw needs that error to push back. User: "trim and
  controls fight each other".
- live_flight_v2.py: yaw integral while the tag steers yaw (tuning yaw_i 0.012 power/(deg*s), 0 = off, max 0.35),
  offset also added in brake/descend/hold; log at landing: learned offset + matching yaw trim.
  tests/test_v2.py, sim with 8 deg/s own right turn: |bearing| 2nd half of approach v1 8.5 deg -> v2 1.3 deg,
  sideways 13 -> 5 cm, learned -0.171 power (-9 bytes). Last-metre scenario: v2 lands 53 cm (target 60), v1 122 cm
  brake-landing. Not flown yet. live_flight.py unchanged in behaviour (only new shared tuning entry).

## 2026-11-11 - scorecard (metric to work towards)
- North star: SUCCESS = lands within 20 cm of the goal (target cm in front of the tag, centred) with no human
  intervention (L / Space / Enter / Esc / focus). Supporting: landing error, interventions, air time, tag seen %,
  searches, sideways swing, heading swing.
- live_flight now measures every approach / hop landing from the ground ("LANDING RESULT: X cm in front, sideways
  S cm -> error E cm") and logs the trims at each takeoff ("tune trim_x = v (at takeoff)", for flight_review).
- NEW tools/scorecard.py <folders>: one row per takeoff + totals, writes <folder>/review/scorecard.csv. Sideways
  swing from flight_review's frames_review.csv when present (the VO-only guess underestimates it).
- Baseline (2026-11-10/11 approach flights, 65 takeoffs): 0 measured successes (landing not measured before
  today), interventions 27/65, air time median 11 s, tag seen median 62 %. 20:54 session (camera level, start
  ~200-250 cm): 2/3 ended "close enough" at 105 / 131 cm, sideways swing 78-117 cm, heading swing 35 deg.

## 2026-11-11 - drift over time + battery proxies (offline, tools/drift_trend.py, report notes/drift_battery.md)
- 114 takeoffs / 8 sessions from control.csv + events.csv (no frame processing). Trims rebuilt from 257 tune lines
  (final values match calib/tuning.json); before 11-10 00:34 assumed 0.
- Biggest drift = takeoff transient: backward lurch during the auto climb (1-3 s after takeoff) -13 cm/s on
  11-10 night, -33 cm/s since 11-10 23:17 (more forward trim did not reduce it; cause open: camera mount /
  wear / battery). After ~10 s the hover drift is near 0 (+-1-3 cm/s, own turn +1..+3 deg/s at the trim).
- No measurable drift / speed change within a session over 12-14 motor-minutes (battery swaps unknown). Auto-climb
  rate 24 cm/s (11-10) vs 40 cm/s (11-11, fresh charge) = best battery proxy so far.
- Battery byte (`93 04`) is not logged -> proposed: status.csv in drone_link + trims logged at takeoff.
- Next test: same battery, 2 takeoffs at trim pitch 22 vs 2 at 0 (tethered) -> is the lurch trim-related?
- 2026-11-11 later: LOGGING ADDED in tools/drone_link.py (both live_flight.py and live_flight_v2.py): `93 04`
  status -> status.csv per flight (fly_state, battery, flags, xor check; on change, at least every 5 s); at every
  takeoff events.csv gets the exact trims as "tune trim_x = v (at takeoff ...)" (flight_review now shows the real
  trims) + "status at takeoff: ... battery N". No flight behaviour change. Tests: new status_log + quick suite
  16/16 pass. drift_trend.py fits drift vs battery once >= 8 takeoffs have it. To verify on the next flights:
  fly_state at takeoff, battery number falling over a session.
