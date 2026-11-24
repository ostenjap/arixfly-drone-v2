# Next steps (written 2026-11-10, after the 01:02 hover session)

## Safe baseline (frozen, do not change without a reason)
- Tuning: `calib/tuning_safe_20261110.json` (copy of `calib/tuning.json`):
  trim_yaw -10, trim_pitch +16, trim_roll -4, hold on (gain 0.035, max 0.4, delay 1 s), yaw_check off,
  approach_gain 0.006, approach_max 0.5, target 60 cm.
  To go back: copy it over `calib/tuning.json`.
- Reference pictures + values: `captures/reference/20261110_safe_baseline/` (ground frame + 4 hover frames from
  `captures/flights/20261110_010237`, 520-545 s). Compare new flights against these.
- Known-good commands:
  - ground feed test: `python tools/live_flight.py --test 10` (18.8 fps verified)
  - hover: `python tools/live_flight.py`
  - approach: `python tools/live_flight.py --approach --axes pitch,yaw,alt` (not yet successful on the drone)
- Known behaviour at this baseline: holds height, heading within about +-10 deg (VO hold), wanders left/right
  (no position sensor; roll trim can't fix random drift).

## Phase 0 results (done 2026-11-10, details in notes/findings.md)
- `python tools/flight_review.py <flight folder>` works; reports + timelines are in each folder's `review/`.
- `python tests/run_all.py` = 19 loopback tests, all pass. `live_flight.py` is split into `drone_link.py`,
  `flight_control.py` and the window file (behaviour unchanged).
- What the footage says (21 takeoffs):
  1. Own right turn 6.6-18 deg/s untrimmed; your trim brings it to -1..+3 deg/s. Keep the trim.
  2. Yaw is weak: ~12-18 deg/s per stick unit. The heading hold (max 0.4) only adds ~6 deg/s. Candidate for the
     next session: `hold_max` 0.4 -> 0.6-0.8 (one change, Tab + =), then check the hover.
  3. In the air the picture is ~20x softer than on the ground and the tag is found in only 1-21 % of frames.
     The single biggest fix: a bigger matte tag (15-20 cm, no tape).
  4. The drone hovers 30-60 cm to one side of the tag line: that is the left/right wander; only the overhead
     webcam (Phase 3) really fixes it.

## Phase 0 - offline, no flying (Claude can do this alone) - DONE
1. `tools/flight_review.py <flight folder>`: frame-by-frame review of saved footage + logs.
   - tag pose per frame, with the sideways (planar) ambiguity resolved over time (look back and forward)
   - VO heading, drift rates (deg/s, cm/s) with sticks centred -> suggested trims
   - response to each command (delay, speed per stick step) -> measured gains
   - every tag loss with its cause (out of view / blur / glare)
   - output: a PNG timeline per flight + a short text summary
   Run it on 20261109_225836, 20261109_234246, 20261110_000232, 20261110_010237.
2. Move the fake drones + simulator from the scratch folder into `tests/`, one command runs everything:
   `python tests/run_all.py` (feed, acks/freeze, controls, kill, cap, crash, watchdog, approach, recovery).
3. Split `tools/live_flight.py` (~900 lines) into network/video, control, window - no behaviour change,
   checked with `tests/run_all.py`.
Done when: review reports exist for the 4 flights and all tests pass after the split.

## Phase 1 - measure the drone (next session, tethered, one axis per flight)
- Add a "pulse" mode: hover 3 s, then one short stick pulse (e.g. yaw left 1 s), hover, land. Only one axis
  and one direction per flight: yaw +/-, pitch +/-, roll +/-, throttle +/-.
- `flight_review.py` turns each pulse into real numbers (delay, speed per step).
- Also check the props/motors after the water crash: a constant right turn of ~15 deg/s is a lot.
Done when: we have measured response numbers for all 4 axes.

## Phase 2 - tune with real numbers
- Set heading-hold and approach gains from Phase 1 (one change per flight, logged in findings).
- Hover test: heading steady, no step-wise turning.
- Approach test from ~150 cm, tag in view: closes to 60 cm and lands.
Done when: 3 approaches in a row land within 20 cm of the target distance.

## Phase 3 - real position: overhead webcam (needed for landing ON table B)
- Decide the camera (USB webcam or phone via USB) and mount it ~2 m high looking down at A, B and the path.
- Tags: 5-6 cm matte tag on top of the drone, tag 2 flat on B.
- Ground test first: move the drone by hand over a ruler, readings within a few cm.
- Then: hover-and-hold over one spot (stops the left/right wander), then A -> B landing.
Done when: lands on B from 3 start spots (Day 4 criterion).

## Phase 4 - Day 5 items
- Decode the `93 04` status packet (battery?) -> land on low battery.
- JSON show script + LLM: one sentence -> script -> validator -> flight.
- Safety supervisor: time cap, geofence (from the webcam), low battery.

## Rules that stay
Tethered, soft ground, nobody within 2 m, Space = kill. One change per test, logged in notes/findings.md.
