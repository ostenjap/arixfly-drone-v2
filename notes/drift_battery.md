# Drift over time and battery effects (offline analysis, 2026-11-11)

Tool: `python tools/drift_trend.py` (live logs only: control.csv + events.csv, no frame processing, ~5 s).
Outputs: `captures/drift_battery/` - `summary.txt`, `takeoffs.csv` (one row per takeoff), `windows.csv`
(2 s windows), `drift_timeline.png`, `drift_vs_session_time.png`, `drift_within_flight.png`.

## Data and how sure it is
- 114 takeoffs, 23 folders, 8 sessions (a new session = more than 25 min between takeoffs).
- **Drift** is measured where the sticks are centred (only trim + heading hold): the hover after the auto climb
  (FLYING / ACQUIRE) and the auto climb itself (1-3 s after takeoff). Position from the logged tag sighting
  (range, bearing, elevation) + VO heading; turn from the VO heading. Robust slopes (Theil-Sen).
- **Trims** per takeoff are rebuilt from all 257 `tune trim_*` lines (first one 2026-11-10 00:34), carried
  forward in time. Check: the rebuilt final values (pitch 22, roll -2, yaw -14) equal `calib/tuning.json` now.
  Before 11-10 00:34 trims are *assumed 0* (arrow-key trim era, not logged). A hand edit of the file between
  sessions would be missed.
- **Limits:** hover drift from the tag exists for only ~8 takeoffs (11-10 00:37 session; the tag was rarely seen
  in the air then; all later flights are approaches with just a 3 s hover). 3 "hovers" were the drone sitting
  still (ground / hand) and are excluded. Session 7 (11-10 23:17) had the camera tilted down: its heights are not
  usable. Battery state is **not logged** - only proxies (below).

## 1. What the drift does
| | value | how sure |
|---|---|---|
| **Takeoff lurch backwards** (auto climb, 1-3 s after takeoff) | 11-10 night: **-13 cm/s**; 11-10 late + 11-11: **-33 cm/s** | good (55 climbs) - bigger now than before |
| Hover drift after the lurch (trimmed), fwd / right | about **-1 / -0.5 cm/s**, scatter 1.5-3 cm/s | weak (8 takeoffs) |
| Own turn, untrimmed (11-09) | **+6.6 deg/s** (right) | ok (VO) |
| Own turn at the trim (11-10, 11-11) | **+0.8 to +2.7 deg/s** | ok, scatter 4.7 deg/s per takeoff |
| Yaw trim effect | +0.24 deg/s per trim byte (se 0.21) - fits flight_review's ~0.3 | weak |
| Roll trim effect | -0.33 cm/s per trim byte (se 0.15) - **sign opposite to "+ = right"?** | weak, check |

**Within a flight** (median of 2 s hover windows): the drift is concentrated in the first ~10 s:

| since takeoff | fwd cm/s | right | up | turn deg/s |
|---|---|---|---|---|
| 3-6 s | -6.1 | +3.8 | +7.8 | +1.5 |
| 6-10 s | -5.0 | +0.1 | -5.3 | +0.1 |
| 10-15 s | +2.3 | -0.2 | +1.6 | -0.9 |
| 15-31 s | ~0 | ~0 | ~0 | ~0 |

So: **a takeoff transient** (backward lurch + climb overshoot + settle), then a nearly steady hover. A fixed trim
cannot fix a transient - it needs a time-since-takeoff feed-forward or a fast estimator.

**Across takeoffs and sessions:** the takeoff lurch got **2.5x bigger** between the 11-10 00:37 session (-13 cm/s,
trim pitch 4-28) and the sessions since 11-10 23:17 (-33 cm/s, trim pitch 22-28). More forward trim did *not*
reduce it (correlation lurch vs trim pitch -0.35: more trim, more backward). Candidates, not separable with these
logs: the camera mount was moved (11-10 22:39 / 11-11 20:33: balance), prop / motor wear after the water crash,
battery, take-off surface. **One controlled test settles it:** same battery, same spot, 2 takeoffs with trim pitch
22, 2 with trim pitch 0 (tethered), compare the climb lurch in `takeoffs.csv`.

## 2. Battery: what the proxies show
Proxy = motor-on minutes earlier in the same session (takeoff -> "landing time over"), and takeoff index.

| session | motor min | climb drift vs min | climb rate vs min | approach speed vs min |
|---|---|---|---|---|
| 11-10 00:37 (34 takeoffs) | 14.0 | +0.1 cm/s/min (se 0.3) | -0.8 cm/s/min (se 0.9) | - |
| 11-10 23:17 (36) | 12.4 | -0.4 (0.5) | (tilted camera) | -0.3 (0.3) |
| 11-11 20:38 (14, fresh charge) | 5.1 | +4.6 (4.1) | -3.2 (4.7) | +1.1 (0.8) |

- **No measurable decline within a session**, even over 12-14 motor-minutes. Either the battery effect is small
  until the last minutes, or the battery was swapped/charged inside those sessions (14 motor-minutes is more than
  one HS420 pack is likely to give) - the logs cannot tell which.
- **Auto-climb rate** is the best battery proxy we have: +24 cm/s (11-10 00:37 session) vs **+40 cm/s** in the
  11-11 session that started on a fresh charge. Suggestive, not proof (other things changed too).
- The pooled approach fit (+1.0 cm/s faster per motor-minute) is confounded by tuning changes and start distance;
  per session it is flat. **Not evidence of a battery effect.**

**What needs the real battery byte:** any battery-vs-drift curve, "land on low battery", and telling swaps from
slow decline. The `93 04` status frame has it (`88 00 <fly-state> <battery> <flags> <xor> 55`, 0x5a = 90 on the
ground, scale unverified).

**Logging added 2026-11-11 (in `tools/drone_link.py`, so live_flight.py and live_flight_v2.py both get it; no
change in how the drone flies):**
- `status.csv` per flight: `t, fly_state, battery, flags, xor_ok, raw_hex` - a row whenever the `93 04` packet
  changes, at least every 5 s.
- At every takeoff, events.csv gets `tune trim_pitch/roll/yaw = v (at takeoff, logged by drone_link)` (the format
  flight_review and this tool already read: exact trims, no reconstruction) and
  `status at takeoff: fly_state F battery B flags 0x..`.
- Test: `python tests/run_all.py status_log` (fake drone sends the real captured packet -> battery 90).
- `drift_trend.py` fits the drifts against the battery byte once >= 8 takeoffs have it.
- Still to verify in flight: does fly_state change at takeoff, does the battery number fall over a session?

## 3. What we can predict
- **The takeoff lurch:** ~-30 cm/s for ~2 s (currently), repeatable within a session (sd ~10 cm/s). Predictable
  enough for a feed-forward correction.
- **Steady hover drift at the trim:** close to 0 (±1-3 cm/s, ±2-5 deg/s per takeoff) - but different every
  takeoff, so it must be *learned in flight*, not set once.
- **Battery:** not predictable from these logs; the climb rate is a usable health warning until the byte is logged.

## 4. Recommended auto-trim / adaptive estimator
1. **Bias states in the Kalman filter** (motion-model work): forward, sideways and turn drift as slowly varying
   states (random walk, ~1 cm/s and ~1 deg/s per 10 s). Learned from every tag sighting / VO update during the
   flight; the controller subtracts the current estimate = an auto-trim that adapts per takeoff.
2. **Start each takeoff from the last estimate** (saved per session), not from zero and not from a hand trim set
   days ago - the drift changes between sessions.
3. **Takeoff transient as its own feed-forward:** a learned profile over time-since-takeoff (pitch forward while
   the lurch happens, first ~3 s), updated after every takeoff from its measured climb drift (exponential average,
   e.g. 0.3 new / 0.7 old). Do not let the steady-state bias states absorb it.
4. **Battery term:** once `93 04` is logged, add the battery byte as an input to the bias / authority model; until
   then, warn when the auto-climb rate drops >30 % below the session's first takeoff.
5. **Measure it:** the scorecard metrics (landing error, sideways swing, heading swing) per takeoff, before/after.
