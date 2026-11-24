"""flight/ = the v2 flight stack for the HS420 (started 2026-11-11). Replaces live_flight.py's picture-by-picture
reactions with a running estimate of where the drone is.

  mission file -> planner -> controller -> supervisor -> link (tools/drone_link.py) -> drone
                               ^
          estimator <- sensors (laptop webcam: drone body; drone camera: tag on B; VO heading; battery)

Modules (each replaceable, all share the same world frame, see geometry.py):
  geometry.py   world frame, rotations, pinhole camera model
  config.py     settings (calib/flight.json + history), mission files (missions/*.json)
  model.py      drone motion model (used by the estimator AND the simulator)
  estimator.py  Kalman filter with delayed measurements (rewind + replay), gating, drift learning
  sensors.py    measurement builders: webcam body, drone-camera tag, VO heading (live and sim use the same ones)
  bodytrack.py  laptop-webcam drone body tracker (background difference, no marker)
  planner.py    site + route -> smooth reference path, pre-flight checks (visibility, box, no-fly, time)
  controller.py follows the reference; gains derived from the model's lag
  supervisor.py separate safety layer with the last word (time cap, box, operator zone, speed, battery, kill)
  pilot.py      the few states (GROUND, TAKEOFF, SETTLE, TRANSIT, FINAL, HOLD, LAND) gluing it together
  links.py      LiveLink (drone_link.Net) and SimLink - the same interface
  sim.py        simulator built from the model; batch runs + scorecard     (python flight/sim.py --runs 50)
  observe.py    standalone webcam tracker + estimator, no drone link       (python flight/observe.py)
  run.py        live window for real flights (observe-only by default)    (python flight/run.py)
  score.py      scorecard for v2 runs (sim and live)
Tests: python tests/flight/run_tests.py   (also part of python tests/run_all.py)
"""
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS = os.path.join(ROOT, "tools")
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)   # tools/drone_link.py, flight_control.py, tracker.py are reused unchanged
