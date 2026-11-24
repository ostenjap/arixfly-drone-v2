"""Safety supervisor: a separate layer with the last word over every stick packet and button.

Two tiers (decided 2026-11-11):
  LAND  (0x02, sticks centred)  leaving the flight box (A's back edge .. B's far edge, +-box_side, ceiling), time cap,
        speed / climb too high, low battery, webcam lost, drone feed lost, operator L / Enter / Esc / focus lost
  ESTOP (0x04, motors off)      box + hard_margin, closer than camera_min to the laptop camera (= you), or the
        estimate is very uncertain WHILE moving toward the camera; operator Space
Command filter: near A's back edge no command may push the drone toward the camera (x < 0 direction).
Decisions latch: once LAND is called it stays; ESTOP overrides LAND.
observe mode (acts=False): estimate-based alarms are logged as "WOULD LAND / WOULD ESTOP" only (the estimate is
still being checked) - EXCEPT the operator zone (closer than camera_min to the laptop = you): that motor stop always
acts. Operator, time cap, battery and link alarms always act.
"""
import math

import numpy as np

from flight.estimator import Estimator
from flight.geometry import rot2
from flight.model import PITCH, PSI, ROLL, VX, VY, VZ, X, Y, Z


class Supervisor:
    def __init__(self, site, cfg, cam_pos, acts=True, log=print):
        self.s, self.c, self.cam_pos, self.acts, self.log = site, cfg["supervisor"], np.asarray(cam_pos, float), acts, log
        self.gf, self.gr = cfg["model"]["fwd_cm_s_per_power"], cfg["model"]["right_cm_s_per_power"]
        self.action, self.reason = None, ""
        self.would = set()
        self.fast_since = None

    def reset(self):
        self.action, self.reason, self.would, self.fast_since = None, "", set(), None

    def _call(self, action, reason, from_estimate, now, always=False):
        if from_estimate and not self.acts and not always:
            key = (action, reason.split(":")[0])
            if key not in self.would:
                self.would.add(key)
                self.log(f"SUPERVISOR WOULD {action.upper()} (observe mode, not acted on): {reason}")
            return
        if self.action == "estop" or (self.action == "land" and action == "land"):
            return
        self.action, self.reason = action, reason
        self.log(f"SUPERVISOR {action.upper()}: {reason}")

    def check(self, now, x, P, sticks, flying, t_air, health):
        """flying: motors on (after takeoff, before landed). health: dict with operator / link / battery flags.
        -> (sticks, action, reason); action None / 'land' / 'estop'."""
        c, s = self.c, self.s
        h = health
        if h.get("kill"):
            self._call("estop", "operator Space", False, now)
        if flying:
            for key, why in (("land_key", "operator L"), ("enter_land", "operator held Enter"), ("esc", "operator Esc"),
                             ("focus_lost", "window lost focus")):
                if h.get(key):
                    self._call("land", why, False, now)
            if t_air is not None and t_air > s.time_cap:
                self._call("land", f"time cap {s.time_cap:.0f} s", False, now)
            b = h.get("battery")
            if b is not None and b < c["battery_min"]:
                self._call("land", f"battery {b} < {c['battery_min']}", False, now)
            if h.get("webcam_age", 0.0) > c["webcam_lost_s"]:
                self._call("land", f"webcam lost {h['webcam_age']:.1f} s", False, now)
            if h.get("feed_age", 0.0) > c["feed_lost_s"]:
                self._call("land", f"drone feed lost {h['feed_age']:.1f} s", False, now)
            p, v = x[X:Z + 1], x[VX:VZ + 1]
            sig = Estimator.sigma_xy(P)
            grace = t_air is not None and t_air < c["takeoff_grace_s"]
            hm = c["hard_margin_cm"]
            ph = p.copy()
            if grace:
                ph[2] = min(ph[2], s.z_max - 1)    # auto climb: ceiling not checked yet
            if h.get("landing"):
                ph[2] = max(ph[2], s.z_min + 1)    # 0x02 descent: going down is the point (also onto the floor)
            dcam = math.hypot(p[0] - self.cam_pos[0], p[1] - self.cam_pos[1])
            if dcam < c["camera_min_cm"]:
                self._call("estop", f"{dcam:.0f} cm from the laptop camera (operator zone)", True, now, always=True)
            elif not s.in_box(ph, margin=hm):
                self._call("estop", f"far outside the box at {np.round(p).tolist()}", True, now)
            elif not s.in_box(ph):
                self._call("land", f"left the box at {np.round(p).tolist()}", True, now)
            for z in s.no_fly:
                if math.hypot(p[0] - z["center"][0], p[1] - z["center"][1]) < z["radius_cm"]:
                    self._call("land", f"in no-fly zone {z.get('name', '')}", True, now)
            if sig > c["uncertain_toward_cm"] and v[0] < -15.0:
                self._call("estop", f"position unknown (sigma {sig:.0f} cm) and moving toward the camera", True, now)
            sp = math.hypot(v[0], v[1])
            if sp > c["max_speed_cm_s"] or abs(v[2]) > c["max_climb_cm_s"] and not grace:
                self.fast_since = self.fast_since or now
                if now - self.fast_since > c["speed_s"]:
                    self._call("land", f"too fast ({sp:.0f} cm/s, climb {v[2]:+.0f} cm/s)", True, now)
            else:
                self.fast_since = None
            sticks = self.filter_toward_camera(list(sticks), x)
        if self.action:
            return [0.0] * 4, self.action, self.reason
        return sticks, None, ""

    def filter_toward_camera(self, sticks, x):
        """Near A's back edge: drop the part of the roll/pitch command that would push the drone toward -x."""
        if x[X] > self.s.x_back + self.c["guard_cm"]:
            return sticks
        body = np.array([self.gf * sticks[PITCH], -self.gr * sticks[ROLL]])
        w = rot2(x[PSI]) @ body
        if w[0] >= 0:
            return sticks
        w[0] = 0.0
        b = rot2(x[PSI]).T @ w
        sticks[PITCH], sticks[ROLL] = b[0] / self.gf, -b[1] / self.gr
        return sticks
