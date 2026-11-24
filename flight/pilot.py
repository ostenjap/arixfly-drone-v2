"""Pilot: the few states of a v2 flight, driven by how sure the estimator is. Shared by the live window (run.py)
and the simulator (sim.py) - both call the same add_*() and step().

  GROUND -> TAKEOFF (0x01, auto climb, sticks centred during the pulse, then hold over A against the lurch)
         -> SETTLE (hover over A at cruise height until still and sure)
         -> TRANSIT (follow the planned path to above B)  -> FINAL (slow over B, down to land height)
         -> LAND (0x02) -> LANDED (result measured on the ground: error to B's centre)
  any air state, position sigma > unsure  -> HOLD (roll/pitch centred - no brake toward you -, heading + height held)
  HOLD longer than hold_max_s, or sigma > lost -> LAND.   Supervisor LAND / ESTOP override everything.
Modes: observe = estimator only, sticks always centred, operator takes off / lands (supervisor alarms logged only);
       hold    = takeoff, settle and hover over A for states.hold_s, then land;
       mission = the full route.
"""
import math

import numpy as np

from flight.controller import Controller, clamp
from flight.estimator import Estimator, initial
from flight.model import PSI, SB, VX, VY, VZ, X, Y, Z, Model, Timeline
from flight.planner import PathRef, check_plan
from flight.sensors import DroneCamSensor, VOSensor, WebcamSensor
from flight.supervisor import Supervisor

TAKEOFF, LAND, ESTOP = 0x01, 0x02, 0x04
AIR = ("TAKEOFF", "SETTLE", "TRANSIT", "FINAL", "HOLD", "FLYING", "LAND")


class Pilot:
    def __init__(self, site, cfg, mode, cam, t0, log=print):
        assert mode in ("observe", "hold", "mission")
        self.site, self.cfg, self.mode, self.log = site, cfg, mode, log
        self.tl = Timeline(t0)
        self.model = Model(cfg["model"], ground_z=site.ground_z)
        self.est = Estimator(self.model, self.tl, cfg["noise"], *initial(site.start, site.heading), t0,
                             cfg["estimator"]["window_s"])
        self.ctrl = Controller(self.model, cfg["control"], cfg["estimator"]["z_trust_cm"], site.max_speed * 1.3)
        self.path = PathRef(site)
        acts = mode != "observe" or cfg["supervisor"]["acts_in_observe"]
        self.sup = Supervisor(site, cfg, cam.pos if cam is not None else [-1e4, 0, 0], acts, log)
        self.cam = cam
        self.webcam = WebcamSensor(cam, cfg["webcam"]) if cam is not None else None
        # one drone-camera sensor per tag in the mission (tag id -> sensor): any tag the drone camera sees helps
        self.dronecams = ({t["id"]: DroneCamSensor(cfg["dronecam"], t["center"], self.tl) for t in site.tags.values()}
                          if cfg["dronecam"]["use"] else {})
        self.vo = VOSensor(cfg["vo"]) if cfg["vo"]["use"] else None
        self.plan_errors, self.plan_warnings, self.plan_info = check_plan(site, cfg, cam, mode if mode != "observe" else "hold")
        self.e = cfg["estimator"]
        self.st = cfg["states"]
        self.state, self.t_state, self.prev = "GROUND", t0, None
        self.sticks = [0.0] * 4
        self.t_takeoff = self.t_last = None
        self.ground_sizes, self.last_det, self.t_det = [], None, None
        self.ok_since = None
        self.result = None
        self.land_reason = ""
        self.trace = []      # rows for estimate.csv
        self.ref = (site.start.copy(), np.zeros(3))
        self.settle_ref = self.path.settle_point
        self.hover_err = []
        self.n_resets = 0
        self.down_since = None

    # ------------------------------------------------------------------ sensors in
    def add_webcam(self, det, t_taken):
        if self.webcam is None or det is None:
            return None
        self.last_det, self.t_det = det, t_taken
        if self.state == "GROUND":
            self.ground_sizes = (self.ground_sizes + [det["size"]])[-30:]
        m = self.webcam.measurement(det, t_taken)
        self.est.add(m)
        st = self.est.stats["webcam"]
        if self.state in AIR and st["streak"] >= 12:
            x, P = self.est.at(t_taken)
            if self.est.sigma_xy(P) > self.e["unsure_cm"] or st["streak"] >= 30:
                self.reset_from_webcam(det, t_taken, x, P)
        return m

    def reset_from_webcam(self, det, t, x, P):
        p = self.webcam.fix(det, x[SB])
        x = x.copy()
        x[X:Z + 1] = p
        x[VX:VZ + 1] = 0.0
        P = P.copy()
        P[:6, :] = 0.0
        P[:, :6] = 0.0
        for i, s in ((X, 15.0), (Y, 15.0), (Z, 10.0), (VX, 30.0), (VY, 30.0), (VZ, 20.0)):
            P[i, i] = s * s
        self.est.reset(x, P, t)
        self.est.stats["webcam"]["streak"] = 0
        self.n_resets += 1
        self.log(f"ESTIMATE RESET from the webcam (12+ rejected in a row): {np.round(p).tolist()}")

    def add_dronecam(self, r, t_taken):
        """r: tag dict (x, y, z cm in drone-camera coordinates, id)."""
        sensor = self.dronecams.get(r.get("id")) if r else None
        if sensor is None:
            return None
        m = sensor.measurement(r, t_taken)
        self.est.add(m)
        return m

    def add_vo(self, heading_deg, t_taken):
        if self.vo is None:
            return None
        t_air = None if self.t_takeoff is None else t_taken - self.t_takeoff
        m = self.vo.measurement(heading_deg, t_taken, t_air)
        self.est.add(m)
        return m

    def start_vo(self, t):
        """Call when a new VO starts (heading 0 = current heading)."""
        if self.vo:
            x, _ = self.est.at(t)
            self.vo.start(x[PSI])

    # ------------------------------------------------------------------ readiness
    def calibrate_size(self):
        if self.webcam is None or len(self.ground_sizes) < 5:
            return None
        s = float(np.median(self.ground_sizes))
        return self.webcam.calibrate_size(s, self.site.start)

    def not_ready(self, now):
        why = []
        if self.plan_errors:
            why.append("plan: " + self.plan_errors[0])
        if self.webcam is not None:
            if self.t_det is None or now - self.t_det > 0.5:
                why.append("webcam does not see the drone")
            elif len(self.ground_sizes) < 5:
                why.append("measuring the drone on table A ...")
            else:
                q = self.cam.project(self.site.start + np.array([0, 0, self.webcam.up]))
                if q is None or math.hypot(self.last_det["u"] - q[0], self.last_det["v"] - q[1]) > 0.08 * self.cam.w:
                    why.append("the blob is not where table A should be (camera moved? recalibrate)")
        return why

    # ------------------------------------------------------------------ the loop
    def set_state(self, now, s, why=""):
        if s != self.state:
            self.log(f"STATE {self.state} -> {s}" + (f": {why}" if why else ""))
            self.prev, self.state, self.t_state = self.state, s, now

    def step(self, now, ops, health):
        """ops: takeoff (operator finished the Enter hold) / land_key / enter_land / esc / focus_lost / kill.
        health: webcam_age, feed_age, battery. -> dict(sticks, button, state, x, P, sig, ref)."""
        dt = 0.0 if self.t_last is None else clamp(now - self.t_last, 0.0, 0.2)
        self.t_last = now
        self.est.advance(now)
        x, P = self.est.at(now)
        sig, sz = self.est.sigma_xy(P), self.est.sigma_z(P)
        button = None
        s = self.state
        t_air = None if self.t_takeoff is None else now - self.t_takeoff
        cmd = [0.0] * 4
        flags = dict(horizontal=False, vertical=False, yaw=False)
        p_ref, v_ref = self.ref
        if s == "GROUND":
            if ops.get("takeoff"):
                why = self.not_ready(now)
                if why:
                    self.log("TAKEOFF REFUSED: " + "; ".join(why))
                else:
                    D = self.calibrate_size()
                    if D:
                        self.log(f"webcam drone size calibrated on table A: {D:.1f} cm")
                    self.t_takeoff, t_air = now, 0.0
                    self.tl.phase(now, "air")
                    self.sup.reset()
                    button = TAKEOFF
                    self.set_state(now, "TAKEOFF", f"mode {self.mode}")
        elif s == "TAKEOFF":
            if self.mode == "observe":
                self.set_state(now, "FLYING", "observe only: sticks stay centred")
            else:
                if t_air > self.st["pulse_s"]:   # damp the speed (no pull back to the start: it may lie toward you)
                    p_ref, v_ref = x[X:Z + 1].copy(), np.zeros(3)
                    flags = dict(horizontal=sig < self.e["unsure_cm"], vertical=False, yaw=True)
                if t_air >= self.st["takeoff_s"]:
                    self.set_state(now, "SETTLE")
        if self.state in ("SETTLE", "TRANSIT", "FINAL") and (sig > self.e["unsure_cm"]):
            self.set_state(now, "HOLD", f"position sigma {sig:.0f} cm > {self.e['unsure_cm']:.0f}")
        s = self.state
        if s == "SETTLE" and self.t_state == now:
            self.settle_ref = self.settle_point(x)
        elif s == "SETTLE" and self.mode == "mission":   # ratchet: never back along the path (toward you)
            q = self.settle_point(x)
            if self.path.project(q) > self.path.project(self.settle_ref):
                self.settle_ref = q
        if s == "SETTLE":
            p_ref, v_ref = self.settle_ref, np.zeros(3)
            flags = dict(horizontal=True, vertical=True, yaw=True)
            err = np.linalg.norm(x[X:Z + 1] - p_ref)
            if self.mode == "hold":
                if now - self.t_state > 1.5:
                    self.hover_err.append(err)
                if now - self.t_state >= self.st["hold_s"]:
                    self.hover_report()
                    self.enter_land(now, f"hold time {self.st['hold_s']:.0f} s over")
            else:
                still = (err < self.st["settle_tol_cm"] and np.linalg.norm(x[VX:VZ + 1]) < self.st["settle_speed_cm_s"]
                         and sig < self.e["sure_cm"])
                self.ok_since = (self.ok_since or now) if still else None
                if self.ok_since and now - self.ok_since >= self.st["settle_hold_s"]:
                    self.path.reset(x[X:Z + 1])
                    self.ok_since = None
                    self.set_state(now, "TRANSIT", f"settled at {np.round(x[X:Z + 1]).tolist()}")
                elif now - self.t_state > self.st["settle_max_s"]:
                    self.enter_land(now, f"could not settle in {self.st['settle_max_s']:.0f} s (error {err:.0f} cm)")
        elif s == "TRANSIT":
            p_ref, v_ref, done = self.path.step(dt, x[X:Z + 1])
            flags = dict(horizontal=True, vertical=True, yaw=True)
            if done and np.linalg.norm(x[X:Y + 1] - self.path.above_goal[:2]) < 15.0:
                self.set_state(now, "FINAL", "above the goal")
        elif s == "FINAL":
            g = self.path.final_point
            z_ref = max(g[2], self.path.above_goal[2] - 10.0 * (now - self.t_state))
            p_ref, v_ref = np.array([g[0], g[1], z_ref]), np.array([0.0, 0.0, -10.0 if z_ref > g[2] else 0.0])
            flags = dict(horizontal=True, vertical=True, yaw=True)
            e_xy = np.linalg.norm(x[X:Y + 1] - g[:2])
            ok = (e_xy < self.st["final_tol_cm"] and np.linalg.norm(x[VX:VY + 1]) < self.st["final_speed_cm_s"]
                  and abs(x[Z] - g[2]) < 6.0 and sig < self.e["sure_cm"])
            self.ok_since = (self.ok_since or now) if ok else None
            if self.ok_since and now - self.ok_since >= self.st["final_hold_s"]:
                self.enter_land(now, f"arrived ({e_xy:.0f} cm from the goal, sigma {sig:.0f} cm)")
            elif now - self.t_state > self.st["final_max_s"] and e_xy < 2 * self.st["final_tol_cm"]:
                self.enter_land(now, f"final approach timeout ({e_xy:.0f} cm from the goal)")
            elif now - self.t_state > 2 * self.st["final_max_s"]:
                self.enter_land(now, f"final approach timeout, landing where it is ({e_xy:.0f} cm from the goal)")
        elif s == "HOLD":
            flags = dict(horizontal=False, vertical=sz < self.e["z_trust_cm"], yaw=True)
            p_ref = np.array([x[X], x[Y], self.ref[0][2]])
            back = (self.e["sure_cm"] + self.e["unsure_cm"]) / 2
            self.ok_since = (self.ok_since or now) if sig < back else None
            if self.ok_since and now - self.ok_since > 0.5:
                self.ok_since = None
                if self.prev == "TRANSIT":
                    self.path.reset(x[X:Z + 1])
                self.set_state(now, self.prev if self.prev in ("SETTLE", "TRANSIT", "FINAL") else "SETTLE",
                               f"sure again (sigma {sig:.0f} cm)")
            elif sig > self.e["lost_cm"]:
                self.enter_land(now, f"position lost (sigma {sig:.0f} cm)")
            elif now - self.t_state > self.st["hold_max_s"]:
                self.enter_land(now, f"unsure for {self.st['hold_max_s']:.0f} s (sigma {sig:.0f} cm)")
        elif s == "FLYING":
            if ops.get("enter_land") or ops.get("land_key"):
                self.enter_land(now, "operator")
        if self.state == "LAND":
            down = x[Z] - self.site.ground_z(x[X], x[Y]) < 3.0 and abs(x[VZ]) < 10.0
            self.down_since = (self.down_since or now) if down else None
            if self.down_since and now - self.down_since > 0.3 and self.tl.pn[-1] == "landing":
                self.tl.phase(now, "ground")       # touchdown: freeze; the blob shrinks when the props stop
                self.est.inflate(now, SB, 0.15)
                x, P = self.est.at(now)
                self.log(f"touchdown at {np.round(x[X:Z + 1]).tolist()}")
            if now - self.t_state >= self.st["land_s"]:
                if self.tl.pn[-1] != "ground":
                    self.tl.phase(now, "ground")
                    self.est.inflate(now, SB, 0.15)
                self.set_state(now, "LANDED", "landing time over")
            elif self.cfg["control"]["control_during_land"]:
                flags = dict(horizontal=sig < self.e["unsure_cm"], vertical=False, yaw=True)
        if self.state == "LANDED" and self.result is None and now - self.t_state > 1.5:
            self.landing_result(x, P)
        if self.state == "KILLED" and now - self.t_state > 1.5 and self.tl.pn[-1] == "falling":
            self.tl.phase(now, "ground")
        self.ref = (p_ref, v_ref)
        if any(flags.values()) and self.mode != "observe":
            vmax = float(np.linalg.norm(v_ref[:2])) + 6.0 if self.state == "TRANSIT" else None  # never outrun the plan
            cmd, _ = self.ctrl.step(x, P, p_ref, v_ref, self.site.heading, t_air, vmax=vmax, **flags)
        # slew like a key press, then the supervisor has the last word
        step = dt / self.cfg["control"]["slew_s"] if dt else 1.0
        self.sticks = [v + clamp(c - v, -step, step) for v, c in zip(self.sticks, cmd)]
        flying = self.state in AIR
        sticks, action, reason = self.sup.check(now, x, P, self.sticks, flying, t_air,
                                                dict(health, landing=self.state == "LAND", **ops))
        if action == "estop" and self.state != "KILLED":
            button = ESTOP
            self.tl.phase(now, "falling")
            self.set_state(now, "KILLED", reason)
            self.land_reason = "ESTOP: " + reason
        elif action == "land" and self.state in AIR and self.state != "LAND":
            self.enter_land(now, "supervisor: " + reason)
        if self.state in ("LAND", "LANDED", "KILLED", "GROUND"):
            if not (self.state == "LAND" and self.cfg["control"]["control_during_land"]):
                sticks = [0.0] * 4
        if self._pending_land:
            self._pending_land = False
            if button is None and self.state == "LAND":
                button = LAND
        self.sticks = list(sticks)
        self.tl.command(now, self.sticks)
        row = dict(t=now, state=self.state, x=x, sig=sig, sz=sz, sticks=list(self.sticks), ref=p_ref, button=button)
        self.trace.append(row)
        return dict(sticks=self.sticks, button=button, state=self.state, x=x, P=P, sig=sig, sz=sz, ref=p_ref)

    _pending_land = False

    def settle_point(self, x):
        """Hover where the takeoff left the drone (on the path line, at cruise height) - never fly back to the start
        first (after the lurch the start may lie toward the operator). Hold mode: right where it is."""
        p = x[X:Z + 1]
        if self.mode == "hold":
            return np.array([p[0], p[1], self.site.start[2] + self.site.cruise])
        q, _ = self.path.point(self.path.project(p))
        return q

    def enter_land(self, now, why):
        if self.state in ("LAND", "LANDED", "KILLED"):
            return None
        self.land_reason = why
        self.tl.phase(now, "landing")
        self.set_state(now, "LAND", why)
        self._pending_land = True
        return LAND

    def hover_report(self):
        if self.hover_err:
            e = np.array(self.hover_err)
            self.log(f"HOVER RESULT: error to the hold point median {np.median(e):.0f} cm, 95 % {np.percentile(e, 95):.0f}"
                     f" cm, max {e.max():.0f} cm")

    def landing_result(self, x, P):
        g = self.site.goal
        err = float(np.linalg.norm(x[X:Y + 1] - g[:2]))
        on = self.site.ground_z(x[X], x[Y])
        tol = 15.0
        ok = self.mode == "mission" and err <= tol and abs(on - g[2]) < 1.0 and not self.land_reason.startswith(
            ("supervisor", "ESTOP", "operator"))
        self.result = {"error_cm": err, "success": ok, "x": float(x[X]), "y": float(x[Y]),
                       "sigma_cm": self.est.sigma_xy(P), "reason": self.land_reason}
        self.log(f"V2 RESULT: landed at x {x[X]:.0f} y {x[Y]:+.0f} cm (sigma {self.est.sigma_xy(P):.0f}), error "
                 f"{err:.0f} cm to {self.site.route[-1]}'s centre (tol {tol:.0f}) -> "
                 f"{'SUCCESS' if ok else 'MISS'} ({self.land_reason})")
