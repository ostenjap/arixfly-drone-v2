"""Drone motion model for the estimator (predict) and the simulator (truth), power units in, cm / rad out.

Per horizontal body axis a first-order response (calib/motion_model.json form):
    v_world -> R(psi) (gain * stick(t - delay) + lurch(t) + drift_body)   with time constant tau
height:  vz -> up_gain * throttle(t - delay) (+ auto climb after takeoff, - descent after 0x02)
heading: r  -> -yaw_gain * yaw(t - delay) + own_turn                      (yaw + = right = psi smaller)
The drift (forward / left cm/s) and own turn (rad/s) are STATES: the filter learns them during each flight.

State vector x (12): X Y Z (cm), VX VY VZ (cm/s), PSI (rad), R (rad/s), DF DL (cm/s body drift), DR (rad/s),
SB = log scale of the drone's blob size in the laptop webcam (spinning props look bigger than the parked drone).
During the auto climb the sticks are assumed to do nothing (sticks_during_climb false): the 2026-11-10/11 lurch did
not shrink with more forward trim (notes/drift_battery.md). Phase 1 / observe flights should check this.
Phases (Timeline): ground (frozen), air (since takeoff), landing (0x02), falling (0x04).
"""
import bisect
import math

import numpy as np

from flight.geometry import rot2

X, Y, Z, VX, VY, VZ, PSI, R, DF, DL, DR, SB = range(12)
N = 12
THR, YAW, ROLL, PITCH = range(4)


class Timeline:
    """Stick commands and flight phases over time (both live and sim append to it as they happen)."""

    def __init__(self, t0=0.0):
        self.ct, self.cu = [t0], [(0.0, 0.0, 0.0, 0.0)]
        self.pt, self.pn = [t0], ["ground"]

    def command(self, t, u):
        u = tuple(float(v) for v in u)
        if u != self.cu[-1]:
            if t <= self.ct[-1]:
                self.cu[-1] = u
            else:
                self.ct.append(t)
                self.cu.append(u)
        self._trim(t)

    def phase(self, t, name):
        if name != self.pn[-1]:
            self.pt.append(max(t, self.pt[-1]))
            self.pn.append(name)

    def u(self, t):
        return self.cu[max(0, bisect.bisect_right(self.ct, t) - 1)]

    def phase_at(self, t):
        i = max(0, bisect.bisect_right(self.pt, t) - 1)
        return self.pn[i], t - self.pt[i]

    def takeoff_time(self, t):
        """Time of the latest takeoff at or before t (None if none)."""
        for i in range(bisect.bisect_right(self.pt, t) - 1, -1, -1):
            if self.pn[i] == "air":
                return self.pt[i]
            if self.pn[i] == "ground":
                return None
        return None

    def _trim(self, t, keep_s=10.0):
        while len(self.ct) > 2 and self.ct[1] < t - keep_s:
            del self.ct[0], self.cu[0]


class Model:
    def __init__(self, p, ground_z=None):
        self.p = dict(p)
        self.gf, self.gr = p["fwd_cm_s_per_power"], p["right_cm_s_per_power"]
        self.gu = p["up_cm_s_per_power"]
        self.gy = math.radians(p["yaw_deg_s_per_power"])
        self.tau_h, self.tau_z, self.tau_y = p["tau_h_s"], p["tau_z_s"], p["tau_yaw_s"]
        self.d_h, self.d_z, self.d_y = p["delay_h_s"], p["delay_z_s"], p["delay_yaw_s"]
        self.ground_z = ground_z or (lambda x, y: 0.0)
        self.lurch_scale = 1.0   # the estimator may use a different lurch than the sim's truth

    def lurch(self, t_air):
        p = self.p
        if t_air is None or not (p["lurch_start_s"] <= t_air <= p["lurch_start_s"] + p["lurch_s"]):
            return 0.0
        return p["lurch_cm_s"] * self.lurch_scale

    def in_climb(self, t_air):
        p = self.p
        return t_air is not None and t_air < p["takeoff_climb_start_s"] + p["takeoff_climb_s"]

    def climb(self, t_air):
        p = self.p
        if t_air is None:
            return 0.0
        s = p["takeoff_climb_start_s"]
        return p["takeoff_climb_cm_s"] if s <= t_air <= s + p["takeoff_climb_s"] else 0.0

    def deriv(self, x, t, tl, want_A=True):
        phase, _ = tl.phase_at(t)
        A = np.zeros((N, N)) if want_A else None
        xd = np.zeros(N)
        if phase == "ground":
            return xd, A, phase
        t_take = tl.takeoff_time(t)
        t_air = None if t_take is None else t - t_take
        uh, uz, uy = tl.u(t - self.d_h), tl.u(t - self.d_z), tl.u(t - self.d_y)
        if not self.p.get("sticks_during_climb", False) and self.in_climb(t_air):
            uh = uz = uy = (0.0, 0.0, 0.0, 0.0)
        psi = x[PSI]
        Rm = rot2(psi)
        if phase == "falling":
            b = np.zeros(2)
            vz_t = -self.p["fall_cm_s"]
        else:
            b = np.array([self.gf * uh[PITCH] + self.lurch(t_air), -self.gr * uh[ROLL]])
            vz_t = -self.p["land_cm_s"] if phase == "landing" else self.gu * uz[THR] + self.climb(t_air)
        bd = b + x[DF:DL + 1]
        vt = Rm @ bd
        xd[X], xd[Y], xd[Z] = x[VX], x[VY], x[VZ]
        xd[VX:VY + 1] = (vt - x[VX:VY + 1]) / self.tau_h
        xd[VZ] = (vz_t - x[VZ]) / self.tau_z
        xd[PSI] = x[R]
        r_t = -self.gy * uy[YAW] + x[DR]
        xd[R] = (r_t - x[R]) / self.tau_y
        if want_A:
            A[X, VX] = A[Y, VY] = A[Z, VZ] = 1.0
            A[VX, VX] = A[VY, VY] = -1.0 / self.tau_h
            c, s = math.cos(psi), math.sin(psi)
            dR = np.array([[-s, -c], [c, -s]])
            A[VX:VY + 1, PSI] = dR @ bd / self.tau_h
            A[VX:VY + 1, DF:DL + 1] = Rm / self.tau_h
            A[VZ, VZ] = -1.0 / self.tau_z
            A[PSI, R] = 1.0
            A[R, R] = -1.0 / self.tau_y
            A[R, DR] = 1.0 / self.tau_y
        return xd, A, phase

    def step(self, x, t, dt, tl, want_F=True):
        """Euler step (dt <= ~0.05 s) -> (x_new, F)."""
        xd, A, phase = self.deriv(x, t, tl, want_F)
        if phase == "ground":
            xn = x.copy()
            xn[VX:VZ + 1] = 0.0
            xn[R] = 0.0
            F = None
            if want_F:
                F = np.eye(N)
                for i in (VX, VY, VZ, R):
                    F[i, i] = 0.0
            return xn, F
        xn = x + dt * xd
        F = np.eye(N) + dt * A if want_F else None
        if phase in ("landing", "falling"):
            g = self.ground_z(xn[X], xn[Y])
            if xn[Z] < g:
                xn[Z], xn[VZ] = g, 0.0
        return xn, F
