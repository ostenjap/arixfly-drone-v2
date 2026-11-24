"""Estimator: one extended Kalman filter for position, speed, heading and the drone's own drift.

Generic: every sensor hands in a Measurement (time it was TAKEN, value z, noise covariance Rm, prediction function
h(x) -> z). The filter does not care which sensor it is, so sensors can be added or dropped freely.
  - Late measurements (video is 0.1-0.3 s old) are fused at the time they were taken: the filter rewinds to that
    time and replays everything after it (out-of-sequence measurements by reprocessing, window `window_s`).
  - Gating: a measurement whose Mahalanobis distance is beyond the 99.9 % chi-square bound is rejected (wrong blob,
    false tag) and counted per sensor.
  - No measurement = no emergency: the model keeps predicting and the uncertainty grows. The pilot decides from
    sigma_xy() what to do (sure / unsure -> hold still / lost -> land).
Prediction uses model.Model with the commands and phases recorded in model.Timeline.
"""
import bisect
import math
from dataclasses import dataclass, field

import numpy as np

from flight.geometry import wrap
from flight.model import DF, DL, DR, N, PSI, R, SB, VX, VY, VZ, X, Y, Z

CHI2_999 = {1: 10.83, 2: 13.82, 3: 16.27, 4: 18.47}
MAX_DT = 0.04


@dataclass
class Measurement:
    t: float                      # when it was taken (monotonic s)
    name: str                     # sensor name (stats, logs)
    z: np.ndarray
    Rm: np.ndarray                # noise covariance
    h: object                     # h(x) -> predicted z (np.ndarray) or None if not predictable
    angles: tuple = ()            # indices of z that are angles (wrapped)
    gate: float = None            # chi-square bound (default 99.9 %)
    info: dict = field(default_factory=dict)
    accepted: bool = None
    d2: float = None


class Estimator:
    def __init__(self, model, timeline, cfg_noise, x0, P0, t0, window_s=1.0):
        self.m, self.tl, self.qn = model, timeline, cfg_noise
        self.window = window_s
        self.base = (t0, np.array(x0, float), np.array(P0, float))
        self.events = []       # measurements after base time, sorted by t
        self.cps = []          # state after each event: (t, x, P)
        self.stats = {}        # name -> {"ok", "rejected", "streak", "late"}
        self.t_now = t0

    # ------------------------------------------------------------------ public
    def reset(self, x0, P0, t0):
        self.base, self.events, self.cps, self.t_now = (t0, np.array(x0, float), np.array(P0, float)), [], [], t0

    def add(self, meas):
        st = self.stats.setdefault(meas.name, {"ok": 0, "rejected": 0, "streak": 0, "late": 0})
        if meas.t < self.base[0]:
            st["late"] += 1
            return False
        ts = [e.t for e in self.events]
        k = bisect.bisect_right(ts, meas.t)
        self.events.insert(k, meas)
        del self.cps[k:]
        t, x, P = self.cps[k - 1] if k > 0 else self.base
        for e in self.events[k:]:
            x, P = self._predict(x, P, t, e.t)
            t = e.t
            x, P = self._update(x, P, e)
            self.cps.append((t, x, P))
        if meas.accepted:
            st["ok"] += 1
            st["streak"] = 0
        else:
            st["rejected"] += 1
            st["streak"] += 1
        self.t_now = max(self.t_now, meas.t)
        self._prune()
        return meas.accepted

    def inflate(self, t, idx, sigma):
        """Forget what is known about state idx (set its sigma), e.g. the blob size scale when the props stop."""
        x, P = self.at(t)
        P = P.copy()
        P[idx, :] = 0.0
        P[:, idx] = 0.0
        P[idx, idx] = sigma * sigma
        self.reset(x.copy(), P, t)

    def at(self, t):
        """(x, P) predicted to time t (nothing is stored)."""
        t0, x, P = self.cps[-1] if self.cps else self.base
        if t > t0:
            x, P = self._predict(x.copy(), P, t0, t)
        self.t_now = max(self.t_now, t)
        return x.copy(), P

    def advance(self, t):
        """Move the base forward when no measurements come (keeps the replay cheap)."""
        self.t_now = max(self.t_now, t)
        self._prune()
        if not self.events and t - self.base[0] > self.window:
            tb = t - self.window
            x, P = self._predict(self.base[1], self.base[2], self.base[0], tb)
            self.base = (tb, x, P)

    @staticmethod
    def sigma_xy(P):
        return math.sqrt(max(np.linalg.eigvalsh(P[X:Y + 1, X:Y + 1])))

    @staticmethod
    def sigma_z(P):
        return math.sqrt(max(P[Z, Z], 0.0))

    # ------------------------------------------------------------------ internals
    def _prune(self):
        lim = self.t_now - self.window
        while self.events and self.events[0].t < lim:
            self.events.pop(0)
            self.base = self.cps.pop(0)

    def _Q(self, t, dt):
        q = self.qn
        phase, t_ph = self.tl.phase_at(t)
        k = q["takeoff_factor"] if (phase in ("landing", "falling") or (phase == "air" and t_ph < 3.5)) else 1.0
        Q = np.zeros((N, N))
        if phase == "ground":
            Q[X, X] = Q[Y, Y] = Q[Z, Z] = 0.05 * dt
            Q[PSI, PSI] = math.radians(0.2) ** 2 * dt
            Q[DF, DF] = Q[DL, DL] = (0.2 * q["drift_walk_cm_s"]) ** 2 * dt
            Q[DR, DR] = (0.2 * math.radians(q["turn_walk_deg_s"])) ** 2 * dt
            Q[SB, SB] = (0.2 * q["size_walk"]) ** 2 * dt
            return Q
        Q[X, X] = Q[Y, Y] = Q[Z, Z] = 1.0 * dt
        Q[VX, VX] = Q[VY, VY] = (k * q["accel_h_cm_s2"]) ** 2 * dt
        Q[VZ, VZ] = (k * q["accel_z_cm_s2"]) ** 2 * dt
        Q[R, R] = (k * math.radians(q["yaw_accel_deg_s2"])) ** 2 * dt
        Q[DF, DF] = Q[DL, DL] = (k * q["drift_walk_cm_s"]) ** 2 * dt
        Q[DR, DR] = (math.radians(q["turn_walk_deg_s"])) ** 2 * dt
        Q[SB, SB] = q["size_walk"] ** 2 * dt + (q["size_takeoff"] ** 2 * dt if phase == "air" and t_ph < 1.0 else 0.0)
        return Q

    def _predict(self, x, P, t0, t1):
        t = t0
        while t < t1 - 1e-9:
            dt = min(MAX_DT, t1 - t)
            x, F = self.m.step(x, t, dt, self.tl)
            P = F @ P @ F.T + self._Q(t, dt)
            t += dt
        x[PSI] = wrap(x[PSI])
        return x, P

    def _update(self, x, P, e):
        zh = e.h(x)
        if zh is None:
            e.accepted, e.d2 = False, float("inf")
            return x, P
        y = np.asarray(e.z, float) - zh
        for i in e.angles:
            y[i] = wrap(y[i])
        H = self._jac(e, x, zh)
        S = H @ P @ H.T + e.Rm
        try:
            Si = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            e.accepted, e.d2 = False, float("inf")
            return x, P
        d2 = float(y @ Si @ y)
        e.d2 = d2
        if d2 > (e.gate or CHI2_999.get(len(y), 20.0)):
            e.accepted = False
            return x, P
        K = P @ H.T @ Si
        x = x + K @ y
        x[PSI] = wrap(x[PSI])
        IKH = np.eye(N) - K @ H
        P = IKH @ P @ IKH.T + K @ e.Rm @ K.T
        e.accepted = True
        return x, P

    @staticmethod
    def _jac(e, x, zh, eps=None):
        H = np.zeros((len(zh), N))
        for i in range(N):
            d = 1e-4 if i in (PSI, R, DR, SB) else 0.01
            xp = x.copy()
            xp[i] += d
            zp = e.h(xp)
            if zp is None:
                continue
            dz = np.asarray(zp) - zh
            for j in e.angles:
                dz[j] = wrap(dz[j])
            H[:, i] = dz / d
        return H


def initial(p0, psi0=0.0, sigma_p=5.0, sigma_psi_deg=5.0, drift=(0.0, 0.0), sigma_drift=8.0, sigma_turn_deg=3.0):
    """Start state on a table: position known to sigma_p, still, heading psi0."""
    x = np.zeros(N)
    x[X:Z + 1] = p0
    x[PSI] = psi0
    x[DF], x[DL] = drift
    P = np.diag([sigma_p ** 2, sigma_p ** 2, 2.0 ** 2, 1, 1, 1, math.radians(sigma_psi_deg) ** 2,
                 math.radians(1) ** 2, sigma_drift ** 2, sigma_drift ** 2, math.radians(sigma_turn_deg) ** 2,
                 0.03 ** 2])
    return x, P
