"""Controller: follow the planned position + speed using the estimate, by inverting the drone model.

  position loop  v_cmd = v_ref + (p_ref - p) / T_pos                     (limited to max speed)
  speed loop     a_cmd = (v_cmd - v) / T_vel
  model inverse  the model says v relaxes to R(psi)(gain * stick + drift + lurch) in tau, so the stick that gives
                 a_cmd is   stick = (R(psi)^T (v + tau * a_cmd) - drift - lurch_ff * lurch) / gain
                 -> the learned drift is cancelled (feed-forward) instead of chased, and so is the takeoff lurch.
  heading        held at psi_ref (nose toward B): sideways moves use ROLL, so the path never curves.
  height         only corrected while the height sigma is small (else throttle 0 = the drone's altitude hold).
Gains come from the model's lag (gains()): T_vel >= tau + delay, T_pos >= 1.6 T_vel, T_yaw >= tau_yaw + 3 delay_yaw,
so the loop is never faster than the drone + video can follow (the old controller overshot and swung).
"""
import math

import numpy as np

from flight.geometry import rot2, wrap
from flight.model import DF, DL, DR, PSI, R, VX, VY, VZ, Z


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class Controller:
    def __init__(self, model, ccfg, z_trust_cm=20.0, max_speed=25.0, max_climb=20.0):
        self.m, self.c = model, ccfg
        self.z_trust, self.vmax, self.climb = z_trust_cm, max_speed, max_climb
        self.T_vel, self.T_pos, self.T_yaw, self.T_z = self.gains()

    def gains(self):
        m, c = self.m, self.c
        T_vel = max(c["vel_T_s"], m.tau_h + m.d_h)
        T_pos = max(c["pos_T_s"], 1.6 * T_vel)
        T_yaw = max(c["yaw_T_s"], m.tau_y + 3 * m.d_y)
        T_z = max(c["z_T_s"], m.tau_z + 3 * m.d_z)
        return T_vel, T_pos, T_yaw, T_z

    def step(self, x, P, p_ref, v_ref, psi_ref, t_air=None, horizontal=True, vertical=True, yaw=True, vmax=None):
        """-> power [throttle, yaw, roll, pitch], debug dict."""
        c, m = self.c, self.m
        out = [0.0, 0.0, 0.0, 0.0]
        dbg = {}
        psi = x[PSI]
        if horizontal:
            e = np.asarray(p_ref[:2]) - x[0:2]
            v_cmd = np.asarray(v_ref[:2]) + e / self.T_pos
            n, lim = np.linalg.norm(v_cmd), min(self.vmax, vmax or self.vmax)
            if n > lim:
                v_cmd *= lim / n
            v = x[VX:VY + 1]
            a_cmd = (v_cmd - v) / self.T_vel
            need = rot2(psi).T @ (v + m.tau_h * a_cmd)
            if c["learn_drift"]:
                need = need - x[DF:DL + 1]
            need[0] -= c["lurch_ff"] * m.lurch(t_air)
            pitch = need[0] / m.gf
            roll = -need[1] / m.gr
            lim = c["max_tilt_power"]
            out[3], out[2] = clamp(pitch, -lim, lim), clamp(roll, -lim, lim)
            dbg.update(v_cmd=v_cmd.tolist(), e=e.tolist())
        if yaw:
            ep = wrap(psi_ref - psi)
            r_cmd = clamp(ep / self.T_yaw, -math.radians(c["max_yaw_rate_deg_s"]), math.radians(c["max_yaw_rate_deg_s"]))
            r_target = r_cmd + 0.5 * (r_cmd - x[R])     # a little damping on the measured turn rate
            yp = -(r_target - (x[DR] if c["learn_drift"] else 0.0)) / m.gy
            out[1] = clamp(yp, -c["max_yaw_power"], c["max_yaw_power"])
            dbg["psi_err_deg"] = math.degrees(ep)
        if vertical:
            sz = math.sqrt(max(P[Z, Z], 0.0))
            trust = clamp((self.z_trust * 1.5 - sz) / (self.z_trust * 0.5), 0.0, 1.0)  # fades out above z_trust
            vz_cmd = clamp((v_ref[2] if len(v_ref) > 2 else 0.0) + (p_ref[2] - x[Z]) / self.T_z, -self.climb, self.climb)
            thr = (vz_cmd + 0.5 * (vz_cmd - x[VZ])) / m.gu
            out[0] = trust * clamp(thr, -c["max_thr_power"], c["max_thr_power"])
            dbg["z_trust"] = trust
        return out, dbg
