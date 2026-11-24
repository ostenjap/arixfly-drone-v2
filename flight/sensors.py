"""Sensor -> Measurement builders. The live window and the simulator feed the SAME builders (webcam blob,
drone-camera tag dict, VO heading), so the simulator tests the real measurement maths.

  WebcamSensor   laptop webcam sees the drone body: z = [u, v, size] (pixel centre + sqrt(blob area)).
                 u, v pin the drone onto a ray (sharp sideways + height); size ~ f * D / depth gives a weak depth.
                 The effective size D is re-measured on table A before every takeoff (known distance there).
  DroneCamSensor drone camera sees the tag on B: z = tag centre in camera coordinates (x right, y down, z ahead).
                 Corrected for the camera tilt, which grows with forward pitch (the tag looks higher then).
  VOSensor       visual-odometry heading of the drone camera (tools/tracker.YawVO, + = right): z = [psi].
"""
import math

import numpy as np

from flight.estimator import Measurement
from flight.geometry import look_rotation
from flight.model import PITCH, PSI, SB, Z


class WebcamSensor:
    def __init__(self, cam, cfg):
        self.cam = cam                     # geometry.Camera at the tracker's processing width
        self.D = float(cfg["drone_size_cm"])
        self.sp, self.sr = float(cfg["sigma_px"]), float(cfg["size_rel_sigma"])
        self.up = float(cfg.get("center_above_cm", 4.0))   # blob centre above the state point (drone feet)
        self.latency = float(cfg["latency_s"])

    def predict(self, x):
        p = np.array([x[0], x[1], x[Z] + self.up])
        q = self.cam.project(p)
        if q is None:
            return None
        return np.array([q[0], q[1], self.cam.f * self.D * math.exp(x[SB]) / q[2]])

    def measurement(self, det, t_taken):
        """det: dict with u, v, size (px at the camera's width), optional 'blur' (px)."""
        s = float(det["size"])
        sp = self.sp + float(det.get("blur", 0.0)) * 0.5
        Rm = np.diag([sp ** 2, sp ** 2, (self.sr * s) ** 2 + 1.0])
        return Measurement(t_taken, "webcam", np.array([det["u"], det["v"], s], float), Rm, self.predict,
                           info={"u": det["u"], "v": det["v"], "size": s})

    def fix(self, det, size_log_scale=0.0):
        """3-D position from one detection (ray + size), for (re)initialising."""
        depth = self.cam.f * self.D * math.exp(size_log_scale) / max(float(det["size"]), 1.0)
        q = np.array([(det["u"] - self.cam.cx) / self.cam.f * depth, (det["v"] - self.cam.cy) / self.cam.f * depth,
                      depth])
        p = self.cam.R @ q + self.cam.pos
        p[2] -= self.up
        return p

    def calibrate_size(self, size_px, p_true):
        """Drone resting at a known world point (feet): set the effective size D from its blob size."""
        q = self.cam.to_cam(np.array([p_true[0], p_true[1], p_true[2] + self.up]))
        self.D = float(size_px) * q[2] / self.cam.f
        return self.D


class DroneCamSensor:
    def __init__(self, cfg, tag_center, timeline, width=1280):
        self.f = cfg["f_px_1280"] * width / 1280
        self.tilt0, self.k_tilt = float(cfg["tilt_down_deg"]), float(cfg["tilt_per_pitch_deg"])
        self.sr, self.srr = float(cfg["sigma_rel"]), float(cfg["sigma_range_rel"])
        self.tag = np.asarray(tag_center, float)
        self.tl = timeline
        self.up = 4.0
        self.latency = float(cfg["latency_s"])

    def tilt_down(self, t):
        return math.radians(self.tilt0 + self.k_tilt * self.tl.u(t)[PITCH])

    def measurement(self, r, t_taken):
        """r: tag dict from flight_control.find_tag (x, y, z in cm, camera coordinates)."""
        z = np.array([r["x"], r["y"], r["z"]], float)
        rng = float(np.linalg.norm(z))
        tilt = self.tilt_down(t_taken)
        sl = self.sr * rng + 1.0
        sv = sl + rng * math.sin(math.radians(3.0))      # tilt is only roughly known
        sz = self.srr * rng + 2.0
        Rm = np.diag([sl ** 2, sv ** 2, sz ** 2])
        tag, up = self.tag, self.up

        def h(x):
            Rwc = look_rotation(x[PSI], -tilt)
            q = Rwc.T @ (tag - np.array([x[0], x[1], x[Z] + up]))
            return q if q[2] > 5.0 else None
        return Measurement(t_taken, f"tag{r.get('id', '')}", z, Rm, h, info={"range": rng, "src": r.get("src", "")})


class VOSensor:
    def __init__(self, cfg):
        self.s0, self.growth = float(cfg["sigma_deg"]), float(cfg["growth_deg_s"])
        self.latency = float(cfg["latency_s"])
        self.offset = 0.0          # psi when the VO started (heading 0)

    def start(self, psi_now):
        self.offset = psi_now

    def measurement(self, heading_deg, t_taken, t_air):
        s = math.radians(math.hypot(self.s0, self.growth * max(0.0, t_air or 0.0)))
        z = np.array([self.offset - math.radians(heading_deg)])
        return Measurement(t_taken, "vo", z, np.array([[s * s]]), lambda x: np.array([x[PSI]]), angles=(0,))
