"""Settings and mission files for the v2 stack.

Settings: ONE file, calib/flight.json (defaults below, the file only holds what differs). Every save appends a line to
calib/flight_history.jsonl (time, reason, old -> new), so a change can always be traced and undone.
Missions: missions/*.json = the site (tables, tags, where the laptop camera is) + the route (tables to fly over).
Missions are checked by planner.check_plan before any takeoff.

Usage (inspect):
  python flight/config.py                     # print the merged settings
  python flight/config.py missions/a_to_b.json   # print a mission as loaded (derived box, no-fly zones)
"""
import copy
import json
import math
import os
import sys
import time

if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from flight import ROOT

SETTINGS_FILE = os.path.join(ROOT, "calib", "flight.json")      # tests point these at a temp folder
HISTORY_FILE = os.path.join(ROOT, "calib", "flight_history.jsonl")

# Power units = old live_flight units: 1.0 = full key press at the tethered limits (sticks 40 %, throttle 55 %).
# Model numbers: first guesses from the 2026-11-08..11 flights (notes/findings.md); NOT measured per axis yet
# (Phase 1 pulses). The simulator randomises around them, so the stack is tested against them being wrong.
DEFAULTS = {
    "model": {
        "fwd_cm_s_per_power": 100.0,     # hop_speed 60 cm/s at power 0.5 (unverified) -> ~120; take 100
        "right_cm_s_per_power": 100.0,   # assumed = forward
        "tau_h_s": 0.5,                  # horizontal speed settles in ~0.5 s
        "delay_h_s": 0.15,               # stick -> motion lag (video lag is in the sensors, not here)
        "up_cm_s_per_power": 75.0,       # motion_model.json: 135.6 cm/s per stick unit x 0.55
        "tau_z_s": 0.3,
        "delay_z_s": 0.1,
        "yaw_deg_s_per_power": 15.0,     # findings 2026-11-10: ~15 deg/s per power unit, + = right
        "tau_yaw_s": 0.5,
        "delay_yaw_s": 0.3,
        "takeoff_climb_cm_s": 30.0,      # auto climb after 0x01: 24-40 cm/s
        "takeoff_climb_s": 2.0,          # -> ~60 cm
        "takeoff_climb_start_s": 0.3,
        "lurch_cm_s": -30.0,             # backward lurch during the auto climb: -13 (11-10) .. -33 cm/s (11-11)
        "lurch_start_s": 1.0,            #   1-3 s after takeoff (notes/drift_battery.md)
        "lurch_s": 2.0,
        "sticks_during_climb": False,    # assumed: the drone ignores sticks in its auto climb (more trim did not
                                         #   reduce the lurch). Observe / hold flights will tell.
        "land_cm_s": 30.0,               # 0x02 descent rate
        "fall_cm_s": 150.0,              # after 0x04 (motors off)
    },
    "noise": {                           # process noise of the estimator (per second)
        "accel_h_cm_s2": 30.0,
        "accel_z_cm_s2": 20.0,
        "yaw_accel_deg_s2": 15.0,
        "drift_walk_cm_s": 3.0,          # drift may change this much per sqrt(second)
        "turn_walk_deg_s": 1.0,
        "takeoff_factor": 4.0,           # more process noise during takeoff / landing
        "size_walk": 0.02,               # webcam blob-size scale random walk (log, per sqrt s)
        "size_takeoff": 0.12,            # extra size-scale uncertainty in the first second after takeoff
    },
    "webcam": {
        "index": 0,
        "width": 1280, "height": 720,
        "hfov_deg": 70.0,                # typical laptop webcam; observe.py --calibrate --tag-distance fixes f
        "pos_cm": [-100.0, 0.0, 15.0],   # laptop camera in the world frame (observe.py --calibrate measures it)
        "pan_deg": 0.0, "tilt_deg": 8.0, "roll_deg": 0.0,
        "R": None,                       # full camera->world rotation from --calibrate (overrides pan/tilt/roll)
        "latency_s": 0.08,
        "proc_width": 640,               # tracker works on this width
        "sigma_px": 4.0,                 # pixel noise at proc_width
        "size_rel_sigma": 0.2,           # blob size noise (relative)
        "drone_size_cm": 15.0,           # effective size (sqrt of blob area); re-measured on table A each run
        "center_above_cm": 4.0,          # blob centre above the drone's feet (the state point)
    },
    "bodytrack": {
        "thresh": 28,                    # colour difference to the background (0..255)
        "shadow": True,                  # drop pixels that look like a shadow (darker, same colour)
        "shadow_ratio": [0.55, 0.95],
        "shadow_chroma": 0.035,
        "min_area_px": 25,               # at proc_width
        "max_area_frac": 0.15,
        "gate_px": 90,
        "adapt": 0.01,                   # background follows slow light changes (outside the drone)
    },
    "dronecam": {
        "f_px_1280": 1151.9,             # calib/drone_cam.json
        "tilt_down_deg": 0.0,            # camera mount tilt (2026-11-10 22:39 setup: tilted down)
        "tilt_per_pitch_deg": 8.0,       # nose-down tilt per forward power (tag looks higher when pitching)
        "latency_s": 0.25,
        "sigma_rel": 0.03,               # sideways / vertical noise per cm of range
        "sigma_range_rel": 0.06,
        "use": True,
    },
    "vo": {"use": True, "sigma_deg": 2.0, "growth_deg_s": 0.3, "latency_s": 0.25},
    "estimator": {
        "window_s": 1.0,                 # measurements up to this late are fused at their true time
        "sure_cm": 12.0,                 # position sigma below this = sure
        "unsure_cm": 25.0,               # above this = hold still; held too long or > lost_cm = land
        "lost_cm": 40.0,
        "z_trust_cm": 20.0,              # height corrected only while its sigma is below this
    },
    "control": {
        "pos_T_s": 1.2,                  # position loop time constant (>= 4 x total lag, see controller.gains)
        "vel_T_s": 0.6,
        "yaw_T_s": 1.5,
        "z_T_s": 1.0,
        "max_tilt_power": 0.35,          # roll / pitch power limit
        "max_yaw_power": 0.6,
        "max_thr_power": 0.5,
        "max_yaw_rate_deg_s": 20.0,
        "slew_s": 0.25,                  # sticks move at most full range per slew_s (like a key press)
        "lurch_ff": 0.5,                 # fraction of the expected takeoff lurch fed forward
        "control_during_land": False,    # keep position control during the 0x02 descent (untested)
        "learn_drift": True,
    },
    "states": {
        "pulse_s": 1.0,                  # button pulse; sticks centred meanwhile
        "takeoff_s": 2.5,
        "settle_tol_cm": 10.0, "settle_speed_cm_s": 10.0, "settle_hold_s": 1.0, "settle_max_s": 8.0,
        "final_tol_cm": 7.0, "final_speed_cm_s": 8.0, "final_hold_s": 0.5, "final_max_s": 6.0,
        "hold_max_s": 3.0,               # unsure this long -> land
        "hold_s": 10.0,                  # --hold mode: hover over A this long, then land
        "land_s": 8.0,
    },
    "supervisor": {
        "acts_in_observe": False,        # observe mode: geofence alarms are logged ("WOULD LAND"), not acted on
        "box_side_cm": 50.0,             # box = A back edge .. B far edge, +-50 cm from the A-B line
        "ceiling_cm": 70.0,              # above the highest table
        "hard_margin_cm": 20.0,          # box + this -> 0x04
        "camera_min_cm": 60.0,           # closer than this to the laptop camera (horizontal) -> 0x04
        "takeoff_grace_s": 5.0,          # ceiling not checked during the auto climb + the way back down
        "max_speed_cm_s": 60.0, "max_climb_cm_s": 60.0, "speed_s": 0.5,
        "uncertain_toward_cm": 30.0,     # sigma above this while moving toward the camera > 15 cm/s -> 0x04
        "guard_cm": 15.0,                # within this of A's back edge: no command may push toward the camera
        "battery_min": 40,               # `93 04` battery byte below this -> land (scale unverified: 90 = full?)
        "webcam_lost_s": 2.0,            # flying without webcam this long -> land
        "feed_lost_s": 2.0,              # drone video (link health) lost this long -> land
        "max_cap_s": 60.0,
    },
}


def merge(base, over):
    out = copy.deepcopy(base)
    for k, v in (over or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = merge(out[k], v)
        else:
            out[k] = v
    return out


def load_settings(path=None):
    path = path or SETTINGS_FILE
    try:
        with open(path) as f:
            return merge(DEFAULTS, json.load(f))
    except (OSError, ValueError):
        return copy.deepcopy(DEFAULTS)


def _diff(a, b, prefix=""):
    out = []
    for k in sorted(set(a) | set(b)):
        va, vb = a.get(k), b.get(k)
        if isinstance(va, dict) and isinstance(vb, dict):
            out += _diff(va, vb, prefix + k + ".")
        elif va != vb:
            out.append((prefix + k, va, vb))
    return out


def save_settings(cfg, reason, path=None, history=None):
    """Write only what differs from DEFAULTS; append the change to the history. Returns the changes."""
    path, history = path or SETTINGS_FILE, history or HISTORY_FILE
    old = load_settings(path)
    changes = _diff(old, cfg)
    small = {}
    for key, _, new in _diff(DEFAULTS, cfg):
        node = small
        parts = key.split(".")
        for p in parts[:-1]:
            node = node.setdefault(p, {})
        node[parts[-1]] = new
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path + ".tmp", "w") as f:
        json.dump(small, f, indent=1)
    os.replace(path + ".tmp", path)
    if changes:
        with open(history, "a") as f:
            f.write(json.dumps({"time": time.strftime("%Y-%m-%d %H:%M:%S"), "reason": reason,
                                "changes": [[k, a, b] for k, a, b in changes]}) + "\n")
    return changes


# ------------------------------------------------------------------ missions
class Site:
    """Tables, tags and the derived flight box / no-fly zones of a mission (world frame, cm)."""

    def __init__(self, m, cfg):
        sv = cfg["supervisor"]
        self.tables = {}
        for name, t in m["tables"].items():
            c = np.array(t["center"], float)
            sx, sy = (float(v) for v in t.get("size", [40, 40]))
            self.tables[name] = {"center": c, "size": (sx, sy)}
        self.route = list(m["route"])
        self.points = [self.point(r) for r in self.route]
        off = np.array(list(m.get("start_offset_cm", [0, 0])) + [0.0])[:3]
        self.points[0] = self.points[0] + off      # where the drone is placed on the first table
        self.start, self.goal = self.points[0], self.points[-1]
        self.heading = math.radians(float(m.get("heading_deg", 0.0)))   # held all flight; 0 = nose toward +x (B)
        self.tags = {}
        for name, t in m.get("tags", {}).items():
            self.tags[name] = {"id": int(t["id"]), "size_cm": float(t["size_cm"]),
                               "center": np.array(t["center"], float),
                               "facing": np.array(t.get("facing", [-1, 0, 0]), float)}
        # box: from the back edge of the first table to the far edge of the last, +-box_side from the line
        a, b = self.tables_on_route()[0], self.tables_on_route()[-1]
        self.x_back = a["center"][0] - a["size"][0] / 2
        self.x_far = b["center"][0] + b["size"][0] / 2
        ys = [p[1] for p in self.points]
        self.y_min, self.y_max = min(ys) - sv["box_side_cm"], max(ys) + sv["box_side_cm"]
        top = max(t["center"][2] for t in self.tables.values())
        self.z_min = min(t["center"][2] for t in self.tables.values()) - 15.0
        self.z_max = top + sv["ceiling_cm"]
        box = m.get("box")
        if box:   # explicit override [[xmin, xmax], [ymin, ymax], [zmin, zmax]]
            (self.x_back, self.x_far), (self.y_min, self.y_max), (self.z_min, self.z_max) = box
        self.no_fly = [dict(z) for z in m.get("no_fly", [])]   # extra cylinders {"center": [x, y], "radius_cm"}
        self.cruise = float(m.get("cruise_height_cm", 40))
        self.land_height = float(m.get("land_height_cm", 20))
        self.max_speed = float(m.get("max_speed_cm_s", 25))
        self.max_accel = float(m.get("max_accel_cm_s2", 25))
        self.time_cap = min(float(m.get("time_cap_s", 30)), sv["max_cap_s"])
        self.name = m.get("name", "mission")

    def tables_on_route(self):
        return [self.tables[r] for r in self.route if isinstance(r, str)]

    def point(self, r):
        if isinstance(r, str):
            return self.tables[r]["center"].copy()
        return np.array(r, float)

    def ground_z(self, x, y):
        """Top of the table under (x, y), else the floor (assumed 75 cm below table A)."""
        for t in self.tables.values():
            c, (sx, sy) = t["center"], t["size"]
            if abs(x - c[0]) <= sx / 2 and abs(y - c[1]) <= sy / 2:
                return float(c[2])
        return -75.0

    def in_box(self, p, margin=0.0):
        return (self.x_back - margin <= p[0] <= self.x_far + margin and self.y_min - margin <= p[1] <= self.y_max + margin
                and self.z_min - margin <= p[2] <= self.z_max + margin)


def webcam_camera(cfg, width=None):
    """geometry.Camera of the laptop webcam from the settings (at `width`, default the capture width)."""
    from flight.geometry import Camera, focal_from_hfov
    w = cfg["webcam"]
    W, H = int(w["width"]), int(w["height"])
    f = float(w.get("f_px") or focal_from_hfov(W, w["hfov_deg"]))
    if w.get("R"):
        cam = Camera(f, W, H, w["pos_cm"], np.array(w["R"], float))
    else:
        cam = Camera.from_pose(f, W, H, w["pos_cm"], w["pan_deg"], w["tilt_deg"], w["roll_deg"])
    return cam.scaled(width) if width else cam


def load_mission(path, cfg=None):
    cfg = cfg or load_settings()
    with open(path) as f:
        m = json.load(f)
    return m, Site(m, cfg)


if __name__ == "__main__":
    cfg = load_settings()
    if len(sys.argv) > 1:
        m, s = load_mission(sys.argv[1], cfg)
        print(json.dumps(m, indent=1))
        print(f"box x {s.x_back:.0f}..{s.x_far:.0f}  y {s.y_min:.0f}..{s.y_max:.0f}  z {s.z_min:.0f}..{s.z_max:.0f}"
              f"  cruise {s.cruise:.0f}  cap {s.time_cap:.0f} s")
    else:
        print(json.dumps(cfg, indent=1))
