"""Planner: mission route -> smooth reference (where the drone should be, how fast) + pre-flight checks.

Route = table names or [x, y, z] points; first = takeoff table, last = landing table. The drone climbs (auto
takeoff), settles over the first table at cruise height, flies the polyline at cruise height, then descends over the
last table to land_height and lands (0x02).
Speed along the path: v(s) = min(max_speed, sqrt(2 a (s + 2 cm)), sqrt(1.2 a (L - s))) - speed up, cruise, slow down
(braking at 0.6 a: the drone's lag makes it overshoot, and the far edge of B is close).
The reference point is a "carrot" that never runs more than lead_cm ahead of the drone, so a slow or held drone
is never yanked toward a far reference.

check_plan(): before takeoff, every point of the path (and the settle / final / auto-climb points) must be inside the
flight box, outside the no-fly zones, far enough from the laptop camera, and VISIBLE to the laptop camera; the
whole plan must fit the time cap. Errors block the takeoff, warnings are shown.

Usage: python flight/planner.py missions/a_to_b.json     # prints the plan and the check
"""
import math
import os
import sys

if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np


class PathRef:
    def __init__(self, site, lead_cm=8.0):
        self.site = site
        pts = []
        for r, p in zip(site.route, site.points):
            q = p.copy()
            if isinstance(r, str):
                q[2] = p[2] + site.cruise
            pts.append(q)
        self.pts = np.array(pts)
        seg = np.diff(self.pts, axis=0)
        self.seg_len = np.linalg.norm(seg, axis=1)
        self.cum = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.L = float(self.cum[-1])
        self.vmax, self.amax, self.lead = site.max_speed, site.max_accel, lead_cm
        self.s_ref = 0.0
        self.settle_point = self.pts[0].copy()
        g = site.goal.copy()
        self.final_point = np.array([g[0], g[1], g[2] + site.land_height])
        self.above_goal = self.pts[-1].copy()

    def point(self, s):
        s = min(max(s, 0.0), self.L)
        i = min(int(np.searchsorted(self.cum, s, side="right")) - 1, len(self.seg_len) - 1)
        i = max(i, 0)
        if self.seg_len[i] < 1e-6:
            return self.pts[i].copy(), np.zeros(3)
        k = (s - self.cum[i]) / self.seg_len[i]
        tan = (self.pts[i + 1] - self.pts[i]) / self.seg_len[i]
        return self.pts[i] + k * (self.pts[i + 1] - self.pts[i]), tan

    def speed(self, s):
        a = self.amax
        return min(self.vmax, math.sqrt(2 * a * (max(s, 0) + 2.0)), math.sqrt(2 * 0.6 * a * max(self.L - s, 0.0)))

    def project(self, p):
        """Arc length of the path point nearest to p."""
        best, bs = None, 0.0
        for i in range(len(self.seg_len)):
            a, b = self.pts[i], self.pts[i + 1]
            d = b - a
            k = 0.0 if self.seg_len[i] < 1e-6 else float(np.clip(np.dot(p - a, d) / np.dot(d, d), 0, 1))
            dist = np.linalg.norm(a + k * d - p)
            if best is None or dist < best:
                best, bs = dist, self.cum[i] + k * self.seg_len[i]
        return bs

    def reset(self, p_est=None):
        self.s_ref = 0.0 if p_est is None else self.project(p_est)

    def step(self, dt, p_est):
        """Advance the carrot; returns (p_ref, v_ref, done)."""
        s_est = self.project(p_est)
        v = self.speed(self.s_ref)
        self.s_ref = min(self.s_ref + v * dt, self.L, s_est + self.lead)
        p, tan = self.point(self.s_ref)
        lead_left = self.lead - (self.s_ref - s_est)
        v_ref = tan * self.speed(self.s_ref) * float(np.clip(lead_left / (0.3 * self.lead), 0.0, 1.0))
        return p, v_ref, self.s_ref >= self.L - 0.5

    def duration(self):
        """Planned transit time (s) at full speed (trapezoid)."""
        t, s, dt = 0.0, 0.0, 0.02
        while s < self.L - 0.5 and t < 120:
            s += self.speed(s) * dt
            t += dt
        return t

    def samples(self, step_cm=5.0):
        n = max(2, int(self.L / step_cm) + 1)
        return [self.point(s)[0] for s in np.linspace(0, self.L, n)]


def check_plan(site, cfg, cam=None, mode="mission"):
    """-> (errors, warnings). cam: geometry.Camera of the laptop webcam (any width) or None."""
    errors, warns = [], []
    sv, md = cfg["supervisor"], cfg["model"]
    path = PathRef(site)
    if mode == "mission" and len(site.points) < 2:
        errors.append("route needs at least 2 points (takeoff table, landing table)")
    start = site.start
    climb_top = start + np.array([0, 0, md["takeoff_climb_cm_s"] * md["takeoff_climb_s"]])
    checks = [("settle point", path.settle_point), ("auto-climb top (estimate)", climb_top)]
    if mode == "mission":
        checks += [(f"path {i * 5} cm", p) for i, p in enumerate(path.samples())]
        checks += [("final point", path.final_point)]
    for name, p in checks:
        if not site.in_box(p, margin=-5.0):
            (warns if name.startswith("auto-climb") else errors).append(
                f"{name} {np.round(p).tolist()} is not inside the flight box (5 cm margin)")
        for z in site.no_fly:
            if math.hypot(p[0] - z["center"][0], p[1] - z["center"][1]) < z["radius_cm"]:
                errors.append(f"{name} is inside no-fly zone {z.get('name', '')}")
        if cam is not None:
            if math.hypot(p[0] - cam.pos[0], p[1] - cam.pos[1]) < sv["camera_min_cm"] + 20:
                errors.append(f"{name} is closer than {sv['camera_min_cm'] + 20:.0f} cm to the laptop camera")
            q = cam.project(p + np.array([0, 0, cfg["webcam"].get("center_above_cm", 4.0)]))
            if q is None or not cam.inside(q[0], q[1], 0.05):
                (warns if name.startswith("auto-climb") else errors).append(
                    f"{name} {np.round(p).tolist()} is NOT visible to the laptop camera" +
                    ("" if q is None else f" (pixel {q[0]:.0f}, {q[1]:.0f} of {cam.w}x{cam.h})"))
    # takeoff lurch: the drone slides back (body frame) ~lurch_cm_s for lurch_s while it ignores the sticks
    lurch = abs(md["lurch_cm_s"]) * md["lurch_s"]
    back = start + lurch * np.array([-math.cos(site.heading), -math.sin(site.heading), 0.0])
    back[2] = start[2] + 20.0
    if cam is not None and math.hypot(back[0] - cam.pos[0], back[1] - cam.pos[1]) < sv["camera_min_cm"]:
        errors.append(f"takeoff lurch (~{lurch:.0f} cm backward) would reach the operator zone at "
                      f"{np.round(back[:2]).tolist()}: move the drone forward on the table (start_offset_cm) or turn it "
                      f"(heading_deg)")
    elif not site.in_box(back, margin=sv["hard_margin_cm"]):
        errors.append(f"takeoff lurch (~{lurch:.0f} cm backward) would end far outside the box (motor stop)")
    elif not site.in_box(back):
        warns.append(f"takeoff lurch (~{lurch:.0f} cm backward, model) may leave the box -> early landing; "
                     f"more room behind the drone helps (start_offset_cm / heading_deg)")
    if site.cruise > sv["ceiling_cm"] - 10:
        errors.append(f"cruise height {site.cruise:.0f} cm is within 10 cm of the ceiling {sv['ceiling_cm']:.0f}")
    st = cfg["states"]
    t_total = (st["takeoff_s"] + 3.0 + (path.duration() if mode == "mission" else st["hold_s"])
               + (site.cruise - site.land_height) / 10.0 + 2.0 + 1.0)
    if t_total > site.time_cap - 3:
        errors.append(f"plan needs ~{t_total:.0f} s, time cap is {site.time_cap:.0f} s (3 s margin)")
    if not site.tags:
        warns.append("no tags in the mission: distance from the laptop camera comes from the blob size only (weak)")
    return errors, warns, {"t_total": t_total, "length_cm": path.L}


def main():
    from flight.config import load_mission, load_settings, webcam_camera
    cfg = load_settings()
    m, site = load_mission(sys.argv[1], cfg)
    cam = webcam_camera(cfg)
    e, w, info = check_plan(site, cfg, cam)
    path = PathRef(site)
    print(f"{site.name}: route {site.route}, path {info['length_cm']:.0f} cm, transit {path.duration():.1f} s, "
          f"whole flight ~{info['t_total']:.0f} s (cap {site.time_cap:.0f} s)")
    for p in path.pts:
        q = cam.project(p)
        print(f"  waypoint {np.round(p).tolist()} -> laptop pixel "
              + ("behind the camera" if q is None else f"{q[0]:.0f}, {q[1]:.0f}"))
    for x in e:
        print("ERROR  ", x)
    for x in w:
        print("warning", x)
    print("plan OK" if not e else "plan BLOCKED")


if __name__ == "__main__":
    main()
