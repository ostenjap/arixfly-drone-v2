"""Simulator for the v2 stack, built from the same model the estimator uses - but every run draws its OWN "true"
drone (gains, lags, drift, own turn, lurch, climb, camera tilt, webcam pose error, video latencies), so the stack
is tested against the model being wrong. Sensors produce the same inputs as live (webcam blob dict, find_tag dict,
VO heading) and go through the same flight/sensors.py builders and flight/pilot.py states.

Scorecard per run uses the TRUTH: success = landed on B within 15 cm of its centre with no supervisor / operator
intervention. Also: landing error, estimate error at landing, sideways swing on the way, holds, resets.

Usage:
  python flight/sim.py                       # 20 runs of missions/a_to_b.json, summary
  python flight/sim.py --runs 100 --csv captures/sim/a_to_b.csv
  python flight/sim.py --mode hold --runs 10
  python flight/sim.py --seed 3 --verbose    # one run with the event log
  python flight/sim.py --harsh               # worse drone: bigger model errors, more dropouts, outliers
  python flight/sim.py --no-dronecam         # webcam only
  python flight/sim.py --plot captures/sim/runs.png   # top/side view of the runs (needs matplotlib)
"""
import argparse
import csv
import json
import math
import os
import sys
import time

if __name__ == "__main__":
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from flight import ROOT
from flight.config import Site, load_mission, load_settings, webcam_camera
from flight.geometry import Camera, look_rotation
from flight.model import DF, DL, DR, N, PITCH, PSI, VX, VY, VZ, X, Y, Z, Model, Timeline
from flight.pilot import ESTOP, LAND, TAKEOFF, Pilot

DEFAULT_MISSION = os.path.join(ROOT, "missions", "a_to_b.json")


class SimWorld:
    """The 'real' drone + its sensors."""

    def __init__(self, cfg, site, seed=0, harsh=False, dronecam=True):
        rng = self.rng = np.random.default_rng(seed)
        k = 2.0 if harsh else 1.0
        p = dict(cfg["model"])

        def f(lo, hi):
            return float(rng.uniform(1 - (1 - lo) * k, 1 + (hi - 1) * k))
        for key, lo, hi in (("fwd_cm_s_per_power", 0.7, 1.3), ("right_cm_s_per_power", 0.7, 1.3),
                            ("tau_h_s", 0.7, 1.5), ("delay_h_s", 0.6, 1.6), ("up_cm_s_per_power", 0.7, 1.3),
                            ("yaw_deg_s_per_power", 0.7, 1.3), ("tau_yaw_s", 0.7, 1.5), ("delay_yaw_s", 0.6, 1.6),
                            ("takeoff_climb_s", 0.75, 1.25), ("land_cm_s", 0.8, 1.2)):
            p[key] = p[key] * f(lo, hi)
        p["takeoff_climb_cm_s"] = float(rng.uniform(24, 40))
        p["lurch_cm_s"] = p["lurch_cm_s"] * float(rng.uniform(0.4, 1.2))   # -12..-36 cm/s at the default
        self.p = p
        self.m = Model(p, ground_z=site.ground_z)
        self.tl = Timeline(0.0)
        self.site, self.cfg = site, cfg
        self.x = np.zeros(N)
        self.x[X:Z + 1] = site.start + np.array([rng.normal(0, 2), rng.normal(0, 2), 0])
        self.x[PSI] = site.heading + math.radians(rng.normal(0, 2))
        self.x[DF], self.x[DL] = rng.normal(0, 4 * k), rng.normal(0, 4 * k)
        self.x[DR] = math.radians(rng.normal(0, 2.5 * k))
        self.drift_walk = 1.5 * k
        self.landed_t = None
        # webcam: true pose = configured pose + calibration error
        wc = cfg["webcam"]
        cam_cfg = webcam_camera(cfg, wc["proc_width"])
        err_R = look_rotation(math.radians(rng.normal(0, 0.4 * k)), math.radians(rng.normal(0, 0.4 * k)),
                              math.radians(rng.normal(0, 0.3 * k)))
        base = look_rotation(0, 0)
        self.cam = Camera(cam_cfg.f * rng.uniform(1 - 0.02 * k, 1 + 0.02 * k), cam_cfg.w, cam_cfg.h,
                          cam_cfg.pos + rng.normal(0, 2 * k, 3), cam_cfg.R @ (base.T @ err_R))
        self.cam_cfg = cam_cfg
        self.D = wc["drone_size_cm"] * rng.uniform(0.8, 1.25)
        self.up = wc.get("center_above_cm", 4.0)
        self.air_size = float(rng.uniform(1.0, 1.2))      # spinning props make the blob bigger
        self.web_lat = float(rng.uniform(0.05, 0.12))
        self.web_drop, self.web_out = 0.05 * k, 0.015 * k
        self.dc = dronecam and bool(site.tags)
        self.tags = [(t["id"], t["center"], t["facing"]) for t in site.tags.values()]
        self.dc_lat = float(rng.uniform(0.2, 0.32))
        self.dc_tilt0 = cfg["dronecam"]["tilt_down_deg"] + rng.normal(0, 1.5 * k)
        self.dc_k = cfg["dronecam"]["tilt_per_pitch_deg"] * rng.uniform(0.5, 1.5)
        self.dc_f = cfg["dronecam"]["f_px_1280"]
        self.p_det = 0.35 / k
        self.vo_bias, self.vo_psi0 = 0.0, None
        self.queue = []   # (deliver_t, kind, payload, true_taken_t)
        self.t = 0.0

    # --- link side
    def command(self, t, sticks, button):
        self.tl.command(t, sticks)
        if button == TAKEOFF:
            self.tl.phase(t, "air")
            self.landed_t = None
        elif button == LAND:
            self.tl.phase(t, "landing")
        elif button == ESTOP:
            self.tl.phase(t, "falling")

    def step(self, dt):
        rng = self.rng
        self.x, _ = self.m.step(self.x, self.t, dt, self.tl, want_F=False)
        self.t += dt
        ph, _ = self.tl.phase_at(self.t)
        if ph == "air":
            s = self.drift_walk * math.sqrt(dt)
            self.x[DF] += rng.normal(0, s) - 0.02 * dt * self.x[DF]
            self.x[DL] += rng.normal(0, s) - 0.02 * dt * self.x[DL]
            self.x[VX] += rng.normal(0, 8 * math.sqrt(dt))     # small gusts
            self.x[VY] += rng.normal(0, 8 * math.sqrt(dt))
        if ph in ("landing", "falling") and self.x[Z] <= self.site.ground_z(self.x[X], self.x[Y]) + 0.01:
            self.landed_t = self.landed_t or self.t
            if self.t - self.landed_t > 0.3:
                self.tl.phase(self.t, "ground")
        if ph == "air" and self.x[Z] < self.site.ground_z(self.x[X], self.x[Y]) - 0.5:
            self.x[Z] = self.site.ground_z(self.x[X], self.x[Y])

    def flying(self):
        return self.tl.phase_at(self.t)[0] != "ground"

    # --- sensors (called at their rates); results come out of deliver() after their latency
    def sense_webcam(self):
        rng = self.rng
        p = self.x[X:Z + 1] + np.array([0, 0, self.up])
        q = self.cam.project(p)
        if q is None or not self.cam.inside(q[0], q[1], 0.01) or rng.random() < self.web_drop:
            return
        u, v = q[0] + rng.normal(0, 1.5), q[1] + rng.normal(0, 1.5)
        blur = 0.0 if not self.flying() else 3.0
        if rng.random() < self.web_out:   # wrong blob (shadow, reflection)
            u, v = u + rng.choice([-1, 1]) * rng.uniform(25, 90), v + rng.uniform(-40, 40)
        size = self.cam.f * self.D / q[2] * (1 + rng.normal(0, 0.08)) * (self.air_size if self.flying() else 1.0)
        self.queue.append((self.t + self.web_lat, "webcam", {"u": u, "v": v, "size": size, "blur": blur}, self.t))

    def sense_dronecam(self):
        if not self.dc or not self.flying():
            return
        rng = self.rng
        tilt = math.radians(self.dc_tilt0 + self.dc_k * self.tl.u(self.t)[PITCH])
        Rwc = look_rotation(self.x[PSI], -tilt)
        cam = self.x[X:Z + 1] + np.array([0, 0, 4.0])
        for tid, c, facing in self.tags:
            if rng.random() > self.p_det:
                continue
            q = Rwc.T @ (c - cam)
            if q[2] < 15 or q[2] > 350:
                continue
            if abs(q[0] / q[2]) > math.tan(math.radians(27)) or abs(q[1] / q[2]) > math.tan(math.radians(15)):
                continue
            view = (cam - c) / np.linalg.norm(cam - c)
            if np.dot(view, facing) < math.cos(math.radians(60)):     # seen too obliquely / from behind
                continue
            rng_cm = float(np.linalg.norm(q))
            q = q + np.array([rng.normal(0, 0.015 * rng_cm), rng.normal(0, 0.015 * rng_cm), rng.normal(0, 0.03 * rng_cm)])
            self.queue.append((self.t + self.dc_lat, "dronecam", {"x": q[0], "y": q[1], "z": q[2], "id": tid,
                                                                  "src": "sim"}, self.t))

    def sense_vo(self):
        if self.vo_psi0 is None:
            self.vo_psi0 = self.x[PSI]
        rng = self.rng
        if self.flying():
            self.vo_bias += rng.normal(0, 0.3 * math.sqrt(1 / 18))
            if rng.random() < 0.2:
                return
        h = -math.degrees(self.x[PSI] - self.vo_psi0) + self.vo_bias + rng.normal(0, 0.3)
        self.queue.append((self.t + self.dc_lat, "vo", h, self.t))

    def deliver(self):
        out = [q for q in self.queue if q[0] <= self.t]
        self.queue = [q for q in self.queue if q[0] > self.t]
        return out


def run(cfg, site, mode="mission", seed=0, harsh=False, dronecam=True, verbose=False, T=45.0, takeoff_at=2.0):
    w = SimWorld(cfg, site, seed, harsh, dronecam)
    logs = []

    def log(s):
        logs.append(f"{w.t:6.2f}  {s}")
        if verbose:
            print(logs[-1])
    pilot = Pilot(site, cfg, mode, w.cam_cfg, 0.0, log)
    if pilot.plan_errors:
        return {"seed": seed, "success": False, "error_cm": float("nan"), "reason": "plan: " + pilot.plan_errors[0]}
    pilot.start_vo(0.0)
    dt, t_pilot, t_web, t_dc = 0.005, 0.0, 0.0, 0.0
    lat_cfg_web, lat_cfg_dc = cfg["webcam"]["latency_s"], cfg["dronecam"]["latency_s"]
    track = []
    tried = False
    while w.t < T:
        w.step(dt)
        if w.t >= t_web:
            t_web += 1 / 30
            w.sense_webcam()
        if w.t >= t_dc:
            t_dc += 1 / 18
            w.sense_dronecam()
            w.sense_vo()
        for _, kind, payload, _true_t in w.deliver():
            if kind == "webcam":
                pilot.add_webcam(payload, w.t - lat_cfg_web)
            elif kind == "dronecam":
                pilot.add_dronecam(payload, w.t - lat_cfg_dc)
            else:
                pilot.add_vo(payload, w.t - lat_cfg_dc)
        if w.t >= t_pilot:
            t_pilot += 1 / 30
            ops = {}
            if w.t >= takeoff_at and not tried and pilot.state == "GROUND":
                ops["takeoff"] = True
                tried = True
            out = pilot.step(w.t, ops, {"webcam_age": 0.0, "feed_age": 0.0, "battery": 90})
            w.command(w.t, out["sticks"], out["button"])
            track.append((w.t, *w.x[X:Z + 1], *out["x"][X:Z + 1], out["sig"], pilot.state))
            if pilot.state == "GROUND" and tried and w.t > takeoff_at + 1:
                break
            if pilot.result is not None or (pilot.state == "KILLED" and not w.flying()):
                break
    g = site.goal
    err = float(math.hypot(w.x[X] - g[0], w.x[Y] - g[1]))
    on_goal = abs(site.ground_z(w.x[X], w.x[Y]) - g[2]) < 1.0
    reason = pilot.land_reason
    clean = not reason.startswith(("supervisor", "ESTOP", "operator"))
    ok = mode == "mission" and err <= 15.0 and on_goal and clean and pilot.state == "LANDED"
    tr = np.array([r[1:8] for r in track], float) if track else np.zeros((0, 7))
    air = [r for r in track if r[8] in ("TRANSIT",)]
    side = max((abs(r[2]) for r in air), default=0.0)
    est_err = float(np.linalg.norm(tr[-1, 0:2] - tr[-1, 3:5])) if len(tr) else float("nan")
    res = {"seed": seed, "success": ok, "error_cm": err, "est_error_cm": est_err, "side_max_cm": side,
           "reason": reason or pilot.state, "holds": sum(1 for s in logs if "-> HOLD" in s),
           "resets": pilot.n_resets, "t_end": w.t, "lurch": w.p["lurch_cm_s"], "logs": logs, "track": track,
           "rej_web": pilot.est.stats.get("webcam", {}).get("rejected", 0),
           "dc_ok": sum(v["ok"] for k, v in pilot.est.stats.items() if k.startswith("tag"))}
    return res


def summary(results, mode):
    n = len(results)
    ok = sum(r["success"] for r in results)
    errs = sorted(r["error_cm"] for r in results if not math.isnan(r["error_cm"]))
    lines = [f"{n} runs, mode {mode}: SUCCESS {ok}/{n} ({100 * ok / max(n, 1):.0f} %)"]
    if errs and mode == "mission":
        lines.append(f"  landing error (truth) median {np.median(errs):.1f} cm, 90 % {np.percentile(errs, 90):.1f} cm, "
                     f"max {max(errs):.1f} cm")
        ee = [r["est_error_cm"] for r in results if not math.isnan(r.get("est_error_cm", float("nan")))]
        lines.append(f"  estimate error at landing median {np.median(ee):.1f} cm; sideways swing in transit median "
                     f"{np.median([r['side_max_cm'] for r in results]):.1f} cm")
    reasons = {}
    for r in results:
        k = r["reason"].split("(")[0].strip()
        reasons[k] = reasons.get(k, 0) + 1
    for k, v in sorted(reasons.items(), key=lambda kv: -kv[1]):
        lines.append(f"  {v:3d} x  {k}")
    return "\n".join(lines)


def plot(results, site, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, (a, b) = plt.subplots(1, 2, figsize=(13, 5))
    for r in results:
        tr = np.array([t[1:4] for t in r["track"]])
        col = "tab:green" if r["success"] else "tab:red"
        a.plot(tr[:, 0], tr[:, 1], color=col, lw=0.8, alpha=0.7)
        b.plot(tr[:, 0], tr[:, 2], color=col, lw=0.8, alpha=0.7)
    for name, t in site.tables.items():
        c, (sx, sy) = t["center"], t["size"]
        a.add_patch(plt.Rectangle((c[0] - sx / 2, c[1] - sy / 2), sx, sy, fill=False, color="k"))
        a.text(c[0], c[1] + sy / 2 + 3, name)
        b.plot([c[0] - sx / 2, c[0] + sx / 2], [c[2], c[2]], "k-", lw=3)
    g = site.goal
    a.add_patch(plt.Circle((g[0], g[1]), 15, fill=False, color="tab:blue", ls="--"))
    a.add_patch(plt.Rectangle((site.x_back, site.y_min), site.x_far - site.x_back, site.y_max - site.y_min,
                              fill=False, color="tab:orange", ls=":"))
    a.set_aspect("equal")
    a.set_title("top view (x toward B, y left), cm")
    b.set_title("side view (height), cm")
    a.invert_yaxis()
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fig.savefig(path, dpi=110, bbox_inches="tight")
    print(f"plot: {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mission", default=DEFAULT_MISSION)
    ap.add_argument("--mode", default="mission", choices=("mission", "hold", "observe"))
    ap.add_argument("--runs", type=int, default=20)
    ap.add_argument("--seed", type=int, default=None, help="one run with this seed")
    ap.add_argument("--harsh", action="store_true")
    ap.add_argument("--no-dronecam", action="store_true")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--heading", type=float, help="override the mission's heading_deg (0 = nose toward B)")
    ap.add_argument("--start-offset", help="override start_offset_cm, e.g. 10,0")
    ap.add_argument("--table", help="table size for A and B, e.g. 60,60 (keeps the 1 m gap)")
    ap.add_argument("--sticks-in-climb", action="store_true", help="assume sticks work during the auto climb")
    ap.add_argument("--lurch", type=float, help="nominal lurch cm/s (model + truth centre)")
    ap.add_argument("--set", action="append", help="override a setting, e.g. --set control.vel_T_s=0.5")
    ap.add_argument("--csv")
    ap.add_argument("--plot")
    a = ap.parse_args()
    cfg = load_settings()
    m, site = load_mission(a.mission, cfg)
    if a.heading is not None or a.start_offset or a.table:
        m = json.loads(json.dumps(m))
        if a.heading is not None:
            m["heading_deg"] = a.heading
        if a.start_offset:
            m["start_offset_cm"] = [float(v) for v in a.start_offset.split(",")]
        if a.table:
            sx, sy = (float(v) for v in a.table.split(","))
            m["tables"]["A"]["size"] = [sx, sy]
            m["tables"]["B"]["size"] = [sx, sy]
            m["tables"]["B"]["center"][0] = sx + 100.0       # keep the 1 m gap
            if "B" in m.get("tags", {}):
                m["tags"]["B"]["center"][0] = 1.5 * sx + 100.0
        site = Site(m, cfg)
    for kv in a.set or []:                      # --set control.vel_T_s=0.5
        key, val = kv.split("=", 1)
        node = cfg
        parts = key.split(".")
        for q in parts[:-1]:
            node = node[q]
        node[parts[-1]] = json.loads(val)
    if a.sticks_in_climb:
        cfg["model"]["sticks_during_climb"] = True
    if a.lurch is not None:
        cfg["model"]["lurch_cm_s"] = a.lurch
    seeds = [a.seed] if a.seed is not None else range(a.runs)
    t0 = time.time()
    results = []
    for s in seeds:
        r = run(cfg, site, a.mode, s, a.harsh, not a.no_dronecam, a.verbose or a.seed is not None)
        results.append(r)
        if not a.verbose and a.seed is None:
            print(f"seed {s:3d}  {'OK  ' if r['success'] else 'MISS'}  error {r['error_cm']:5.1f} cm  "
                  f"est {r.get('est_error_cm', float('nan')):4.1f}  side {r.get('side_max_cm', 0):4.1f}  "
                  f"holds {r.get('holds', 0)}  {r['reason'][:70]}", flush=True)
    print(summary(results, a.mode))
    print(f"({time.time() - t0:.1f} s)")
    if a.csv:
        os.makedirs(os.path.dirname(os.path.abspath(a.csv)), exist_ok=True)
        keys = ["seed", "success", "error_cm", "est_error_cm", "side_max_cm", "holds", "resets", "reason", "lurch"]
        with open(a.csv, "w", newline="") as f:
            wr = csv.writer(f)
            wr.writerow(keys)
            for r in results:
                wr.writerow([r.get(k, "") for k in keys])
    if a.plot:
        plot(results, site, a.plot)


if __name__ == "__main__":
    main()
