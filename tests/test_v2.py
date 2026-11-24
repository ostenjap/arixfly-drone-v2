"""Simulator checks of tools/live_flight_v2.py against tools/live_flight.py, same scenarios.
1. last metre: like the 23:36 video - camera tilted 20 deg down, drone 110 cm above the tag, start 230 cm.
2. yaw learning: the drone turns right 8 deg/s by itself (like the extra right turn while pitching forward,
   2026-11-11 00:09 flights) - v1 flies with the tag off-centre, v2 should learn the offset and centre it.
Both with axes pitch,yaw. Loopback only (never the drone, Wi-Fi or calib/).

Usage:  python tests/test_v2.py
"""
import csv
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "tools"))

import run_all  # noqa: E402  (sets the temp tuning file, helpers)
import live_flight  # noqa: E402
import live_flight_v2  # noqa: E402
from sim_drone2 import SimDrone2  # noqa: E402

KEYS = ("APPROACH:", "BRAKE", "DESCEND", "FOUND", "LAND:", "RECOVER", "yaw learned")


def run(mod, name, **sim_kw):
    run_all.lf = mod
    tilt, climb = sim_kw.pop("tilt"), sim_kw.pop("climb")
    sim = SimDrone2(dist=230, drift_back=0.0, **sim_kw)
    sim.TILT, sim.CLIMB_Z = tilt, climb
    ev, fin = run_all.sim_run(name, sim, axes="pitch,yaw", end=40)
    air = [p for p in sim.track if p[1]]
    rows = list(csv.DictReader(open(os.path.join(run_all.TMP, name, "control.csv"))))
    b = [abs(float(r["bearing_deg"])) for r in rows if r["state"] == "APPROACH" and r["fresh"] == "1"]
    tail = b[len(b) // 2:]   # second half of the approach (after learning)
    keep = [line.split(",", 1)[1] for line in ev.splitlines() if any(k in line for k in KEYS)]
    return fin, (min(p[2] for p in air) if air else None), (sum(tail) / len(tail) if tail else None), keep


if __name__ == "__main__":
    print(f"logs: {run_all.TMP}")
    for title, kw in (("last metre", dict(tilt=20.0, climb=110.0, yaw_drift=0.0)),
                      ("yaw learning", dict(tilt=20.0, climb=60.0, yaw_drift=8.0))):
        print(f"\n=== {title}: {kw}")
        for mod, name in ((live_flight, "v1"), (live_flight_v2, "v2")):
            fin, closest, bear, keep = run(mod, f"{name}_{title.replace(' ', '_')}", **dict(kw))
            bs = "-" if bear is None else f"{bear:.1f}"
            print(f"{name}: ended {fin[2]} cm in front, sideways {fin[3]} cm, closest {closest} cm, "
                  f"|bearing| in 2nd half of approach {bs} deg")
            for k in keep:
                print("     ", k)
