"""Scorecard: one row per takeoff from live_flight.py logs (fast: logs only, no frame processing).

North star: SUCCESS = landed within --tol cm (default 20) of the goal (target cm in front of the tag, centred)
with no human intervention (L, Space/kill, Enter held, Esc, window focus lost). The landing error comes from the
"LANDING RESULT" line live_flight logs after each approach / hop landing (tag read from the ground); flights
before 2026-11-11 have none ("-").

Per takeoff: mode, end reason, intervention, air time, landing error, tag visible % (air), searches, sideways
swing and heading swing (cm / deg, 5-95 % spread). Sideways comes from <folder>/review/frames_review.csv when
tools/flight_review.py has run on the folder (tag pose per frame, reliable); otherwise a rough guess from range,
bearing and VO heading (underestimates: sideways motion also looks like turning to the VO) -> run the review.

Usage:
  python tools/scorecard.py captures/flights/20261111_205409
  python tools/scorecard.py captures/flights/2026111*            # several folders + totals
  python tools/scorecard.py captures/flights/2026111* --tol 30
Writes <folder>/review/scorecard.csv per folder.
"""
import argparse
import csv
import glob
import math
import os
import re
import statistics

INTERVENTIONS = ("L pressed", "Enter held", "Esc", "window lost focus")
AIR = ("FLYING", "ACQUIRE", "APPROACH", "RECOVER", "HOP", "BRAKE")


def spread(v):
    if len(v) < 5:
        return None
    v = sorted(v)
    return v[int(0.95 * (len(v) - 1))] - v[int(0.05 * (len(v) - 1))]


def flights(folder):
    ev = [(float(r["t"]), r["event"]) for r in csv.DictReader(open(os.path.join(folder, "events.csv")))]
    p = os.path.join(folder, "control.csv")
    ctl = list(csv.DictReader(open(p))) if os.path.exists(p) else []
    q = os.path.join(folder, "review", "frames_review.csv")
    fr = list(csv.DictReader(open(q))) if os.path.exists(q) else []
    starts = [t for t, e in ev if e.startswith("TAKEOFF")]
    out = []
    for i, t0 in enumerate(starts):
        t1 = starts[i + 1] if i + 1 < len(starts) else float("inf")
        seg = [(t, e) for t, e in ev if t0 <= t < t1]
        land = next(((t, e) for t, e in seg if e.startswith("LAND:")), None)
        kill = next((t for t, e in seg if e.startswith("KILL")), None)
        end_t = min(x for x in (land[0] if land else None, kill, t1) if x is not None)
        reason = "KILL" if kill and (not land or kill < land[0]) else (land[1][6:] if land else "?")
        inter = reason == "KILL" or any(reason.startswith(x) for x in INTERVENTIONS)
        mode = ("hop" if any(e.startswith("HOP") for _, e in seg)
                else "approach" if any(e.startswith(("ACQUIRE", "APPROACH")) for _, e in seg) else "hover")
        res = next((e for _, e in seg if e.startswith("LANDING RESULT")), None)
        err = along = side = None
        if res:
            m = re.search(r"(-?\d+) cm in front of the tag, sideways ([+-]?\d+) cm, target \d+ -> error (\d+) cm", res)
            if m:
                along, side, err = float(m.group(1)), float(m.group(2)), float(m.group(3))
        rows = [r for r in ctl if t0 <= float(r["t"]) < end_t and r["state"] in AIR]
        seen = sum(r["fresh"] == "1" for r in rows)
        world, head = [], []
        for r in rows:
            if r.get("vo_heading_deg"):
                head.append(float(r["vo_heading_deg"]))
            if r["fresh"] == "1" and r["range_cm"] and r.get("vo_heading_deg"):
                world.append((float(r["range_cm"]), float(r["vo_heading_deg"]) + float(r["bearing_deg"])))
        side_sw = None
        if fr:   # frame review (tools/flight_review.py) ran: sideways from the tag pose, per frame
            v = [float(x["side_cm"]) for x in fr if t0 <= float(x["t"]) < end_t and x["state"] in AIR
                 and x["side_cm"]]
            side_sw = spread(v)
        elif len(world) >= 5:   # rough: VO heading also turns with sideways motion -> underestimates
            a0 = statistics.median(a for _, a in world[:5])
            side_sw = spread([rg * math.sin(math.radians(a - a0)) for rg, a in world])
        out.append(dict(
            takeoff=round(t0, 1), mode=mode, end=reason[:60], intervention=int(inter),
            air_s=round(end_t - t0, 1), landing_err_cm=err, landed_along_cm=along, landed_side_cm=side,
            tag_seen_pct=round(100 * seen / len(rows)) if rows else None,
            searches=sum(bool(re.match(r"RECOVER \d", e)) for _, e in seg),
            side_swing_cm=None if side_sw is None else round(side_sw),
            heading_swing_deg=None if spread(head) is None else round(spread(head))))
    return out


def fmt(v):
    return "-" if v is None else str(v)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folders", nargs="+")
    ap.add_argument("--tol", type=float, default=20.0, help="success: landing error at most this, cm")
    a = ap.parse_args()
    paths = [p for f in a.folders for p in sorted(glob.glob(f)) if os.path.exists(os.path.join(p, "events.csv"))]
    cols = ["takeoff", "mode", "success", "landing_err_cm", "intervention", "air_s", "tag_seen_pct", "searches",
            "side_swing_cm", "heading_swing_deg", "end"]
    allrows = []
    for p in paths:
        rows = flights(p)
        if not rows:
            continue
        for r in rows:
            r["success"] = ("-" if r["landing_err_cm"] is None and not r["intervention"]
                            else int(not r["intervention"] and r["landing_err_cm"] is not None
                                     and r["landing_err_cm"] <= a.tol))
            r["folder"] = os.path.basename(os.path.normpath(p))
        allrows += rows
        os.makedirs(os.path.join(p, "review"), exist_ok=True)
        with open(os.path.join(p, "review", "scorecard.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["folder"] + cols + ["landed_along_cm", "landed_side_cm"],
                               extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)
        print(f"\n{os.path.basename(os.path.normpath(p))}")
        print("  " + " | ".join(c for c in cols))
        for r in rows:
            print("  " + " | ".join(fmt(r[c]) for c in cols))
    if not allrows:
        print("no takeoffs found")
        return
    appr = [r for r in allrows if r["mode"] in ("approach", "hop")]
    scored = [r for r in appr if r["success"] != "-"]
    errs = [r["landing_err_cm"] for r in appr if r["landing_err_cm"] is not None]
    med = lambda v: round(statistics.median(v)) if v else "-"
    print(f"\nTOTAL: {len(allrows)} takeoffs, {len(appr)} approach/hop")
    print(f"  SUCCESS (<= {a.tol:g} cm, no intervention): {sum(r['success'] == 1 for r in scored)}/{len(scored)}"
          f" scored  (unscored: no landing measurement)")
    print(f"  landing error median {med(errs)} cm   interventions {sum(r['intervention'] for r in appr)}/{len(appr)}"
          f"   air time median {med([r['air_s'] for r in appr])} s")
    print(f"  tag seen median {med([r['tag_seen_pct'] for r in appr if r['tag_seen_pct'] is not None])} %"
          f"   searches median {med([r['searches'] for r in appr])}"
          f"   side swing median {med([r['side_swing_cm'] for r in appr if r['side_swing_cm'] is not None])} cm"
          f"   heading swing median {med([r['heading_swing_deg'] for r in appr if r['heading_swing_deg'] is not None])}"
          f" deg")


if __name__ == "__main__":
    main()
