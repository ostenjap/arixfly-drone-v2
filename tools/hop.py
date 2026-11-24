"""Day 3 open-loop hop: fly a JSON show script (timed moves) with hard caps and a kill key.

Show script (shows/*.json), checked before anything is sent; any error = refuse to fly:
  {"name": "hover_test", "trim": {"roll": 0, "pitch": 0},
   "moves": [{"move": "takeoff", "s": 3}, {"move": "hover", "s": 3},
             {"move": "forward", "s": 1.0, "power": 0.5}, {"move": "land"}]}
  power 0..1 is a fraction of a FULL KEY PRESS in the flight window (sticks 40 % -> byte 179,
  throttle 55 % -> byte 198, the tethered 2026-11-08 defaults), so power 1.0 never exceeds what was flown.
  First move must be takeoff, last must be land, nothing else may be takeoff/land. Per-move caps in MOVES.
Directions = the keyboard window's keys (forward = arrow up = pitch byte up, right = arrow right,
up = W, yaw_right = D). Throttle up verified in the air; the other signs only as flown tethered.

Flight window (must stay in front; focus lost while flying -> land):
  C connect   G calibrate (drone level)   hold Enter 1 s = run the script
  Space = KILL (0x04 e-stop, then packets stop)   L = land now   Esc = quit (lands first if airborne)
Total cap (--cap, default 30 s, max 60) from takeoff -> land. After land 0x02: LAND_S centred, then stop.
After the flight the terminal asks where it landed (cm from the start spot) -> captures/hops/hops.csv.

Usage:
  python tools/hop.py shows/hover_test.json --check     # validate + print the timeline, send nothing
  python tools/hop.py shows/hover_test.json --dry       # window, no Wi-Fi, nothing sent
  python tools/hop.py shows/hover_test.json             # LIVE (tethered, soft ground, nobody near)
  python tools/hop.py --fit                             # drift + cm/s per power from hops.csv -> calib/moves.json
  python tools/hop.py --to-b 80 [--power 0.5]           # write shows/hop_to_b_80cm.json from calib/moves.json
"""
import argparse
import csv
import json
import os
import sys
import time

import pygame

from joystick_fly import (CALIBRATE, CAL_S, ESTOP, KILL_S, LAND, LAND_S, PULSE_S, TAKEOFF, TRIM_MAX, Link)
from replay_test import DRONE_SSID, RATE_HZ, build

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
HOPS_DIR = os.path.join(ROOT, "captures", "hops")
HOPS_CSV = os.path.join(HOPS_DIR, "hops.csv")
MOVES_JSON = os.path.join(ROOT, "calib", "moves.json")
SHOWS_DIR = os.path.join(ROOT, "shows")

STICK_LIMIT, THROTTLE_LIMIT = 0.40, 0.55  # tethered defaults; power 1.0 = these
MAX_CAP_S = 60
ARM_HOLD_S = 1.0
RAMP_S = 0.25       # stick slews centre -> full power in this time (both ways), like a key press
# move -> (packet byte or None, sign, min s, max s, default s)
MOVES = {
    "takeoff":   (None, 0, 2.0, 6.0, 3.0),   # 0x01 pulse, then centred sticks while it climbs
    "hover":     (None, 0, 0.0, 10.0, 2.0),
    "forward":   (23, +1, 0.1, 3.0, None),
    "back":      (23, -1, 0.1, 3.0, None),
    "right":     (22, +1, 0.1, 3.0, None),
    "left":      (22, -1, 0.1, 3.0, None),
    "up":        (20, +1, 0.1, 2.0, None),
    "down":      (20, -1, 0.1, 2.0, None),
    "yaw_right": (21, +1, 0.1, 3.0, None),
    "yaw_left":  (21, -1, 0.1, 3.0, None),
    "land":      (None, 0, 0.0, 0.0, 0.0),   # 0x02 pulse; LAND_S of centred sticks follow, outside the cap
}
TRANSLATE = ("forward", "back", "right", "left")


# ---------------------------------------------------------------- script check
def validate(show, cap):
    """Return (moves with defaults filled in, total airborne seconds). Raise ValueError on any problem."""
    if not isinstance(show, dict) or not isinstance(show.get("moves"), list) or not show["moves"]:
        raise ValueError("script needs a non-empty 'moves' list")
    unknown = set(show) - {"name", "note", "trim", "moves"}
    if unknown:
        raise ValueError(f"unknown top-level keys {sorted(unknown)}")
    trim = show.get("trim", {})
    if set(trim) - {"roll", "pitch"}:
        raise ValueError("trim takes only 'roll' and 'pitch'")
    for k, v in trim.items():
        if not isinstance(v, int) or abs(v) > TRIM_MAX:
            raise ValueError(f"trim {k} must be an integer within +/-{TRIM_MAX}")
    out, total = [], 0.0
    names = [m.get("move") if isinstance(m, dict) else None for m in show["moves"]]
    if names[0] != "takeoff" or names[-1] != "land":
        raise ValueError("first move must be 'takeoff' and last must be 'land'")
    if names.count("takeoff") != 1 or names.count("land") != 1:
        raise ValueError("exactly one takeoff and one land")
    for i, m in enumerate(show["moves"]):
        name = m.get("move")
        if name not in MOVES:
            raise ValueError(f"move {i}: unknown '{name}' (known: {', '.join(MOVES)})")
        if set(m) - {"move", "s", "power"}:
            raise ValueError(f"move {i} ({name}): unknown keys {sorted(set(m) - {'move', 's', 'power'})}")
        byte, _, lo, hi, default = MOVES[name]
        s = m.get("s", default)
        if name == "land":
            s = 0.0
        if not isinstance(s, (int, float)) or not lo <= s <= hi:
            raise ValueError(f"move {i} ({name}): s must be {lo}..{hi} seconds, got {s}")
        power = m.get("power", 0.0 if byte is None else None)
        if byte is None and power:
            raise ValueError(f"move {i} ({name}) takes no power")
        if byte is not None and (not isinstance(power, (int, float)) or not 0.05 <= power <= 1.0):
            raise ValueError(f"move {i} ({name}): power must be 0.05..1.0, got {power}")
        out.append({"move": name, "s": float(s), "power": float(power)})
        total += s
    if total > cap:
        raise ValueError(f"script is {total:.1f} s airborne, over the {cap:.0f} s cap")
    return out, total


def axes_for(move, power):
    """Target sticks for a move: list of 4 values -1..1 for bytes 20..23 (before limits)."""
    v = [0.0] * 4
    byte, sign = MOVES[move][:2]
    if byte is not None:
        v[byte - 20] = sign * power
    return v


def to_bytes(v, trim):
    out = []
    for i, x in enumerate(v):
        limit = THROTTLE_LIMIT if i == 0 else STICK_LIMIT
        out.append(max(0, min(255, round(0x80 + x * limit * 127))))
    out[2] = max(0, min(255, out[2] + trim.get("roll", 0)))
    out[3] = max(0, min(255, out[3] + trim.get("pitch", 0)))
    return out


def timeline(moves, trim):
    t = 0.0
    for m in moves:
        b = to_bytes(axes_for(m["move"], m["power"]), trim)
        extra = "  +0x01 pulse" if m["move"] == "takeoff" else ("  +0x02 pulse, then centred "
                                                                  f"{LAND_S:.0f} s, stop" if m["move"] == "land" else "")
        print(f"  {t:5.1f}s  {m['move']:9} {m['s']:4.1f}s  power {m['power']:.2f}  "
              f"thr {b[0]} yaw {b[1]} roll {b[2]} pitch {b[3]}{extra}")
        t += m["s"]


# ---------------------------------------------------------------- calibration from logged hops
def read_hops():
    if not os.path.exists(HOPS_CSV):
        return []
    with open(HOPS_CSV, newline="") as f:
        return list(csv.DictReader(f))


def fit():
    """Hover drift = mean landing offset of hops with no translate move; forward cm/s per power from the rest."""
    rows = [r for r in read_hops() if r["outcome"] == "landed" and r["dx_cm"] != "" and r["live"] == "1"]
    hover = [r for r in rows if all(float(r[f"{k}_s"]) == 0 for k in TRANSLATE)]
    if not hover:
        print("no landed hover-only hops with a measured offset yet - fly shows/hover_test.json first")
        return
    ddx = sum(float(r["dx_cm"]) for r in hover) / len(hover)
    ddy = sum(float(r["dy_cm"]) for r in hover) / len(hover)
    print(f"hover drift from {len(hover)} hops: forward {ddx:+.0f} cm, right {ddy:+.0f} cm per flight")
    speeds = {}
    for r in rows:
        if float(r["forward_s"]) > 0 and all(float(r[f"{k}_s"]) == 0 for k in TRANSLATE if k != "forward"):
            speeds.setdefault(r["forward_power"], []).append((float(r["dx_cm"]) - ddx) / float(r["forward_s"]))
    for p, v in sorted(speeds.items()):
        print(f"  forward at power {p}: {sum(v) / len(v):.0f} cm/s from {len(v)} hops ({', '.join(f'{x:.0f}' for x in v)})")
    cal = {"hover_drift_cm": {"forward": round(ddx, 1), "right": round(ddy, 1), "n": len(hover)},
           "forward_cm_per_s": {p: round(sum(v) / len(v), 1) for p, v in speeds.items()},
           "fitted": time.strftime("%Y-%m-%d %H:%M")}
    os.makedirs(os.path.dirname(MOVES_JSON), exist_ok=True)
    with open(MOVES_JSON, "w") as f:
        json.dump(cal, f, indent=2)
    print("written", os.path.normpath(MOVES_JSON))


def plan_to_b(dist_cm, power):
    if not os.path.exists(MOVES_JSON):
        sys.exit("no calib/moves.json - fly hover_test + forward_test, log offsets, then run --fit")
    cal = json.load(open(MOVES_JSON))
    speed = cal["forward_cm_per_s"].get(f"{power:g}")
    if not speed:
        sys.exit(f"no forward speed measured at power {power:g} (have {list(cal['forward_cm_per_s'])})")
    fwd_s = round((dist_cm - cal["hover_drift_cm"]["forward"]) / speed, 2)
    show = {"name": f"hop_to_b_{dist_cm:g}cm",
            "note": f"open loop, {speed} cm/s at power {power:g}, hover drift {cal['hover_drift_cm']} subtracted",
            "moves": [{"move": "takeoff", "s": 3}, {"move": "hover", "s": 2},
                      {"move": "forward", "s": fwd_s, "power": power}, {"move": "hover", "s": 2},
                      {"move": "land"}]}
    try:
        moves, total = validate(show, MAX_CAP_S)  # refuses a hop that needs more than one 3 s forward move
    except ValueError as e:
        sys.exit(f"REFUSED: {e} - use a higher --power or a shorter hop")
    path = os.path.join(SHOWS_DIR, show["name"] + ".json")
    with open(path, "w") as f:
        json.dump(show, f, indent=2)
    print(f"written {os.path.normpath(path)} ({total:.1f} s airborne):")
    timeline(moves, {})


def log_hop(name, moves, trim, outcome, airborne_s, live):
    """Ask for the landing offset in the terminal and append one row to hops.csv."""
    print(f"\nhop '{name}' ended: {outcome}, {airborne_s:.1f} s airborne")
    dx = dy = ""
    if outcome == "landed":
        try:
            dx = input("landed how far FORWARD of the start spot, cm (nose direction, back = negative, Enter = skip)? ").strip()
            dy = input("landed how far RIGHT of the start spot, cm (left = negative)? ").strip() if dx else ""
            float(dx or 0), float(dy or 0)
        except (ValueError, EOFError):
            print("not a number - offset not logged")
            dx = dy = ""
    note = ""
    try:
        note = input("note (e.g. 'on B', 'tether pulled', Enter = none)? ").strip()
    except EOFError:
        pass
    row = {"time": time.strftime("%Y-%m-%d %H:%M:%S"), "show": name, "live": int(live), "outcome": outcome,
           "airborne_s": f"{airborne_s:.1f}", "trim_roll": trim.get("roll", 0), "trim_pitch": trim.get("pitch", 0)}
    for k in TRANSLATE:
        row[f"{k}_s"] = sum(m["s"] for m in moves if m["move"] == k)
    fp = [m["power"] for m in moves if m["move"] == "forward"]
    row["forward_power"] = f"{fp[0]:g}" if fp and len(set(fp)) == 1 else ""
    row.update({"dx_cm": dx, "dy_cm": dy, "note": note})
    os.makedirs(HOPS_DIR, exist_ok=True)
    new = not os.path.exists(HOPS_CSV)
    with open(HOPS_CSV, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(row))
        if new:
            w.writeheader()
        w.writerow(row)
    print("logged to", os.path.normpath(HOPS_CSV))


# ---------------------------------------------------------------- flight window
def draw(screen, font, big, lines, state):
    color = {"FLYING": (251, 191, 36), "LANDING": (167, 139, 250), "KILLING": (248, 113, 113),
             "READY": (45, 212, 191), "SAFE": (74, 222, 128), "DONE": (113, 113, 122)}.get(state, (244, 244, 245))
    screen.fill((9, 9, 11))
    screen.blit(big.render(state, True, color), (30, 20))
    for i, (txt, col) in enumerate(lines):
        screen.blit(font.render(txt, True, col), (30, 110 + i * 34))
    pygame.display.flip()


def fly(path, show, moves, total, cap, dry, auto):
    trim = show.get("trim", {})
    name = show.get("name", os.path.splitext(os.path.basename(path))[0])
    if auto:
        os.environ["SDL_VIDEODRIVER"] = "dummy"
    pygame.init()
    screen = pygame.display.set_mode((900, 520))
    pygame.display.set_caption(f"HS420 hop: {name}" + (" (DRY RUN)" if dry else ""))
    font, big = pygame.font.SysFont("consolas", 24), pygame.font.SysFont("consolas", 64, bold=True)
    clock, link = pygame.time.Clock(), Link(dry)
    state, note = "OFFLINE", "press C to connect"
    seq, pulse_bit, pulse_until, hold_t, kill_until = 0, 0, 0.0, None, 0.0
    start_t, land_t, move_i, move_t0, end_t = None, None, 0, 0.0, None
    sticks, outcome, rows, t0, logged = [0.0] * 4, "not flown", [], time.perf_counter(), None
    script_keys = {0.3: "c", 1.5: "g", 3.0: "enter"} if auto else {}
    try:
        while state != "DONE":
            now = time.perf_counter()
            keys_down = set()
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    keys_down.add("esc")
                elif ev.type == pygame.KEYDOWN:
                    keys_down.add({pygame.K_SPACE: "space", pygame.K_ESCAPE: "esc", pygame.K_c: "c",
                                   pygame.K_g: "g", pygame.K_l: "l"}.get(ev.key, ""))
            for at in [k for k in script_keys if now - t0 >= k]:
                keys_down.add(script_keys.pop(at))
            enter = "enter" in keys_down or (not auto and pygame.key.get_pressed()[pygame.K_RETURN])
            focused = auto or pygame.key.get_focused()
            airborne = state in ("FLYING", "LANDING")

            # --- state machine
            if "space" in keys_down and state in ("SAFE", "READY", "FLYING", "LANDING"):
                pulse_bit, pulse_until, kill_until = ESTOP, now + KILL_S, now + KILL_S
                state, note, outcome = "KILLING", "EMERGENCY STOP 0x04, then packets stop", "killed"
            if state == "KILLING" and now >= kill_until:
                state = "DONE"
            if "esc" in keys_down:
                if state == "FLYING":
                    keys_down.add("l")
                elif state != "LANDING":
                    state = "DONE"
            if state == "OFFLINE" and "c" in keys_down:
                link.connect()
                state, note = "CONNECTING", f"joining {DRONE_SSID} ..."
            if state == "CONNECTING" and not link.busy:
                state, note = ("SAFE", "drone level on table A, press G to calibrate") if link.sock else \
                    ("OFFLINE", (link.error or "join failed") + " - C to retry")
            if state in ("SAFE", "READY") and "g" in keys_down:
                pulse_bit, pulse_until = CALIBRATE, now + CAL_S
                state, note = "SAFE", "calibrating gyro ..."
            if state == "SAFE" and pulse_bit == CALIBRATE and now >= pulse_until:
                state, note = "READY", "hold Enter 1 s to run the script   (Space = KILL)"
            if state == "READY" and enter:
                hold_t = hold_t or now
                if now - hold_t >= ARM_HOLD_S or auto:
                    state, start_t, move_i, move_t0 = "FLYING", now, 0, now
                    pulse_bit, pulse_until = TAKEOFF, now + PULSE_S
            elif not enter:
                hold_t = None
            if state == "FLYING":
                reason = ("L pressed" if "l" in keys_down else
                          "window lost focus" if not focused else
                          f"time cap {cap:.0f} s" if now - start_t >= cap else None)
                while reason is None and move_i < len(moves) - 1 and now - move_t0 >= moves[move_i]["s"]:
                    move_i, move_t0 = move_i + 1, move_t0 + moves[move_i]["s"]
                if reason or moves[move_i]["move"] == "land":
                    outcome = "landed" if not reason else f"aborted ({reason})"
                    pulse_bit, pulse_until, land_t = LAND, now + PULSE_S, now
                    state, note = "LANDING", ("script done" if not reason else reason) + " - landing, Space = KILL"
                    end_t = now
                else:
                    m = moves[move_i]
                    note = f"move {move_i + 1}/{len(moves)}: {m['move']} {m['s']:.1f}s power {m['power']:.2f}"
            if state == "LANDING" and now - land_t >= LAND_S:
                state, note = "DONE", "landed - packets stopped"
            if now >= pulse_until:
                pulse_bit = 0

            # --- sticks: slew toward the current move's target
            target = axes_for(moves[move_i]["move"], moves[move_i]["power"]) if state == "FLYING" else [0.0] * 4
            step = (1.0 / RATE_HZ) / RAMP_S
            sticks = [s + max(-step, min(step, t - s)) for s, t in zip(sticks, target)]
            axes = to_bytes(sticks, trim) if state in ("FLYING", "LANDING") else [0x80] * 4
            button = pulse_bit if now < pulse_until else 0
            if state in ("SAFE", "READY", "FLYING", "LANDING", "KILLING") and link.sock:
                if link.send(build(seq, axes=axes, button=button)):
                    seq += 1
                elif airborne:
                    state, outcome = "DONE", f"link lost ({link.error}) - drone failsafe stops motors"
                else:
                    state, note = "OFFLINE", f"{link.error} - C to reconnect"
            if state in ("FLYING", "LANDING", "KILLING"):
                rows.append((round(now - t0, 3), state, moves[move_i]["move"] if state == "FLYING" else "",
                             *axes, button))
            if (state, note) != logged:
                print(f"{now - t0:7.2f}s  {state:10} {note}", flush=True)
                logged = (state, note)

            left = cap - (now - start_t) if state == "FLYING" else None
            draw(screen, font, big, [
                (note, (244, 244, 245)),
                (f"show {name}: {len(moves)} moves, {total:.1f} s airborne, cap {cap:.0f} s"
                 + (f", {left:.1f} s left" if left is not None else ""), (113, 113, 122)),
                (f"sticks thr {axes[0]} yaw {axes[1]} roll {axes[2]} pitch {axes[3]}  button 0x{button:02x}  "
                 f"seq {seq}  link {link.ip or '-'}", (113, 113, 122)),
                ("", (0, 0, 0)),
                ("SPACE = KILL     L = land now     Esc = quit (lands first)", (248, 113, 113)),
                ("C connect   G calibrate   hold Enter 1 s = run", (113, 113, 122)),
                ("tethered, soft ground, nobody within 2 m, Mullvad off", (251, 191, 36)),
            ], state)
            clock.tick(RATE_HZ)
    finally:
        link.home()
        pygame.quit()
    airborne_s = ((end_t or time.perf_counter()) - start_t) if start_t else 0.0
    print(f"closed ({seq} packets sent)")
    if rows:
        os.makedirs(HOPS_DIR, exist_ok=True)
        out = os.path.join(HOPS_DIR, f"{time.strftime('%Y%m%d_%H%M%S')}_{name}{'_dry' if dry else ''}.csv")
        with open(out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "state", "move", "throttle", "yaw", "roll", "pitch", "button"])
            w.writerows(rows)
        print("packet log", os.path.normpath(out))
    if start_t and not auto:
        log_hop(name, moves, trim, outcome, airborne_s, live=not dry)
    return outcome


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("show", nargs="?", help="show script JSON")
    ap.add_argument("--check", action="store_true", help="validate and print the timeline only")
    ap.add_argument("--dry", action="store_true", help="window without Wi-Fi, nothing sent")
    ap.add_argument("--auto", action="store_true", help="with --dry: no window, keys pressed by a timer (self-test)")
    ap.add_argument("--cap", type=float, default=30, help=f"airborne seconds before auto land (max {MAX_CAP_S})")
    ap.add_argument("--fit", action="store_true", help="fit drift and forward speed from captures/hops/hops.csv")
    ap.add_argument("--to-b", type=float, metavar="CM", help="write an A->B hop script for this distance")
    ap.add_argument("--power", type=float, default=0.5, help="forward power for --to-b")
    a = ap.parse_args()
    if a.fit:
        return fit()
    if a.to_b:
        return plan_to_b(a.to_b, a.power)
    if not a.show:
        ap.error("give a show script, --fit or --to-b")
    if a.auto and not a.dry:
        ap.error("--auto only with --dry")
    cap = min(a.cap, MAX_CAP_S)
    with open(a.show) as f:
        show = json.load(f)
    try:
        moves, total = validate(show, cap)
    except ValueError as e:
        sys.exit(f"REFUSED: {e}")
    print(f"{a.show}: OK, {len(moves)} moves, {total:.1f} s airborne (cap {cap:.0f} s), trim {show.get('trim', {})}")
    timeline(moves, show.get("trim", {}))
    if a.check:
        return
    print(f"result: {fly(a.show, show, moves, total, cap, a.dry, a.auto)}")


if __name__ == "__main__":
    main()
