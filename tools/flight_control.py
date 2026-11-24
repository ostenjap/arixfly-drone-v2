"""Flight control for tools/live_flight.py: control law, tag 2 detection, live-tuning store and the control
constants (split out of live_flight.py on 2026-11-10, code unchanged). See tools/live_flight.py for usage.
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))


TAG_ID, TAG_CM = 2, 10.0   # the BOTTOM tag (on the shoe box): the target

TOP_ID = 1                 # the TOP tag (on the stand above the box): used when the bottom one is not found
TOP_TO_BOTTOM_CM = (0.0, 37.0, 10.0)  # bottom-tag centre from the top-tag centre, camera axes (x right, y down,
                                      # z away): measured 2026-11-10 21:55 on the ground at 250 cm (1 frame)

CLIMB_S, ACQUIRE_S, APPROACH_S = 3.0, 6.0, 15.0

LOCK_FRAMES, ARRIVE_FRAMES = 3, 3

FRESH_S, LOST_S = 0.5, 1.5           # sighting age used for control / tag lost -> land

TARGET_CM, ARRIVE_TOL_CM, OVERSHOOT_CM, AWAY_CM = 60.0, 8.0, 20.0, 30.0

KP_RANGE, MAX_FWD, MAX_BACK, OFF_CENTRE_DEG = 0.006, 0.5, 0.3, 20.0

KI_RANGE, MAX_I = 0.002, 0.3   # integral on range (power per cm*s): beats steady drift (sim: P alone stalls at

                               # ~100 cm against 20 cm/s backward drift)
KP_YAW, MAX_YAW, YAW_DEAD = 0.02, 0.4, 2.0

SMOOTH, RAMP_S = 0.5, 0.25

STICK_LIMIT, THROTTLE_LIMIT = 0.40, 0.55   # power 1.0 = these (tethered defaults, as hop.py / keyboard)

AIR = ("FLYING", "ACQUIRE", "APPROACH", "RECOVER", "HOP", "BRAKE", "DESCEND")

# brake = pitch back for BRAKE_S before landing / searching (22:39 flight: tag lost at 134 cm while moving ~30
# cm/s, sticks centred -> glided over the box). Strong because the pitch trim (+16..+22 bytes) is added on top.
BRAKE_POWER, BRAKE_S = -0.7, 0.5   # strongest brake; used: 1.4 x the recent forward push, at least 0.2
# tag leaves the picture at the BOTTOM this close (camera tilted down, drone above the tag) -> brake + land
CLOSE_ELEV_DEG = -3.0   # (distance: tuning close_cm) 22:39 sim: last sighting before the edge was at -8 deg

HOP_CLIMB_S, HOP_MAX_S = 2.5, 8.0   # hop: hover this long after takeoff, then forward at most this long

KP_ALT, MAX_ALT, ALT_DEAD = 0.03, 0.35, 3.0   # approach with --axes ...,alt: tag below centre -> descend

LOST_RECOVER_S = 0.5      # approach: tag not seen this long -> RECOVER (tools/tracker.py), not land

ACQUIRE_SEARCH_S = 1.5    # acquire: no tag this long but seen before (ground / earlier) -> RECOVER search

REACQUIRE_FRAMES = 2      # sightings in a row that end a recovery

MAX_RECOVERIES = 3        # per flight, then land

TUNING_FILE = os.path.join(HERE, "..", "calib", "tuning.json")

OLD_TRIM_FILE = os.path.join(HERE, "..", "calib", "trim.json")

TRIM_FILE = TUNING_FILE   # tests point this at a scratch file

# name, default, step, min, max, help (shown in the window). Live keys: Tab / Shift+Tab pick, = / - change,
# arrows = pitch/roll trim, [ / ] = yaw trim. Every change is saved at once and logged in events.csv.
TUNE_SPEC = [
    ("trim_yaw", 0, 2, -60, 60, "yaw trim: + turns right, - turns left"),
    ("trim_pitch", 0, 2, -40, 40, "pitch trim: + goes forward, - goes back"),
    ("trim_roll", 0, 2, -40, 40, "roll trim: + goes right, - goes left"),
    ("hold_on", 1, 1, 0, 1, "heading hold by camera: 1 on, 0 off"),
    ("hold_gain", 0.03, 0.005, 0.0, 0.1, "heading hold strength per deg"),
    ("hold_max", 0.4, 0.05, 0.0, 0.8, "strongest heading-hold yaw"),
    ("hold_delay", 1.0, 0.25, 0.0, 5.0, "s after takeoff before the hold starts"),
    ("yaw_check", 0, 1, 0, 1, "wrong-yaw check (off: false alarms, sign verified)"),
    ("approach_gain", 0.006, 0.001, 0.0, 0.02, "forward power per cm to the target"),
    ("approach_max", 0.5, 0.05, 0.1, 0.8, "strongest forward push"),
    ("target_cm", 60, 5, 30, 150, "approach: land this far from the tag"),
    ("hop_power", 0.5, 0.05, 0.1, 0.8, "hop: forward push (power)"),
    ("hop_speed", 60.0, 2.0, 5.0, 150.0, "hop: cm/s at hop_power (auto-set after each hop)"),
    ("brake_max", 0.7, 0.05, 0.0, 1.0, "brake: strongest pull back before landing / searching"),
    ("close_cm", 80, 10, 0, 200, "tag lost below centre within target+this -> brake+land"),
    ("alt_on", 1, 1, 0, 1, "approach height control (off: camera tilted down)"),
    ("recover_s", 4.0, 0.5, 1.0, 9.0, "search this long for a lost tag, then land"),
    ("yaw_i", 0.012, 0.002, 0.0, 0.05, "v2: yaw learning (power per deg*s), 0 = off"),
]

TUNE_DEFAULT = {k: d for k, d, *_ in TUNE_SPEC}

def load_tuning():
    import json
    t = dict(TUNE_DEFAULT)
    try:
        t.update({k: v for k, v in json.load(open(TRIM_FILE)).items() if k in t})
    except (OSError, ValueError):
        try:   # first run: take the arrow-key trim saved by the previous version
            old = json.load(open(OLD_TRIM_FILE))
            t["trim_pitch"], t["trim_roll"] = int(old.get("pitch", 0)), int(old.get("roll", 0))
        except (OSError, ValueError):
            pass
    return t

def save_tuning(t):
    import json
    os.makedirs(os.path.dirname(TRIM_FILE), exist_ok=True)
    with open(TRIM_FILE + ".tmp", "w") as f:
        json.dump(t, f, indent=1)
    os.replace(TRIM_FILE + ".tmp", TRIM_FILE)   # atomic: a live reader never sees half a file

def tuning_mtime():
    try:
        return os.path.getmtime(TRIM_FILE)
    except OSError:
        return None

def reload_tuning(t):
    """Values changed in the file by someone else (Claude between flights) -> into t. Returns [(name, old,
    new)]; a half-written or broken file is skipped (never resets anything)."""
    import json
    try:
        new = json.load(open(TRIM_FILE))
    except (OSError, ValueError):
        return []
    out = []
    for k, v in new.items():
        if k in t and isinstance(v, (int, float)) and v != t[k]:
            _, _, step, lo, hi, _ = next(x for x in TUNE_SPEC if x[0] == k)
            v = clamp(v, lo, hi)
            out.append((k, t[k], v))
            t[k] = v
    return out

def tune_step(t, name, sign):
    _, _, step, lo, hi, _ = next(x for x in TUNE_SPEC if x[0] == name)
    v = clamp(t[name] + sign * step, lo, hi)
    t[name] = round(v, 4) if isinstance(step, float) else int(v)
    return t[name]

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def control(sm, axes, target, integ=0.0, kp=KP_RANGE, max_fwd=MAX_FWD):
    """Smoothed sighting {dist, bearing, elev} -> sticks [throttle, yaw, roll, pitch] in power units.
    integ = integral of (range - target) over time (cm*s), kept by the caller."""
    out = [0.0] * 4
    if "pitch" in axes:
        i_term = clamp(KI_RANGE * integ, -MAX_I, MAX_I)
        fwd = clamp(kp * (sm["dist"] - target) + i_term, -MAX_BACK, max_fwd)
        if fwd > 0:   # pitch only: no forward while off-centre (it would lose the tag sideways); with yaw on, the
            # tag is being turned toward, so keep >= 30 % (23:42: tag at -27 deg -> forward 0 -> drifted back)
            fade = (max(0.3, 1 - abs(sm["bearing"]) / 45.0) if "yaw" in axes
                    else max(0.0, 1 - abs(sm["bearing"]) / OFF_CENTRE_DEG))
            fwd *= fade
        out[3] = fwd
    if "yaw" in axes and abs(sm["bearing"]) > YAW_DEAD:
        out[1] = clamp(KP_YAW * sm["bearing"], -MAX_YAW, MAX_YAW)
    if "alt" in axes and abs(sm["elev"]) > ALT_DEAD:
        out[0] = clamp(KP_ALT * sm["elev"], -MAX_ALT, MAX_ALT)
    return out

def to_bytes(power):
    """[throttle, yaw, roll, pitch] power -> bytes 20..23 (centre 0x80), throttle at its own limit."""
    return [clamp(round(0x80 + v * (THROTTLE_LIMIT if i == 0 else STICK_LIMIT) * 127), 0, 255)
            for i, v in enumerate(power)]

_DETECTORS = None

_GAMMA2 = None

def find_tag(img, f, use_top=False):
    """Bottom-tag (tag 2) pose or None. Detector tuned on the drone's own frames (2026-11-09, taped tag at 138 cm):
    pass 1: adaptive threshold window up to 73 px, constant 3 -> 106/190 frames (defaults: 58);
    pass 2 (only if pass 1 finds nothing): image darkened with gamma 2, constant 1 -> 74/95 alone (pass-1
    settings on the same frames: 48/95). 0 false hits on 56 tag-free frames each. Tape glare (22:31 run) defeats
    every setting tried (0/115) -> matte tag without tape.
    use_top: if the bottom tag is not found but the top tag (TOP_ID) is, return where the BOTTOM tag must be
    (top-tag position + TOP_TO_BOTTOM_CM), so the controller is unchanged. Result "src" = "bottom" / "top"."""
    global _DETECTORS, _GAMMA2
    import math
    import cv2
    import numpy as np
    from aruco_distance import pose
    if _DETECTORS is None:
        dets = []
        for const in (3, 1):
            prm = cv2.aruco.DetectorParameters()
            prm.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            prm.adaptiveThreshWinSizeMax, prm.adaptiveThreshConstant = 73, const
            dets.append(cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), prm))
        _DETECTORS = dets
        _GAMMA2 = np.array([255 * (i / 255.0) ** 2 for i in range(256)], np.uint8)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    top = None
    for det, view in ((_DETECTORS[0], gray), (_DETECTORS[1], None)):
        if view is None:
            view = cv2.LUT(gray, _GAMMA2)
        corners, ids, _ = det.detectMarkers(view)
        found = [] if ids is None else list(ids.flatten())
        if TAG_ID in found:
            c, src = corners[found.index(TAG_ID)][0], "bottom"
            break
        if use_top and top is None and TOP_ID in found:
            top = corners[found.index(TOP_ID)][0]
    else:
        if top is None:
            return None
        c, src = top, "top"
    side = float(np.mean([np.linalg.norm(c[k] - c[(k + 1) % 4]) for k in range(4)]))
    h, w = gray.shape
    r = pose({"corners": c, "side": side}, TAG_CM, f, w, h)
    if not r:
        return None
    if src == "top":
        x, y, z = (r[k] + d for k, d in zip(("x", "y", "z"), TOP_TO_BOTTOM_CM))
        r = dict(r, x=x, y=y, z=z, dist=math.sqrt(x * x + y * y + z * z),
                 bearing=math.degrees(math.atan2(x, z)), elev=math.degrees(math.atan2(-y, z)))
    return dict(r, corners=c, src=src)

