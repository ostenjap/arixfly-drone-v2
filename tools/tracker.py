"""Tag memory + recovery for live_flight.py: where was tag 2 last seen, where is it now, how to get it back.

Proven pieces, kept small:
  1. Visual odometry for heading (YawVO): sparse optical flow (Shi-Tomasi corners + pyramidal Lucas-Kanade) on a
     320x180 copy of each frame; the median horizontal shift of the room = the drone's yaw change, the median
     vertical shift ~ its tilt/climb seen as elevation change. Rejected when the flow is inconsistent (blur,
     close objects). Checked on the 2026-11-09 22:58 flight: VO yaw vs. the tag's own bearing change over 42 frame
     pairs: median error 0.02 deg, max 0.13 deg; valid on 78 % of in-flight frame pairs (the rest: motion blur).
  2. Tag map (TagMap): every sighting is stored in a world frame anchored at the start (heading 0 = nose at the
     first frame): world bearing = VO heading + camera bearing, plus elevation, range and an alpha-beta rate filter
     (the constant-velocity predictor SORT/Kalman trackers use during occlusion). While the tag is hidden its
     camera bearing/elevation is predicted from the map and the current VO heading/tilt.
  3. Recovery (Recover): the last-known-position search used by follow-me drones: settle while the picture is
     blurred, turn toward the predicted bearing and climb/descend toward the predicted elevation, then back off a
     little (wider view), then a +-30 deg sector sweep around the remembered bearing, then give up (land).
     A yaw-sign check compares the commanded turn with the VO heading and aborts if they disagree. Never steers
     while VO is blind > 0.5 s (blind > 2.5 s: hover centred and wait for a tag, no landing); climb/descend at most 2 s per search.
  4. Heading hold (heading_hold): P control on the VO heading against the drone's own yaw drift (+6.7 deg/s on
     the 23:42 flight with centred sticks), used whenever the tag is not steering yaw.
Usage: imported by tools/live_flight.py (--approach). Self-check on recorded frames:
  python tools/tracker.py captures/flights/20261109_225836
"""
import math
import sys

import numpy as np

VO_W, VO_H = 320, 180
VO_MIN_POINTS, VO_MAX_SPREAD_PX = 15, 3.0
VO_REF_MAX_S = 0.6           # keep comparing against the last good frame this long through blurred frames

KP_YAW_REC, MAX_YAW_REC = 0.02, 0.35
KP_ALT_REC, MAX_ALT_REC = 0.03, 0.35
SETTLE_S, BACKOFF_AFTER_S, BACKOFF_S, BACKOFF_POWER = 1.0, 1.5, 1.0, -0.2
SWEEP_DEG, SWEEP_POWER, RECOVER_S = 30.0, 0.3, 9.0
CENTRED_DEG, ELEV_DEAD = 6.0, 4.0
VFOV_HALF = 17.0             # camera: +-17 deg vertical (hfov 58 deg at 16:9)
YAW_CHECK_S, YAW_CHECK_DEG = 0.8, 3.0
BLIND_PAUSE_S, BLIND_HOVER_S, ALT_BUDGET_S = 0.5, 2.5, 2.0
KP_HOLD, MAX_HOLD = 0.03, 0.4   # heading hold: power per deg of VO heading error


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class YawVO:
    """Heading (deg, + = turned right) and tilt (deg, + = view moved up) since start, from optical flow."""

    def __init__(self, f_full, width_full):
        import cv2
        self.cv2 = cv2
        self.f = f_full * VO_W / width_full
        self.heading = self.tilt = 0.0
        self.ref = self.t_ref = None
        self.valid = False
        self.t_valid = None
        self.pairs = self.good = 0

    def update(self, img_bgr, t):
        cv2 = self.cv2
        g = cv2.resize(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY), (VO_W, VO_H), interpolation=cv2.INTER_AREA)
        if self.ref is None:
            self.ref, self.t_ref, self.t_valid = g, t, t
            return
        self.pairs += 1
        dyaw = dtilt = None
        p0 = cv2.goodFeaturesToTrack(self.ref, 150, 0.01, 6)
        if p0 is not None and len(p0) >= VO_MIN_POINTS:
            p1, ok, _ = cv2.calcOpticalFlowPyrLK(self.ref, g, p0, None, winSize=(21, 21), maxLevel=3)
            ok = ok.ravel() == 1
            if ok.sum() >= VO_MIN_POINTS:
                d = (p1 - p0).reshape(-1, 2)[ok]
                dx, dy = float(np.median(d[:, 0])), float(np.median(d[:, 1]))
                if float(np.median(np.abs(d[:, 0] - dx))) <= VO_MAX_SPREAD_PX:
                    dyaw = -math.degrees(math.atan2(dx, self.f))   # scene moves left = drone turned right
                    dtilt = math.degrees(math.atan2(dy, self.f))  # scene moves down = view moved up
        if dyaw is not None:
            self.heading += dyaw
            self.tilt += dtilt
            self.ref, self.t_ref, self.valid, self.t_valid = g, t, True, t
            self.good += 1
        else:
            self.valid = False
            if t - self.t_ref > VO_REF_MAX_S:   # blurred too long: restart from here (heading may be off)
                self.ref, self.t_ref = g, t

    def blind_for(self, t):
        return 0.0 if self.t_valid is None else t - self.t_valid

    def blind_for_now(self):
        import time
        return self.blind_for(time.monotonic())


class TagMap:
    """Last sightings of the tag in the world frame + alpha-beta rates, prediction while hidden."""
    ALPHA, BETA = 0.6, 0.2

    def __init__(self):
        self.last = None   # dict(t, wb world bearing, we world elevation, dist, rate_b, rate_e)
        self.count = 0

    def see(self, t, heading, tilt, bearing, elev, dist):
        wb, we = heading + bearing, elev + tilt
        if self.last is None or t - self.last["t"] > 1.5:
            self.last = dict(t=t, wb=wb, we=we, dist=dist, rb=0.0, re=0.0)
        else:
            L, dt = self.last, max(1e-3, t - self.last["t"])
            for k, rk, z in (("wb", "rb", wb), ("we", "re", we)):
                pred = L[k] + L[rk] * dt
                r = z - pred
                L[k] = pred + self.ALPHA * r
                L[rk] = clamp(L[rk] + self.BETA * r / dt, -30, 30)   # deg/s, clamped
            L["t"], L["dist"] = t, dist
        self.count += 1

    def predict(self, t, heading, tilt):
        """(camera bearing, camera elevation, age s) where the tag should be now, or None."""
        if self.last is None:
            return None
        L = self.last
        age = t - L["t"]
        k = min(age, 1.0)  # trust the rate for at most 1 s
        wb = L["wb"] + L["rb"] * k
        we = L["we"] + L["re"] * k
        return wb - heading, we - tilt, age


class YawSignCheck:
    """A firm yaw command must change the TURN RATE its own way, compared with the rate just before it.
    Judging the heading itself gave false alarms: right after takeoff the drone turns right ~15 deg/s by itself,
    faster than the ramping hold correction (2026-11-10 00:02, 4 flights landed). The sign itself was confirmed
    by VO on the 23:42 flight (yaw left 110 -> heading 27.8 -> 21.0 deg)."""
    MIN_CMD, WINDOW_S, RATE_TOL = 0.2, 1.0, 3.0   # power, s, deg/s

    def __init__(self):
        self.hist = []          # (t, heading) while VO is valid
        self.run = None         # (t start, rate before, sign) of the current firm command
        self.bad = False

    def _rate(self, t0, t1):
        pts = [(t, h) for t, h in self.hist if t0 <= t <= t1]
        return None if len(pts) < 3 or pts[-1][0] - pts[0][0] < 0.3 else             (pts[-1][1] - pts[0][1]) / (pts[-1][0] - pts[0][0])

    def update(self, t, yaw_cmd, vo):
        if vo.valid:
            self.hist = [p for p in self.hist if t - p[0] < 3.0] + [(t, vo.heading)]
        if abs(yaw_cmd) < self.MIN_CMD or not vo.valid:
            self.run = None
            return
        sgn = 1 if yaw_cmd > 0 else -1
        if self.run is None or self.run[2] != sgn:
            self.run = (t, self._rate(t - 0.6, t), sgn)
            return
        if t - self.run[0] >= self.WINDOW_S:
            before, during = self.run[1], self._rate(self.run[0] + 0.3, t)
            if before is not None and during is not None and (during - before) * sgn < -self.RATE_TOL:
                self.bad = True
            self.run = (t, during, sgn)


def heading_hold(ref, vo, kp=KP_HOLD, mx=MAX_HOLD):
    """Yaw power that holds the VO heading at ref (drone turned right by itself 6.7 deg/s, 23:42 flight)."""
    if ref is None or vo is None or vo.blind_for_now() > 0.3:
        return 0.0
    return clamp(-kp * (vo.heading - ref), -mx, mx)


class Recover:
    """Stick commands [throttle, yaw, roll, pitch] (power units) to get the tag back into view."""

    def __init__(self, t, heading, check_on=True, max_s=RECOVER_S):
        self.t0, self.phase, self.check_on, self.max_s = t, "SETTLE", check_on, max_s
        self.sweep_dir, self.sweep_centre = 1, None
        self.check = YawSignCheck()
        self.abort = None
        self.t_centred = None
        self.t_backoff = None
        self.alt_used, self.t_prev = 0.0, t

    def step(self, t, pred, vo):
        """pred = TagMap.predict(...) or None; vo = YawVO. Returns (sticks, phase text)."""
        out = [0.0] * 4
        dt, self.t_prev = t - self.t_prev, t
        if t - self.t0 > self.max_s:
            self.abort = f"tag not recovered in {self.max_s:g} s"
            return out, "GIVE UP"
        # blind long = hover with centred sticks and wait for a tag until RECOVER_S, no landing (21:55 flights: plain
        # curtain/wall -> VO blind already during the climb -> every search landed at once). Never steers blind.
        blind = vo.blind_for(t)
        if blind > BLIND_HOVER_S:
            return out, f"HOVER (camera blind {blind:.1f} s: plain wall / blur) - waiting for a tag"
        if blind > 0.3 and t - self.t0 < SETTLE_S + 1.0:
            self.phase = "SETTLE"           # picture blurred (swing / jerk): wait for it to calm down
            return out, "SETTLE (picture blurred)"
        if blind > BLIND_PAUSE_S:           # never steer without VO: 23:42 flight spun for 6 s turning blind
            return out, f"{self.phase} paused (VO blind {blind:.1f} s)"
        if pred is None:
            self.phase = "SWEEP"
            pred_b, pred_e = None, None
        else:
            pred_b, pred_e, _ = pred
        if self.phase in ("SETTLE", "TURN") and pred_b is not None:
            self.phase = "TURN"
            if abs(pred_b) > CENTRED_DEG:
                out[1] = clamp(KP_YAW_REC * pred_b, -MAX_YAW_REC, MAX_YAW_REC)
                self.t_centred = None
            else:
                self.t_centred = self.t_centred or t
            if pred_e is not None and abs(pred_e) > ELEV_DEAD and self.alt_used < ALT_BUDGET_S:
                out[0] = clamp(KP_ALT_REC * pred_e, -MAX_ALT_REC, MAX_ALT_REC)
            if self.t_centred and t - self.t_centred > BACKOFF_AFTER_S:
                self.phase, self.t_backoff = "BACKOFF", t
        if self.phase == "BACKOFF":
            out[3] = BACKOFF_POWER
            if pred_e is not None and abs(pred_e) > ELEV_DEAD and self.alt_used < ALT_BUDGET_S:
                out[0] = clamp(KP_ALT_REC * pred_e, -MAX_ALT_REC, MAX_ALT_REC)
            if t - self.t_backoff > BACKOFF_S:
                self.phase = "SWEEP"
        if self.phase == "SWEEP":
            if self.sweep_centre is None:
                self.sweep_centre = vo.heading + (pred_b or 0.0)
            off = vo.heading - self.sweep_centre
            if off > SWEEP_DEG:
                self.sweep_dir = -1
            elif off < -SWEEP_DEG:
                self.sweep_dir = 1
            out[1] = SWEEP_POWER * self.sweep_dir
        if out[0]:
            self.alt_used += dt             # climb/descend budget per search (23:42: descended 6 s)
        self.check.update(t, out[1], vo)
        if self.check.bad and self.check_on:
            self.abort = "yaw turns the wrong way (VO) - yaw sign?"
        txt = self.phase if pred_b is None else f"{self.phase} (tag {pred_b:+.0f} deg, elev {pred_e:+.0f})"
        return out, txt


# ---------------------------------------------------------------- self-check on a recorded flight
def selfcheck(folder):
    import csv
    import os
    import cv2
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from aruco_distance import load_focal
    from flight_control import find_tag
    f = load_focal(1280)[0]
    rows = list(csv.DictReader(open(os.path.join(folder, "frames.csv"))))
    vo, tm = YawVO(f, 1280), TagMap()
    errs, preds = [], []
    last_seen_n = None
    for r in rows:
        n, t = int(r["frame"]), float(r["t"])
        img = cv2.imread(os.path.join(folder, f"frame_{n:04d}.jpg"))
        if img is None:
            continue
        vo.update(img, t)
        p = tm.predict(t, vo.heading, vo.tilt)
        s = find_tag(img, f)
        if s:
            if p and p[2] > 0.2:   # tag back after a gap: how good was the prediction?
                preds.append((n, round(p[0] - s["bearing"], 1), round(p[1] - s["elev"], 1), round(p[2], 1)))
            if p and p[2] < 0.2:
                errs.append(abs(p[0] - s["bearing"]))
            tm.see(t, vo.heading, vo.tilt, s["bearing"], s["elev"], s["dist"])
            last_seen_n = n
    print(f"VO valid {vo.good}/{vo.pairs} frame pairs; final heading {vo.heading:+.1f} deg; sightings {tm.count}")
    if errs:
        print(f"next-frame bearing prediction error: median {float(np.median(errs)):.2f} deg (n={len(errs)})")
    print("re-sightings after a gap (frame, bearing error, elevation error, gap s):")
    for p in preds[:20]:
        print("  ", p)
    return last_seen_n


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    selfcheck(sys.argv[1])
