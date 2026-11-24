"""Flight window: fly the HS420 from an Xbox controller (controller -> laptop -> UDP `ef 02` -> drone).

Layout decoded from the HS420 app itself (com.mjx.holystone 1.0.23, class ba2 "lxMjxCtrlPro", 2026-11-05;
see notes/findings.md). This replaces the turbodrone-based guesses that crashed the drone into water.
  bytes 20..23 = throttle, yaw, roll (+ aileron trim), pitch (+ elevator trim)
  byte 25: 0x01 takeoff, 0x02 land, 0x04 EMERGENCY STOP, 0x20 flip, 0x80 GYRO CALIBRATION
Verified on the HS420: 0x01 takeoff lifts off; no packets -> motors stop ~1.2 s later; 0x04 stops the motors
at once (props off, 2026-11-08).
NOT verified yet: stick DIRECTIONS (signs below; sibling-app reference agrees), 0x80 calibration effect, 0x02 land.
Do not fly free until those are checked (props off / tethered, see findings).

Controls:
  Y               connect to the drone Wi-Fi (laptop goes offline until the window closes)
  X               gyro calibration 0x80 (drone level on the ground; required before every takeoff)
  hold A 1 s      take off 0x01 (once calibrated); release, hold A 1 s again = land 0x02. Start works like A.
  B  or  Space    KILL: emergency stop 0x04 for KILL_S, then stop sending (no-packet failsafe as backup)
  left stick      throttle (up/down) + yaw (left/right)
  right stick     pitch (up/down)    + roll (left/right)
  D-pad up/down   stick strength +/- 5 % (10..80 %), shown as LIMIT in the window
  D-pad left/right  roll trim -/+ TRIM_STEP;  LB / RB  pitch trim -/+ TRIM_STEP (against drift)
  F11             full screen on/off (window can also be resized)
  Esc / close     stop, go back to home Wi-Fi, quit
Time cap (--cap, from takeoff): the drone is sent the land command, then sending stops after LAND_S.

Keyboard instead of the pad (--keys, e.g. when the laptop's USB is down). The window must be in front:
  C connect   G calibrate   hold Enter 1 s takeoff / land   Space KILL   Esc quit
  W/S throttle   A/D yaw   arrow up/down pitch   arrow left/right roll   +/- stick strength   V speed 1/2
Speed (pad Back, key V): the app's speed button, byte 24 = 0 (slow, app default) or 1 (fast). --speed sets the start.
Throttle strength on its own: --throttle-limit (default 0.55, full W = 198), PgUp/PgDn +/- 5 % live.
Default set (tethered flight 2026-11-08, user: "perfect"): sticks 40 %, throttle 55 %, speed 1 (byte 24 = 0).
  Keys ease the stick in over KEY_RAMP_S and spring back on release. Window loses focus while flying -> auto land.

Usage:
  python tools/joystick_fly.py --dry              # everything except the Wi-Fi switch and sending
  python tools/joystick_fly.py --keys             # LIVE with the laptop keyboard
  python tools/joystick_fly.py                    # LIVE, 30 s per armed run, sticks limited to 30 %
  python tools/joystick_fly.py --limit 0.5 --cap 60 --fullscreen
"""
import argparse
import os
import socket
import threading
import time

# SDL drops controller input while its window is not in front; the KILL button must work regardless.
os.environ["SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS"] = "1"
import pygame

import flight_ui
from replay_test import DRONE, DRONE_SSID, HOME_SSID, RATE_HZ, build, wait_for_ip, wifi_connect

# SDL2 XInput layout on Windows
AX_LX, AX_LY, AX_RX, AX_RY = 0, 1, 2, 3
# Standard XInput order, confirmed on this pad with tools/controller_test.py (2026-11-04: the swapped map
# 1/0/3/2 showed wrong labels). B is the KILL button, so re-check with the test panel after any change.
BTN_A, BTN_B, BTN_X, BTN_Y, BTN_LB, BTN_RB, BTN_BACK, BTN_START = 0, 1, 2, 3, 4, 5, 6, 7
BUTTON_NAMES = {BTN_A: "A", BTN_B: "B", BTN_X: "X", BTN_Y: "Y", 4: "LB", 5: "RB", 6: "Back", BTN_START: "Start",
                8: "LS", 9: "RS", 10: "Guide"}

# controller axis -> packet byte (20..23), +1 or -1 for direction (stick up is -1 in SDL).
# Byte order from the app's ba2.a(); the SIGNS are not confirmed yet (needs an app capture with known moves).
AXIS_MAP = [  # (packet byte, controller axis, sign)
    (20, AX_LY, -1),  # throttle: left stick up = climb (direction unverified)
    (21, AX_LX, +1),  # yaw: left stick left/right
    (22, AX_RX, +1),  # roll: right stick left/right   (+ aileron trim)
    (23, AX_RY, -1),  # pitch: right stick up = forward (+ elevator trim)
]
THROTTLE_BYTE, ROLL_BYTE, PITCH_BYTE = 20, 22, 23
TAKEOFF, LAND, ESTOP, CALIBRATE = 0x01, 0x02, 0x04, 0x80  # byte 25 bits, from ba2.a()
DEADZONE = 0.15       # second-hand pad rests at up to ~0.09 off centre
ARM_HOLD_S = 1.0      # hold A this long for takeoff / land
PULSE_S = 1.0         # how long a command bit stays set (as captured from the app)
CAL_S = 1.0           # calibration pulse; the app's duration is unknown
KILL_S = 1.0          # emergency stop bit before packets stop
LAND_S = 8.0          # after a land command (or the cap), keep sending this long, then stop
TRIM_STEP = 2         # trim in byte counts; the app's trim slider spans about +/- 94
TRIM_MAX = 40
MAX_CAP_S = 120
LIMIT_MIN, LIMIT_MAX = 0.1, 0.8  # D-pad range for the stick strength (60 % felt too aggressive)

WINDOW = (1280, 720)

# Keyboard stand-in for the pad (--keys). Same SDL axis convention: up / left = -1.
KEY_AXES = {AX_LY: (pygame.K_w, pygame.K_s), AX_LX: (pygame.K_a, pygame.K_d),
            AX_RY: (pygame.K_UP, pygame.K_DOWN), AX_RX: (pygame.K_LEFT, pygame.K_RIGHT)}
KEY_BUTTONS = {pygame.K_c: BTN_Y, pygame.K_g: BTN_X, pygame.K_v: BTN_BACK}  # Space = KILL, Enter = A separately
SPEEDS = 2            # app speed button: E.j = n % A.b() with A = ba2(..., 2) -> levels 0 and 1, default 0
KEY_LIMIT = {pygame.K_EQUALS: 1, pygame.K_PLUS: 1, pygame.K_KP_PLUS: 1, pygame.K_MINUS: -1, pygame.K_KP_MINUS: -1}
KEY_RAMP_S = 0.4      # key held: stick reaches full deflection in this time; released: back to centre twice as fast


class KeyPad:
    """Looks like a pygame joystick to the flight loop, driven by the keyboard."""

    def __init__(self):
        self.v, self.t = {ax: 0.0 for ax in KEY_AXES}, None

    def update(self, now):
        keys, dt = pygame.key.get_pressed(), 0.0 if self.t is None else now - self.t
        self.t = now
        for ax, (neg, pos) in KEY_AXES.items():
            target = float(keys[pos]) - float(keys[neg])
            rate = (1 if target else 2) / KEY_RAMP_S
            step = max(-rate * dt, min(rate * dt, target - self.v[ax]))
            self.v[ax] += step

    def get_axis(self, ax):
        return self.v.get(ax, 0.0)

    def get_button(self, b):
        keys = pygame.key.get_pressed()
        return b == BTN_A and bool(keys[pygame.K_RETURN] or keys[pygame.K_KP_ENTER])

    def get_name(self):
        return "Keyboard"

    def get_instance_id(self):
        return -1


KEY_WORDS = (("press Y", "press C"), ("press X", "press G"), ("hold A", "hold Enter"), ("B / Space", "Space"),
             ("Y to reconnect", "C to reconnect"), ("X to calibrate", "G to calibrate"))


def keyify(note):
    """Status notes name the pad buttons; in keyboard mode name the keys instead."""
    for pad, key in KEY_WORDS:
        note = note.replace(pad, key)
    return note


def to_byte(v, limit, expo=0.0):
    """Stick -1..1 -> byte around 0x80. expo 0..1 softens the centre: (1-e)*v + e*v^3."""
    if abs(v) < DEADZONE:
        v = 0.0
    v = (1 - expo) * v + expo * v ** 3
    return max(0, min(255, round(0x80 + v * limit * 127)))


class Link:
    """Drone Wi-Fi + UDP socket. Joining runs in a thread so the window never freezes."""

    def __init__(self, dry):
        self.dry, self.sock, self.ip, self.busy, self.error, self.joined = dry, None, None, False, "", False

    def connect(self):
        if self.busy or self.sock:
            return
        self.busy, self.error = True, ""
        threading.Thread(target=self._join, daemon=True).start()

    def _join(self):
        if self.dry:
            time.sleep(0.5)
            self.ip, self.sock = "dry-run", "dry"
        else:
            self.joined = True
            wifi_connect(DRONE_SSID)
            ip = wait_for_ip("192.168.169.")
            if ip:
                self.ip, self.sock = ip, socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            else:
                self.error = "no 192.168.169.x address - is the drone on?"
        self.busy = False

    def send(self, pkt):
        """True if sent. Any socket error (Windows left the drone Wi-Fi) drops the link."""
        if self.sock == "dry":
            return True
        try:
            self.sock.sendto(pkt, DRONE)
            return True
        except OSError as e:
            self.error = f"link lost: {e.strerror or e}"
            self.close()
            return False

    def close(self):
        if self.sock not in (None, "dry"):
            self.sock.close()
        self.sock, self.ip = None, None

    def home(self):
        self.close()
        if self.joined:
            wifi_connect(HOME_SSID)


def set_screen(fullscreen):
    if fullscreen:
        return pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    return pygame.display.set_mode(WINDOW, pygame.RESIZABLE)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry", action="store_true")
    ap.add_argument("--cap", type=float, default=30, help="seconds per armed run (max 120)")
    ap.add_argument("--limit", type=float, default=0.4, help="stick range, 0.4 = 40 %%; D-pad up/down changes it live")
    ap.add_argument("--expo", type=float, default=0.6, help="0 = linear, 1 = very soft around the centre")
    ap.add_argument("--fullscreen", action="store_true")
    ap.add_argument("--keys", action="store_true", help="fly with the laptop keyboard instead of the pad")
    ap.add_argument("--speed", type=int, default=0, choices=range(SPEEDS), help="app speed level at start (0 = slow)")
    ap.add_argument("--throttle-limit", type=float, default=0.55,
                    help="throttle range only (0.55 = tethered 2026-11-08 'perfect'); PgUp/PgDn live")
    a = ap.parse_args()
    speed = a.speed
    cap = min(a.cap, MAX_CAP_S)
    limit = min(max(a.limit, LIMIT_MIN), LIMIT_MAX)
    thr_limit = min(max(a.throttle_limit if a.throttle_limit is not None else limit, LIMIT_MIN), LIMIT_MAX)

    pygame.init()
    fullscreen = a.fullscreen
    screen = set_screen(fullscreen)
    pygame.display.set_caption("HS420 flight control" + (" (DRY RUN)" if a.dry else ""))
    fonts = flight_ui.Fonts(screen.get_height())
    clock = pygame.time.Clock()

    js, link = None, Link(a.dry)
    keypad = KeyPad() if a.keys else None
    state, note = "NO CONTROLLER", ""
    seq, armed_at, start_held, sent_t = 0, 0.0, None, []
    pulse_bit, pulse_until, land_at, need_release, last_axes_log = 0, 0.0, 0.0, False, 0.0
    roll_trim, pitch_trim, kill_until = 0, 0, 0.0
    t0, logged, events = time.perf_counter(), None, []
    try:
        running = True
        while running:
            now = time.perf_counter()
            stop_pressed = False
            if keypad:  # keyboard counts as connected only while the window has focus
                keypad.update(now)
                js = keypad if pygame.key.get_focused() else None
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key == pygame.K_ESCAPE):
                    running = False
                elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_SPACE:
                    stop_pressed = True
                elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_F11:
                    fullscreen = not fullscreen
                    screen = set_screen(fullscreen)
                    fonts = flight_ui.Fonts(screen.get_height())
                elif ev.type == pygame.VIDEORESIZE and not fullscreen:
                    fonts = flight_ui.Fonts(screen.get_height())
                elif ev.type == pygame.JOYDEVICEADDED and js is None and not keypad:
                    js = pygame.joystick.Joystick(ev.device_index)
                elif ev.type == pygame.JOYDEVICEREMOVED and js and not keypad and ev.instance_id == js.get_instance_id():
                    js = None
                elif ev.type == pygame.KEYDOWN and ev.key in (pygame.K_PAGEUP, pygame.K_PAGEDOWN):  # throttle only
                    step = 0.05 if ev.key == pygame.K_PAGEUP else -0.05
                    thr_limit = round(min(max(thr_limit + step, LIMIT_MIN), LIMIT_MAX), 2)
                    print(f"{now - t0:7.2f}s    throttle limit {thr_limit:.0%} (full W = {to_byte(1, thr_limit)})",
                          flush=True)
                elif keypad and ev.type == pygame.KEYDOWN and ev.key in KEY_LIMIT:  # +/-: stick strength
                    limit = round(min(max(limit + 0.05 * KEY_LIMIT[ev.key], LIMIT_MIN), LIMIT_MAX), 2)
                    print(f"{now - t0:7.2f}s    stick limit {limit:.0%}", flush=True)
                elif ev.type == pygame.JOYHATMOTION and js and ev.value[1]:  # D-pad up/down: stick strength
                    limit = round(min(max(limit + 0.05 * ev.value[1], LIMIT_MIN), LIMIT_MAX), 2)
                    print(f"{now - t0:7.2f}s    stick limit {limit:.0%}", flush=True)
                elif ev.type == pygame.JOYHATMOTION and js and ev.value[0]:  # D-pad left/right: roll trim
                    roll_trim = max(-TRIM_MAX, min(TRIM_MAX, roll_trim + TRIM_STEP * ev.value[0]))
                    print(f"{now - t0:7.2f}s    roll trim {roll_trim:+d}", flush=True)
                elif ev.type == pygame.JOYBUTTONDOWN and js and ev.button in (BTN_LB, BTN_RB):  # pitch trim
                    pitch_trim = max(-TRIM_MAX, min(TRIM_MAX, pitch_trim + TRIM_STEP * (1 if ev.button == BTN_RB else -1)))
                    print(f"{now - t0:7.2f}s    pitch trim {pitch_trim:+d}", flush=True)
                elif js and ((ev.type == pygame.JOYBUTTONDOWN and not keypad) or
                             (keypad and ev.type == pygame.KEYDOWN and ev.key in KEY_BUTTONS)):
                    button_id = ev.button if ev.type == pygame.JOYBUTTONDOWN else KEY_BUTTONS[ev.key]
                    if button_id == BTN_B:
                        stop_pressed = True
                    elif button_id == BTN_BACK:  # app speed button: cycles 0 -> 1 -> 0
                        speed = (speed + 1) % SPEEDS
                        print(f"{now - t0:7.2f}s    speed {speed + 1}/{SPEEDS} (byte 24 = {speed})", flush=True)
                    elif button_id == BTN_Y and state in ("OFFLINE", "STOPPED", "LINK LOST"):
                        if link.sock:
                            state, note = "SAFE", "link up - drone level on the ground, press X to calibrate"
                        else:
                            link.connect()
                    elif button_id == BTN_X and state in ("SAFE", "READY"):
                        pulse_bit, pulse_until = CALIBRATE, now + CAL_S
                        state, note = "SAFE", "calibrating gyro ..."

            # --- state machine -------------------------------------------------------------------
            if js is None:
                if state == "FLYING":  # no pilot: land instead of dropping, KILL still works on Space
                    pulse_bit, pulse_until, land_at = LAND, now + PULSE_S, now
                    state, note = "LANDING", ("window lost focus" if keypad else "controller lost") + \
                        " - auto landing, Space = KILL"
                elif state not in ("LANDING", "KILLING"):
                    state = "NO CONTROLLER" if not link.sock else "STOPPED"
            elif state == "NO CONTROLLER":
                state, note = "OFFLINE", "press Y to connect to the drone"
            if link.busy:
                state, note = "CONNECTING", f"joining {DRONE_SSID} ..."
            elif state == "CONNECTING":
                state, note = (("SAFE", "link up - drone level on the ground, press X to calibrate") if link.sock
                               else ("OFFLINE", link.error))
            if stop_pressed and state in ("SAFE", "READY", "FLYING", "LANDING"):
                pulse_bit, pulse_until, kill_until = ESTOP, now + KILL_S, now + KILL_S
                state, note = "KILLING", "EMERGENCY STOP 0x04 - then packets stop"
            if state == "KILLING" and now >= kill_until:
                state, note = "STOPPED", "KILL - packets stopped (failsafe ~1 s). Y to reconnect"
            if state == "SAFE" and pulse_bit == CALIBRATE and now >= pulse_until:
                state, note = "READY", "calibrated - hold A 1 s to take off"
            if now >= pulse_until:
                pulse_bit = 0  # an old calibration must not carry over a reconnect

            holding = bool(js and (js.get_button(BTN_A) or js.get_button(BTN_START)))
            if not holding:
                need_release = False  # A must be let go between takeoff and land
            if state in ("READY", "FLYING") and holding and not need_release and now >= pulse_until:
                start_held = start_held or now
                if now - start_held >= ARM_HOLD_S:
                    start_held, need_release = None, True
                    if state == "READY":
                        pulse_bit, pulse_until = TAKEOFF, now + PULSE_S
                        state, armed_at, note = "FLYING", now, "hold A = land   B / Space = KILL"
                    else:
                        pulse_bit, pulse_until = LAND, now + PULSE_S
                        state, land_at, note = "LANDING", now, "landing - B / Space = KILL"
            elif not holding:
                start_held = None
            if state == "FLYING" and now - armed_at >= cap:  # time cap: land, don't drop
                pulse_bit, pulse_until, land_at = LAND, now + PULSE_S, now
                state, note = "LANDING", f"time cap {cap:.0f} s - auto landing"
            if state == "LANDING" and now - land_at >= LAND_S:
                state, note = "STOPPED", "landed - packets stopped. Y to reconnect, X to calibrate again"

            # --- packet ------------------------------------------------------------------------------
            axes = [0x80] * 4
            if js and state in ("FLYING", "LANDING"):
                for byte, ax, sign in AXIS_MAP:
                    if state == "LANDING" and byte == THROTTLE_BYTE:
                        continue  # the drone descends on its own; only steer while landing
                    axes[byte - 20] = to_byte(sign * js.get_axis(ax), thr_limit if byte == THROTTLE_BYTE else limit,
                                              a.expo)
            if state in ("FLYING", "LANDING"):  # trims are added like the app adds its trim sliders
                axes[ROLL_BYTE - 20] = max(0, min(255, axes[ROLL_BYTE - 20] + roll_trim))
                axes[PITCH_BYTE - 20] = max(0, min(255, axes[PITCH_BYTE - 20] + pitch_trim))
            button = pulse_bit if now < pulse_until else 0x00
            if state in ("FLYING", "LANDING") and axes != [0x80] * 4 and now - last_axes_log >= 0.5:
                print(f"{now - t0:7.2f}s    sticks -> throttle {axes[0]} yaw {axes[1]} roll {axes[2]} pitch {axes[3]}",
                      flush=True)  # what actually went to the drone
                last_axes_log = now
            if state in ("SAFE", "READY", "FLYING", "LANDING", "KILLING") and link.sock:
                if link.send(build(seq, axes=axes, button=button, speed=speed)):
                    seq += 1
                    sent_t.append(now)
                else:
                    state, note = "LINK LOST", f"{link.error} - packets stopped, Y to reconnect"
            sent_t = [t for t in sent_t if now - t < 1.0]
            if keypad:
                note = keyify(note)
            if (state, note) != logged:  # flight log in the terminal and in the window
                print(f"{now - t0:7.2f}s  {state:13} {note}", flush=True)
                if not logged or state != logged[0]:
                    events.append((now - t0, state, note))
                logged = (state, note)

            # --- draw --------------------------------------------------------------------------------
            armed = state in ("FLYING", "LANDING")
            sticks = [(js.get_axis(AX_LX), js.get_axis(AX_LY), "THROTTLE / YAW"),
                      (js.get_axis(AX_RX), js.get_axis(AX_RY), "PITCH / ROLL")] if js else \
                     [(0, 0, "THROTTLE / YAW"), (0, 0, "PITCH / ROLL")]
            flight_ui.draw(screen, fonts, {
                "state": state, "note": note, "dry": a.dry, "limit": limit, "deadzone": DEADZONE,
                "arm_frac": min(1.0, (now - start_held) / ARM_HOLD_S) if start_held and state in ("READY", "FLYING") else 0.0,
                "t_armed": now - armed_at if armed else None, "cap": cap,
                "cap_left": max(0.0, cap - (now - armed_at)) if armed else None,
                "axes": axes, "button": button, "pps": len(sent_t), "seq": seq,
                "link": link.ip, "controller": js.get_name() if js else None, "mic": None,
                "trim": (roll_trim, pitch_trim), "sticks": sticks, "log": events, "keys": bool(keypad),
                "speed": f"{speed + 1}/{SPEEDS}"})
            pygame.display.flip()
            # pace the loop at the app's send rate
            clock.tick(RATE_HZ)
    finally:
        link.home()  # stops sending and returns the laptop to the home Wi-Fi
        pygame.quit()
        print(f"closed (last state {state}, {seq} packets sent)")


if __name__ == "__main__":
    main()
