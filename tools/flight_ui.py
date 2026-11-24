"""Mission-control style drawing for the flight window (used by joystick_fly.py).

Dark, thin lines, uppercase labels, monospace telemetry; cards with 1 px borders and rounded corners.
Everything is laid out from the current window size, so it works windowed and full screen.

Preview without a controller or drone:
  python tools/flight_ui.py            # writes captures/ui_preview_*.png for 1280x720 and 1920x1080
"""
import math
import os
import time

import pygame

BG = (9, 9, 11)
CARD = (17, 17, 20)
BORDER = (39, 39, 42)
FG = (244, 244, 245)
MUTED = (113, 113, 122)
FAINT = (63, 63, 70)
STATE_COLORS = {"NO CONTROLLER": (113, 113, 122), "OFFLINE": (113, 113, 122), "CONNECTING": (96, 165, 250),
                "SAFE": (74, 222, 128), "READY": (45, 212, 191), "ARMED": (251, 191, 36),
                "FLYING": (251, 191, 36), "LANDING": (167, 139, 250), "STOPPED": (248, 113, 113),
                "KILLING": (248, 113, 113), "LINK LOST": (248, 113, 113)}
CHANNELS = ("THROTTLE", "YAW", "ROLL", "PITCH")  # packet bytes 20..23, from the HS420 app's ba2.a()
# the flight sequence shown as a stepper; the active step is highlighted, earlier ones get a check
STEPS = (("CONTROLLER", "plug in USB"), ("CONNECT", "press Y"), ("CALIBRATE", "drone level, press X"),
         ("TAKEOFF", "hold A  1 s"), ("FLY", "sticks  ·  hold A = land"), ("KILL", "B  or  Space"))
KEY_STEPS = (("CONTROLLER", "click this window"), ("CONNECT", "press C"), ("CALIBRATE", "drone level, press G"),
             ("TAKEOFF", "hold Enter  1 s"), ("FLY", "WASD + arrows"), ("KILL", "Space"))
PAD_HINTS = (("Y", "CONNECT"), ("X", "CALIBRATE"), ("A", "HOLD: TAKEOFF / LAND"), ("B / SPACE", "KILL"),
             ("D-PAD", "STRENGTH / ROLL TRIM"), ("LB RB", "PITCH TRIM"), ("ESC", "QUIT"))
KEY_HINTS = (("C", "CONNECT"), ("G", "CALIBRATE"), ("ENTER", "TAKEOFF/LAND"), ("SPACE", "KILL"),
             ("WASD", "THR/YAW"), ("ARROWS", "PITCH/ROLL"), ("+ -", "STRENGTH"), ("V", "SPEED"))
STEP_OF_STATE = {"NO CONTROLLER": 0, "OFFLINE": 1, "CONNECTING": 1, "LINK LOST": 1, "STOPPED": 1,
                 "SAFE": 2, "READY": 3, "ARMED": 4, "FLYING": 4, "LANDING": 4, "KILLING": 5}
RULES = ("PROPS CHECKED A/B", "TETHERED, SOFT GROUND, NO WATER", "NOBODY WITHIN 2 M", "MULLVAD OFF",
         "! STICK DIRECTIONS UNVERIFIED")
AIRBORNE = ("ARMED", "FLYING", "LANDING")


class Fonts:
    """Font set scaled to the window height; rebuilt when the window size changes."""

    def __init__(self, h):
        s = h / 900
        f = pygame.font.SysFont
        self.label = f("bahnschrift", max(11, int(15 * s)))
        self.body = f("segoeui", max(12, int(18 * s)))
        self.mono = f("consolas", max(12, int(18 * s)))
        self.mono_big = f("consolas", max(18, int(34 * s)), bold=True)
        self.hero = f("bahnschrift", max(36, int(104 * s)))
        self.title = f("bahnschrift", max(14, int(20 * s)))
        self.h = h


def text(surf, font, s, color, pos, anchor="topleft", spacing=0):
    """Draw text; spacing > 0 adds letter tracking (for uppercase labels)."""
    if spacing:
        glyphs = [font.render(c, True, color) for c in s]
        w = sum(g.get_width() for g in glyphs) + spacing * (len(glyphs) - 1)
        img = pygame.Surface((max(w, 1), font.get_height()), pygame.SRCALPHA)
        x = 0
        for g in glyphs:
            img.blit(g, (x, 0))
            x += g.get_width() + spacing
    else:
        img = font.render(s, True, color)
    r = img.get_rect(**{anchor: pos})
    surf.blit(img, r)
    return r


def card(surf, rect, title, fonts, accent=None):
    pygame.draw.rect(surf, CARD, rect, border_radius=12)
    pygame.draw.rect(surf, accent or BORDER, rect, 1, border_radius=12)
    pad = int(fonts.h * 0.022)
    if title:
        text(surf, fonts.label, title, MUTED, (rect.x + pad, rect.y + pad), spacing=2)
    inner = rect.inflate(-2 * pad, -2 * pad)
    if title:  # leave room under the card title
        off = int(fonts.label.get_height() * 2.2)
        inner = pygame.Rect(inner.x, inner.y + off, inner.w, inner.h - off)
    return inner


def dot(surf, color, center, r):
    pygame.draw.aacircle(surf, color, center, r)
    pygame.draw.circle(surf, color, center, r)


def stick(surf, fonts, center, r, x, y, label, deadzone):
    cx, cy = center
    pygame.draw.aacircle(surf, BORDER, center, r, 1)
    pygame.draw.aacircle(surf, BORDER, center, int(r * 0.5), 1)
    pygame.draw.circle(surf, (26, 26, 30), center, max(2, int(r * deadzone)))
    pygame.draw.line(surf, BORDER, (cx - r, cy), (cx + r, cy))
    pygame.draw.line(surf, BORDER, (cx, cy - r), (cx, cy + r))
    px, py = int(cx + x * r), int(cy + y * r)
    pygame.draw.line(surf, FAINT, center, (px, py), 1)
    dot(surf, FG, (px, py), max(5, r // 9))
    text(surf, fonts.label, label, MUTED, (cx, cy + r + int(fonts.h * 0.02)), "midtop", spacing=2)


def channel_bar(surf, fonts, rect, name, value, color):
    """Centered bar: 128 = middle, 0 = full left, 255 = full right."""
    text(surf, fonts.label, name, MUTED, rect.topleft, spacing=2)
    text(surf, fonts.mono, f"{value:3d}", FG, rect.topright, "topright")
    by = rect.y + int(fonts.label.get_height() * 1.5)
    track = pygame.Rect(rect.x, by, rect.w, max(4, int(fonts.h * 0.007)))
    pygame.draw.rect(surf, (30, 30, 34), track, border_radius=3)
    mid = track.centerx
    pygame.draw.line(surf, FAINT, (mid, track.y - 4), (mid, track.bottom + 3))
    w = int((value - 128) / 128 * track.w / 2)
    if w:
        pygame.draw.rect(surf, color, (min(mid, mid + w), track.y, abs(w), track.h), border_radius=3)


# Expected motor speeds from the stick bytes, standard X-quad mixer, "props in" layout seen from above:
# front-left CW, front-right CCW, rear-left CCW, rear-right CW. Yaw right speeds up the CCW pair.
# Not measured on the HS420 - it is what the user compares the real motors against.
MOTORS = (("FL", -1, -1, +1, -1), ("FR", -1, +1, -1, +1), ("RL", +1, -1, +1, +1), ("RR", +1, +1, -1, -1))
#          name  front/rear(pitch) left/right(roll sign) roll  yaw


def expected_motors(axes, airborne):
    """0..1 per motor for the (throttle, yaw, roll, pitch) bytes 20..23; 0 on the ground."""
    if not airborne:
        return {name: 0.0 for name, *_ in MOTORS}
    t, y, r, p = ((v - 128) / 127 for v in axes)
    out = {}
    for name, fr, _, rs, ys in MOTORS:
        # pitch forward (p > 0): rear (fr = +1) faster; roll right (r > 0): left motors (rs = +1) faster
        out[name] = max(0.0, min(1.0, 0.5 + 0.4 * t + 0.25 * p * fr + 0.25 * r * rs + 0.2 * y * ys))
    return out


def motor_diagram(surf, fonts, rect, axes, airborne, color):
    """X quad seen from above, nose up; each prop spins at its expected speed with % and a ring."""
    speeds = expected_motors(axes, airborne)
    cx, cy = rect.center
    lh = fonts.label.get_height() + fonts.mono.get_height() + 12  # name above + % below each prop
    arm = int(min(rect.w * 0.28, (rect.h - lh) / 2.9))  # 2 arms + 2 prop radii (0.45 arm) fit the height
    pr = int(arm * 0.45)
    pygame.draw.line(surf, FAINT, (cx - arm, cy - arm), (cx + arm, cy + arm), 3)
    pygame.draw.line(surf, FAINT, (cx + arm, cy - arm), (cx - arm, cy + arm), 3)
    pygame.draw.rect(surf, BORDER, (cx - 14, cy - 18, 28, 36), border_radius=6)
    pygame.draw.polygon(surf, color, [(cx, cy - 30), (cx - 8, cy - 20), (cx + 8, cy - 20)])  # nose
    t = pygame.time.get_ticks() / 1000
    for name, fr, lr, _, ys in MOTORS:
        mx, my = cx + lr * arm, cy + fr * arm
        s = speeds[name]
        # slower than hover = blue, about hover = green, faster = amber
        ring = FAINT if s == 0 else (96, 165, 250) if s < 0.45 else (74, 222, 128) if s <= 0.55 else (251, 191, 36)
        pygame.draw.aacircle(surf, BORDER, (mx, my), pr, 2)
        if s > 0:
            pygame.draw.arc(surf, ring, (mx - pr, my - pr, 2 * pr, 2 * pr), math.pi / 2,
                            math.pi / 2 + s * 2 * math.pi, 4)
            ang = (t * (2 + 18 * s) * (-ys)) % (2 * math.pi)  # spin direction follows CW/CCW
            for k in (0, math.pi):
                ex, ey = mx + math.cos(ang + k) * pr * 0.75, my + math.sin(ang + k) * pr * 0.75
                pygame.draw.line(surf, FG, (mx, my), (int(ex), int(ey)), 3)
        dot(surf, FG if s else FAINT, (mx, my), 4)
        text(surf, fonts.mono, f"{s:4.0%}" if airborne else " -- ", FG if s else MUTED,
             (mx, my + pr + 4), "midtop")
        text(surf, fonts.label, name, MUTED, (mx, my - pr - 4), "midbottom", spacing=2)


def sequence(surf, fonts, rect, m, color):
    """Stepper 01..06 with the active step highlighted, then the safety rules as a checklist row."""
    active = STEP_OF_STATE[m["state"]]
    steps = KEY_STEPS if m.get("keys") else STEPS
    n = len(steps)
    gap = int(fonts.h * 0.012)
    sw = (rect.w - gap * (n - 1)) // n
    sh = int(rect.h * 0.62)
    for i, (name, how) in enumerate(steps):
        r = pygame.Rect(rect.x + i * (sw + gap), rect.y, sw, sh)
        done = i < active
        live = i == active or (i == n - 1 and m["state"] in AIRBORNE)  # KILL is always live in the air
        fill = (24, 24, 28) if live else CARD
        edge = color if live else BORDER
        pygame.draw.rect(surf, fill, r, border_radius=10)
        pygame.draw.rect(surf, edge, r, 2 if live else 1, border_radius=10)
        p = int(fonts.h * 0.016)
        num_col = color if live else ((74, 222, 128) if done else FAINT)
        text(surf, fonts.label, f"{i + 1:02d}", num_col, (r.x + p, r.y + p), spacing=2)
        if done:
            cx, cy = r.right - p - 8, r.y + p + fonts.label.get_height() // 2
            pygame.draw.lines(surf, (74, 222, 128), False, [(cx - 7, cy), (cx - 2, cy + 5), (cx + 7, cy - 5)], 2)
        text(surf, fonts.title, name, FG if (live or done) else MUTED, (r.x + p, r.centery - 2), "midleft", spacing=3)
        text(surf, fonts.body, how, FG if live else MUTED, (r.x + p, r.bottom - p), "bottomleft")
        if i < n - 1:  # small arrow between steps
            ax, ay = r.right + gap // 2, r.centery
            pygame.draw.line(surf, FAINT, (ax - 3, ay - 4), (ax + 1, ay), 1)
            pygame.draw.line(surf, FAINT, (ax - 3, ay + 4), (ax + 1, ay), 1)

    # rules row: always visible
    y = rect.y + sh + int(fonts.h * 0.018)
    x = rect.x
    rules = list(RULES) + (["DRY RUN - NOTHING IS SENT"] if m["dry"] else [])
    widths = [fonts.label.size(r)[0] + 2 * (len(r) - 1) + 22 for r in rules]  # text + tracking + box
    spare = rect.w - sum(widths)
    step = max(10, min(34, spare // max(1, len(rules) - 1)))  # shrink the gaps so the row fits
    for rule in rules:
        warn = rule.startswith(("DRY", "!"))
        col = (96, 165, 250) if rule.startswith("DRY") else (251, 191, 36) if warn else MUTED
        rr = text(surf, fonts.label, rule, col, (x + 22, y), spacing=2)
        if warn:
            pygame.draw.aacircle(surf, col, (x + 7, rr.centery), 6, 2)
        else:
            pygame.draw.rect(surf, FAINT, (x, rr.centery - 7, 14, 14), 1, border_radius=3)
        x = rr.right + step


def draw(surf, fonts, m):
    """m: dict with the window model, see joystick_fly.py."""
    W, H = surf.get_size()
    color = STATE_COLORS[m["state"]]
    surf.fill(BG)
    pad = int(H * 0.035)
    gap = int(H * 0.022)

    # --- top bar -------------------------------------------------------------------------------------
    text(surf, fonts.title, "HS420", FG, (pad, pad), spacing=3)
    t = text(surf, fonts.title, "  /  FLIGHT CONTROL", MUTED, (pad + fonts.title.size("HS420")[0] + 30, pad), spacing=3)
    if m["dry"]:
        badge = pygame.Rect(t.right + 18, t.y - 2, fonts.label.size("DRY RUN")[0] + 40, t.h + 4)
        pygame.draw.rect(surf, (96, 165, 250), badge, 1, border_radius=20)
        text(surf, fonts.label, "DRY RUN", (96, 165, 250), badge.center, "center", spacing=2)
    x = W - pad
    for label, ok in (("MIC", m.get("mic")), ("LINK", bool(m["link"])), ("CONTROLLER", bool(m["controller"]))):
        if ok is None:
            continue
        r = text(surf, fonts.label, label, FG if ok else MUTED, (x, pad + 2), "topright", spacing=2)
        dot(surf, (74, 222, 128) if ok else FAINT, (r.x - 14, r.centery), 5)
        x = r.x - 44
    pygame.draw.line(surf, BORDER, (pad, pad + fonts.title.get_height() + gap), (W - pad, pad + fonts.title.get_height() + gap))

    # --- hero: state + mission clock -----------------------------------------------------------------
    hy = pad + fonts.title.get_height() + 2 * gap
    text(surf, fonts.label, "STATUS", MUTED, (pad, hy), spacing=3)
    sr = text(surf, fonts.hero, m["state"], color, (pad - 4, hy + fonts.label.get_height()), spacing=4)
    if m["arm_frac"]:
        r = int(sr.h * 0.32)
        c = (sr.right + r + 30, sr.centery)
        pygame.draw.aacircle(surf, BORDER, c, r, 3)
        pygame.draw.arc(surf, STATE_COLORS["ARMED"], (c[0] - r, c[1] - r, 2 * r, 2 * r),
                        math.pi / 2, math.pi / 2 + m["arm_frac"] * 2 * math.pi, 5)
        text(surf, fonts.label, "HOLD A", MUTED, (c[0] + r + 14, c[1]), "midleft", spacing=2)
    nr = text(surf, fonts.body, m["note"], FG, (pad, sr.bottom + int(gap * 0.4)))

    tx = W - pad
    text(surf, fonts.label, "T+  MISSION", MUTED, (tx, hy), "topright", spacing=3)
    tm = m["t_armed"]
    tm_show = tm or 0.0
    clock = f"{int(tm_show // 60):02d}:{tm_show % 60:04.1f}"
    cr = text(surf, fonts.hero, clock, FG if tm is not None else FAINT, (tx, hy + fonts.label.get_height()), "topright", spacing=4)
    if m["cap_left"] is not None:  # thin cap progress line, SpaceX-style
        frac = max(0.0, min(1.0, m["cap_left"] / m["cap"]))
        ly = cr.bottom + int(gap * 0.6)
        pygame.draw.line(surf, BORDER, (cr.x, ly), (cr.right, ly), 2)
        pygame.draw.line(surf, color, (cr.x, ly), (cr.x + int(cr.w * frac), ly), 2)
        text(surf, fonts.label, f"CAP {m['cap_left']:4.1f} S", MUTED, (cr.right, ly + 8), "topright", spacing=2)

    # --- sequence + rules ----------------------------------------------------------------------------
    sy = max(nr.bottom, cr.bottom + int(gap * 1.5)) + int(gap * 1.4)
    seq_h = int(H * 0.16)
    sequence(surf, fonts, pygame.Rect(pad, sy, W - 2 * pad, seq_h), m, color)

    # --- cards ---------------------------------------------------------------------------------------
    top = sy + seq_h + int(gap * 1.2)
    bottom = H - pad - int(fonts.label.get_height() * 2.6)
    ch = bottom - top
    cw = (W - 2 * pad - 2 * gap)
    c1 = pygame.Rect(pad, top, int(cw * 0.36), ch)
    c2 = pygame.Rect(c1.right + gap, top, int(cw * 0.34), ch)
    c3 = pygame.Rect(c2.right + gap, top, W - pad - (c2.right + gap), ch)

    # left: sticks on top, the four bytes that go to the drone below
    inner = card(surf, c1, "STICKS  ->  DRONE", fonts, color if m["state"] in AIRBORNE else None)
    sticks_h = int(inner.h * 0.56)
    r = int(min(inner.w / 4.8, sticks_h / 2.7))
    for i, (sx, sy, label) in enumerate(m["sticks"]):
        stick(surf, fonts, (inner.x + int(inner.w * (0.25 + 0.5 * i)), inner.y + int(sticks_h * 0.42)), r,
              sx, sy, label, m["deadzone"])
    by = inner.y + sticks_h
    row = (inner.bottom - by) / 4
    for i, (name, v) in enumerate(zip(CHANNELS, m["axes"])):
        channel_bar(surf, fonts, pygame.Rect(inner.x, by + int(row * i), inner.w, int(row)), name, v, color)

    # middle: expected motor speeds, full card height so it reads from across the room
    inner = card(surf, c2, "MOTORS  -  EXPECTED SPEED", fonts)
    text(surf, fonts.label, f"LIMIT {m['limit']:.0%}", MUTED, (inner.right, c2.y + int(fonts.h * 0.022)),
         "topright", spacing=2)
    key_y = inner.bottom - fonts.label.get_height()
    kx = inner.x
    for col, what in (((96, 165, 250), "SLOWER"), ((74, 222, 128), "HOVER"), ((251, 191, 36), "FASTER")):
        dot(surf, col, (kx + 6, key_y + fonts.label.get_height() // 2), 5)
        kx = text(surf, fonts.label, what, MUTED, (kx + 18, key_y), spacing=2).right + 26
    motor_diagram(surf, fonts, pygame.Rect(inner.x, inner.y, inner.w, key_y - inner.y - 8),
                  m["axes"], m["state"] in AIRBORNE, color)

    inner = card(surf, c3, "TELEMETRY", fonts)
    cmd = {0x01: "TAKEOFF", 0x02: "LAND", 0x04: "E-STOP", 0x80: "CALIBRATE"}.get(m["button"], "")
    rt, pt = m.get("trim", (0, 0))
    rows = [("PACKETS / S", f"{m['pps']:3d}"), ("COMMAND BYTE", f"0x{m['button']:02X} {cmd}".strip()),
            ("TRIM ROLL / PITCH", f"{rt:+d} / {pt:+d}"), ("SPEED", m.get("speed", "-")), ("LINK", m["link"] or "-"),
            ("CONTROLLER", m["controller"] or "-")]
    rh = fonts.mono.get_height() * 1.55
    y = inner.y
    for k, v in rows:
        kr = text(surf, fonts.label, k, MUTED, (inner.x, y + 3), spacing=2)
        room = inner.right - kr.right - 16
        while v and fonts.mono.size(v)[0] > room:  # shorten long values to the free width
            v = v[:-2] + "…"
        text(surf, fonts.mono, v, FG, (inner.right, y), "topright")
        y += rh
    pygame.draw.line(surf, BORDER, (inner.x, y + 4), (inner.right, y + 4))
    y += 16
    text(surf, fonts.label, "EVENT LOG", MUTED, (inner.x, y), spacing=2)
    y += fonts.label.get_height() + 8
    for ts, st, _ in m["log"][-max(1, int((inner.bottom - y) // (fonts.mono.get_height() * 1.2))):]:
        text(surf, fonts.mono, f"{ts:7.2f}", MUTED, (inner.x, y))
        text(surf, fonts.mono, st, STATE_COLORS.get(st, FG), (inner.x + fonts.mono.size("0000.00  ")[0], y))
        y += fonts.mono.get_height() * 1.2

    # --- key hints -----------------------------------------------------------------------------------
    x, y = pad, H - pad - fonts.label.get_height() - 6
    for key, what in (KEY_HINTS if m.get("keys") else PAD_HINTS):
        kw = fonts.label.size(key)[0] + 22
        kr = pygame.Rect(x, y - 4, kw, fonts.label.get_height() + 10)
        pygame.draw.rect(surf, BORDER, kr, 1, border_radius=6)
        text(surf, fonts.label, key, FG, kr.center, "center")
        r = text(surf, fonts.label, what, MUTED, (kr.right + 10, kr.centery), "midleft", spacing=2)
        x = r.right + 34


def demo_model(state="FLYING"):
    flying = state in AIRBORNE
    return {"state": state, "note": "hold A = land   B / Space = KILL", "arm_frac": 0.0,
            "t_armed": 12.4 if flying else None,
            "cap": 30, "cap_left": 17.6 if flying else None, "axes": [176, 128, 140, 101], "button": 0,
            "trim": (4, -2),
            "pps": 39, "seq": 1284, "link": "192.168.169.2", "controller": "Xbox One Controller", "mic": None,
            "dry": False, "limit": 0.3, "deadzone": 0.15,
            "sticks": [(0.0, -0.75, "THROTTLE / YAW"), (0.2, 0.42, "PITCH / ROLL")],
            "log": [(0.0, "OFFLINE", ""), (3.1, "CONNECTING", ""), (6.8, "SAFE", ""), (8.0, "READY", ""),
                    (9.2, "FLYING", "")]}


if __name__ == "__main__":
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    pygame.init()
    out = os.path.join(os.path.dirname(__file__), "..", "captures")
    for size, state in (((1280, 720), "READY"), ((1920, 1080), "FLYING")):
        s = pygame.Surface(size)
        m = demo_model(state)
        if state == "READY":
            m.update(note="calibrated - hold A 1 s to take off", arm_frac=0.6, axes=[128] * 4, dry=True,
                     sticks=[(0, 0, "THROTTLE / YAW"), (0, 0, "PITCH / ROLL")])
        draw(s, Fonts(size[1]), m)
        p = os.path.normpath(os.path.join(out, f"ui_preview_{size[0]}x{size[1]}.png"))
        pygame.image.save(s, p)
        print("wrote", p)
