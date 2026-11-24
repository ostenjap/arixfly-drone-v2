"""Live test screen for the Xbox controller (no drone involved). Built for checking a second-hand pad.

Shows both sticks with their raw values and drift at rest, triggers, every button and the D-pad,
the range each stick has reached so far, and the drone byte values joystick_fly.py would send.
The controller can be plugged in after start.

Keys: R = rumble test, C = clear the recorded stick range, Esc / Q = quit

Usage:
  python tools/controller_test.py
"""
import pygame

from joystick_fly import AXIS_MAP, BTN_A, BTN_B, BTN_START, BTN_X, BTN_Y, BUTTON_NAMES, DEADZONE, to_byte

W, H = 900, 560
BG, FG, DIM, OK, WARN, ACC = (18, 20, 24), (230, 232, 236), (90, 95, 105), (60, 200, 120), (240, 170, 50), (80, 160, 255)
BUTTONS = BUTTON_NAMES  # same index -> label map as the flight window, so a wrong label shows up here first
DRIFT_WARN = 0.10  # resting stick further than this from centre = worn stick


def stick(screen, font, cx, cy, x, y, rng, label):
    r = 110
    pygame.draw.circle(screen, DIM, (cx, cy), r, 2)
    pygame.draw.circle(screen, (40, 44, 52), (cx, cy), int(r * DEADZONE))
    if rng:  # range reached so far
        pts = [(cx + px * r, cy + py * r) for px, py in rng]
        if len(pts) > 2:
            pygame.draw.lines(screen, (50, 70, 100), False, pts[-400:], 1)
    mag = (x * x + y * y) ** 0.5
    col = WARN if 0.02 < mag < DRIFT_WARN and not pygame.key.get_pressed()[pygame.K_SPACE] else ACC
    pygame.draw.line(screen, DIM, (cx - r, cy), (cx + r, cy))
    pygame.draw.line(screen, DIM, (cx, cy - r), (cx, cy + r))
    pygame.draw.circle(screen, col, (int(cx + x * r), int(cy + y * r)), 12)
    screen.blit(font.render(f"{label}  x {x:+.3f}  y {y:+.3f}", True, FG), (cx - r, cy + r + 10))


def bar(screen, font, x, y, v, label):
    v = (v + 1) / 2  # SDL triggers rest at -1
    pygame.draw.rect(screen, DIM, (x, y, 30, 120), 2)
    h = int(116 * max(0, min(1, v)))
    pygame.draw.rect(screen, ACC, (x + 2, y + 118 - h, 26, h))
    screen.blit(font.render(f"{label} {v:.2f}", True, FG), (x - 8, y + 126))


def main():
    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Controller test (R rumble, C clear, Q quit)")
    font, big = pygame.font.SysFont("consolas", 16), pygame.font.SysFont("consolas", 22, bold=True)
    clock = pygame.time.Clock()
    js, seen, rng, rest = None, set(), {0: [], 1: []}, {}
    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT or (ev.type == pygame.KEYDOWN and ev.key in (pygame.K_ESCAPE, pygame.K_q)):
                running = False
            elif ev.type == pygame.JOYDEVICEADDED and js is None:
                js = pygame.joystick.Joystick(ev.device_index)
                js.init()
            elif ev.type == pygame.JOYDEVICEREMOVED and js is not None and ev.instance_id == js.get_instance_id():
                js = None
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_r and js:
                js.rumble(0.6, 0.6, 600)
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_c:
                rng, rest = {0: [], 1: []}, {}

        screen.fill(BG)
        if js is None:
            screen.blit(big.render("No controller - plug it in with the USB cable", True, WARN), (40, 40))
            pygame.display.flip()
            clock.tick(30)
            continue

        ax = [js.get_axis(i) for i in range(js.get_numaxes())]
        get = lambda i: ax[i] if i < len(ax) else 0.0
        screen.blit(big.render(f"{js.get_name()}  ({len(ax)} axes, {js.get_numbuttons()} buttons)", True, FG), (30, 20))

        for k, (xi, yi) in enumerate(((0, 1), (2, 3))):
            x, y = get(xi), get(yi)
            rng[k].append((x, y))
            if abs(x) < 0.25 and abs(y) < 0.25:  # treat as "at rest": track worst resting offset
                rest[k] = max(rest.get(k, 0), (x * x + y * y) ** 0.5)
        stick(screen, font, 170, 200, get(0), get(1), rng[0], "Left ")
        stick(screen, font, 470, 200, get(2), get(3), rng[1], "Right")
        bar(screen, font, 640, 120, get(4), "LT")
        bar(screen, font, 700, 120, get(5), "RT")

        # buttons + D-pad
        bx, by = 30, 380
        for i in range(js.get_numbuttons()):
            on = js.get_button(i)
            if on:
                seen.add(i)
            name = BUTTONS.get(i, str(i))
            col = OK if on else ((60, 90, 70) if i in seen else DIM)
            pygame.draw.rect(screen, col, (bx, by, 70, 34), 0 if on else 2, border_radius=6)
            screen.blit(font.render(name, True, FG), (bx + 8, by + 8))
            bx += 78
            if bx > W - 90:
                bx, by = 30, by + 44
        for h in range(js.get_numhats()):
            hx, hy = js.get_hat(h)
            screen.blit(font.render(f"D-pad {h}: ({hx:+d},{hy:+d})", True, OK if (hx or hy) else FG), (780, 120))

        # health summary
        y0 = 470
        for k, name in ((0, "Left"), (1, "Right")):
            d = rest.get(k, 0)
            reach = max(((px * px + py * py) ** 0.5 for px, py in rng[k]), default=0)
            ok = d < DRIFT_WARN
            txt = f"{name} stick: worst rest offset {d:.3f} ({'ok' if ok else 'DRIFT - bigger deadzone needed'}),  max reach {reach:.2f}"
            screen.blit(font.render(txt, True, OK if ok else WARN), (30, y0))
            y0 += 22
        missing = [BUTTONS[i] for i in (BTN_A, BTN_B, BTN_X, BTN_Y) if i not in seen]
        screen.blit(font.render("Drone controls (A, B, X, Y) " + ("all pressed once - ok" if not missing else
                                f"not pressed yet: {', '.join(missing)}"), True, OK if not missing else FG), (30, y0))

        # what joystick_fly would send (stick limit 30 %, LB held)
        vals = {b: to_byte(s * get(a), 0.3) for b, a, s in AXIS_MAP}
        screen.blit(font.render(f"drone bytes @30%: thr {vals[20]} yaw {vals[21]} pit {vals[22]} rol {vals[23]}",
                                True, DIM), (560, 330))
        pygame.display.flip()
        clock.tick(60)
    pygame.quit()


if __name__ == "__main__":
    main()
