#!/usr/bin/env python3
"""Interactive real-time viewer for the sculpture simulation.

Drives exactly the same model the sweep reports on, so what you see on screen
is what the numbers describe -- not a separate illustration.

    ./.venv/bin/python view.py
    ./.venv/bin/python view.py --speed 11 --two-ball

Controls
    LEFT / RIGHT   speed setting  0..15   (wave period 20.1 s .. 0.26 s)
    UP / DOWN      amplitude      25/50/75/100 %
    S              toggle spread  (0 = full wavelength, 1 = half)
    M              toggle mirror  (two-ball mode, PRD §6.7)
    N              reversal period: 2 / 3 / 4 / 6 wave cycles (REVERSE[1:0])
    W              toggle the wave off entirely -- the control experiment.
                   The ball should stop dead. If it keeps moving, the track
                   geometry is doing the work, not the wave.
    R              reset          SPACE  pause          Q / ESC  quit
"""

import argparse
import sys

import pygame

from mechanics import Geometry, Sculpture
from wave_model import WaveEngine

W, H = 1100, 400
PX_PER_MM = 3.4
ORIGIN_X, ORIGIN_Y = 185, 250          # screen px for model (0, 0)

BG = (18, 20, 24)
FG = (232, 234, 238)
DIM = (120, 126, 138)
ROD = (90, 170, 255)
BALL = (255, 190, 60)
BALL2 = (120, 230, 150)
RAMP = (70, 76, 88)
ACCENT = (255, 120, 120)

AMP_NAME = {0: "25%", 1: "50%", 2: "75%", 3: "100%"}


def to_screen(x_mm, y_mm):
    return (int(ORIGIN_X + x_mm * PX_PER_MM), int(ORIGIN_Y - y_mm * PX_PER_MM))


class FlatEngine(WaveEngine):
    """Wave disabled -- the on-screen control experiment."""

    def positions(self):
        return [128] * 8

    def step(self):
        pass


REVERSE_CHOICES = [2, 3, 4, 6]      # the REVERSE[1:0] pin field, PRD §6.5


def build(speed, amp, spread, mirror, wave_on, geo, reverse_cycles):
    cls = WaveEngine if wave_on else FlatEngine
    eng = cls(speed=speed, amp=amp, spread=spread, mirror=mirror,
              reverse_cycles=reverse_cycles)
    sc = Sculpture(eng, geo, n_balls=2 if mirror else 1)
    sc.settle()
    return eng, sc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=int, default=8)
    ap.add_argument("--amp", type=int, default=3)
    ap.add_argument("--spread", type=int, default=0)
    ap.add_argument("--two-ball", action="store_true")
    args = ap.parse_args()

    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Kinematic Wave Engine - mechanical simulation")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 15)
    big = pygame.font.SysFont("monospace", 19, bold=True)

    geo = Geometry()
    speed, amp, spread = args.speed, args.amp, args.spread
    mirror = 1 if args.two_ball else 0
    wave_on = True
    paused = False

    rev_i = 0                        # index into REVERSE_CHOICES; default 2 cycles (PRD §6.5)
    eng, sc = build(speed, amp, spread, mirror, wave_on, geo,
                    REVERSE_CHOICES[rev_i])
    traverses, zone = 0, 0
    left_edge = geo.span / 3.0
    right_edge = 2.0 * geo.span / 3.0

    dt = 1e-3
    steps_per_frame = 16          # 16 ms of model per rendered frame ~ real time

    while True:
        rebuild = False
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pygame.quit()
                return 0
            if ev.type != pygame.KEYDOWN:
                continue
            k = ev.key
            if k in (pygame.K_q, pygame.K_ESCAPE):
                pygame.quit()
                return 0
            elif k == pygame.K_SPACE:
                paused = not paused
            elif k == pygame.K_RIGHT:
                speed = min(15, speed + 1); rebuild = True
            elif k == pygame.K_LEFT:
                speed = max(0, speed - 1); rebuild = True
            elif k == pygame.K_UP:
                amp = min(3, amp + 1); rebuild = True
            elif k == pygame.K_DOWN:
                amp = max(0, amp - 1); rebuild = True
            elif k == pygame.K_s:
                spread ^= 1; rebuild = True
            elif k == pygame.K_m:
                mirror ^= 1; rebuild = True
            elif k == pygame.K_w:
                wave_on = not wave_on; rebuild = True
            elif k == pygame.K_n:
                rev_i = (rev_i + 1) % len(REVERSE_CHOICES); rebuild = True
            elif k == pygame.K_r:
                rebuild = True

        if rebuild:
            eng, sc = build(speed, amp, spread, mirror, wave_on, geo,
                            REVERSE_CHOICES[rev_i])
            traverses, zone = 0, 0

        if not paused:
            for _ in range(steps_per_frame):
                sc.step_once(dt)
            x = sc.balls[0].position.x
            if x < left_edge:
                if zone == 1:
                    traverses += 1
                zone = -1
            elif x > right_edge:
                if zone == -1:
                    traverses += 1
                zone = 1

        screen.fill(BG)

        # ground reference line
        pygame.draw.line(screen, (34, 38, 46),
                         to_screen(-80, 0), to_screen(240, 0), 1)

        # end ramps
        for sign, x0 in ((-1.0, geo.rod_x(0)), (+1.0, geo.rod_x(geo.n_rods - 1))):
            a = to_screen(x0, -geo.ramp_drop)
            b = to_screen(x0 + sign * geo.ramp_run, -geo.ramp_drop + geo.ramp_rise)
            pygame.draw.line(screen, RAMP, a, b, 5)

        # cradles, with their pushrods
        for n, body in enumerate(sc.cradles):
            cx, cy = body.position.x, body.position.y
            half = geo.cradle_width / 2
            pygame.draw.line(screen, (44, 50, 60),
                             to_screen(cx, -geo.ramp_drop), to_screen(cx, cy), 3)
            pygame.draw.line(screen, ROD,
                             to_screen(cx - half, cy), to_screen(cx + half, cy), 6)
            lbl = font.render(str(n), True, DIM)
            screen.blit(lbl, (to_screen(cx, 0)[0] - 4, ORIGIN_Y + 78))

        # balls
        for i, b in enumerate(sc.balls):
            pygame.draw.circle(screen, BALL if i == 0 else BALL2,
                               to_screen(b.position.x, b.position.y),
                               int(geo.ball_diameter / 2 * PX_PER_MM))

        # HUD
        period = eng.wave_period_s()
        slew = "clipped" if period < 1.2 else ("marginal" if period < 2.4 else "clean")
        lines = [
            (big, f"speed {speed:2d}   period {period:6.2f} s   amp {AMP_NAME[amp]:>4}"
                  f"   spread {spread}   mirror {mirror}   rev {REVERSE_CHOICES[rev_i]}", FG),
            (font, f"servo tracking: {slew}      traverses: {traverses}"
                   f"      ball x: {sc.balls[0].position.x:7.1f} mm", DIM),
        ]
        if not wave_on:
            lines.append((big, "WAVE OFF (control) - ball should not move", ACCENT))
        if paused:
            lines.append((big, "PAUSED", ACCENT))
        y = 14
        for f, text, col in lines:
            screen.blit(f.render(text, True, col), (16, y))
            y += 26

        help_text = ("arrows speed/amp   S spread   M mirror(2-ball)   N reversal   "
                     "W wave off   R reset   SPACE pause   Q quit")
        screen.blit(font.render(help_text, True, DIM), (16, H - 26))

        pygame.display.flip()
        clock.tick(60)


if __name__ == "__main__":
    sys.exit(main())
