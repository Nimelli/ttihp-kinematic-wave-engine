#!/usr/bin/env python3
"""Sweep the frozen parameter tables and report which settings transport a ball.

This is the deliverable: it answers the three questions that PRD §6.6 and §6.7
would otherwise have to be frozen blind.

    ./sweep.py              one-ball sweep over all 16 speeds x 2 spreads
    ./sweep.py --two-ball   two-ball mirrored mode (PRD §6.7)
    ./sweep.py --plot out.png   trajectory plot for the best setting found
"""

import argparse
import sys

from mechanics import Geometry, Sculpture
from wave_model import SPEED_TABLE, WaveEngine


def run_one(speed, amp, spread, mirror, n_balls, duration, geo):
    eng = WaveEngine(speed=speed, amp=amp, spread=spread, mirror=mirror)
    return eng, Sculpture(eng, geo, n_balls=n_balls).run(duration_s=duration)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--two-ball", action="store_true",
                    help="mirrored geometry with two balls (PRD §6.7 / P2)")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--amp", type=int, default=None,
                    help="fix amplitude 0..3; default sweeps 1,2,3")
    ap.add_argument("--plot", metavar="PNG", default=None)
    ap.add_argument("--control", action="store_true",
                    help="hold the array flat (wave disabled) and report. Any "
                         "traverses here are the U-shaped track acting as a "
                         "pendulum, not wave transport -- run this whenever the "
                         "geometry changes, or the sweep means nothing.")
    args = ap.parse_args()

    if args.control:
        class Flat(WaveEngine):
            def positions(self):
                return [128] * 8

            def step(self):
                pass

        geo = Geometry()
        r = Sculpture(Flat(), geo,
                      n_balls=2 if args.two_ball else 1).run(duration_s=args.duration)
        print(f"CONTROL (wave disabled, array flat): "
              f"traverses={r.traverses}  x {r.x_min:.0f}..{r.x_max:.0f}")
        print("expected: 0 traverses. Anything else means the track geometry "
              "moves the ball by itself and the sweep results are not "
              "attributable to the wave.")
        return 0

    geo = Geometry()
    mirror = 1 if args.two_ball else 0
    n_balls = 2 if args.two_ball else 1
    amps = [args.amp] if args.amp is not None else [1, 2, 3]
    amp_name = {0: "25%", 1: "50%", 2: "75%", 3: "100%"}

    print(f"track span {geo.span:.0f} mm, ball {geo.ball_diameter:.0f} mm, "
          f"rod spacing {geo.rod_spacing:.0f} mm")
    print(f"servo: {geo.servo_span_deg:.0f} deg span, "
          f"{geo.servo_slew_deg_s:.0f} deg/s slew, {geo.horn_mm:.0f} mm horn")
    print(f"mode: {'TWO-BALL mirrored' if mirror else 'single ball travelling wave'}, "
          f"{args.duration:.0f} s per run\n")

    header = f"{'spd':>3} {'period':>8} {'amp':>5} {'spr':>3} " \
             f"{'trav/s':>7} {'x range':>14} {'ends':>5}"
    print(header)
    print("-" * len(header))

    best = None
    for spread in (0, 1):
        for speed in range(len(SPEED_TABLE)):
            for amp in amps:
                eng, r = run_one(speed, amp, spread, mirror,
                                 n_balls, args.duration, geo)
                period = eng.wave_period_s()
                frac = r.span_travelled / geo.span
                mark = "YES" if r.reached_both_ends else ""
                print(f"{speed:>3} {period:>7.2f}s {amp_name[amp]:>5} {spread:>3} "
                      f"{r.traverses:>7} "
                      f"{r.x_min:>6.0f}..{r.x_max:<6.0f} {mark:>5}")
                score = (r.traverses, r.reached_both_ends, frac)
                if best is None or score > best[0]:
                    best = (score, speed, amp, spread, period, r)
        print()

    if best is None:
        print("no runs completed")
        return 1

    score, speed, amp, spread, period, r = best
    print("=" * len(header))
    print(f"best: speed={speed} ({period:.2f} s)  amp={amp_name[amp]}  spread={spread}")
    print(f"      {r.traverses} completed traverses, swept {r.span_travelled:.0f} mm of {geo.span:.0f} mm")
    if not r.reached_both_ends:
        print("      NOTE: no setting moved a ball end to end. Either the wave "
              "amplitude is too small for this geometry, or the horn/spacing "
              "estimates in Geometry need revisiting.")

    if args.plot:
        plot(r, geo, args.plot, speed, period, amp_name[amp], spread)
    return 0


def plot(result, geo, path, speed, period, amp, spread):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ts = [t for t, _ in result.trace]
    n_balls = len(result.trace[0][1]) if result.trace else 1

    fig, ax = plt.subplots(figsize=(10, 4.5))
    for i in range(n_balls):
        ax.plot(ts, [xs[i] for _, xs in result.trace],
                label=f"ball {i}", linewidth=1.6)
    for n in range(geo.n_rods):
        ax.axhline(geo.rod_x(n), color="0.85", linewidth=0.7, zorder=0)

    ax.set_xlabel("time (s)")
    ax.set_ylabel("position along track (mm)")
    ax.set_title(f"speed={speed} ({period:.2f} s wave)  amp={amp}  spread={spread}")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    print(f"wrote {path}")


if __name__ == "__main__":
    sys.exit(main())
