# Mechanical simulation

There is no hardware before tapeout, so the SPEED/AMP tables in PRD §6.6 must be frozen
without physical validation. This is the substitute: the **bit-exact** wave datapath
driving a 2D rigid-body model of the sculpture.

Answers are **directional** — "the table covers the right range", "spread=0 is more
reliable than spread=1" — not precise. Friction, elasticity, horn length and cradle shape
are estimates. Do not read a specific speed setting off this and call it correct.

## Setup

```bash
cd sim
python3 -m venv .venv
./.venv/bin/pip install pymunk numpy matplotlib
```

pymunk (Chipmunk2D) is used rather than Box2D: `pybox2d` has had no release since 2019
and no wheels for Python 3.14, while pymunk installs in one command and does the same job.

## Running

```bash
./.venv/bin/python view.py                       # interactive real-time window
./.venv/bin/python sweep.py                      # single ball, all 16 speeds x 2 spreads
./.venv/bin/python sweep.py --two-ball           # mirrored two-ball mode (PRD §6.7 / P2)
./.venv/bin/python sweep.py --amp 3 --plot p.png # fix amplitude, write a trajectory plot
./.venv/bin/python sweep.py --control            # wave disabled -- see "Control" below
```

## Watching it (`view.py`)

A real-time pygame window. It drives the *same* `Sculpture.step_once()` the sweep uses, so
the picture is the simulation being reported on, not a separate illustration.

| Key | Does |
|---|---|
| LEFT / RIGHT | speed setting 0..15 (period 20.1 s .. 0.26 s) |
| UP / DOWN | amplitude 25 / 50 / 75 / 100 % |
| `S` | toggle spread (0 = full wavelength, 1 = half) |
| `M` | toggle mirror -- two-ball mode (PRD §6.7) |
| `N` | reversal period: 2 / 3 / 4 / 6 wave cycles (`REVERSE[1:0]`, PRD §6.5) |
| `W` | **wave off** -- the control experiment, live |
| `R` reset, SPACE pause, `Q` quit | |

`W` is the one to reach for when a result looks too good. A shallow-U track is a pendulum;
with the wave off the ball must sit still. If it keeps moving, the geometry is doing the
work and nothing on screen is attributable to the wave.

Sweeping LEFT/RIGHT through the speed settings is the fastest way to see the two regimes
described below: smooth sinusoidal motion at low speeds, and the servos visibly slamming
between stops at speed 10+.

pygame-ce is used rather than pygame -- upstream pygame has no Python 3.14 wheel.

## Files

| File | What |
|---|---|
| `wave_model.py` | Bit-exact Python model of the RTL datapath. Same sine table, same quadrant mirror, same amplitude shifts, same accumulator. Doubles as the golden reference for the cocotb tests. |
| `mechanics.py` | Geometry, servo slew model, pymunk world |
| `sweep.py` | Parameter sweep and reporting |

Because `wave_model.py` is bit-exact, conclusions here apply to the real chip, not to an
idealised sine.

## Control — run this whenever geometry changes

```
CONTROL (wave disabled, array flat): traverses=0  x 81..81
```

A shallow-U track is a pendulum: a ball in it oscillates on its own. If the control shows
any traverses, the track geometry is moving the ball by itself and **every sweep result is
meaningless**. It currently reads 0, so the motion below is wave-driven.

## Findings

### 1. End geometry matters more than the wave — the biggest result here

With **vertical end stops**, the ball arrives at one end and stays there *permanently*.
Every speed, every amplitude. It perches on the last cradle leaning against the wall, in a
stable equilibrium the wave cannot break. Direction reversal does not recover it.

With **shallow-U end ramps** (45 mm run, 35 mm rise, starting 22 mm below the rod line),
every setting sustains continuous back-and-forth motion.

That is a mechanical design requirement, not a chip one, and it would have been very
expensive to discover after the sculpture was built. PRD §4 already says "shallow U-shaped
guide" — this says the U is load-bearing, not decorative, and the ends must slope enough
to return the ball into the array.

### 2. The SPEED table range is correct as frozen — do not re-centre it

Single ball, 100% amplitude, spread=0, 60 s runs:

| Speed | Period | Traverses | Servo tracking |
|---|---|---|---|
| 0–5 | 20.1–4.7 s | 1–4 | tracks cleanly |
| 8 | 2.0 s | 10 | marginal (5.5 deg error) |
| 10–13 | 1.1–0.47 s | 18–41 | slew-limited, waveform clipped |
| 14–15 | 0.35–0.26 s | 32, 16 | heavily clipped, transport degrades |

Two distinct regimes, and **both are useful**:

- **Speeds 0–8** are the smooth aesthetic regime. Servos track the sine faithfully, motion
  is quiet, the ball drifts gracefully. Fewer traverses but this is what a kinetic art
  piece wants.
- **Speeds 10–13** move the ball far more vigorously, but only because the servos are in
  slew limiting and the sine is clipped toward a square wave. Loud, jerky, hard on the
  gears. Legitimate as a "high energy" mode; not what you would leave running.
- **Speeds 14–15** are over-driven and transport gets *worse* again.

So the 16-entry table spanning 0.26–20 s is justified end to end. An earlier reading of
partial results suggested re-centring the table slower; that was wrong, and came from
measuring `max(x) - min(x)` instead of completed traverses.

### 3. spread=0 is the reliable geometry

`spread=0` (one full wavelength across the array) transports at every speed. `spread=1`
works too but is erratic — several settings park the ball. Reset default should be
`spread=0`.

### 4. The reversal period was hardcoded wrong — now `REVERSE[1:0]` on `uio[6:5]`

PRD §6.5 originally flipped the wave direction after a fixed **4 wave cycles**. That number
was never derived from anything: the reversal is open loop, and nothing in the design knows
where the ball is or how long a crossing takes.

A ball crosses in roughly 2 cycles. At N=4 it arrives and then sits at the end of the track
for two more cycles doing nothing — tens of seconds of visible stall at slow settings.

Traverses per minute, 180 s runs, single ball, 100% amplitude, spread 0:

| Speed band | N=2 | N=3 | N=4 | N=6 |
|---|---|---|---|---|
| 0-2 (20-11 s) | 1.3 / 0.3 / 1.7 | 0.7 / 2.0 / 2.0 | 1.0 / 0.7 / 1.0 | 0.7 / 0.7 / 1.0 |
| 3-8 (8.4-2.0 s) | **6.0 - 22.7** | 3.3 - 15.0 | 2.3 - 10.0 | 1.3 - 6.0 |
| 9-12 (1.5-0.62 s) | **36.3 - 65.7** | 19.7 - 46.3 | 13.3 - 31.0 | 8.0 - 19.0 |
| 13-15 (0.47-0.26 s) | 53.3 / 19.3 / 3.7 | **62.0** / 31.3 / 14.3 | 41.3 / **32.3** / **14.7** | 25.0 / 28.7 / 10.3 |

**N=2 wins 10 of 16 speeds and every setting in the usable band (3-12), often by 2-3x.**
It loses only where nothing works (0-2, all values under 2/min) or where the servos are
already slew-clipped into uselessness (13-15).

Reset default is now **2 cycles**. The pin field is kept anyway: 20 GE, two otherwise-unused
`uio` pins, and the friction figures here are estimates.

*Methodology note:* an earlier 120 s sweep showed speed 5 preferring N=4 by a wide margin.
That did not reproduce at 180 s. Short runs give noisy per-cell results in this model —
use 180 s or longer before drawing conclusions from a single cell.

### 5. Two-ball mirrored mode (P2) probably does not work as intended

Most settings produce **zero** traverses; the balls stay bunched in one half. Only speeds
10–11 show partial converge/return behaviour.

This matches the asymmetry flagged in PRD §6.7: the outward leg is a clean travelling
trough, but the inward leg has a transient barrier the balls do not clear. **Keep P2** —
it costs ~26 GE, roughly 1% of the area budget, and a free option worth having — but do
not plan the exhibit around it.

## What is modelled

- The bit-exact wave datapath at 50.08 Hz
- **SG90 slew limiting** (600 deg/s). This is what makes the fast end of the table clip,
  and it is the single most important dynamic term here.
- 8 narrow cradles as kinematic bodies; narrow so a 40 mm ball always bridges 2–3 and
  feels the slope of their envelope, per PRD §4
- Ball(s) as dynamic circles with friction, contact and rolling inertia
- Shallow-U end ramps

## What is not

- Cradle cup shape (flat-topped segments here)
- Servo torque limits — irrelevant, 1.8 kg·cm against a 2.7 g ball
- Air drag, spin decay, guide-channel sidewalls, the third dimension
- Manufacturing slop, servo centring error, linkage backlash

## Known modelling traps found while building this

Both produced confident, completely wrong results before being caught:

1. **Wide flat cradle tops.** A ball perched on a single flat platform feels no lateral
   force however the platform moves, so it sat motionless forever. Cradles must be narrow
   relative to rod spacing so the ball always bridges.
2. **`max(x) - min(x)` as the success metric.** Cannot distinguish "swept the track once
   and parked" from "oscillates continuously". Count completed traverses instead.
