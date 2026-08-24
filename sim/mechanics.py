"""Mechanical simulation of the kinetic sculpture, driven by the real wave model.

Units are millimetres, grams and seconds throughout (so gravity is 9810 mm/s^2).

WHAT THIS IS FOR
    There is no hardware before tapeout, so the SPEED/AMP tables in PRD §6.6 are
    frozen without physical validation. This model is the substitute. It gives
    DIRECTIONAL answers -- "the table is centred in roughly the right decade",
    "spread=1 carries better than spread=0" -- not precise ones. Friction,
    linkage geometry and cradle shape are estimates.

WHAT IS MODELLED
    - the bit-exact wave datapath (wave_model.py), updating at 50.08 Hz
    - SG90 slew limiting: the servo cannot jump to its commanded angle, it
      travels at a finite deg/s. This is what makes fast wave settings fail.
    - 8 cradles as kinematic bodies whose height follows the servo angle
    - ball(s) as dynamic circles with friction, contact and rolling inertia
    - end stops at both ends of the track

WHAT IS NOT MODELLED
    - the cradle cup shape (cradles are flat-topped segments here)
    - servo torque limits (irrelevant: 1.8 kg.cm against a 2.7 g ball)
    - air drag, spin decay, the guide channel's sidewalls
"""

from dataclasses import dataclass, field

import pymunk

from wave_model import WaveEngine, FRAME_S


@dataclass
class Geometry:
    n_rods: int = 8
    rod_spacing: float = 23.0      # mm, PRD §4 (12 mm is impossible: SG90 is 12.2 mm wide)
    ball_diameter: float = 40.0    # mm, standard ping-pong ball
    ball_mass: float = 2.7         # g
    horn_mm: float = 20.0          # effective pushrod arm; vertical = horn * sin(angle)
    servo_span_deg: float = 120.0  # travel across the full 1.0-2.0 ms pulse range
    servo_slew_deg_s: float = 600.0  # SG90: 60 deg / 0.1 s
    # Cradle tops must be NARROW relative to the 23 mm spacing, so a 40 mm ball
    # always bridges 2-3 of them and therefore always feels the slope of their
    # envelope. This matches PRD §4 ("the cup rims collectively form the wave
    # surface"). With a wide flat top the ball can perch on a single cradle,
    # feel a perfectly level surface, and sit there permanently no matter what
    # the wave does -- which is a modelling artefact, not real behaviour.
    cradle_width: float = 8.0      # mm
    friction: float = 0.4
    elasticity: float = 0.2

    # Shallow-U end ramps (PRD §4). Each starts just below the last rod and
    # climbs outward, so a ball reaching the end is returned into the array.
    ramp_run: float = 45.0         # mm outward from the last rod
    ramp_rise: float = 35.0        # mm of climb over that run (~38 deg)
    ramp_drop: float = 22.0        # mm below the rod centre line at the start

    @property
    def span(self):
        return (self.n_rods - 1) * self.rod_spacing

    def rod_x(self, n):
        return n * self.rod_spacing

    def pos_to_deg(self, pos):
        """Commanded servo angle. pos 128 (1.5 ms) is centre, 0 deg."""
        return (pos - 128) / 128.0 * (self.servo_span_deg / 2.0)


@dataclass
class Result:
    span_travelled: float = 0.0
    x_min: float = 0.0
    x_max: float = 0.0
    reached_both_ends: bool = False
    reversals: int = 0
    # Completed end-to-end trips: the ball entering the far third having last
    # been in the near third. This is the metric that matters -- span_travelled
    # cannot tell "swept the track once and parked" apart from "oscillates".
    traverses: int = 0
    trace: list = field(default_factory=list)   # (t, [ball x...])
    collisions: int = 0


class Sculpture:
    def __init__(self, engine: WaveEngine, geo: Geometry = None, n_balls=1):
        self.geo = geo or Geometry()
        self.engine = engine
        self.n_balls = n_balls

        self.space = pymunk.Space()
        self.space.gravity = (0.0, -9810.0)

        g = self.geo
        self.servo_deg = [0.0] * g.n_rods       # actual angle, slew-limited
        self.cmd_deg = [0.0] * g.n_rods         # commanded angle, updated per frame

        # Cradles: kinematic bodies driven by velocity each step.
        self.cradles = []
        for n in range(g.n_rods):
            body = pymunk.Body(body_type=pymunk.Body.KINEMATIC)
            body.position = (g.rod_x(n), 0.0)
            shape = pymunk.Segment(body,
                                   (-g.cradle_width / 2, 0.0),
                                   (g.cradle_width / 2, 0.0),
                                   2.0)
            shape.friction = g.friction
            shape.elasticity = g.elasticity
            self.space.add(body, shape)
            self.cradles.append(body)

        # End ramps, not vertical walls.
        #
        # PRD §4 calls for a "shallow U-shaped guide". That shape matters more
        # than it looks: with a vertical end stop the ball perches on the last
        # cradle leaning against the wall, in a stable equilibrium that the wave
        # cannot break -- it arrives once and never comes back. A ramp sloping up
        # and outward always has a component pushing the ball back into the
        # array, so the wave can pick it up again on the return sweep.
        for sign, x0 in ((-1.0, g.rod_x(0)), (+1.0, g.rod_x(g.n_rods - 1))):
            ramp = pymunk.Body(body_type=pymunk.Body.STATIC)
            seg = pymunk.Segment(
                ramp,
                (x0, -g.ramp_drop),
                (x0 + sign * g.ramp_run, -g.ramp_drop + g.ramp_rise),
                2.0)
            seg.friction = g.friction
            seg.elasticity = g.elasticity
            self.space.add(ramp, seg)

        # Balls. Two-ball mode starts them at opposite ends.
        r = g.ball_diameter / 2.0
        self.balls = []
        if n_balls == 1:
            starts = [g.span / 2.0]
        else:
            starts = [g.rod_x(0), g.rod_x(g.n_rods - 1)]
        for x0 in starts:
            moment = pymunk.moment_for_circle(g.ball_mass, 0, r)
            body = pymunk.Body(g.ball_mass, moment)
            body.position = (x0, r + 30.0)
            shape = pymunk.Circle(body, r)
            shape.friction = g.friction
            shape.elasticity = g.elasticity
            self.space.add(body, shape)
            self.balls.append(body)

    def _cradle_y(self, n):
        import math
        return self.geo.horn_mm * math.sin(math.radians(self.servo_deg[n]))

    def _apply_frame(self):
        """New commanded positions once per 19.968 ms frame."""
        for n, pos in enumerate(self.engine.positions()):
            self.cmd_deg[n] = self.geo.pos_to_deg(pos)
        self.engine.step()

    def _slew(self, dt):
        """Servos travel toward the command at a finite rate.

        This is the term that makes the fast end of the SPEED table unusable:
        beyond ~600 deg/s demand the waveform is lag-clipped by the servo.
        """
        max_step = self.geo.servo_slew_deg_s * dt
        for n in range(self.geo.n_rods):
            err = self.cmd_deg[n] - self.servo_deg[n]
            if err > max_step:
                err = max_step
            elif err < -max_step:
                err = -max_step
            self.servo_deg[n] += err

    def settle(self, settle_s=1.0, dt=1e-3):
        """Let the balls come to rest on a flat array before the wave starts."""
        self._apply_frame()
        for _ in range(int(settle_s / dt)):
            self._drive_cradles(dt)
            self.space.step(dt)

    def step_once(self, dt=1e-3):
        """Advance the whole model by one physics step.

        Split out of run() so the interactive viewer can drive the same code
        path -- the picture on screen is the same simulation the sweep reports.
        """
        self._frame_acc = getattr(self, "_frame_acc", 0.0) + dt
        if self._frame_acc >= FRAME_S:
            self._frame_acc -= FRAME_S
            self._apply_frame()
        self._slew(dt)
        self._drive_cradles(dt)
        self.space.step(dt)

    def run(self, duration_s=30.0, dt=1e-3, settle_s=1.0, trace_every=0.02):
        g = self.geo
        frame_acc = 0.0
        trace_acc = 0.0
        result = Result()
        xs_seen = []

        # Let the balls settle onto a flat array before the wave starts.
        self._apply_frame()
        for _ in range(int(settle_s / dt)):
            self._drive_cradles(dt)
            self.space.step(dt)

        t = 0.0
        prev_dir = 0
        prev_x = self.balls[0].position.x
        left_edge = g.rod_x(0) + g.span / 3.0
        right_edge = g.rod_x(0) + 2.0 * g.span / 3.0
        zone = 0  # -1 = in left third, +1 = in right third, 0 = middle/unknown
        while t < duration_s:
            frame_acc += dt
            if frame_acc >= FRAME_S:
                frame_acc -= FRAME_S
                self._apply_frame()

            self._slew(dt)
            self._drive_cradles(dt)
            self.space.step(dt)

            x = self.balls[0].position.x
            xs_seen.append(x)
            d = 1 if x > prev_x + 0.05 else (-1 if x < prev_x - 0.05 else prev_dir)
            if d != 0 and prev_dir != 0 and d != prev_dir:
                result.reversals += 1
            if d != 0:
                prev_dir = d
            prev_x = x

            if x < left_edge:
                if zone == 1:
                    result.traverses += 1
                zone = -1
            elif x > right_edge:
                if zone == -1:
                    result.traverses += 1
                zone = 1

            if len(self.balls) == 2:
                sep = abs(self.balls[0].position.x - self.balls[1].position.x)
                if sep < g.ball_diameter + 1.0:
                    result.collisions += 1

            trace_acc += dt
            if trace_acc >= trace_every:
                trace_acc = 0.0
                result.trace.append((t, [b.position.x for b in self.balls]))

            # A ball that has fallen out of the channel ends the run.
            if any(b.position.y < -100.0 for b in self.balls):
                break

            t += dt

        result.x_min = min(xs_seen)
        result.x_max = max(xs_seen)
        result.span_travelled = result.x_max - result.x_min
        margin = g.rod_spacing
        result.reached_both_ends = (result.x_min < g.rod_x(0) + margin and
                                    result.x_max > g.rod_x(g.n_rods - 1) - margin)
        return result

    def _drive_cradles(self, dt):
        """Set kinematic velocity so pymunk lands exactly on the target height.

        Velocity (not a position teleport) is what lets contact friction carry
        the ball -- a teleported surface imparts no momentum.
        """
        for n, body in enumerate(self.cradles):
            target = self._cradle_y(n)
            body.velocity = (0.0, (target - body.position.y) / dt)
