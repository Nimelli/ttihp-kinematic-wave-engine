"""Bit-exact Python model of the Kinematic Wave Engine datapath.

This mirrors the RTL spec in PRD-v2 §6.3-§6.7 exactly: same sine table, same
quadrant mirror, same amplitude shifts, same phase accumulator. It is the golden
reference the cocotb tests compare against, and the position source for the
mechanical simulation.

Because it is bit-exact, anything the mechanical sim concludes about wave
parameters applies to the real chip -- not to an idealised sine.
"""

import math

# --- PRD §6.4: quarter-wave sine table -------------------------------------
# Half-step centring (the +0.5) is what makes the quadrant mirror exact.
SINE_LUT = [round(127 * math.sin((i + 0.5) * math.pi / 64)) for i in range(32)]

# --- PRD §6.6: 16 phase increments, ~1.35x apart, 0.26 s to 20 s -----------
SPEED_TABLE = [65, 87, 119, 156, 208, 278, 374, 503,
               654, 872, 1190, 1577, 2111, 2784, 3739, 5033]

FRAME_S = 19.968e-3        # 50.08 Hz, PRD §6.1
ACC_BITS = 16
TURN = 128                 # angle steps in one full wave cycle
REVERSE_AFTER_CYCLES = 4   # PRD §6.5


def sine(angle):
    """angle 0..127 -> -127..+127. Mirrors kwe_sine."""
    quad = (angle >> 5) & 3
    idx = angle & 31
    lut_idx = 31 - idx if quad in (1, 3) else idx
    mag = SINE_LUT[lut_idx]
    return -mag if quad >= 2 else mag


def amp_scale(s, amp):
    """Mirrors kwe_amp_scale: shifts and one subtract, no multiplier.

    Python's >> on negative ints floors, which matches Verilog's signed >>>.
    """
    if amp == 0:
        v = s >> 2          # 25%
    elif amp == 1:
        v = s >> 1          # 50%
    elif amp == 2:
        v = s - (s >> 2)    # 75%
    else:
        v = s               # 100%
    return max(1, min(255, 128 + v))


def angle_for(phase, slot, spread, mirror):
    """Mirrors kwe_angle_map. `mirror` selects the two-ball geometry."""
    if mirror:
        m = slot if slot < 4 else 7 - slot      # 0,1,2,3,3,2,1,0
    else:
        m = slot
    delta = 8 if spread else 16
    return (phase + m * delta) & (TURN - 1)


class WaveEngine:
    """Frame-by-frame position generator, one frame per call to step()."""

    def __init__(self, speed=8, amp=1, spread=0, mirror=0, auto_reverse=True,
                 reverse_cycles=REVERSE_AFTER_CYCLES):
        self.inc = SPEED_TABLE[speed]
        self.amp = amp
        self.spread = spread
        self.mirror = mirror
        self.auto_reverse = auto_reverse
        self.reverse_cycles = reverse_cycles
        self.acc = 0
        self.forward = True
        self.cycles = 0

    @property
    def phase(self):
        return (self.acc >> (ACC_BITS - 7)) & (TURN - 1)

    def positions(self):
        """The 8 commanded positions (0..255) for the current phase."""
        return [amp_scale(sine(angle_for(self.phase, n, self.spread, self.mirror)),
                          self.amp)
                for n in range(8)]

    def step(self):
        """Advance one 19.968 ms frame. Mirrors kwe_phase_gen at phase_tick."""
        prev = self.acc
        delta = self.inc if self.forward else -self.inc
        self.acc = (self.acc + delta) & ((1 << ACC_BITS) - 1)

        wrapped = (self.acc < prev) if self.forward else (self.acc > prev)
        if wrapped:
            self.cycles += 1
            if self.auto_reverse and self.cycles >= self.reverse_cycles:
                self.cycles = 0
                self.forward = not self.forward

    def wave_period_s(self):
        return (1 << ACC_BITS) / (self.inc / FRAME_S)


def pos_to_pulse_ticks(pos):
    """Mirrors kwe_servo_pwm: {1'b1, pos} == 256 + pos."""
    return 256 + pos


def pos_to_pulse_ms(pos):
    return pos_to_pulse_ticks(pos) * 3.9e-3
