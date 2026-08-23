# SPDX-License-Identifier: Apache-2.0
"""
Unit tests for kwe_sine  --  Kinematic Wave Engine stage 3b.

Contract under test (PRD-v2 §6.4):

    angle[6:0]  unsigned 0..127, one full turn of the wave
    sine[8:0]   signed, -127 .. +127

    angle[6:5] selects the quadrant, angle[4:0] indexes kwe_sine_lut:

        00  ->  +LUT[idx]
        01  ->  +LUT[31 - idx]
        10  ->  -LUT[idx]
        11  ->  -LUT[31 - idx]

Only the first quadrant is stored; the other three come from mirroring the index
and negating the output. That is the whole reason a full 128-point sine costs 32
table entries instead of 128.

Purely combinational, so no clock. 128 possible inputs means these tests are
EXHAUSTIVE against Python's math.sin -- once green, this module is proven for every
input it can ever see, not sampled.

Run:  source ~/oss-cad-suite/environment && make MOD=kwe_sine
"""

import math

import cocotb
from cocotb.triggers import Timer

TURN = 128
AMPLITUDE = 127
SINE_BITS = 9


def reference(angle):
    """Ideal value: round(127 * sin((angle + 0.5) * 2pi/128)).

    Identical by construction to mirroring the quarter-wave table, because the
    half-step centring makes the mirror mathematically exact.
    """
    return round(AMPLITUDE * math.sin((angle + 0.5) * 2 * math.pi / TURN))


def to_signed(sig, width=SINE_BITS):
    """Two's-complement read.

    Done by hand rather than via LogicArray.to_signed() so the test does not
    depend on the DUT actually declaring the port `signed` -- a common slip that
    would otherwise show up as a confusing API error instead of a value mismatch.
    """
    v = int(sig.value)
    return v - (1 << width) if v & (1 << (width - 1)) else v


async def read(dut, angle):
    dut.angle.value = angle
    await Timer(1, unit="ns")
    return to_signed(dut.sine)


@cocotb.test()
async def test_exhaustive_vs_reference(dut):
    """All 128 angles match an ideal sine exactly.

    This is the test that matters. Everything below is diagnostic: it narrows
    down *which* part of the quadrant logic is wrong when this one fails.
    """
    wrong = []
    for angle in range(TURN):
        got = await read(dut, angle)
        want = reference(angle)
        if got != want:
            wrong.append((angle, got, want))

    if wrong:
        head = "\n".join(
            f"  angle={a:3d} (quadrant {a >> 5}, idx {a & 31:2d}): "
            f"got {g:4d}, expected {w:4d}"
            for a, g, w in wrong[:12]
        )
        more = f"\n  ... and {len(wrong) - 12} more" if len(wrong) > 12 else ""
        raise AssertionError(
            f"{len(wrong)} of {TURN} angles wrong:\n{head}{more}"
        )


@cocotb.test()
async def test_quadrant_signs(dut):
    """Quadrants 0 and 1 are positive; 2 and 3 are negative."""
    for angle in range(TURN):
        got = await read(dut, angle)
        quadrant = angle >> 5
        if quadrant < 2:
            assert got > 0, (
                f"angle={angle} (quadrant {quadrant}): got {got}, expected positive. "
                "The first half turn of a sine is above zero -- check the sign term."
            )
        else:
            assert got < 0, (
                f"angle={angle} (quadrant {quadrant}): got {got}, expected negative. "
                "Quadrants 2 and 3 must negate the table output."
            )


@cocotb.test()
async def test_half_turn_antisymmetry(dut):
    """sin(x + pi) == -sin(x), i.e. sine(a + 64) == -sine(a).

    Catches a wrong negation or a quadrant swap even when the magnitudes happen
    to look plausible.
    """
    for angle in range(TURN // 2):
        a = await read(dut, angle)
        b = await read(dut, angle + TURN // 2)
        assert b == -a, (
            f"sine({angle}) = {a} but sine({angle + TURN // 2}) = {b}, expected {-a}. "
            "Angles half a turn apart must be exact negatives."
        )


@cocotb.test()
async def test_quarter_turn_mirror(dut):
    """sin(pi - x) == sin(x), i.e. sine(63 - a) == sine(a) for a in 0..31.

    This is the index mirror itself. If quadrant 1 uses LUT[idx] instead of
    LUT[31 - idx], the ramp runs backwards and only this test says so plainly.
    """
    for angle in range(TURN // 4):
        a = await read(dut, angle)
        b = await read(dut, (TURN // 2 - 1) - angle)
        assert b == a, (
            f"sine({angle}) = {a} but sine({(TURN // 2 - 1) - angle}) = {b}, expected {a}. "
            "Quadrant 1 must index the table backwards: LUT[31 - idx]."
        )


@cocotb.test()
async def test_range_and_peaks(dut):
    """Output stays in [-127, +127] and peaks where a sine should."""
    values = [await read(dut, a) for a in range(TURN)]

    for angle, got in enumerate(values):
        assert -AMPLITUDE <= got <= AMPLITUDE, (
            f"angle={angle}: got {got}, outside +/-{AMPLITUDE}. "
            f"128 + sine must fit in 8 bits, so amplitude cannot exceed {AMPLITUDE}."
        )

    assert max(values) == AMPLITUDE, f"peak is {max(values)}, expected +{AMPLITUDE}"
    assert min(values) == -AMPLITUDE, f"trough is {min(values)}, expected -{AMPLITUDE}"

    # The peak is a plateau, not a single point: LUT[30] and LUT[31] are both 127,
    # so by the quadrant mirror the maximum is held across angles 30..33 -- centred
    # on the quarter turn at 31.5. Same by antisymmetry for the trough at 94..97.
    peaks = [a for a, v in enumerate(values) if v == AMPLITUDE]
    troughs = [a for a, v in enumerate(values) if v == -AMPLITUDE]

    assert set(peaks) <= {30, 31, 32, 33}, (
        f"maximum reached at angles {peaks}, expected within 30..33 "
        "(the plateau centred on a quarter turn)"
    )
    assert set(troughs) <= {94, 95, 96, 97}, (
        f"minimum reached at angles {troughs}, expected within 94..97 "
        "(the plateau centred on three quarters of a turn)"
    )


@cocotb.test()
async def test_max_step_around_the_turn(dut):
    """No step larger than 7 LSB anywhere, including across quadrant seams.

    The seams are where a mirror or sign error hides: the values on either side
    can both be individually plausible while the join has a visible jump. A wave
    with a discontinuity at a quadrant boundary would jerk the servos four times
    per cycle.
    """
    values = [await read(dut, a) for a in range(TURN)]

    worst_step = 0
    worst_at = None
    for a in range(TURN):
        step = abs(values[(a + 1) % TURN] - values[a])
        if step > worst_step:
            worst_step, worst_at = step, a

    assert worst_step <= 7, (
        f"step of {worst_step} LSB between angle {worst_at} "
        f"({values[worst_at]}) and {(worst_at + 1) % TURN} "
        f"({values[(worst_at + 1) % TURN]}), expected <= 7. "
        "A jump at a quadrant seam (angle 31/32, 63/64, 95/96, 127/0) means the "
        "index mirror or the sign is wrong on one side."
    )
