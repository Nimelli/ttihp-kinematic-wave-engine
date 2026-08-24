# SPDX-License-Identifier: Apache-2.0
"""
Unit tests for kwe_amp_scale  --  Kinematic Wave Engine stage 5.

Contract under test (PRD-v2 §6.6):

    amp = 00 -> v = sine >>> 2            25%
    amp = 01 -> v = sine >>> 1            50%
    amp = 10 -> v = sine - (sine >>> 2)   75%
    amp = 11 -> v = sine                  100%

    pos = 128 + v, clamped to 1..255

Shifts and one subtract. **No multiplier belongs in this module** -- that is the whole
reason the amplitude presets are 25/50/75/100 rather than arbitrary percentages.

`>>>` is an ARITHMETIC shift and `sine` is signed, so the sign must be preserved:
-127 >>> 1 is -64, not +64 and not 191. This is BUILD-PLAN Trap 3, and it is the most
likely bug in this module: if any operand in the expression is unsigned, Verilog
evaluates the whole thing unsigned and the negative half of the wave turns to garbage.
Python's >> on negative ints floors, which matches Verilog's signed >>>, so the
reference below is directly comparable.

Purely combinational, no clock. 255 sine values x 4 amplitudes = 1020 inputs, so these
tests are EXHAUSTIVE.

Run:  source ~/oss-cad-suite/environment && make MOD=kwe_amp_scale
"""

import cocotb
from cocotb.triggers import Timer

AMP_NAME = {0: "25%", 1: "50%", 2: "75%", 3: "100%"}
CENTRE = 128


def reference(s, amp):
    if amp == 0:
        v = s >> 2
    elif amp == 1:
        v = s >> 1
    elif amp == 2:
        v = s - (s >> 2)
    else:
        v = s
    return max(1, min(255, CENTRE + v))


async def read(dut, sine, amp):
    dut.sine.value = sine          # cocotb handles the two's-complement encoding
    dut.amp.value = amp
    await Timer(1, unit="ns")
    return int(dut.pos.value)


@cocotb.test()
async def test_exhaustive(dut):
    """All 255 sine values x 4 amplitudes match the spec."""
    wrong = []
    for amp in range(4):
        for s in range(-127, 128):
            got = await read(dut, s, amp)
            want = reference(s, amp)
            if got != want:
                wrong.append((s, amp, got, want))

    if wrong:
        head = "\n".join(
            f"  sine={s:5d} amp={AMP_NAME[a]:>5}: got {g:4d}, expected {w:4d}"
            for s, a, g, w in wrong[:12]
        )
        more = f"\n  ... and {len(wrong) - 12} more" if len(wrong) > 12 else ""
        neg = sum(1 for s, _, _, _ in wrong if s < 0)
        hint = ""
        if neg == len(wrong):
            hint = ("\n\nEvery failure is at a NEGATIVE sine value. That is the signature "
                    "of unsigned arithmetic: declare every intermediate `signed` and use "
                    "$signed() where unsure. See BUILD-PLAN Trap 3.")
        raise AssertionError(f"{len(wrong)} of 1020 combinations wrong:\n{head}{more}{hint}")


@cocotb.test()
async def test_centre_is_centre(dut):
    """sine = 0 gives 128 (a 1.5 ms pulse) at every amplitude.

    Scaling zero must give zero regardless of amplitude, otherwise the array does not
    sit flat when the wave passes through its zero crossing.
    """
    for amp in range(4):
        got = await read(dut, 0, amp)
        assert got == CENTRE, (
            f"amp={AMP_NAME[amp]}: sine=0 gave pos={got}, expected {CENTRE}"
        )


@cocotb.test()
async def test_output_never_leaves_servo_range(dut):
    """pos stays in 1..255 for every input -- the servo-safety property."""
    for amp in range(4):
        for s in range(-127, 128):
            got = await read(dut, s, amp)
            assert 1 <= got <= 255, (
                f"sine={s} amp={AMP_NAME[amp]}: pos={got}, outside 1..255. "
                "Clamp in a wider signed intermediate BEFORE truncating to 8 bits."
            )


@cocotb.test()
async def test_sign_is_preserved(dut):
    """Negative sine must give pos below centre. Catches lost signedness instantly.

    Magnitudes are kept at 8 or above deliberately. At 25% amplitude a sine of 1
    shifts down to 0, so pos is exactly 128 -- correct quantisation, not a sign bug,
    and asserting strict inequality there would fail a correct design.
    """
    for amp in range(4):
        for s in (-127, -100, -64, -33, -8):
            got = await read(dut, s, amp)
            assert got < CENTRE, (
                f"sine={s} amp={AMP_NAME[amp]}: pos={got}, expected below {CENTRE}. "
                "A negative input produced a non-negative offset -- the arithmetic "
                "went unsigned. See BUILD-PLAN Trap 3."
            )
        for s in (8, 33, 64, 100, 127):
            got = await read(dut, s, amp)
            assert got > CENTRE, (
                f"sine={s} amp={AMP_NAME[amp]}: pos={got}, expected above {CENTRE}"
            )


@cocotb.test()
async def test_zero_crossing_asymmetry_is_bounded(dut):
    """Documents an accepted asymmetry rather than testing for a bug.

    Verilog's signed >>> floors, so +1 >>> 2 is 0 while -1 >>> 2 is -1. Near the
    wave's zero crossings the positive and negative deflections therefore differ
    slightly, and the direction of the bias is not uniform: at 25% and 50% the
    output leans low, but at 75% the expression is s - (s>>>2), where the floor
    lands on the *subtracted* term and the bias flips high.

    What matters is only that it is bounded at 1 LSB -- 0.4% of full scale, well
    under the SG90's own 5-10 us deadband, so it is invisible in the mechanism.

    Pinned so nobody later "fixes" it into a rounding network that costs gates and
    changes nothing observable.
    """
    for amp in range(4):
        up = await read(dut, 1, amp) - CENTRE
        down = CENTRE - await read(dut, -1, amp)
        assert abs(up - down) <= 1, (
            f"amp={AMP_NAME[amp]}: deflection is +{up} up vs {down} down at sine=+/-1. "
            f"Asymmetry of {abs(up - down)} LSB exceeds the 1 LSB that plain "
            "floor-rounding produces -- check the shift is arithmetic and applied once."
        )


@cocotb.test()
async def test_amplitude_ordering(dut):
    """Larger amp setting means larger deflection from centre, at every sine value."""
    for s in (-127, -90, -40, -5, 5, 40, 90, 127):
        devs = [abs(await read(dut, s, amp) - CENTRE) for amp in range(4)]
        for a in range(3):
            assert devs[a] <= devs[a + 1], (
                f"sine={s}: amp {AMP_NAME[a]} deflects {devs[a]} but "
                f"amp {AMP_NAME[a + 1]} deflects {devs[a + 1]} -- larger amplitude "
                "must never deflect less"
            )
        assert devs[3] > devs[0], (
            f"sine={s}: 100% ({devs[3]}) should deflect more than 25% ({devs[0]})"
        )


@cocotb.test()
async def test_monotonic_in_sine(dut):
    """pos rises monotonically with sine at fixed amplitude.

    A shift implemented with the wrong sign extension typically produces a sawtooth
    here rather than a clean ramp.
    """
    for amp in range(4):
        prev = None
        for s in range(-127, 128):
            got = await read(dut, s, amp)
            if prev is not None:
                assert got >= prev, (
                    f"amp={AMP_NAME[amp]}: pos dropped from {prev} to {got} as sine "
                    f"went to {s}. Output must be non-decreasing in sine."
                )
            prev = got


@cocotb.test()
async def test_full_scale_endpoints(dut):
    """The extremes land where the spec says, per amplitude."""
    expected = {
        0: (reference(-127, 0), reference(127, 0)),
        1: (reference(-127, 1), reference(127, 1)),
        2: (reference(-127, 2), reference(127, 2)),
        3: (1, 255),
    }
    for amp in range(4):
        lo = await read(dut, -127, amp)
        hi = await read(dut, 127, amp)
        want_lo, want_hi = expected[amp]
        assert (lo, hi) == (want_lo, want_hi), (
            f"amp={AMP_NAME[amp]}: extremes are ({lo}, {hi}), expected "
            f"({want_lo}, {want_hi})"
        )
