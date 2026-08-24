# SPDX-License-Identifier: Apache-2.0
"""
Unit tests for kwe_angle_map  --  Kinematic Wave Engine stage 4.

Contract under test (PRD-v2 §6.3, §6.7):

    m     = mirror ? (slot < 4 ? slot : 7 - slot) : slot      // 0,1,2,3,3,2,1,0
    delta = spread ? 8 : 16
    angle = (phase + m * delta) mod 128

This is the module that decides the *shape of the sculpture*:

    mirror=0  one travelling trough  -> a single ball crosses the track
    mirror=1  two troughs, symmetric -> two balls converge, collide, return (P2)

    spread=0  delta 16 -> one full wavelength across the 8 rods
    spread=1  delta  8 -> half a wavelength; the array is one broad tilting ramp

The mirror costs two XOR gates: `slot ^ {3{slot[2]}}` gives 0,1,2,3,3,2,1,0 directly.
Writing it as a comparison works too -- the tests check behaviour, not spelling.

Purely combinational, no clock. 128 phases x 8 slots x 2 spread x 2 mirror = 4096
inputs, so these tests are EXHAUSTIVE.

Run:  source ~/oss-cad-suite/environment && make MOD=kwe_angle_map
"""

import cocotb
from cocotb.triggers import Timer

TURN = 128
SLOTS = 8


def mirrored_index(slot):
    """0,1,2,3,3,2,1,0 -- the array folded about its centre."""
    return slot if slot < 4 else 7 - slot


def reference(phase, slot, spread, mirror):
    m = mirrored_index(slot) if mirror else slot
    delta = 8 if spread else 16
    return (phase + m * delta) % TURN


async def read(dut, phase, slot, spread, mirror):
    dut.phase.value = phase
    dut.slot.value = slot
    dut.spread.value = spread
    dut.mirror.value = mirror
    await Timer(1, unit="ns")
    return int(dut.angle.value)


@cocotb.test()
async def test_exhaustive(dut):
    """All 4096 input combinations match the spec."""
    wrong = []
    for mirror in (0, 1):
        for spread in (0, 1):
            for slot in range(SLOTS):
                for phase in range(TURN):
                    got = await read(dut, phase, slot, spread, mirror)
                    want = reference(phase, slot, spread, mirror)
                    if got != want:
                        wrong.append((phase, slot, spread, mirror, got, want))

    if wrong:
        head = "\n".join(
            f"  phase={p:3d} slot={s} spread={sp} mirror={mi}: got {g:3d}, expected {w:3d}"
            for p, s, sp, mi, g, w in wrong[:12]
        )
        more = f"\n  ... and {len(wrong) - 12} more" if len(wrong) > 12 else ""
        raise AssertionError(f"{len(wrong)} of 4096 combinations wrong:\n{head}{more}")


@cocotb.test()
async def test_output_always_in_range(dut):
    """angle is a 7-bit position around the circle; it must wrap, never saturate."""
    for mirror in (0, 1):
        for spread in (0, 1):
            for slot in range(SLOTS):
                for phase in (0, 1, 63, 64, 100, 126, 127):
                    got = await read(dut, phase, slot, spread, mirror)
                    assert 0 <= got < TURN, (
                        f"angle={got} out of range for phase={phase} slot={slot}. "
                        "The sum must wrap modulo 128 -- just truncate to 7 bits, "
                        "do not clamp."
                    )


@cocotb.test()
async def test_wrap_is_modulo_not_saturating(dut):
    """phase near the top of the range must wrap past zero, not stick at 127."""
    # slot 7, spread 0, mirror 0 -> offset 7*16 = 112. phase 100 -> 212 mod 128 = 84.
    got = await read(dut, 100, 7, 0, 0)
    assert got == 84, (
        f"phase=100 slot=7 spread=0 gave angle={got}, expected 84 (212 mod 128). "
        "A saturating adder here would put a discontinuity in the wave."
    )


@cocotb.test()
async def test_spread_step_between_adjacent_rods(dut):
    """Adjacent rods differ by delta: 16 when spread=0, 8 when spread=1."""
    for spread, delta in ((0, 16), (1, 8)):
        for phase in (0, 37, 100):
            for slot in range(SLOTS - 1):
                a = await read(dut, phase, slot, spread, 0)
                b = await read(dut, phase, slot + 1, spread, 0)
                step = (b - a) % TURN
                assert step == delta, (
                    f"spread={spread}: rods {slot}->{slot + 1} differ by {step}, "
                    f"expected {delta}. spread=0 must put one full wavelength across "
                    "the array, spread=1 half a wavelength."
                )


@cocotb.test()
async def test_no_mirror_spans_the_array(dut):
    """With mirror=0 the offsets march monotonically 0,16,32,...,112 (spread=0)."""
    for slot in range(SLOTS):
        got = await read(dut, 0, slot, 0, 0)
        assert got == slot * 16, (
            f"mirror=0 phase=0 slot={slot}: angle={got}, expected {slot * 16}. "
            "Offsets must increase linearly with slot -- that is what makes the "
            "trough travel."
        )


@cocotb.test()
async def test_mirror_folds_the_array(dut):
    """With mirror=1, rod N and rod 7-N share a phase. This IS the two-ball mode.

    It is the whole of P2: fold the index and the array becomes symmetric about its
    centre, so a trough travelling inward on the left is mirrored on the right.
    """
    for spread in (0, 1):
        for phase in (0, 20, 64, 111):
            for slot in range(SLOTS):
                a = await read(dut, phase, slot, spread, 1)
                b = await read(dut, phase, SLOTS - 1 - slot, spread, 1)
                assert a == b, (
                    f"mirror=1 phase={phase} spread={spread}: rod {slot} gives {a} "
                    f"but rod {SLOTS - 1 - slot} gives {b}; they must match. "
                    "The mirrored index is 0,1,2,3,3,2,1,0."
                )


@cocotb.test()
async def test_mirror_index_sequence(dut):
    """The folded offsets are 0,1,2,3,3,2,1,0 scaled by delta."""
    for spread, delta in ((0, 16), (1, 8)):
        expected = [mirrored_index(s) * delta for s in range(SLOTS)]
        got = [await read(dut, 0, s, spread, 1) for s in range(SLOTS)]
        assert got == expected, (
            f"mirror=1 spread={spread} phase=0 gave {got}, expected {expected} "
            "(index folded to 0,1,2,3,3,2,1,0)"
        )


@cocotb.test()
async def test_phase_shifts_every_rod_equally(dut):
    """Advancing phase moves all 8 rods by the same amount -- that is the wave.

    If phase does not apply uniformly the array does not carry a coherent wave,
    it just wobbles.
    """
    for mirror in (0, 1):
        for spread in (0, 1):
            base = [await read(dut, 0, s, spread, mirror) for s in range(SLOTS)]
            for step in (1, 5, 40, 127):
                moved = [await read(dut, step, s, spread, mirror) for s in range(SLOTS)]
                for s in range(SLOTS):
                    assert moved[s] == (base[s] + step) % TURN, (
                        f"mirror={mirror} spread={spread} rod {s}: phase +{step} gave "
                        f"{moved[s]}, expected {(base[s] + step) % TURN}"
                    )
