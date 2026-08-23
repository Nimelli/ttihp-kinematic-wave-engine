# SPDX-License-Identifier: Apache-2.0
"""
Unit tests for kwe_sine_lut  --  Kinematic Wave Engine stage 3a.

Contract under test (PRD-v2 §6.4):

    LUT[i] = round(127 * sin((i + 0.5) * pi/64)),  i = 0..31

Just the first quadrant of a sine, 32 entries, 8-bit unsigned. kwe_sine builds the
other three quadrants from it by mirroring the index and negating the output.

The half-step centring (the `+ 0.5`) is not cosmetic: it is what makes the mirror
exact, so no zero entry and no 33rd entry are needed.

Purely combinational, so there is no clock in this file. 32 possible inputs means
this test is EXHAUSTIVE -- once green, this module is proven, not sampled.

Run:  source ~/oss-cad-suite/environment && make MOD=kwe_sine_lut
"""

import math

import cocotb
from cocotb.triggers import Timer

N = 32
AMPLITUDE = 127


def reference(i):
    """The PRD §6.4 table, as a formula rather than copied digits."""
    return round(AMPLITUDE * math.sin((i + 0.5) * math.pi / 64))


# The table as written in PRD §6.4. Kept literal so that if someone edits the
# formula above, the mismatch is caught rather than silently redefining the spec.
TABLE = [
    3, 9, 16, 22, 28, 34, 40, 46,
    51, 57, 63, 68, 73, 78, 83, 88,
    92, 96, 100, 104, 107, 111, 113, 116,
    118, 121, 122, 124, 125, 126, 127, 127,
]


def test_reference_matches_prd_table():
    """Sanity check on the test itself, not the DUT."""
    assert [reference(i) for i in range(N)] == TABLE


async def read(dut, idx):
    dut.idx.value = idx
    await Timer(1, unit="ns")
    return int(dut.val.value)


@cocotb.test()
async def test_all_entries_exhaustive(dut):
    """Every one of the 32 entries matches the spec table."""
    test_reference_matches_prd_table()

    wrong = []
    for i in range(N):
        got = await read(dut, i)
        if got != TABLE[i]:
            wrong.append((i, got, TABLE[i]))

    assert not wrong, "LUT entries wrong:\n" + "\n".join(
        f"  LUT[{i}] = {got}, expected {want}" for i, got, want in wrong
    )


@cocotb.test()
async def test_monotonic_non_decreasing(dut):
    """A quarter sine only ever rises. Catches transposed or mistyped entries.

    An exhaustive value check already covers this, but a monotonicity failure
    points straight at *which* entry is out of order, which the value check does
    not make obvious when several digits are wrong.
    """
    prev = -1
    for i in range(N):
        got = await read(dut, i)
        assert got >= prev, (
            f"LUT[{i}] = {got} is less than LUT[{i - 1}] = {prev}. "
            "The first quadrant of a sine never decreases -- check for a "
            "transposed pair of entries."
        )
        prev = got


@cocotb.test()
async def test_endpoints_and_range(dut):
    """Endpoints are right and nothing exceeds the 8-bit amplitude."""
    first = await read(dut, 0)
    last = await read(dut, N - 1)

    assert first == 3, (
        f"LUT[0] = {first}, expected 3. A 0 here means the half-step centring "
        "(the +0.5) was dropped -- that breaks the quadrant mirror in kwe_sine."
    )
    assert last == AMPLITUDE, f"LUT[31] = {last}, expected {AMPLITUDE} (the peak)"

    for i in range(N):
        got = await read(dut, i)
        assert 0 <= got <= AMPLITUDE, (
            f"LUT[{i}] = {got}, outside 0..{AMPLITUDE}. Amplitude must stay at "
            f"{AMPLITUDE} so that 128 +/- sine fits in 8 bits without clipping."
        )


@cocotb.test()
async def test_max_step_between_entries(dut):
    """No two adjacent entries differ by more than 7.

    This is the number PRD §6.4 rests on: the LUT's worst-case quantisation step
    (7 LSB) sits just below the intrinsic step from updating at only 50 Hz (8 LSB
    at the nominal 2 s wave period). That is why 32 entries is the right size and
    a bigger table would buy nothing visible.
    """
    values = [await read(dut, i) for i in range(N)]
    steps = [values[i + 1] - values[i] for i in range(N - 1)]
    worst = max(steps)

    assert worst <= 7, (
        f"largest step between adjacent entries is {worst} LSB, expected <= 7. "
        "Above 8 the LUT becomes the dominant artefact rather than the 50 Hz "
        "frame rate, and the servos will visibly staircase."
    )
