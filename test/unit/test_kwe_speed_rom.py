# SPDX-License-Identifier: Apache-2.0
"""
Unit tests for kwe_speed_rom  --  Kinematic Wave Engine stage 6a.

Contract under test (PRD-v2 §6.6): a 16-entry table of 13-bit phase increments.

    wave period T = 2^16 / (inc / 19.968 ms)

    sel:  0    1    2    3    4    5    6    7
    inc: 65   87  119  156  208  278  374  503
    T:  20.1 15.0 11.0  8.4  6.3  4.7  3.5  2.6  s

    sel:  8    9   10   11   12   13   14   15
    inc: 654  872 1190 1577 2111 2784 3739 5033
    T:   2.0  1.5  1.1 0.83 0.62 0.47 0.35 0.26 s

Spacing is ~1.35x per step. Both halves of the range are load-bearing: speeds 0-8 are
the smooth regime where the servos track the sine faithfully, 10-15 drive the ball
harder but only because the servos are slew-limited and the sine is clipped toward a
square wave. Mechanical simulation validated the range as frozen -- see sim/README.md.

Purely combinational, no clock. 16 inputs, so this is EXHAUSTIVE.

Run:  source ~/oss-cad-suite/environment && make MOD=kwe_speed_rom
"""

import cocotb
from cocotb.triggers import Timer

FRAME_S = 19.968e-3
ACC_SPAN = 1 << 16

TABLE = [65, 87, 119, 156, 208, 278, 374, 503,
         654, 872, 1190, 1577, 2111, 2784, 3739, 5033]


def period_s(inc):
    return ACC_SPAN / (inc / FRAME_S)


async def read(dut, sel):
    dut.sel.value = sel
    await Timer(1, unit="ns")
    return int(dut.inc.value)


@cocotb.test()
async def test_all_entries_exhaustive(dut):
    """Every one of the 16 increments matches PRD §6.6."""
    wrong = []
    for sel in range(16):
        got = await read(dut, sel)
        if got != TABLE[sel]:
            wrong.append((sel, got, TABLE[sel]))

    assert not wrong, "speed table wrong:\n" + "\n".join(
        f"  sel={s:2d}: inc={g} (T={period_s(g):.2f} s), "
        f"expected {w} (T={period_s(w):.2f} s)"
        for s, g, w in wrong
    )


@cocotb.test()
async def test_monotonic_increasing(dut):
    """Higher sel means a larger increment, i.e. a faster wave.

    If two entries are transposed the DIP switch stops behaving like a speed
    control, which is confusing on a device whose only tuning is DIP switches.
    """
    prev = 0
    for sel in range(16):
        got = await read(dut, sel)
        assert got > prev, (
            f"sel={sel}: inc={got} is not greater than the previous entry {prev}. "
            "The table must increase monotonically -- check for transposed values."
        )
        prev = got


@cocotb.test()
async def test_period_range_covers_the_useful_band(dut):
    """Slowest and fastest entries bracket the range the simulation needs."""
    slowest = period_s(await read(dut, 0))
    fastest = period_s(await read(dut, 15))

    assert 19.0 < slowest < 21.0, (
        f"sel=0 gives T={slowest:.2f} s, expected ~20.1 s (the slow end of the range)"
    )
    assert 0.24 < fastest < 0.28, (
        f"sel=15 gives T={fastest:.2f} s, expected ~0.26 s (the fast end)"
    )


@cocotb.test()
async def test_fits_in_thirteen_bits(dut):
    """No entry may overflow the 13-bit increment path into kwe_phase_gen."""
    for sel in range(16):
        got = await read(dut, sel)
        assert 0 < got < (1 << 13), (
            f"sel={sel}: inc={got} does not fit in 13 bits (max 8191)"
        )


@cocotb.test()
async def test_spacing_is_roughly_geometric(dut):
    """Consecutive entries are ~1.35x apart -- no gaps, no near-duplicates.

    A gap would leave a wave period unreachable; a near-duplicate wastes a DIP
    setting. Both matter because these 16 values are the entire tuning range
    available without SPI.
    """
    incs = [await read(dut, sel) for sel in range(16)]
    for i in range(15):
        ratio = incs[i + 1] / incs[i]
        assert 1.2 < ratio < 1.55, (
            f"sel {i} -> {i + 1}: ratio {ratio:.2f} "
            f"({incs[i]} -> {incs[i + 1]}), expected ~1.35"
        )
