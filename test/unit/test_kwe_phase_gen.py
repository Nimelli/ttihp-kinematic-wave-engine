# SPDX-License-Identifier: Apache-2.0
"""
Unit tests for kwe_phase_gen  --  Kinematic Wave Engine stage 6b.

Contract under test (PRD-v2 §6.3, §6.5, P0.3):

    16-bit accumulator, advanced by +/- inc on every phase_tick.
    phase = acc[15:9]                        -- 7-bit angle, one full turn
    direction flips after `reverse_sel` complete accumulator wraps
    reverse_sel:  00 -> 2   01 -> 3   10 -> 4   11 -> 6   wave cycles
    hold is asserted for the first 25 phase_ticks after reset (500 ms centre hold)
    and freezes the accumulator while it is high.

This is the last sequential module in P0 and the only one with real state beyond
counters. Three separate behaviours live here -- accumulate, reverse, startup hold --
and they interact, so they are tested independently first and then together.

The tests drive phase_tick directly rather than waiting real 19.968 ms frames, so a
full 16-speed sweep costs milliseconds of simulation instead of minutes. That is the
payoff for kwe_timebase and kwe_phase_gen being separate modules.

Run:  source ~/oss-cad-suite/environment && make MOD=kwe_phase_gen
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge

CLK_NS = 100
ACC_BITS = 16
ACC_SPAN = 1 << ACC_BITS
TURN = 128
HOLD_TICKS = 25
FRAME_S = 19.968e-3

SPEED_TABLE = [65, 87, 119, 156, 208, 278, 374, 503,
               654, 872, 1190, 1577, 2111, 2784, 3739, 5033]
REVERSE_CHOICES = [2, 3, 4, 6]


def u(sig):
    return int(sig.value)


_clock_task = None


def ensure_clock(dut):
    global _clock_task
    if _clock_task is not None and not _clock_task.done():
        _clock_task.cancel()
    _clock_task = cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())


async def reset(dut, speed=8, reverse=0):
    ensure_clock(dut)
    dut.phase_tick.value = 0
    dut.speed_sel.value = speed
    dut.reverse_sel.value = reverse
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 4)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1
    await FallingEdge(dut.clk)


async def tick(dut):
    """One phase_tick pulse, one clock wide, as kwe_timebase would deliver it."""
    dut.phase_tick.value = 1
    await RisingEdge(dut.clk)
    await FallingEdge(dut.clk)
    dut.phase_tick.value = 0
    await FallingEdge(dut.clk)


async def release_hold(dut):
    """Consume the startup hold so later tests start from free-running motion."""
    for _ in range(HOLD_TICKS + 1):
        await tick(dut)


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_reset_state(dut):
    """After reset: phase at 0, hold asserted."""
    await reset(dut)
    assert u(dut.phase) == 0, f"phase={u(dut.phase)} after reset, expected 0"
    assert u(dut.hold) == 1, (
        "hold must be asserted out of reset -- the array holds centre for 500 ms "
        "so it physically assembles flat before the wave starts (P0.3)"
    )


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def test_hold_lasts_25_ticks_and_freezes_phase(dut):
    """hold covers exactly 25 phase_ticks, and phase does not move during it."""
    await reset(dut, speed=15)          # fastest, so any leak is obvious

    for i in range(HOLD_TICKS):
        assert u(dut.hold) == 1, (
            f"hold dropped after {i} ticks, expected it to cover {HOLD_TICKS} "
            f"(25 frames = 500 ms)"
        )
        assert u(dut.phase) == 0, (
            f"phase moved to {u(dut.phase)} during the hold at tick {i}. "
            "The accumulator must be frozen while hold is high, or the array "
            "starts waving before the servos have reached centre."
        )
        await tick(dut)

    assert u(dut.hold) == 0, (
        f"hold still asserted after {HOLD_TICKS} ticks, expected it to release"
    )


@cocotb.test(timeout_time=5, timeout_unit="ms")
async def test_phase_advances_after_hold(dut):
    """Once hold releases, phase actually moves."""
    await reset(dut, speed=15)
    await release_hold(dut)

    start = u(dut.phase)
    for _ in range(5):
        await tick(dut)
    assert u(dut.phase) != start, (
        f"phase stuck at {start} after the hold released -- the accumulator "
        "is not advancing"
    )


@cocotb.test(timeout_time=400, timeout_unit="ms")
async def test_wave_period_matches_speed_table(dut):
    """For every speed setting, the accumulator cycles at the rate PRD §6.6 says.

    Measured by counting wraps rather than looking for phase == 0: at the fast
    settings phase steps by ~10 per tick and never lands on an exact value.
    """
    for sel, inc in enumerate(SPEED_TABLE):
        await reset(dut, speed=sel, reverse=3)   # 6 cycles: no reversal while measuring
        await release_hold(dut)

        expected_frames = ACC_SPAN / inc

        # Measure wrap-to-wrap, not over the whole window: the run neither starts
        # nor ends on a wrap boundary, and over only 2 wraps those partial cycles
        # dominate. Timing from the first wrap to the last is exact to +/-1 tick.
        wrap_at = []
        cap = int(expected_frames * 5) + 50
        t = 0
        prev = u(dut.phase)
        while len(wrap_at) < 4 and t < cap:
            await tick(dut)
            t += 1
            now = u(dut.phase)
            if now < prev:
                wrap_at.append(t)
            prev = now

        assert len(wrap_at) >= 2, (
            f"sel={sel} (inc={inc}): only {len(wrap_at)} wraps in {cap} ticks; "
            f"expected a full cycle every {expected_frames:.0f} ticks"
        )
        measured = (wrap_at[-1] - wrap_at[0]) / (len(wrap_at) - 1)
        err = abs(measured - expected_frames) / expected_frames
        assert err < 0.03, (
            f"sel={sel}: measured {measured:.1f} ticks per wave cycle, expected "
            f"{expected_frames:.1f} (T={ACC_SPAN / (inc / FRAME_S):.2f} s). "
            "Check the increment reaching the accumulator."
        )


@cocotb.test(timeout_time=200, timeout_unit="ms")
async def test_reverse_after_selected_cycles(dut):
    """Direction flips after exactly `reverse_sel` accumulator wraps.

    PRD §6.5: 2 cycles is the default. Mechanical simulation showed the original
    hardcoded 4 stalls the ball at the end of the track for whole wave periods.
    """
    for rsel, n_cycles in enumerate(REVERSE_CHOICES):
        await reset(dut, speed=14, reverse=rsel)   # fast: short cycles
        await release_hold(dut)

        frames_per_cycle = ACC_SPAN / SPEED_TABLE[14]

        # Direction is read from the modular step: forward gives a small positive
        # delta, backward gives one just under a full turn. That classifies
        # correctly across wraps without special-casing them.
        wraps = 0
        prev = u(dut.phase)
        going_fwd = True
        reversed_after = None
        for _ in range(int(frames_per_cycle * (n_cycles + 3)) + 20):
            await tick(dut)
            now = u(dut.phase)
            step = (now - prev) % TURN
            if step != 0:
                fwd = step < TURN // 2
                if fwd != going_fwd:
                    reversed_after = wraps
                    break
                if fwd and now < prev:
                    wraps += 1
                elif not fwd and now > prev:
                    wraps += 1
                going_fwd = fwd
            prev = now

        assert reversed_after is not None, (
            f"reverse_sel={rsel} ({n_cycles} cycles): direction never reversed "
            f"within {n_cycles + 3} cycles"
        )
        assert abs(reversed_after - n_cycles) <= 1, (
            f"reverse_sel={rsel}: reversed after {reversed_after} cycles, "
            f"expected {n_cycles}. Reset default is 00 = 2 cycles (PRD §6.5)."
        )


@cocotb.test(timeout_time=50, timeout_unit="ms")
async def test_phase_stays_in_range(dut):
    """phase is a 7-bit position on a circle: always 0..127, wrapping not clamping."""
    await reset(dut, speed=12, reverse=0)
    await release_hold(dut)
    for _ in range(3000):
        await tick(dut)
        p = u(dut.phase)
        assert 0 <= p < TURN, f"phase={p} out of range 0..127"


@cocotb.test(timeout_time=50, timeout_unit="ms")
async def test_reversal_actually_runs_backwards(dut):
    """After reversing, phase decreases -- the wave travels the other way.

    A reversal that stops the wave, or restarts it forwards, would leave the ball
    parked at one end forever.
    """
    await reset(dut, speed=15, reverse=0)   # fastest + earliest reversal
    await release_hold(dut)

    saw_up = saw_down = False
    prev = u(dut.phase)
    for _ in range(2000):
        await tick(dut)
        now = u(dut.phase)
        d = now - prev
        if 0 < d < TURN // 2:
            saw_up = True
        elif -TURN // 2 < d < 0:
            saw_down = True
        prev = now
        if saw_up and saw_down:
            break

    assert saw_up, "phase never increased -- forward direction is not working"
    assert saw_down, (
        "phase never decreased -- the reversal is not making the wave travel "
        "backwards. Negate the increment, do not just stop."
    )
