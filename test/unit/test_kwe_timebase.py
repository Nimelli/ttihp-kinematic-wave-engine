# SPDX-License-Identifier: Apache-2.0
"""
Unit tests for kwe_timebase  --  Kinematic Wave Engine stage 1.

Contract under test (PRD-v2 §6.1, BUILD-PLAN §2):

    clk 10.000 MHz  (100 ns period)
      /39 prescaler         -> tick_en @ 256.41 kHz  (3900 ns)
      640 ticks             -> one slot              (2.496 ms)
      8 slots               -> one frame             (19.968 ms = 50.08 Hz)

    phase_tick fires once per frame, mid-slot-7 at tick_cnt == 600.
    Tick 600 is after the longest possible servo pulse (511) and before the slot
    ends (639), so advancing the phase there cannot disturb any latched position.
    See BUILD-PLAN Trap 2 -- this placement is load-bearing, not arbitrary.

These tests assert the *contract*, not your internal naming beyond the port list.

Every test carries a sim-time timeout, so a module that never asserts tick_en fails
in seconds instead of hanging. If you see "Timeout", the signal the test was waiting
on is not toggling at all -- that is the first thing to go and look at.

Run:  source ~/oss-cad-suite/environment && make MOD=kwe_timebase
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, ReadOnly, RisingEdge
from cocotb.utils import get_sim_time

CLK_NS = 100  # 10.000 MHz
PRESCALE = 39
TICKS_PER_SLOT = 640
SLOTS_PER_FRAME = 8
PHASE_TICK_AT = 600

TICK_NS = PRESCALE * CLK_NS  # 3_900
SLOT_NS = TICKS_PER_SLOT * TICK_NS  # 2_496_000
FRAME_NS = SLOTS_PER_FRAME * SLOT_NS  # 19_968_000


def u(sig):
    return int(sig.value)


def now_ns():
    """Sim time in ns, as an exact integer.

    get_sim_time(unit="ns") returns a float and will hand back
    3899.9999999999964 for what is exactly 3900 ns. Every deadline in this
    design is a whole number of ns, so rounding is lossless here and removes
    a whole class of spurious failures.
    """
    return round(get_sim_time(unit="ns"))


# All tests in a run share one simulation and one DUT. Each test needs a live clock,
# but two live Clock tasks driving dut.clk at once produce extra edges and every
# timing measurement becomes meaningless -- the symptom is tests that pass when run
# alone and fail in the suite. So: cancel any clock still running, then start a fresh
# one. Do not "start it only once" either; the task does not reliably survive the test
# that created it, and the next test then hangs with no clock at all.
_clock_task = None


def ensure_clock(dut):
    global _clock_task
    if _clock_task is not None and not _clock_task.done():
        _clock_task.cancel()
    _clock_task = cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())


async def start(dut):
    """Ensure the clock is running, then pulse reset."""
    ensure_clock(dut)
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 5)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1
    await FallingEdge(dut.clk)


async def next_tick(dut):
    """Advance to the assertion of tick_en, with all values settled.

    Waiting on tick_en rather than clk means one Python wake-up per 39 clocks
    instead of one per clock -- the difference between tests that run in seconds
    and tests that run in minutes. tick_en is safe to edge-trigger on: it decodes
    a single register that updates atomically.

    The ReadOnly() puts us at the end of the current time step, after every delta
    has settled, so combinational outputs (slot_start, phase_tick) read correctly.
    It does not advance simulation time, so period measurements stay exact -- which
    is why this is used instead of a Timer.
    """
    await RisingEdge(dut.tick_en)
    await ReadOnly()


async def wait_phase_tick(dut):
    """Advance to the tick on which phase_tick is asserted.

    Deliberately polls the *level* rather than using RisingEdge(dut.phase_tick).
    phase_tick is combinational, and it glitches high for zero simulation time at
    the clock edge where tick_cnt reaches 600 while tick_en is still falling. The
    glitch is harmless in hardware -- phase_tick is only ever sampled at a clock
    edge -- but cocotb reports it as a genuine edge, which made this test measure
    a 3800 ns "frame".

    Rule of thumb: never edge-trigger on a combinational signal.
    """
    while True:
        await next_tick(dut)
        if u(dut.phase_tick):
            return


@cocotb.test(timeout_time=20, timeout_unit="us")
async def test_reset_state(dut):
    """Counters sit at zero while rst_n is low."""
    ensure_clock(dut)
    dut.rst_n.value = 0

    for _ in range(20):
        await FallingEdge(dut.clk)
        assert u(dut.tick_cnt) == 0, f"tick_cnt={u(dut.tick_cnt)} during reset, expected 0"
        assert u(dut.slot) == 0, f"slot={u(dut.slot)} during reset, expected 0"


@cocotb.test(timeout_time=1, timeout_unit="ms")
async def test_tick_period_and_width(dut):
    """tick_en is a 1-clock pulse every 39 clocks."""
    await start(dut)

    await RisingEdge(dut.tick_en)
    t_prev = now_ns()

    for i in range(20):
        await FallingEdge(dut.tick_en)
        width = now_ns() - t_prev
        assert width == CLK_NS, (
            f"tick_en pulse {i} is {width} ns wide, expected {CLK_NS} ns "
            "(must be exactly one clock)"
        )

        await RisingEdge(dut.tick_en)
        t_now = now_ns()
        period = t_now - t_prev
        assert period == TICK_NS, (
            f"tick_en period {i} is {period} ns, expected {TICK_NS} ns "
            f"(prescaler must divide by {PRESCALE})"
        )
        t_prev = t_now


@cocotb.test(timeout_time=12, timeout_unit="ms")
async def test_tick_cnt_walks_one_slot(dut):
    """tick_cnt counts 0..639 in order, once per slot, then wraps."""
    await start(dut)

    while True:
        await next_tick(dut)
        if u(dut.tick_cnt) == 0:
            break

    seen = []
    for _ in range(TICKS_PER_SLOT + 1):
        seen.append(u(dut.tick_cnt))
        await next_tick(dut)

    expected = list(range(TICKS_PER_SLOT)) + [0]
    if seen != expected:
        bad = next(i for i, (a, b) in enumerate(zip(seen, expected)) if a != b)
        raise AssertionError(
            f"tick_cnt sequence wrong at index {bad}: got {seen[bad]}, expected {expected[bad]}. "
            f"Sequence must be 0..{TICKS_PER_SLOT - 1} then wrap to 0."
        )


@cocotb.test(timeout_time=35, timeout_unit="ms")
async def test_slot_sequence_and_duration(dut):
    """slot advances 0..7, each slot is exactly 640 ticks, slot_nxt leads by one."""
    await start(dut)

    while True:
        await next_tick(dut)
        if u(dut.tick_cnt) == 0:
            break

    for _ in range(SLOTS_PER_FRAME + 1):
        slot = u(dut.slot)
        nxt = u(dut.slot_nxt)
        assert nxt == (slot + 1) % SLOTS_PER_FRAME, (
            f"slot_nxt={nxt} while slot={slot}; expected {(slot + 1) % SLOTS_PER_FRAME}. "
            "slot_nxt drives the wave datapath -- see BUILD-PLAN Trap 1"
        )

        t0 = now_ns()
        starts = 0
        for _ in range(TICKS_PER_SLOT):
            if u(dut.slot_start):
                starts += 1
            await next_tick(dut)

        assert starts == 1, (
            f"slot_start asserted {starts} times during slot {slot}, expected exactly 1"
        )

        elapsed = now_ns() - t0
        assert elapsed == SLOT_NS, f"slot {slot} lasted {elapsed} ns, expected {SLOT_NS} ns"

        new_slot = u(dut.slot)
        assert new_slot == (slot + 1) % SLOTS_PER_FRAME, (
            f"after slot {slot} the counter went to {new_slot}, "
            f"expected {(slot + 1) % SLOTS_PER_FRAME}"
        )


@cocotb.test(timeout_time=70, timeout_unit="ms")
async def test_frame_period(dut):
    """Successive phase_tick pulses are exactly one 50.08 Hz frame apart."""
    await start(dut)

    await wait_phase_tick(dut)
    t0 = now_ns()
    await wait_phase_tick(dut)
    t1 = now_ns()

    period = t1 - t0
    assert period == FRAME_NS, (
        f"frame period is {period} ns ({1e9 / period:.3f} Hz), "
        f"expected {FRAME_NS} ns (50.08 Hz). Servos need 40-60 Hz."
    )


@cocotb.test(timeout_time=90, timeout_unit="ms")
async def test_phase_tick_placement(dut):
    """phase_tick fires in slot 7 at tick 600, one clock wide, qualified by tick_en."""
    await start(dut)

    for _ in range(2):
        await wait_phase_tick(dut)

        assert u(dut.slot) == SLOTS_PER_FRAME - 1, (
            f"phase_tick fired in slot {u(dut.slot)}, expected slot {SLOTS_PER_FRAME - 1}"
        )
        assert u(dut.tick_cnt) == PHASE_TICK_AT, (
            f"phase_tick fired at tick_cnt={u(dut.tick_cnt)}, expected {PHASE_TICK_AT}. "
            "Must land after the longest pulse (511) and before the slot ends (639) "
            "-- see BUILD-PLAN Trap 2"
        )
        assert u(dut.tick_en), "phase_tick must be qualified by tick_en"

        # One tick wide: it must be gone by the following tick. Checked as a level
        # rather than with FallingEdge, for the glitch reason in wait_phase_tick().
        await next_tick(dut)
        assert not u(dut.phase_tick), (
            "phase_tick still asserted on the following tick; it must be "
            "exactly one clock wide (qualify it with tick_en)"
        )


@cocotb.test(timeout_time=90, timeout_unit="ms")
async def test_phase_tick_once_per_frame(dut):
    """Exactly one phase_tick per frame, counted over two full frames."""
    await start(dut)

    await wait_phase_tick(dut)

    hits = 0
    for _ in range(2 * SLOTS_PER_FRAME * TICKS_PER_SLOT):
        await next_tick(dut)
        if u(dut.phase_tick):
            hits += 1

    assert hits == 2, f"{hits} phase_tick pulses over 2 frames, expected exactly 2"
