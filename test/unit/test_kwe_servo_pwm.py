# SPDX-License-Identifier: Apache-2.0
"""
Unit tests for kwe_servo_pwm  --  Kinematic Wave Engine stage 2.

Contract under test (PRD-v2 §6.1/§6.2, BUILD-PLAN §2):

    pos_r  <= pos_next            on slot_start
    servo_pwm = (tick_cnt < {1'b1, pos_r}) ? (8'd1 << slot) : 8'd0

    {1'b1, pos_r} is a CONCATENATION, not an addition -- it *is* 256 + pos.
    No adder belongs in this module.

    pulse width = 256 + pos ticks:
        pos =   0  ->  256 ticks  ->  0.998 ms   (one end of servo travel)
        pos = 128  ->  384 ticks  ->  1.498 ms   (centre)
        pos = 255  ->  511 ticks  ->  1.993 ms   (other end)

    Exactly one channel is ever active -- channel N owns slot N.

Why this test does not instantiate kwe_timebase: stage 1 already proved a tick is
exactly 3900 ns. Stage 2 only has to prove pulse width *in ticks*; composing the two
gives real time. That also lets the testbench drive tick_cnt straight to a boundary
rather than counting to it, so sweeping all 256 positions costs ~1300 clocks instead
of 164,000.

Unlike stage 1, this file measures nothing in simulation time -- widths come from the
tick model -- so a settling Timer is safe here.

Run:  source ~/oss-cad-suite/environment && make MOD=kwe_servo_pwm
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge, Timer

CLK_NS = 100
TICKS_PER_SLOT = 640
SLOTS = 8

PULSE_BASE = 256  # ticks at pos = 0
TICK_NS = 3900  # proven in stage 1, used only for human-readable messages

# Distinct position per channel, so a one-slot registration skew is unmissable:
# each channel's width would come out as its neighbour's.
POS_PATTERN = [10, 40, 70, 100, 130, 160, 190, 220]


def u(sig):
    return int(sig.value)


def expected_width(pos):
    return PULSE_BASE + pos


def ms(ticks):
    return ticks * TICK_NS / 1e6


_clock_task = None


def ensure_clock(dut):
    """One live clock, always. See test_kwe_timebase for why this matters."""
    global _clock_task
    if _clock_task is not None and not _clock_task.done():
        _clock_task.cancel()
    _clock_task = cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())


async def settle(dut):
    """Let combinational outputs resolve after driving inputs."""
    await Timer(1, unit="ns")


async def reset(dut):
    ensure_clock(dut)
    dut.rst_n.value = 0
    dut.slot_start.value = 0
    dut.pos_next.value = 0
    dut.tick_cnt.value = 0
    dut.slot.value = 0
    await FallingEdge(dut.clk)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1
    await FallingEdge(dut.clk)


async def latch_pos(dut, pos, slot):
    """Latch `pos` as the active position for `slot`.

    Reproduces what kwe_timebase + the wave datapath do at a slot boundary:
    slot_start is asserted during the last tick of the previous slot, with
    pos_next already holding the value computed for the slot about to begin.
    """
    await FallingEdge(dut.clk)
    dut.slot.value = (slot - 1) % SLOTS
    dut.tick_cnt.value = TICKS_PER_SLOT - 1
    dut.slot_start.value = 1
    dut.pos_next.value = pos

    await RisingEdge(dut.clk)  # pos_r <= pos_next here

    await FallingEdge(dut.clk)
    dut.slot_start.value = 0
    dut.slot.value = slot


async def probe(dut, tick):
    """Read servo_pwm with tick_cnt forced to `tick`."""
    await FallingEdge(dut.clk)
    dut.tick_cnt.value = tick
    await settle(dut)
    return u(dut.servo_pwm)


async def measure_width(dut, pos, slot):
    """Width in ticks, found from the two boundary samples rather than by counting."""
    await latch_pos(dut, pos, slot)
    w = expected_width(pos)

    last_high = await probe(dut, w - 1)
    first_low = await probe(dut, w)
    return last_high, first_low


@cocotb.test(timeout_time=50, timeout_unit="us")
async def test_reset_outputs_low(dut):
    """servo_pwm is all-zero throughout reset, whatever tick_cnt does.

    This cannot be satisfied by pos_r's reset value alone: pos_r = 0 still encodes
    a 256-tick (1 ms) pulse. servo_pwm must be explicitly gated by rst_n.
    Servos must be free to be positioned by hand during mechanical assembly.
    """
    ensure_clock(dut)
    dut.rst_n.value = 0
    dut.slot_start.value = 0
    dut.pos_next.value = 200
    dut.slot.value = 3

    for tick in (0, 1, 100, 255, 256, 400, 511, 639):
        await FallingEdge(dut.clk)
        dut.tick_cnt.value = tick
        await settle(dut)
        assert u(dut.servo_pwm) == 0, (
            f"servo_pwm = 0b{u(dut.servo_pwm):08b} at tick {tick} during reset, "
            "expected 0. Outputs must be gated by rst_n -- pos_r = 0 is still a 1 ms pulse."
        )


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_pulse_width_encoding(dut):
    """Pulse width is exactly 256 + pos ticks, at the spec's reference points."""
    await reset(dut)

    for pos in (0, 1, 64, 128, 192, 254, 255):
        last_high, first_low = await measure_width(dut, pos, slot=0)
        w = expected_width(pos)

        assert last_high != 0, (
            f"pos={pos}: servo_pwm low at tick {w - 1}, expected still high. "
            f"Width must be {w} ticks ({ms(w):.3f} ms). "
            "Check the comparator is '<' against {1'b1, pos} as a 9-bit value."
        )
        assert first_low == 0, (
            f"pos={pos}: servo_pwm still high at tick {w}, expected low. "
            f"Width must be exactly {w} ticks ({ms(w):.3f} ms), not longer. "
            "An off-by-one here is '<=' where the spec says '<'."
        )


@cocotb.test(timeout_time=500, timeout_unit="us")
async def test_all_positions_within_servo_limits(dut):
    """Sweep all 256 positions: width always in [256, 511] ticks.

    This is the servo-safety test. A pulse outside 1.0-2.0 ms drives an SG90 past
    its mechanical stops, which stalls it against the linkage. It is the one property
    that must hold for every reachable input, so it is checked exhaustively.
    """
    await reset(dut)

    for pos in range(256):
        last_high, first_low = await measure_width(dut, pos, slot=0)
        w = expected_width(pos)

        assert PULSE_BASE <= w <= 511, f"internal: bad expectation {w}"
        assert last_high != 0 and first_low == 0, (
            f"pos={pos}: width is not {w} ticks ({ms(w):.3f} ms). "
            f"high@{w - 1}={last_high != 0}, high@{w}={first_low != 0}. "
            f"Every position must land in [{PULSE_BASE}, 511] ticks "
            f"({ms(PULSE_BASE):.3f}-{ms(511):.3f} ms) or the servo hits its stops."
        )


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_channel_routing(dut):
    """Channel N drives bit N of servo_pwm and no other bit."""
    await reset(dut)

    for slot in range(SLOTS):
        await latch_pos(dut, 128, slot)
        out = await probe(dut, 0)
        assert out == (1 << slot), (
            f"slot {slot}: servo_pwm = 0b{out:08b}, expected 0b{1 << slot:08b}. "
            "Channel N must drive bit N -- check the 3-to-8 decoder."
        )


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_position_is_latched_not_transparent(dut):
    """pos_next changing mid-slot must not alter the pulse in progress.

    If pos_next feeds the comparator directly instead of through a register, the
    servo sees its pulse width change partway through, and every channel ends up
    driven by the *next* channel's position. That is BUILD-PLAN Trap 1 seen from
    inside this module.
    """
    await reset(dut)

    await latch_pos(dut, 100, slot=2)

    # Yank pos_next to an extreme without a slot_start; the latched 100 must hold.
    await FallingEdge(dut.clk)
    dut.pos_next.value = 255
    await settle(dut)

    w = expected_width(100)
    assert await probe(dut, w - 1) != 0, (
        f"pulse ended early after pos_next changed mid-slot; "
        f"latched pos=100 means {w} ticks regardless of pos_next"
    )
    assert await probe(dut, w) == 0, (
        f"pulse extended past {w} ticks after pos_next changed mid-slot -- "
        "pos_next is reaching the comparator without being registered. "
        "Latch it on slot_start."
    )


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_pulse_is_contiguous_from_tick_zero(dut):
    """The pulse starts at tick 0 and is one contiguous run, with no second edge."""
    await reset(dut)

    pos = 64
    w = expected_width(pos)
    await latch_pos(dut, pos, slot=5)

    for tick in (0, 1, 2, w // 2, w - 2, w - 1):
        assert await probe(dut, tick) == (1 << 5), (
            f"pos={pos}: expected high at tick {tick} (pulse spans 0..{w - 1})"
        )

    for tick in (w, w + 1, 600, TICKS_PER_SLOT - 1):
        assert await probe(dut, tick) == 0, (
            f"pos={pos}: expected low at tick {tick} (pulse ends at {w - 1}). "
            "The servo must see one clean pulse per slot, never two."
        )


@cocotb.test(timeout_time=20, timeout_unit="ms")
async def test_one_hot_across_a_full_frame(dut):
    """Across a full 8-slot frame, never more than one channel high at once.

    Staggering is the whole point of the architecture (PRD §6.2): it is what removes
    seven comparators and flattens the 5 V supply transient. If two channels are ever
    high together, the stagger is broken.

    Each channel carries a distinct position, so this also catches a registration
    skew -- a channel showing its neighbour's width.
    """
    await reset(dut)

    tick = TICKS_PER_SLOT - 1
    slot = SLOTS - 1
    widths = {}

    # One priming iteration for the (slot 7, tick 639) boundary that latches
    # channel 0's position, then exactly eight full slots. Overshooting wraps back
    # into slot 0 and inflates its tally.
    for _ in range(1 + SLOTS * TICKS_PER_SLOT):
        nxt = (slot + 1) % SLOTS

        await FallingEdge(dut.clk)
        dut.tick_cnt.value = tick
        dut.slot.value = slot
        dut.slot_start.value = 1 if tick == TICKS_PER_SLOT - 1 else 0
        dut.pos_next.value = POS_PATTERN[nxt]
        await settle(dut)

        out = u(dut.servo_pwm)
        assert out == 0 or (out & (out - 1)) == 0, (
            f"servo_pwm = 0b{out:08b} at slot {slot} tick {tick}: "
            "more than one channel high. Slots must not overlap."
        )
        if out:
            assert out == (1 << slot), (
                f"slot {slot} tick {tick}: servo_pwm = 0b{out:08b}, "
                f"expected bit {slot}"
            )
            widths[slot] = widths.get(slot, 0) + 1

        tick += 1
        if tick == TICKS_PER_SLOT:
            tick = 0
            slot = (slot + 1) % SLOTS

    for slot in range(SLOTS):
        want = expected_width(POS_PATTERN[slot])
        got = widths.get(slot, 0)
        assert got == want, (
            f"channel {slot} was high for {got} ticks, expected {want} "
            f"(pos={POS_PATTERN[slot]}). "
            f"Got channel {(slot - 1) % SLOTS}'s width instead? That is a "
            "registration skew -- see BUILD-PLAN Trap 1."
        )
