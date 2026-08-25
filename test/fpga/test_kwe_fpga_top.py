# SPDX-License-Identifier: Apache-2.0
"""
Tests for the Cmod A7 FPGA wrapper -- fpga/kwe_fpga_top.v.

This verifies the TEST HARNESS, not the tapeout. The point is to prove the
pulse monitor, the ASCII formatter and the UART are correct *before* any time
is spent in Vivado: on the board there is no scope and no servos, so if the
reported numbers were wrong there would be no way to tell.

What is covered:
    - the wrapper reproduces the pulse contract at the pins
    - kwe_pulse_monitor measures the width exactly, in 10 MHz clocks
    - the report line is well-formed and carries the right values
    - single-character serial commands change what the DUT is told to do
    - BTN1 steps the speed, through the debouncer
    - the wave actually moves once the 500 ms startup hold expires

What is NOT covered: the MMCM. Xilinx primitives need the unisim libraries
and glbl, so KWE_SIM bypasses it and drives sysclk at 10 MHz directly. The
MMCM is confirmed on hardware instead -- if it were wrong, every measured
width would be scaled by 12/10 and nothing would match.

Run:  source ~/oss-cad-suite/environment && make
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, RisingEdge, Timer

CLK_NS = 100  # 10 MHz -- the MMCM output, not the 12 MHz board oscillator

# UART: kwe_fpga_top uses CLKS_PER_BIT = 87, so a bit is exactly 8700 ns.
# The testbench uses the generated rate rather than the nominal 115200 so it
# decodes what the RTL actually emits.
CLKS_PER_BIT = 87
BIT_NS = CLKS_PER_BIT * CLK_NS

# Pulse widths in 10 MHz clocks: (256 + pos) ticks x 39 clocks per tick.
TICK_CLKS = 39
CENTRE_CLKS = (256 + 128) * TICK_CLKS  # 14976 = 1497.6 us
MIN_CLKS = (256 + 0) * TICK_CLKS  # 9984  =  998.4 us
MAX_CLKS = (256 + 255) * TICK_CLKS  # 19929 = 1992.9 us

FRAME_NS = 8 * 640 * TICK_CLKS * CLK_NS  # 19_968_000 ns

# kwe_phase_gen holds the array flat for 25 frames after reset (PRD P0.3).
HOLD_FRAMES = 25

# Defaults baked into kwe_fpga_top.
DEF_SPEED, DEF_AMP, DEF_SPREAD, DEF_MIRROR, DEF_REVERSE = 8, 3, 0, 0, 0

# Fast enough that a dozen frames (~240 ms) show obvious movement.
TARGET_SPEED = 12


def ctrl_word(speed, amp, spread, mirror, reverse):
    """Field 0 of the report line, packed as kwe_fpga_top packs it."""
    return (spread << 9) | (amp << 7) | (speed << 3) | (mirror << 2) | reverse


async def start(dut):
    """Bring the wrapper up. It self-resets out of configuration.

    Note what this does NOT do: reset the control registers. They live in the
    power-on domain on purpose, so that resetting the DUT does not discard the
    settings you dialled in -- physical switches would not spring back either.
    In a single simulation every test after the first therefore inherits the
    control state the previous one left behind, which is why the tests below
    assert on the *change* they cause rather than on absolute values.
    """
    dut.btn.value = 0
    dut.uart_txd_in.value = 1  # serial line idles high
    cocotb.start_soon(Clock(dut.sysclk, CLK_NS, unit="ns").start())
    # por_sync then rst_sync are three flops each; ten clocks is ample.
    await Timer(10 * CLK_NS, "ns")


async def uart_get_byte(dut):
    """Decode one byte from the FPGA's transmit pin, sampling mid-bit."""
    await FallingEdge(dut.uart_rxd_out)  # start bit
    await Timer(BIT_NS + BIT_NS // 2, "ns")  # centre of bit 0
    value = 0
    for i in range(8):
        value |= int(dut.uart_rxd_out.value) << i  # LSB first
        await Timer(BIT_NS, "ns")
    return value


async def uart_sync(dut):
    """Wait for an inter-line idle gap, so the next falling edge is a start bit.

    Needed after any jump in time that could land mid-transmission: the
    decoder would otherwise latch onto a data-bit edge and return garbage.
    Consecutive uart_get_line() calls do not need this -- a line ends with the
    stop bit of '\\n' and the line then idles for ~15 ms before the next frame.
    """
    while True:
        while dut.uart_rxd_out.value == 0:
            await RisingEdge(dut.sysclk)
        # A byte is 10 bit times. Staying high for 11 means no byte is in
        # flight, so the transmitter really is between lines.
        for _ in range(11):
            await Timer(BIT_NS, "ns")
            if dut.uart_rxd_out.value == 0:
                break
        else:
            return


async def uart_get_line(dut):
    """Collect bytes up to and including a newline, returned as text."""
    out = bytearray()
    while True:
        byte = await uart_get_byte(dut)
        out.append(byte)
        if byte == 0x0A:
            return out.decode("ascii", errors="replace").strip()
        assert len(out) < 200, f"no newline after {len(out)} bytes: {out!r}"


async def uart_send_byte(dut, byte):
    """Drive one byte into the FPGA's receive pin."""
    dut.uart_txd_in.value = 0  # start bit
    await Timer(BIT_NS, "ns")
    for i in range(8):
        dut.uart_txd_in.value = (byte >> i) & 1
        await Timer(BIT_NS, "ns")
    dut.uart_txd_in.value = 1  # stop bit, then idle
    await Timer(2 * BIT_NS, "ns")


def unpack(ctrl):
    """Split a control word back into the five settings."""
    return {
        "spread": (ctrl >> 9) & 0x1,
        "amp": (ctrl >> 7) & 0x3,
        "speed": (ctrl >> 3) & 0xF,
        "mirror": (ctrl >> 2) & 0x1,
        "reverse": ctrl & 0x3,
    }


def parse(line):
    """'W cccc wwww ... ' -> (ctrl, [w0..w7]). Raises on a malformed line."""
    parts = line.split()
    assert parts and parts[0] == "W", f"bad header in {line!r}"
    assert len(parts) == 10, f"expected 9 fields, got {len(parts) - 1} in {line!r}"
    values = [int(p, 16) for p in parts[1:]]
    return values[0], values[1:]


async def measure_frame(dut):
    """Time all 8 pulses at the pins directly, in clocks. Returns 8 widths."""
    widths = []
    for channel in range(8):
        # Wait for this channel, and only this channel, to go high.
        while not (int(dut.servo.value) >> channel) & 1:
            await RisingEdge(dut.sysclk)
        assert int(dut.servo.value) == (1 << channel), (
            f"outputs not one-hot at channel {channel}: "
            f"{int(dut.servo.value):#010b}"
        )
        start_ns = cocotb.utils.get_sim_time("ns")
        while (int(dut.servo.value) >> channel) & 1:
            await RisingEdge(dut.sysclk)
        widths.append(round((cocotb.utils.get_sim_time("ns") - start_ns) / CLK_NS))
    return widths


@cocotb.test()
async def test_power_on_defaults(dut):
    """The DUT comes up on the defaults baked into kwe_fpga_top.

    Must run first: the control registers deliberately survive a DUT reset
    (see start()), so this is the only point in the simulation where the
    power-on state can still be observed. cocotb runs tests in definition
    order, so keep this at the top of the file.
    """
    await start(dut)
    await uart_get_line(dut)

    ctrl, _ = parse(await uart_get_line(dut))
    expected = ctrl_word(DEF_SPEED, DEF_AMP, DEF_SPREAD, DEF_MIRROR, DEF_REVERSE)
    assert ctrl == expected, (
        f"control word {ctrl:#06x} at power-on, expected {expected:#06x} "
        f"(speed={DEF_SPEED}, amp={DEF_AMP}, spread={DEF_SPREAD}, "
        f"mirror={DEF_MIRROR}, reverse={DEF_REVERSE})"
    )


@cocotb.test()
async def test_hold_widths(dut):
    """During the startup hold every channel sits dead centre, in slot order."""
    await start(dut)

    await measure_frame(dut)  # frame 0 starts mid-pulse; discard it

    for frame in range(2):
        widths = await measure_frame(dut)
        for channel, width in enumerate(widths):
            assert width == CENTRE_CLKS, (
                f"frame {frame} channel {channel}: {width} clocks "
                f"({width * CLK_NS / 1000:.1f} us), expected {CENTRE_CLKS} "
                f"({CENTRE_CLKS * CLK_NS / 1000:.1f} us). During the 500 ms "
                f"startup hold every rod must be commanded flat."
            )


@cocotb.test()
async def test_uart_report(dut):
    """The report line is well-formed and agrees with the pins."""
    await start(dut)

    # The first line describes frame 0, whose channel 0 pulse was already
    # under way at reset. Skip it and check steady-state lines.
    await uart_get_line(dut)

    first_ctrl = None
    for _ in range(2):
        ctrl, widths = parse(await uart_get_line(dut))

        # Nothing is touching the controls here, so the field must not drift.
        if first_ctrl is None:
            first_ctrl = ctrl
        assert ctrl == first_ctrl, (
            f"control word changed from {first_ctrl:#06x} to {ctrl:#06x} "
            f"with no command sent"
        )

        for channel, width in enumerate(widths):
            assert width == CENTRE_CLKS, (
                f"reported channel {channel} = {width} clocks, "
                f"expected {CENTRE_CLKS}"
            )


@cocotb.test()
async def test_serial_command(dut):
    """Single-character commands reach the DUT's control pins."""
    await start(dut)
    await uart_get_line(dut)

    ctrl, _ = parse(await uart_get_line(dut))
    state = unpack(ctrl)

    # Each command touches exactly one setting and leaves the rest alone --
    # a wrong bit position in the packing would show up as a second field
    # moving. Wrap-around is part of the contract: the registers are 4 and 2
    # bits wide and are meant to roll over.
    for char, field, apply in [
        ("s", "speed", lambda v: (v + 1) & 0xF),
        ("S", "speed", lambda v: (v - 1) & 0xF),
        ("a", "amp", lambda v: (v + 1) & 0x3),
        ("A", "amp", lambda v: (v - 1) & 0x3),
        ("p", "spread", lambda v: v ^ 1),
        ("m", "mirror", lambda v: v ^ 1),
        ("r", "reverse", lambda v: (v + 1) & 0x3),
        ("R", "reverse", lambda v: (v - 1) & 0x3),
        ("?", None, None),  # unknown command must be ignored
    ]:
        expected = dict(state)
        if field is not None:
            expected[field] = apply(state[field])

        await uart_send_byte(dut, ord(char))
        # uart_sync returns in the gap between lines, so the next line was
        # snapshotted after the command had already been applied.
        await uart_sync(dut)
        ctrl, _ = parse(await uart_get_line(dut))
        got = unpack(ctrl)

        assert got == expected, (
            f"after command {char!r}: got {got}, expected {expected}"
        )
        state = got


@cocotb.test()
async def test_button_speed(dut):
    """BTN1 steps the speed once per press, and bounce does not double-count.

    DEBOUNCE_CLKS is overridden to 50 by the Makefile, so a "press" here is
    microseconds rather than the 10 ms the hardware requires.
    """
    await start(dut)
    await uart_get_line(dut)

    ctrl, _ = parse(await uart_get_line(dut))
    before = unpack(ctrl)

    settle = 200 * CLK_NS  # comfortably longer than DEBOUNCE_CLKS = 50

    # A bouncing press: several fast transitions, then a stable high.
    for _ in range(4):
        dut.btn.value = 0b10
        await Timer(3 * CLK_NS, "ns")
        dut.btn.value = 0b00
        await Timer(3 * CLK_NS, "ns")
    dut.btn.value = 0b10
    await Timer(settle, "ns")
    dut.btn.value = 0b00
    await Timer(settle, "ns")

    await uart_sync(dut)
    ctrl, _ = parse(await uart_get_line(dut))
    after = unpack(ctrl)

    expected = dict(before)
    expected["speed"] = (before["speed"] + 1) & 0xF
    assert after == expected, (
        f"one bouncing press took the controls from {before} to {after}, "
        f"expected {expected} -- the debouncer counted bounce as extra presses"
    )


@cocotb.test()
async def test_wave_moves(dut):
    """After the 500 ms hold the rods separate, and stay inside servo travel.

    Slow: simulates ~600 ms, about 6 million clocks. Skip with
    COCOTB_TEST_FILTER if iterating on something else.
    """
    await start(dut)

    # Do not inherit whatever speed the previous test left behind: at
    # speed_sel = 0 the wave period is 20.1 s and 12 frames of samples would
    # show no movement at all. Dial in a fast setting explicitly.
    await uart_get_line(dut)
    ctrl, _ = parse(await uart_get_line(dut))
    for _ in range((TARGET_SPEED - unpack(ctrl)["speed"]) % 16):
        await uart_send_byte(dut, ord("s"))
    await uart_sync(dut)
    ctrl, _ = parse(await uart_get_line(dut))
    assert unpack(ctrl)["speed"] == TARGET_SPEED, (
        f"could not set speed to {TARGET_SPEED}, got {unpack(ctrl)['speed']}"
    )

    # Run out the startup hold, plus a couple of frames of margin. This lands
    # at an arbitrary point, very likely mid-line, so resynchronise before
    # decoding anything.
    await Timer((HOLD_FRAMES + 2) * FRAME_NS, "ns")
    await uart_sync(dut)

    seen = []
    for _ in range(12):
        _, widths = parse(await uart_get_line(dut))
        seen.append(widths)

        for channel, width in enumerate(widths):
            assert MIN_CLKS <= width <= MAX_CLKS, (
                f"channel {channel} = {width} clocks "
                f"({width * CLK_NS / 1000:.1f} us) is outside the "
                f"{MIN_CLKS}..{MAX_CLKS} clock envelope -- a real SG90 would "
                f"be driven past its stop"
            )

    # A travelling wave means the rods are not all at the same angle, and that
    # any given rod does not sit still.
    assert any(len(set(w)) > 1 for w in seen), (
        "every channel reported the same width in all 12 frames -- the array "
        "is still flat, so the wave never started"
    )
    channel0 = [w[0] for w in seen]
    assert len(set(channel0)) > 1, (
        f"channel 0 never moved across 12 frames: {channel0}"
    )
