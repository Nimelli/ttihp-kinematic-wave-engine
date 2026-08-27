"""
Unit tests for spis_phy.v

Run:  source ~/oss-cad-suite/environment && make MOD=spis_phy

byte_valid is a 1-cycle strobe that fires on the same edge as the 8th
spi_clk_rising. Sampling it after the stimulus helper returns always misses it,
so every test watches it with ByteMonitor instead.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, ReadOnly, RisingEdge

CLK_NS = 100  # 10.000 MHz system clock


def u(sig):
    return int(sig.value)


""" start fresh clock for each tests"""
_clock_task = None


def ensure_clock(dut):
    global _clock_task
    if _clock_task is not None and not _clock_task.done():
        _clock_task.cancel()
    _clock_task = cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())


class ByteMonitor:
    """Records rx_data on every clock edge where byte_valid is high.

    One entry per high cycle, so a strobe stretched to 2 cycles shows up as a
    duplicate entry and fails the length check.
    """

    def __init__(self, dut):
        self.dut = dut
        self.bytes = []
        self._task = cocotb.start_soon(self._run())

    async def _run(self):
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            if u(self.dut.byte_valid):
                self.bytes.append(u(self.dut.rx_data))

    def stop(self):
        self._task.cancel()


async def reset_dut(dut):
    """Resets the PHY and initializes all input ports."""
    ensure_clock(dut)
    dut.rst_n.value = 0
    dut.spi_mosi_sync.value = 0
    dut.spi_cs_sync.value = 1
    dut.spi_clk_rising.value = 0
    dut.spi_clk_falling.value = 0
    dut.spi_cs_falling.value = 0

    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)


async def spi_assert_cs(dut):
    """Simulates CS assertion (spi_cs_falling 1-cycle strobe)."""
    await FallingEdge(dut.clk)
    dut.spi_cs_sync.value = 0
    dut.spi_cs_falling.value = 1

    await FallingEdge(dut.clk)
    dut.spi_cs_falling.value = 0
    await ClockCycles(dut.clk, 2)


async def spi_deassert_cs(dut):
    """Simulates CS deassertion (CS returns high)."""
    await FallingEdge(dut.clk)
    dut.spi_cs_sync.value = 1
    await ClockCycles(dut.clk, 2)


async def spi_send_bit(dut, bit):
    """One SCK period: MOSI setup, 1-cycle rising strobe, 1-cycle falling strobe.

    Inputs change on the falling edge of clk so each strobe is high across
    exactly one rising edge, matching what spis_synchro emits.
    """
    await FallingEdge(dut.clk)
    dut.spi_mosi_sync.value = bit

    await FallingEdge(dut.clk)
    dut.spi_clk_rising.value = 1

    await FallingEdge(dut.clk)
    dut.spi_clk_rising.value = 0

    await FallingEdge(dut.clk)
    dut.spi_clk_falling.value = 1

    await FallingEdge(dut.clk)
    dut.spi_clk_falling.value = 0


async def spi_send_byte(dut, byte_val: int):
    """Transfers 1 byte MSB-first by toggling edge strobes."""
    for bit_idx in range(7, -1, -1):
        await spi_send_bit(dut, (byte_val >> bit_idx) & 1)


@cocotb.test(timeout_time=20, timeout_unit="us")
async def test_reset_state(dut):
    """Verify all outputs are cleared while rst_n is asserted."""
    ensure_clock(dut)
    dut.rst_n.value = 0
    dut.spi_mosi_sync.value = 1
    dut.spi_cs_sync.value = 0
    dut.spi_clk_rising.value = 1
    dut.spi_clk_falling.value = 0
    dut.spi_cs_falling.value = 0

    for _ in range(10):
        await FallingEdge(dut.clk)
        assert u(dut.rx_data) == 0, f"rx_data={u(dut.rx_data):#04x} during reset, expected 0x00"
        assert u(dut.byte_valid) == 0, "byte_valid asserted during reset"


@cocotb.test(timeout_time=20, timeout_unit="us")
async def test_single_byte_rx(dut):
    """Verify 8-bit shift deserialization into rx_data and 1-cycle byte_valid strobe."""
    await reset_dut(dut)
    mon = ByteMonitor(dut)
    test_byte = 0xA5  # 1010_0101

    await spi_assert_cs(dut)
    await spi_send_byte(dut, test_byte)
    await spi_deassert_cs(dut)
    mon.stop()

    assert len(mon.bytes) == 1, (
        f"byte_valid pulsed {len(mon.bytes)} cycles, expected exactly 1"
    )
    assert mon.bytes[0] == test_byte, (
        f"rx_data = 0x{mon.bytes[0]:02X} at byte_valid, expected 0x{test_byte:02X}"
    )


@cocotb.test(timeout_time=40, timeout_unit="us")
async def test_multi_byte_burst_rx(dut):
    """Verify consecutive multi-byte transfers while CS remains low."""
    await reset_dut(dut)
    mon = ByteMonitor(dut)
    test_bytes = [0x55, 0xAA, 0xBE, 0xEF]

    await spi_assert_cs(dut)
    for byte_val in test_bytes:
        await spi_send_byte(dut, byte_val)
    await spi_deassert_cs(dut)
    mon.stop()

    assert mon.bytes == test_bytes, (
        "burst mismatch: got ["
        + ", ".join(f"0x{b:02X}" for b in mon.bytes)
        + "], expected ["
        + ", ".join(f"0x{b:02X}" for b in test_bytes)
        + "]"
    )


@cocotb.test(timeout_time=20, timeout_unit="us")
async def test_cs_deassert_resets_bit_counter(dut):
    """Verify mid-byte CS deassertion resets the bit counter and suppresses byte_valid."""
    await reset_dut(dut)
    mon = ByteMonitor(dut)

    # Send only 5 bits (partial transfer)
    await spi_assert_cs(dut)
    for _ in range(5):
        await spi_send_bit(dut, 1)

    # Prematurely abort frame via CS high
    await spi_deassert_cs(dut)

    # Send 3 stray clock pulses while CS is high
    for _ in range(3):
        await spi_send_bit(dut, 1)

    assert mon.bytes == [], "byte_valid pulsed despite frame being aborted by CS high"

    # A fresh frame must decode from bit 0 again: the aborted 5 bits and the 3
    # stray pulses must not count toward it.
    await spi_assert_cs(dut)
    await spi_send_byte(dut, 0x3C)
    await spi_deassert_cs(dut)
    mon.stop()

    assert mon.bytes == [0x3C], (
        "frame after abort mismatch: got ["
        + ", ".join(f"0x{b:02X}" for b in mon.bytes)
        + "], expected [0x3C]"
    )
