"""
Unit tests for spis_synchro.v

Run:  source ~/oss-cad-suite/environment && make MOD=spis_synchro
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, ReadOnly, RisingEdge
from cocotb.utils import get_sim_time

CLK_NS = 100  # 10.000 MHz

def u(sig):
    return int(sig.value)

""" start fresh clock for each tests"""
_clock_task = None
def ensure_clock(dut):
    global _clock_task
    if _clock_task is not None and not _clock_task.done():
        _clock_task.cancel()
    _clock_task = cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())

@cocotb.test(timeout_time=20, timeout_unit="us")
async def test_reset_state(dut):
    ensure_clock(dut)
    dut.rst_n.value = 0

    for _ in range(20):
        await FallingEdge(dut.clk)
        assert u(dut.spi_clk_sync) == 0, f"spi_clk_sync={u(dut.spi_clk_sync)} during reset, expected 0"
        assert u(dut.spi_cs_sync) == 1, f"spi_cs_sync={u(dut.spi_cs_sync)} during reset, expected 1"
        assert u(dut.spi_mosi_sync) == 0, f"spi_mosi_sync={u(dut.spi_mosi_sync)} during reset, expected 0"
        assert u(dut.spi_clk_rising) == 0, f"spi_clk_rising={u(dut.spi_clk_rising)} during reset, expected 0"
        assert u(dut.spi_clk_falling) == 0, f"spi_clk_falling={u(dut.spi_clk_falling)} during reset, expected 0"
        assert u(dut.spi_cs_falling) == 0, f"spi_cs_falling={u(dut.spi_cs_falling)} during reset, expected 0"




@cocotb.test(timeout_time=20, timeout_unit="us")
async def test_single_pulse_and_cdc_latency(dut):
    """
    Verify that an incoming asynchronous SCK pulse generates 
    EXACTLY 1-cycle strobes with a 2-cycle double-flop latency.
    """
    ensure_clock(dut)
    
    # Release reset
    dut.rst_n.value = 1
    dut.spi_clk_async.value = 0
    dut.spi_cs_async.value = 1
    dut.spi_mosi_async.value = 0
    await ClockCycles(dut.clk, 5)

    # 1. Drive SCK High asynchronously mid-cycle
    await FallingEdge(dut.clk)
    dut.spi_clk_async.value = 1

    # Cycle +1 after edge: FF1 samples 1, FF2 is 0 -> Sync output still 0, no strobe
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert u(dut.spi_clk_sync) == 0
    assert u(dut.spi_clk_rising) == 0

    # Cycle +2 after edge: FF2 samples 1, FF3 is 0 -> Sync becomes 1, rising strobe MUST pulse
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert u(dut.spi_clk_sync) == 1
    assert u(dut.spi_clk_rising) == 1, "spi_clk_rising should pulse HIGH for exactly 1 cycle"

    # Cycle +3 after edge: FF3 samples 1 -> Strobe MUST drop back to 0
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert u(dut.spi_clk_sync) == 1
    assert u(dut.spi_clk_rising) == 0, "spi_clk_rising failed to drop to 0 on next cycle"

    # 2. Drive SCK Low asynchronously
    await FallingEdge(dut.clk)
    dut.spi_clk_async.value = 0

    # Cycle +1 after falling edge
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert u(dut.spi_clk_falling) == 0

    # Cycle +2 after falling edge: Falling strobe MUST pulse
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert u(dut.spi_clk_sync) == 0
    assert u(dut.spi_clk_falling) == 1, "spi_clk_falling should pulse HIGH for exactly 1 cycle"

    # Cycle +3: Strobe drops
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert u(dut.spi_clk_falling) == 0

@cocotb.test(timeout_time=20, timeout_unit="us")
async def test_cs_falling_edge_detection(dut):
    """
    Verify Chip Select (CS_N) assertion generates a single-cycle cs_falling strobe.
    """
    ensure_clock(dut)
    dut.rst_n.value = 1
    dut.spi_cs_async.value = 1
    await ClockCycles(dut.clk, 5)

    # Assert CS (Active Low)
    await FallingEdge(dut.clk)
    dut.spi_cs_async.value = 0

    # Wait for CDC latency (2 cycles)
    await ClockCycles(dut.clk, 2)
    await ReadOnly()
    assert u(dut.spi_cs_sync) == 0
    assert u(dut.spi_cs_falling) == 1, "spi_cs_falling did not pulse upon CS assertion"

    # Next cycle check
    await RisingEdge(dut.clk)
    await ReadOnly()
    assert u(dut.spi_cs_falling) == 0, "spi_cs_falling stayed high for more than 1 cycle"


@cocotb.test(timeout_time=20, timeout_unit="us")
async def test_mosi_synchronization_alignment(dut):
    """
    Verify that MOSI data aligns with SCK sampling edges after CDC propagation.
    """
    ensure_clock(dut)
    dut.rst_n.value = 1
    dut.spi_clk_async.value = 0
    dut.spi_cs_async.value = 0
    dut.spi_mosi_async.value = 0
    await ClockCycles(dut.clk, 5)

    # Set MOSI = 1 and SCK = 1 simultaneously
    await FallingEdge(dut.clk)
    dut.spi_mosi_async.value = 1
    dut.spi_clk_async.value = 1

    # Both MOSI and SCK rising strobe should become valid on the same clock cycle
    await ClockCycles(dut.clk, 2)
    await ReadOnly()
    assert u(dut.spi_mosi_sync) == 1, "spi_mosi_sync was not updated correctly"
    assert u(dut.spi_clk_rising) == 1, "spi_clk_rising did not align with MOSI update"