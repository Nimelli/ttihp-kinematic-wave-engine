"""
Unit tests for spis_phy.v

Run:  source ~/oss-cad-suite/environment && make MOD=spis_phy

rx_byte_valid is a 1-cycle strobe that fires on the same edge as the 8th
spi_clk_rising. Sampling it after the stimulus helper returns always misses it,
so every test watches it with SpiMonitor instead.

The monitor samples after each rising clk edge, i.e. once the non-blocking
assignments of that edge have settled, so record k shows the state produced BY
edge k. The strobe inputs are driven on falling clk edges, so record k also
shows the strobe values that were present AT edge k.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, ReadOnly, RisingEdge

CLK_NS = 100  # 10.000 MHz system clock


def u(sig):
    return int(sig.value)


def bits_of(byte_val: int):
    """MSB-first bit list of a byte, the order it goes on the wire."""
    return [(byte_val >> i) & 1 for i in range(7, -1, -1)]


def bits_of_all(byte_list):
    return [b for byte_val in byte_list for b in bits_of(byte_val)]


def hexlist(byte_list):
    return "[" + ", ".join(f"0x{b:02X}" for b in byte_list) + "]"


""" start fresh clock for each tests"""
_clock_task = None


def ensure_clock(dut):
    global _clock_task
    if _clock_task is not None and not _clock_task.done():
        _clock_task.cancel()
    _clock_task = cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())


class SpiMonitor:
    """Records every port of interest on each clock edge.

    rx_bytes has one entry per cycle where rx_byte_valid is high, so a strobe
    stretched to 2 cycles shows up as a duplicate entry and fails the length
    check.

    format_trace() renders what a mode 0 master would have seen on the wire. The
    FST dump cannot show that -- this module is fed 1-cycle strobes, not SCK, so
    there is no clock in the waveform to read MOSI and MISO against. Read the
    trace in the log instead; the waveform is only useful for chasing an internal
    signal (tx_shift, bit_cnt) once the trace has told you which byte is wrong.
    """

    def __init__(self, dut):
        self.dut = dut
        self.records = []
        self._task = cocotb.start_soon(self._run())

    async def _run(self):
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            self.records.append(
                {
                    "rising": u(self.dut.spi_clk_rising),
                    "falling": u(self.dut.spi_clk_falling),
                    "cs_fall": u(self.dut.spi_cs_falling),
                    "cs": u(self.dut.spi_cs_sync),
                    "mosi": u(self.dut.spi_mosi_sync),
                    "miso": u(self.dut.spi_miso),
                    "oe": u(self.dut.spi_miso_oe),
                    "rx_valid": u(self.dut.rx_byte_valid),
                    "rx_data": u(self.dut.rx_data),
                    "tx_load": u(self.dut.tx_load),
                }
            )

    def stop(self):
        self._task.cancel()

    @property
    def rx_bytes(self):
        return [r["rx_data"] for r in self.records if r["rx_valid"]]

    @property
    def miso_at_rising(self):
        """What the master reads: MISO sampled on every rising SCK edge.

        Valid as a model of the real master only because spis_synchro bounds SCK
        to 2.5 MHz. The master samples at the PHYSICAL rising edge, 2-3 system
        clocks before the strobe seen here; no falling strobe can land in that
        window unless SCK exceeds its limit, so the two sample points agree.
        """
        return [r["miso"] for r in self.records if r["rising"]]

    @property
    def mosi_at_rising(self):
        """What the master drove, sampled where the PHY samples it."""
        return [r["mosi"] for r in self.records if r["rising"]]

    @property
    def tx_load_count(self):
        return sum(r["tx_load"] for r in self.records)

    def illegal_miso_changes(self):
        """Cycles where MISO moved when it had no right to.

        In mode 0 the master samples on the rising edge, so MISO may only take a
        new value on a falling edge or when CS falls (the MSB). While CS is high
        the only legal move is the release to 0.
        """
        bad = []
        for k in range(1, len(self.records)):
            prev, cur = self.records[k - 1], self.records[k]
            if cur["miso"] == prev["miso"]:
                continue
            if cur["falling"] or cur["cs_fall"]:
                continue
            if cur["cs"] and cur["miso"] == 0:
                continue  # bus released
            bad.append((k, prev["miso"], cur["miso"]))
        return bad

    def tx_load_misalignments(self):
        """Cycles where tx_load broke its contract with rx_byte_valid.

        tx_load must pulse on exactly two occasions: with cs_falling (the frame's
        first capture) and with rx_byte_valid (every byte boundary). If the two
        ever come apart, the link layer's idea of which byte it is answering is
        off by one, which is invisible in a stream comparison.
        """
        bad = []
        for k, r in enumerate(self.records):
            if r["rx_valid"] and not r["tx_load"]:
                bad.append((k, "rx_byte_valid without tx_load"))
            if r["tx_load"] and not (r["rx_valid"] or r["cs_fall"]):
                bad.append((k, "tx_load without rx_byte_valid or cs_falling"))
        return bad

    def format_trace(self):
        """Render the transfer as a mode 0 master would have seen it."""
        mosi, miso = self.mosi_at_rising, self.miso_at_rising
        rx = self.rx_bytes
        lines = ["", "        SCK edge   1 2 3 4 5 6 7 8"]
        for idx in range(0, len(mosi), 8):
            m_bits, s_bits = mosi[idx:idx + 8], miso[idx:idx + 8]
            n = idx // 8
            got = f" -> rx 0x{rx[n]:02X}" if n < len(rx) else " -> rx (incomplete)"
            if len(m_bits) == 8:
                shifted = int("".join(str(b) for b in m_bits), 2)
                got += "" if n < len(rx) and rx[n] == shifted else f"  MISMATCH, wire says 0x{shifted:02X}"
                tx = f" -> tx 0x{int(''.join(str(b) for b in s_bits), 2):02X}"
            else:
                tx = " -> tx (incomplete)"
            lines.append(f"byte {n}  mosi     " + " ".join(str(b) for b in m_bits) + got)
            lines.append(f"        miso     " + " ".join(str(b) for b in s_bits) + tx)
        if not mosi:
            lines.append("(no SCK edges captured)")
        return "\n".join(lines)

    def log_trace(self, dut, header=""):
        dut._log.info(header + self.format_trace())


async def reset_dut(dut):
    """Resets the PHY and initializes all input ports."""
    ensure_clock(dut)
    dut.rst_n.value = 0
    dut.spi_mosi_sync.value = 0
    dut.spi_cs_sync.value = 1
    dut.spi_clk_rising.value = 0
    dut.spi_clk_falling.value = 0
    dut.spi_cs_falling.value = 0
    dut.tx_data.value = 0

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


async def spi_send_bit(dut, bit, gap=0):
    """One SCK period: MOSI setup, 1-cycle rising strobe, 1-cycle falling strobe.

    Inputs change on the falling edge of clk so each strobe is high across
    exactly one rising edge, matching what spis_synchro emits. The default pace
    is 5 system clocks per bit, i.e. SCK = 2 MHz, near the 2.5 MHz ceiling;
    `gap` adds idle cycles after the falling strobe to model a slower master.
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

    if gap:
        await ClockCycles(dut.clk, gap)


async def spi_send_byte(dut, byte_val: int, gap=0):
    """Transfers 1 byte MSB-first by toggling edge strobes."""
    for bit_idx in range(7, -1, -1):
        await spi_send_bit(dut, (byte_val >> bit_idx) & 1, gap=gap)


# ----------------------------------------------------------------------
# RX
# ----------------------------------------------------------------------


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
    dut.tx_data.value = 0xFF

    for _ in range(10):
        await FallingEdge(dut.clk)
        assert u(dut.rx_data) == 0, f"rx_data={u(dut.rx_data):#04x} during reset, expected 0x00"
        assert u(dut.rx_byte_valid) == 0, "rx_byte_valid asserted during reset"
        assert u(dut.tx_load) == 0, "tx_load asserted during reset"
        assert u(dut.spi_miso) == 0, "spi_miso driven high during reset"
        assert u(dut.spi_miso_oe) == 0, (
            "spi_miso_oe asserted during reset -- the pad must stay Hi-Z until the "
            "PHY is out of reset, even if CS happens to be low"
        )


@cocotb.test(timeout_time=20, timeout_unit="us")
async def test_single_byte_rx(dut):
    """Verify 8-bit shift deserialization into rx_data and 1-cycle rx_byte_valid strobe."""
    await reset_dut(dut)
    mon = SpiMonitor(dut)
    test_byte = 0xA5  # 1010_0101

    await spi_assert_cs(dut)
    await spi_send_byte(dut, test_byte)
    await spi_deassert_cs(dut)
    mon.stop()

    assert len(mon.rx_bytes) == 1, (
        f"rx_byte_valid pulsed {len(mon.rx_bytes)} cycles, expected exactly 1"
    )
    assert mon.rx_bytes[0] == test_byte, (
        f"rx_data = 0x{mon.rx_bytes[0]:02X} at rx_byte_valid, expected 0x{test_byte:02X}"
    )


@cocotb.test(timeout_time=40, timeout_unit="us")
async def test_multi_byte_burst_rx(dut):
    """Verify consecutive multi-byte transfers while CS remains low."""
    await reset_dut(dut)
    mon = SpiMonitor(dut)
    test_bytes = [0x55, 0xAA, 0xBE, 0xEF]

    await spi_assert_cs(dut)
    for byte_val in test_bytes:
        await spi_send_byte(dut, byte_val)
    await spi_deassert_cs(dut)
    mon.stop()

    assert mon.rx_bytes == test_bytes, (
        f"burst mismatch: got {hexlist(mon.rx_bytes)}, expected {hexlist(test_bytes)}"
    )


@cocotb.test(timeout_time=20, timeout_unit="us")
async def test_cs_deassert_resets_bit_counter(dut):
    """Verify mid-byte CS deassertion resets the bit counter and suppresses rx_byte_valid."""
    await reset_dut(dut)
    mon = SpiMonitor(dut)

    # Send only 5 bits (partial transfer)
    await spi_assert_cs(dut)
    for _ in range(5):
        await spi_send_bit(dut, 1)

    # Prematurely abort frame via CS high
    await spi_deassert_cs(dut)

    # Send 3 stray clock pulses while CS is high
    for _ in range(3):
        await spi_send_bit(dut, 1)

    assert mon.rx_bytes == [], "rx_byte_valid pulsed despite frame being aborted by CS high"

    # A fresh frame must decode from bit 0 again: the aborted 5 bits and the 3
    # stray pulses must not count toward it.
    await spi_assert_cs(dut)
    await spi_send_byte(dut, 0x3C)
    await spi_deassert_cs(dut)
    mon.stop()

    assert mon.rx_bytes == [0x3C], (
        f"frame after abort mismatch: got {hexlist(mon.rx_bytes)}, expected [0x3C]"
    )


# ----------------------------------------------------------------------
# TX
# ----------------------------------------------------------------------


@cocotb.test(timeout_time=20, timeout_unit="us")
async def test_miso_msb_ready_before_first_edge(dut):
    """Verify the MSB is on the pad as soon as CS falls, before any SCK edge.

    CPHA=0 means the master samples bit 7 on the first rising edge, and there is
    no falling edge before it, so presenting the MSB on the first falling edge
    would lose it.
    """
    await reset_dut(dut)
    test_byte = 0xC3  # MSB 1, so a stuck-at-0 MISO fails here

    dut.tx_data.value = test_byte
    assert u(dut.spi_miso_oe) == 0, "spi_miso_oe asserted while CS is high"

    await spi_assert_cs(dut)

    assert u(dut.spi_miso_oe) == 1, "spi_miso_oe not asserted after CS fell"
    assert u(dut.spi_miso) == 1, (
        f"MISO = {u(dut.spi_miso)} after CS fell, expected bit 7 of "
        f"0x{test_byte:02X} = 1"
    )


@cocotb.test(timeout_time=20, timeout_unit="us")
async def test_single_byte_tx(dut):
    """Verify one byte is serialized MSB-first, sampled the way a mode 0 master does."""
    await reset_dut(dut)
    test_byte = 0x96  # 1001_0110, no run longer than 2

    dut.tx_data.value = test_byte
    mon = SpiMonitor(dut)

    await spi_assert_cs(dut)
    await spi_send_byte(dut, 0x00)
    await spi_deassert_cs(dut)
    mon.stop()
    mon.log_trace(dut, f"single byte TX, tx_data = 0x{test_byte:02X}")

    expected = bits_of(test_byte)
    assert mon.miso_at_rising == expected, (
        f"MISO at the 8 rising edges = {mon.miso_at_rising}, expected {expected} "
        f"(0x{test_byte:02X} MSB first)"
    )


@cocotb.test(timeout_time=40, timeout_unit="us")
async def test_miso_only_changes_on_falling_edge(dut):
    """Verify MISO never moves on a rising edge, including at byte boundaries.

    The byte boundary is the risky one: tx_data is captured on the 8th rising
    edge, the same edge the master uses to sample the last bit. If that capture
    reached the pad the last bit of every byte would be corrupted.
    """
    await reset_dut(dut)
    tx_bytes = [0xFF, 0x00, 0xFF]  # every boundary flips all 8 bits

    dut.tx_data.value = tx_bytes[0]
    mon = SpiMonitor(dut)

    await spi_assert_cs(dut)
    for next_byte in tx_bytes[1:] + [0x00]:
        # Presented during the current byte, captured at its boundary, shifted
        # out during the next one.
        dut.tx_data.value = next_byte
        await spi_send_byte(dut, 0x00)
    await spi_deassert_cs(dut)
    mon.stop()

    bad = mon.illegal_miso_changes()
    assert bad == [], (
        "MISO changed outside a falling edge at cycles "
        + ", ".join(f"{k} ({a}->{b})" for k, a, b in bad)
    )


@cocotb.test(timeout_time=60, timeout_unit="us")
async def test_multi_byte_tx(dut):
    """Verify tx_data captured at a byte boundary is what goes out next."""
    await reset_dut(dut)
    tx_bytes = [0x01, 0x80, 0xA5, 0x5A]

    dut.tx_data.value = tx_bytes[0]
    mon = SpiMonitor(dut)

    await spi_assert_cs(dut)
    for next_byte in tx_bytes[1:] + [0x00]:
        dut.tx_data.value = next_byte
        await spi_send_byte(dut, 0x00)
    await spi_deassert_cs(dut)
    mon.stop()
    mon.log_trace(dut, f"multi byte TX, tx_data sequence {hexlist(tx_bytes)}")

    expected = bits_of_all(tx_bytes)
    assert mon.miso_at_rising == expected, (
        f"MISO stream mismatch for {hexlist(tx_bytes)}:\n"
        f"  got      {mon.miso_at_rising}\n"
        f"  expected {expected}"
    )

    # One capture when CS fell, one per byte boundary.
    assert mon.tx_load_count == 1 + len(tx_bytes), (
        f"tx_load pulsed {mon.tx_load_count} times, expected {1 + len(tx_bytes)} "
        "(CS falling + one per byte boundary)"
    )
    assert mon.tx_load_misalignments() == [], (
        f"tx_load / rx_byte_valid contract broken: {mon.tx_load_misalignments()}"
    )


@cocotb.test(timeout_time=60, timeout_unit="us")
async def test_registered_link_layer_response_latency(dut):
    """Pin down the response latency seen by a link layer registered on rx_byte_valid.

    tx_data is sampled ON the boundary edge, one cycle before rx_byte_valid
    appears, so such a link layer always misses that capture and lands in the
    next one: the answer to byte N goes out during byte N+2, and the master owes
    one turnaround byte between a command and its reply.

    This is the contract the register layer will be written against, so it is
    asserted exactly rather than left to whichever SCK frequency is in use.
    """
    await reset_dut(dut)
    table = {0x10: 0xDE, 0x11: 0xAD, 0x12: 0xBE}
    idle = 0x5A  # what tx_data holds before any command arrives

    async def responder():
        """Link-layer stand-in: registers its answer one cycle after rx_byte_valid."""
        while True:
            await RisingEdge(dut.clk)
            await ReadOnly()
            if u(dut.rx_byte_valid):
                answer = table.get(u(dut.rx_data), 0x00)
                await FallingEdge(dut.clk)
                dut.tx_data.value = answer

    dut.tx_data.value = idle
    mon = SpiMonitor(dut)
    resp = cocotb.start_soon(responder())

    # Two trailing bytes to clock out the answers to the last two commands.
    mosi_bytes = [0x10, 0x11, 0x12, 0x00, 0x00]
    await spi_assert_cs(dut)
    for byte_val in mosi_bytes:
        await spi_send_byte(dut, byte_val)
    await spi_deassert_cs(dut)
    mon.stop()
    resp.cancel()
    mon.log_trace(dut, "registered link layer, answers arrive 2 bytes after their command")

    assert mon.tx_load_misalignments() == [], (
        f"tx_load / rx_byte_valid contract broken: {mon.tx_load_misalignments()}"
    )
    assert mon.rx_bytes == mosi_bytes, (
        f"RX mismatch: got {hexlist(mon.rx_bytes)}, expected {hexlist(mosi_bytes)}"
    )

    # Byte 0 and byte 1 both carry the idle byte: byte 0 from the CS-falling
    # capture, byte 1 from the boundary capture the responder was too late for.
    expected_tx = [idle, idle, 0xDE, 0xAD, 0xBE]
    expected = bits_of_all(expected_tx)
    assert mon.miso_at_rising == expected, (
        f"MISO stream mismatch, expected {hexlist(expected_tx)} "
        "(idle, turnaround byte, then each answer two bytes after its command):\n"
        f"  got      {mon.miso_at_rising}\n"
        f"  expected {expected}"
    )


@cocotb.test(timeout_time=40, timeout_unit="us")
async def test_miso_released_and_frozen_while_cs_high(dut):
    """Verify the bus is released on CS high and stray SCK edges do not shift."""
    await reset_dut(dut)

    dut.tx_data.value = 0xFF
    await spi_assert_cs(dut)
    await spi_send_byte(dut, 0x00)
    await spi_deassert_cs(dut)

    assert u(dut.spi_miso_oe) == 0, "spi_miso_oe still asserted after CS went high"
    assert u(dut.spi_miso) == 0, "MISO not parked low after CS went high"

    mon = SpiMonitor(dut)
    for _ in range(12):
        await spi_send_bit(dut, 1)
    mon.stop()

    assert mon.tx_load_count == 0, "tx_load pulsed on stray SCK edges while CS was high"
    assert all(r["oe"] == 0 for r in mon.records), (
        "spi_miso_oe asserted by stray SCK edges while CS was high"
    )
    assert all(r["miso"] == 0 for r in mon.records), (
        "MISO shifted on stray SCK edges while CS was high"
    )

    # The next frame must still start from the MSB of the byte presented at CS fall.
    dut.tx_data.value = 0x80
    await spi_assert_cs(dut)
    assert u(dut.spi_miso) == 1, "MISO did not reload from tx_data on the next frame"


@cocotb.test(timeout_time=60, timeout_unit="us")
async def test_full_duplex(dut):
    """Verify RX and TX run simultaneously without disturbing each other."""
    await reset_dut(dut)
    rx_in = [0x3C, 0xF0, 0x0F]
    tx_out = [0xC3, 0x0F, 0xF0]

    dut.tx_data.value = tx_out[0]
    mon = SpiMonitor(dut)

    await spi_assert_cs(dut)
    for idx, byte_val in enumerate(rx_in):
        dut.tx_data.value = tx_out[idx + 1] if idx + 1 < len(tx_out) else 0x00
        await spi_send_byte(dut, byte_val)
    await spi_deassert_cs(dut)
    mon.stop()

    mon.log_trace(dut, "full duplex")
    assert mon.rx_bytes == rx_in, (
        f"RX corrupted by TX activity: got {hexlist(mon.rx_bytes)}, expected {hexlist(rx_in)}"
    )
    expected = bits_of_all(tx_out)
    assert mon.miso_at_rising == expected, (
        f"TX corrupted by RX activity, expected {hexlist(tx_out)}:\n"
        f"  got      {mon.miso_at_rising}\n"
        f"  expected {expected}"
    )
    assert mon.illegal_miso_changes() == [], "MISO changed outside a falling edge"
    assert mon.tx_load_misalignments() == [], (
        f"tx_load / rx_byte_valid contract broken: {mon.tx_load_misalignments()}"
    )



@cocotb.test(timeout_time=40, timeout_unit="us")
async def test_cs_abort_mid_byte_tx(dut):
    """Verify a frame aborted mid-byte leaves no TX state behind.

    The RX side already has this test; the TX side has its own leftovers -- a
    half-shifted tx_shift and a bit_cnt that no longer marks a byte boundary.
    The next frame must still start from the MSB of tx_data.
    """
    await reset_dut(dut)

    dut.tx_data.value = 0xAA  # 1010_1010, MSB 1
    await spi_assert_cs(dut)
    for _ in range(3):  # 3 bits into the byte, mid-shift
        await spi_send_bit(dut, 0)
    await spi_deassert_cs(dut)

    assert u(dut.spi_miso_oe) == 0, "spi_miso_oe still asserted after a mid-byte abort"
    assert u(dut.spi_miso) == 0, "MISO not parked low after a mid-byte abort"

    # New frame, new byte: must come out whole, from bit 7.
    next_byte = 0x96
    dut.tx_data.value = next_byte
    mon = SpiMonitor(dut)
    await spi_assert_cs(dut)
    await spi_send_byte(dut, 0x00)
    await spi_deassert_cs(dut)
    mon.stop()
    mon.log_trace(dut, f"frame after a mid-byte abort, tx_data = 0x{next_byte:02X}")

    expected = bits_of(next_byte)
    assert mon.miso_at_rising == expected, (
        f"TX did not restart cleanly after a mid-byte abort:\n"
        f"  got      {mon.miso_at_rising}\n"
        f"  expected {expected} (0x{next_byte:02X} MSB first)"
    )


@cocotb.test(timeout_time=40, timeout_unit="us")
async def test_reset_during_frame(dut):
    """Verify a reset mid-frame releases the bus and stays quiet until the next CS.

    rst_n is asynchronous, so it can land anywhere in a transfer. What matters is
    that the PHY does not resume half-driving MISO for the rest of a frame it has
    lost track of: CS is still low at that point, and the frame is unrecoverable.
    """
    await reset_dut(dut)

    dut.tx_data.value = 0xFF  # so a failure to release shows up as a stuck 1
    await spi_assert_cs(dut)
    for _ in range(3):
        await spi_send_bit(dut, 1)
    assert u(dut.spi_miso_oe) == 1, "test setup: expected an active frame before reset"

    await FallingEdge(dut.clk)
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 2)

    assert u(dut.spi_miso) == 0, "MISO not cleared by reset"
    assert u(dut.spi_miso_oe) == 0, "spi_miso_oe not released by reset"
    assert u(dut.rx_data) == 0, "rx_data not cleared by reset"

    await FallingEdge(dut.clk)
    dut.rst_n.value = 1

    # CS is still low, so no cs_falling will arrive: the PHY must stay off the bus.
    mon = SpiMonitor(dut)
    for _ in range(8):
        await spi_send_bit(dut, 1)
    mon.stop()

    assert all(r["oe"] == 0 for r in mon.records), (
        "PHY drove MISO in the middle of the frame it was reset out of; it must "
        "wait for the next CS falling edge"
    )
    assert mon.rx_bytes == [0xFF], (
        "RX should still deserialize after reset; got " + hexlist(mon.rx_bytes)
    )

    # And a fresh frame works normally.
    dut.tx_data.value = 0x5A
    await spi_deassert_cs(dut)
    mon2 = SpiMonitor(dut)
    await spi_assert_cs(dut)
    await spi_send_byte(dut, 0x3C)
    await spi_deassert_cs(dut)
    mon2.stop()
    mon2.log_trace(dut, "frame after a mid-frame reset")

    assert mon2.rx_bytes == [0x3C], f"RX broken after reset: {hexlist(mon2.rx_bytes)}"
    assert mon2.miso_at_rising == bits_of(0x5A), "TX broken after reset"


@cocotb.test(timeout_time=100, timeout_unit="us")
async def test_slow_sck_and_stalled_master(dut):
    """Verify a slow master, and one that stalls mid-frame with CS held low.

    Everything else runs at 5 system clocks per bit, so a bug that depends on the
    rising and falling strobes being 2 cycles apart would pass unnoticed. Here
    they are 8 cycles apart, and the master pauses for 20 idle cycles between
    bytes with CS still asserted -- MISO must hold the bit it is presenting for
    the whole stall.
    """
    await reset_dut(dut)
    tx_bytes = [0x81, 0x7E]
    rx_bytes = [0xC3, 0x18]

    dut.tx_data.value = tx_bytes[0]
    mon = SpiMonitor(dut)

    await spi_assert_cs(dut)
    for idx, byte_val in enumerate(rx_bytes):
        dut.tx_data.value = tx_bytes[idx + 1] if idx + 1 < len(tx_bytes) else 0x00
        await spi_send_byte(dut, byte_val, gap=6)
        await ClockCycles(dut.clk, 20)  # master stalls, CS stays low
    await spi_deassert_cs(dut)
    mon.stop()
    mon.log_trace(dut, "slow SCK (1 MHz) with a 2 us stall between bytes")

    assert mon.rx_bytes == rx_bytes, (
        f"RX mismatch at slow SCK: got {hexlist(mon.rx_bytes)}, expected {hexlist(rx_bytes)}"
    )
    assert mon.miso_at_rising == bits_of_all(tx_bytes), (
        f"TX mismatch at slow SCK, expected {hexlist(tx_bytes)}:\n"
        f"  got      {mon.miso_at_rising}\n"
        f"  expected {bits_of_all(tx_bytes)}"
    )
    assert mon.illegal_miso_changes() == [], (
        "MISO moved while the master was stalled: "
        + ", ".join(f"cycle {k} ({a}->{b})" for k, a, b in mon.illegal_miso_changes())
    )
    assert mon.tx_load_misalignments() == [], (
        f"tx_load / rx_byte_valid contract broken: {mon.tx_load_misalignments()}"
    )
