"""
Unit tests for spis_top.v -- the full SPI slave stack.

Run:  source ~/oss-cad-suite/environment && make MOD=spis_top

Unlike test_spis_phy.py, which pokes 1-cycle strobes straight into the PHY,
this file drives the *pads*: spi_clk, spi_mosi, spi_cs. Everything below them
(CDC, edge detect, bit assembly, protocol decode, register read) is exercised
as one unit, so the waveform in sim_build/spis_top/spis_top.fst has a real SCK
in it and can be read like a scope capture.

Protocol under test (spis_app.v):

    READ  (0x03), 4 bytes: opcode, ADDR, dummy (MISO 0x00), dummy (MISO = data)
    WRITE (0x02), 3 bytes: opcode, ADDR, DATA   (MISO 0x00 throughout)

The dummy byte at index 2 of a READ is not padding: spis_phy grabs the byte it
will shift out during byte N+1 at the END of byte N, one cycle before spis_app
has latched byte N. Byte 3 is therefore the first byte whose content can depend
on the address. A WRITE never turns the bus around, so it needs no dummy. See
the header of spis_app.v.

Master model: mode 0 (CPOL=0, CPHA=0). MOSI is set up while SCK is low, the
slave samples it on the rising edge; the master samples MISO on the rising edge
too, and the slave may only move MISO while SCK is low.

Why the master drives on falling clk edges: a real master is asynchronous to
clk, which is the whole reason spis_synchro exists. A simulator cannot model
metastability anyway, and driving asynchronously just creates delta-cycle races
that make failures unreproducible. Driving on the falling edge keeps every pad
transition a clean half-cycle away from the flops that sample it, which is what
the synchroniser guarantees in silicon.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, ReadOnly, RisingEdge

CLK_NS = 100        # 10.000 MHz system clock

# System clocks per SCK half period. spis_synchro needs SCK high and low for at
# least 2 clocks each (SCK <= 2.5 MHz); 5 gives ~830 kHz with margin to spare.
SCK_HALF = 5

# Clocks between CS falling and the first rising SCK edge. spis_phy.v's header
# asks for ~4: the synchroniser delays CS by 2-3 clocks and the MSB has to reach
# the pad before the master samples it.
CS_SETUP = 10
CS_HOLD = 4

CMD_READ = 0x03
CMD_WRITE = 0x02

# Index of the byte carrying the register data on a READ. Every other MISO byte,
# in either command, is 0x00.
DATA_BYTE = 3


# Seeded by the initial block in registers.v
SEEDED = {0x00: 0xA5, 0x01: 0x3C, 0x02: 0xFF}


def u(sig):
    return int(sig.value)


def hexlist(byte_list):
    return "[" + ", ".join("--" if b is None else f"0x{b:02X}" for b in byte_list) + "]"


_clock_task = None


def ensure_clock(dut):
    """Start a fresh system clock, cancelling any left over from a prior test."""
    global _clock_task
    if _clock_task is not None and not _clock_task.done():
        _clock_task.cancel()
    _clock_task = cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())


# ----------------------------------------------------------------------
# Pad-level monitor
# ----------------------------------------------------------------------


class PadMonitor:
    """Samples the SPI pads every clock edge, as a logic analyser would.

    Its job is the checks a byte-stream comparison cannot make: that MISO never
    moves while SCK is high, and that the slave never drives the bus outside a
    frame. Those are protocol violations that still produce the right bytes when
    the master happens to sample at the right moment.
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
                    "sck": u(self.dut.spi_clk),
                    "cs": u(self.dut.spi_cs),
                    "mosi": u(self.dut.spi_mosi),
                    "miso": self.dut.spi_miso.value,
                    "oe": self.dut.spi_miso_oe.value,
                }
            )

    def stop(self):
        self._task.cancel()

    def illegal_miso_changes(self):
        """Cycles where MISO moved while SCK was high.

        In mode 0 the master samples on the rising edge and holds until the next
        one, so any MISO transition during the high phase is a setup/hold
        violation on the master side even if the bit value happens to be right.
        """
        bad = []
        for k in range(1, len(self.records)):
            prev, cur = self.records[k - 1], self.records[k]
            if cur["miso"] == prev["miso"] and cur["oe"] == prev["oe"]:
                continue
            if not cur["sck"] and not prev["sck"]:
                continue        # SCK low: legal window
            if cur["cs"]:
                continue        # deselected: release is legal any time
            bad.append((k, str(prev["miso"]), str(cur["miso"])))
        return bad

    def drive_while_deselected(self, grace=4):
        """Cycles where spi_miso_oe was high while CS was high.

        On the TT pad this is uio_oe: driving it outside a frame fights whatever
        else shares the bus.

        `grace` covers the clocks right after CS rises. spis_phy releases the pad
        on spi_cs_sync, which the synchroniser delays by 2-3 clocks, so the slave
        physically cannot let go any sooner -- that lag is the oversampled
        design's t_dis, not a bug. Anything past the window is.
        """
        bad = []
        since_rise = None
        for k, r in enumerate(self.records):
            prev_cs = self.records[k - 1]["cs"] if k else 1
            if r["cs"] and not prev_cs:
                since_rise = 0
            elif r["cs"]:
                since_rise = None if since_rise is None else since_rise + 1
            else:
                since_rise = None
            driving = r["oe"].is_resolvable and int(r["oe"]) == 1
            if r["cs"] and driving and (since_rise is None or since_rise >= grace):
                bad.append(k)
        return bad

    def x_on_pads(self):
        """Cycles where the slave drove an X onto MISO.

        Only counted while oe is high -- an undriven pad is legitimately unknown.
        """
        bad = []
        for k, r in enumerate(self.records):
            if r["oe"].is_resolvable and int(r["oe"]) == 1 and not r["miso"].is_resolvable:
                bad.append(k)
        return bad


# ----------------------------------------------------------------------
# Mode 0 master BFM
# ----------------------------------------------------------------------


class SpiMaster:
    """Drives the pads as a CPOL=0 CPHA=0 master and records what came back."""

    def __init__(self, dut, half=SCK_HALF):
        self.dut = dut
        self.half = half
        self.miso_bits = []     # every bit sampled, across every frame
        self.oe_at_sample = []  # spi_miso_oe at each of those instants

    async def idle(self):
        await FallingEdge(self.dut.clk)
        self.dut.spi_cs.value = 1
        self.dut.spi_clk.value = 0
        self.dut.spi_mosi.value = 0

    async def select(self):
        await FallingEdge(self.dut.clk)
        self.dut.spi_cs.value = 0
        await ClockCycles(self.dut.clk, CS_SETUP)

    async def deselect(self):
        await ClockCycles(self.dut.clk, CS_HOLD)
        await FallingEdge(self.dut.clk)
        self.dut.spi_cs.value = 1
        await ClockCycles(self.dut.clk, 5)

    async def xfer_bit(self, out_bit):
        """One SCK period: MOSI setup, rising edge, falling edge.

        Returns the MISO bit the master would have latched, or None if the slave
        drove an X. A Hi-Z line reads as 1 through the usual pull-up.
        """
        # SCK low phase: present the next MOSI bit.
        await FallingEdge(self.dut.clk)
        self.dut.spi_mosi.value = out_bit
        await ClockCycles(self.dut.clk, self.half)

        # The master latches MISO at the rising edge. MISO has been stable since
        # the previous falling edge, so sampling in the read-only phase just
        # before we raise SCK reads the same value the master would, without
        # racing the driver.
        await ReadOnly()
        oe = self.dut.spi_miso_oe.value
        miso = self.dut.spi_miso.value
        self.oe_at_sample.append(oe)
        if not oe.is_resolvable or int(oe) == 0:
            bit = 1
        elif not miso.is_resolvable:
            bit = None
        else:
            bit = int(miso)
        self.miso_bits.append(bit)

        await FallingEdge(self.dut.clk)
        self.dut.spi_clk.value = 1
        await ClockCycles(self.dut.clk, self.half)
        await FallingEdge(self.dut.clk)
        self.dut.spi_clk.value = 0
        return bit

    async def xfer_byte(self, out_byte):
        """One byte out on MOSI, one byte in on MISO, MSB first."""
        in_byte = 0
        for bit_idx in range(7, -1, -1):
            bit = await self.xfer_bit((out_byte >> bit_idx) & 1)
            # An X stays None so it cannot silently become a plausible byte.
            in_byte = None if (bit is None or in_byte is None) else (in_byte << 1) | bit
        return in_byte

    async def transfer(self, tx_bytes):
        """Full frame: CS low, N bytes, CS high. Returns the N MISO bytes."""
        await self.select()
        rx = []
        for b in tx_bytes:
            rx.append(await self.xfer_byte(b))
        await self.deselect()
        return rx


async def reset_dut(dut):
    ensure_clock(dut)
    dut.rst_n.value = 0
    dut.spi_cs.value = 1
    dut.spi_clk.value = 0
    dut.spi_mosi.value = 0

    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    return SpiMaster(dut)


def read_frame(addr):
    """The 4-byte READ frame from the spis_app.v header."""
    return [CMD_READ, addr, 0x00, 0x00]


def write_frame(addr, data):
    """The 3-byte WRITE frame from the spis_app.v header."""
    return [CMD_WRITE, addr, data]


async def read_reg(master, addr):
    """Convenience: run a READ frame and return just the data byte."""
    return (await master.transfer(read_frame(addr)))[DATA_BYTE]


async def write_reg(master, addr, value):
    """Convenience: run a WRITE frame."""
    await master.transfer(write_frame(addr, value))


_reg_count = None


async def reg_count(master, limit=64):
    """How many registers reg_file actually has, discovered over the SPI bus.

    Hardcoding the size makes every write test fail the next time N_REGS moves,
    for a reason that has nothing to do with what the test is checking. Reads
    cannot tell the difference -- an unwritten register and an out-of-range one
    both return 0x00 -- but writes can: an out-of-range write is dropped, so a
    probe value that survives a write/read round trip marks an address that
    exists. Each probe restores what it found.

    Cached: the DUT is rebuilt per simulation, not per test. Whichever test runs
    first pays ~3 frames per address probed, so callers need a timeout with room
    for it -- 2 ms, rather than the 200-500 us a plain write test needs.
    """
    global _reg_count
    if _reg_count is not None:
        return _reg_count

    n = 0
    for addr in range(limit):
        original = await read_reg(master, addr)
        if original is None:
            break
        probe = original ^ 0xFF     # always differs from what is already there
        await write_reg(master, addr, probe)
        if await read_reg(master, addr) != probe:
            break
        await write_reg(master, addr, original)
        n = addr + 1

    assert n > 0, "reg_file accepted no writes at all -- nothing else can be tested"
    _reg_count = n
    return n


def describe(mosi, miso, note=""):
    lines = ["", note] if note else [""]
    for k, (m, s) in enumerate(zip(mosi, miso)):
        roles = (
            {0: "cmd  ", 1: "addr ", 2: "wdata"}
            if mosi and mosi[0] == CMD_WRITE
            else {0: "cmd  ", 1: "addr ", 2: "dummy", 3: "data "}
        )
        role = roles.get(k, f"byte{k}")
        lines.append(
            f"  byte {k} {role}  mosi 0x{m:02X}   miso "
            + ("XX (unresolvable)" if s is None else f"0x{s:02X}")
        )
    return "\n".join(lines)


# ----------------------------------------------------------------------
# Reset and bus ownership
# ----------------------------------------------------------------------


@cocotb.test(timeout_time=20, timeout_unit="us")
async def test_reset_state(dut):
    """The pad must stay Hi-Z and low through reset, even with CS already low."""
    ensure_clock(dut)
    dut.rst_n.value = 0
    dut.spi_cs.value = 0        # hostile: master already selected us
    dut.spi_clk.value = 0
    dut.spi_mosi.value = 1

    for _ in range(10):
        await FallingEdge(dut.clk)
        assert u(dut.spi_miso_oe) == 0, (
            "spi_miso_oe asserted during reset -- the pad must stay Hi-Z until the "
            "slave is out of reset, even if CS happens to be low"
        )
        assert u(dut.spi_miso) == 0, "spi_miso driven high during reset"


@cocotb.test(timeout_time=50, timeout_unit="us")
async def test_miso_hiz_when_deselected(dut):
    """spi_miso_oe must be low outside a frame and high throughout one."""
    master = await reset_dut(dut)
    mon = PadMonitor(dut)

    await ClockCycles(dut.clk, 20)
    assert u(dut.spi_miso_oe) == 0, "slave drove MISO before CS ever fell"

    await master.transfer(read_frame(0x00))
    await ClockCycles(dut.clk, 20)
    mon.stop()

    assert u(dut.spi_miso_oe) == 0, "slave still driving MISO after CS went high"
    assert all(int(oe) == 1 for oe in master.oe_at_sample), (
        "spi_miso_oe was low at a rising SCK edge inside the frame -- the master "
        "would have sampled a floating pad"
    )
    assert mon.drive_while_deselected() == [], (
        f"spi_miso_oe high while CS was high at cycles {mon.drive_while_deselected()[:8]}"
    )


# ----------------------------------------------------------------------
# READ protocol
# ----------------------------------------------------------------------


@cocotb.test(timeout_time=100, timeout_unit="us")
async def test_read_seeded_register(dut):
    """READ 0x03 / ADDR 0x00 must return 0xA5 on the third byte."""
    master = await reset_dut(dut)
    mon = PadMonitor(dut)

    mosi = read_frame(0x00)
    miso = await master.transfer(mosi)
    mon.stop()
    dut._log.info(describe(mosi, miso, "READ addr 0x00:"))

    assert miso[DATA_BYTE] == SEEDED[0x00], (
        f"data byte = {'XX' if miso[DATA_BYTE] is None else f'0x{miso[DATA_BYTE]:02X}'}, "
        f"expected 0x{SEEDED[0x00]:02X}\n" + describe(mosi, miso)
    )

    assert all(b == 0x00 for b in miso[:DATA_BYTE]), (
        "MISO carried something other than 0x00 before the data byte -- the "
        "slave is leaking state from an earlier frame\n" + describe(mosi, miso)
    )


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_read_each_seeded_register(dut):
    """Every seeded address must read back its own value, in its own frame."""
    master = await reset_dut(dut)

    for addr, expected in SEEDED.items():
        mosi = read_frame(addr)
        miso = await master.transfer(mosi)
        await ClockCycles(dut.clk, 10)
        assert miso[DATA_BYTE] == expected, (
            f"addr 0x{addr:02X} returned "
            f"{'XX' if miso[DATA_BYTE] is None else f'0x{miso[DATA_BYTE]:02X}'}, "
            f"expected 0x{expected:02X}\n" + describe(mosi, miso)
        )
        assert all(b == 0x00 for b in miso[:DATA_BYTE]), (
            f"addr 0x{addr:02X}: MISO was not 0x00 before the data byte\n"
            + describe(mosi, miso)
        )


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_read_latency_probe(dut):
    """Diagnostic: run a long frame and report which byte the data lands on.

    A read that is off by one byte still looks like a plain mismatch in every
    other test. This one drives extra dummy bytes and names the offset, so the
    log says whether the value is missing or merely late.
    """
    master = await reset_dut(dut)

    addr, expected = 0x01, SEEDED[0x01]
    mosi = [CMD_READ, addr] + [0x00] * 4
    miso = await master.transfer(mosi)
    dut._log.info(describe(mosi, miso, f"latency probe, addr 0x{addr:02X}:"))

    hits = [k for k, b in enumerate(miso) if b == expected]
    assert hits, (
        f"0x{expected:02X} never appeared on MISO in a 6-byte frame\n"
        + describe(mosi, miso)
    )
    assert hits[0] == DATA_BYTE, (
        f"register data first appeared on byte {hits[0]}, but the protocol in "
        f"spis_app.v puts it on byte {DATA_BYTE} -- the read is "
        f"{hits[0] - DATA_BYTE:+d} byte(s) off\n"
        + describe(mosi, miso)
    )


@cocotb.test(timeout_time=100, timeout_unit="us")
async def test_unknown_opcode_returns_zero(dut):
    """A non-READ opcode must not leak register contents onto MISO."""
    master = await reset_dut(dut)

    mosi = [0x99, 0x00, 0x00, 0x00]
    miso = await master.transfer(mosi)
    dut._log.info(describe(mosi, miso, "unknown opcode 0x99:"))

    assert all(b == 0x00 for b in miso), (
        f"opcode 0x99 put {hexlist(miso)} on MISO, expected all 0x00 -- an "
        "unrecognised command must not leak register contents\n"
        + describe(mosi, miso)
    )


# ----------------------------------------------------------------------
# Framing
# ----------------------------------------------------------------------


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_back_to_back_frames(dut):
    """Two frames in a row: the second must not inherit the first one's state."""
    master = await reset_dut(dut)

    first = await master.transfer(read_frame(0x00))
    second = await master.transfer(read_frame(0x02))

    assert first[DATA_BYTE] == SEEDED[0x00], (
        f"frame 1 returned {hexlist(first)}, expected 0x{SEEDED[0x00]:02X} "
        f"on byte {DATA_BYTE}"
    )
    assert second[DATA_BYTE] == SEEDED[0x02], (
        f"frame 2 returned {hexlist(second)}, expected 0x{SEEDED[0x02]:02X} "
        f"on byte {DATA_BYTE} -- the phase/addr state did not reset on CS falling"
    )
    assert second[0] == 0x00, (
        f"frame 2 opened with 0x{second[0]:02X} on MISO -- spis_phy captures "
        "tx_data on the cs_falling cycle, while spis_app still holds the "
        "previous frame's phase, so byte 0 is echoing the last register read"
    )


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_aborted_frame_then_clean_read(dut):
    """CS rising mid-byte must abort cleanly; the next frame decodes from byte 0."""
    master = await reset_dut(dut)

    # Half a frame: opcode plus 3 stray bits, then CS goes high.
    await master.select()
    await master.xfer_byte(CMD_READ)
    for _ in range(3):
        await master.xfer_byte(0xFF)    # never completes a meaningful phase
    await master.deselect()

    await ClockCycles(dut.clk, 20)

    mosi = read_frame(0x01)
    miso = await master.transfer(mosi)
    assert miso[DATA_BYTE] == SEEDED[0x01], (
        f"read after an aborted frame returned "
        f"{'XX' if miso[DATA_BYTE] is None else f'0x{miso[DATA_BYTE]:02X}'}, "
        f"expected 0x{SEEDED[0x01]:02X}\n" + describe(mosi, miso)
    )


@cocotb.test(timeout_time=100, timeout_unit="us")
async def test_stray_sck_while_deselected(dut):
    """SCK toggling with CS high must not advance any state."""
    master = await reset_dut(dut)
    await master.idle()

    for _ in range(3):
        await master.xfer_byte(0xFF)    # CS is high the whole time

    await ClockCycles(dut.clk, 10)
    assert u(dut.spi_miso_oe) == 0, "stray SCK edges made the slave drive the bus"

    mosi = read_frame(0x00)
    miso = await master.transfer(mosi)
    assert miso[DATA_BYTE] == SEEDED[0x00], (
        f"frame after stray SCK returned "
        f"{'XX' if miso[DATA_BYTE] is None else f'0x{miso[DATA_BYTE]:02X}'}, "
        f"expected 0x{SEEDED[0x00]:02X}\n" + describe(mosi, miso)
    )


# ----------------------------------------------------------------------
# WRITE protocol
#
# These tests derive every address from reg_count() rather than assuming a
# size, so they follow N_REGS in registers.v instead of breaking when it moves.
# ----------------------------------------------------------------------


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def test_write_then_read(dut):
    """A WRITE must be visible to a subsequent READ of the same address."""
    master = await reset_dut(dut)
    n = await reg_count(master)

    addr = n - 1                    # the top of the range, most likely to be mis-decoded
    original = await read_reg(master, addr)
    value = original ^ 0x5A

    mosi = write_frame(addr, value)
    miso = await master.transfer(mosi)
    dut._log.info(describe(mosi, miso, f"WRITE 0x{value:02X} -> addr 0x{addr:02X}:"))

    got = await read_reg(master, addr)
    assert got == value, (
        f"wrote 0x{value:02X} to addr 0x{addr:02X}, read back "
        f"{'XX' if got is None else f'0x{got:02X}'}"
    )


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_write_drives_zero_on_miso(dut):
    """A WRITE frame must put 0x00 on every MISO byte.

    The bus never turns around for a write, so anything non-zero here means the
    read path is driving when it should not be. Address 0x00 is seeded, which
    makes it the most tempting thing to leak.
    """
    master = await reset_dut(dut)
    mon = PadMonitor(dut)

    mosi = write_frame(0x00, 0x77)
    miso = await master.transfer(mosi)
    mon.stop()
    dut._log.info(describe(mosi, miso, "WRITE over a seeded register:"))

    assert all(b == 0x00 for b in miso), (
        f"WRITE frame returned {hexlist(miso)} on MISO, expected all 0x00\n"
        + describe(mosi, miso)
    )
    assert mon.x_on_pads() == [], "WRITE frame drove X onto MISO"


@cocotb.test(timeout_time=500, timeout_unit="us")
async def test_write_overwrites_reset_default(dut):
    """A reset default must be overwritable, and stay overwritten."""
    master = await reset_dut(dut)

    addr, value = 0x01, 0xC3        # 0x01 resets to 0x3C
    assert await read_reg(master, addr) == SEEDED[addr], (
        "addr 0x01 did not come out of reset at its default"
    )

    await write_reg(master, addr, value)
    assert await read_reg(master, addr) == value, "write over a seeded register was lost"
    assert await read_reg(master, addr) == value, "value did not persist across two reads"


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def test_write_does_not_disturb_neighbours(dut):
    """Writing one address must leave every other address alone."""
    master = await reset_dut(dut)
    n = await reg_count(master)

    addr = n // 2
    before = {a: await read_reg(master, a) for a in range(n) if a != addr}
    value = (await read_reg(master, addr)) ^ 0x9E

    await write_reg(master, addr, value)

    assert await read_reg(master, addr) == value, "the write itself did not land"
    for a, was in before.items():
        now = await read_reg(master, a)
        assert now == was, (
            f"writing addr 0x{addr:02X} changed addr 0x{a:02X} from "
            f"0x{was:02X} to {'XX' if now is None else f'0x{now:02X}'}"
        )


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def test_write_out_of_range_is_dropped(dut):
    """A write past the end of reg_file must be dropped, not aliased onto a real entry."""
    master = await reset_dut(dut)
    n = await reg_count(master)

    before = {a: await read_reg(master, a) for a in range(n)}

    # n is the first out-of-range address and catches an index that was merely
    # truncated to ADDR_W bits instead of guarded; 0xFF is out of range outright.
    await write_reg(master, n, 0xAD)
    await write_reg(master, 0xFF, 0xDE)

    for a in (n, 0xFF):
        got = await read_reg(master, a)
        assert got == 0x00, (
            f"out-of-range READ of addr 0x{a:02X} returned "
            f"{'XX' if got is None else f'0x{got:02X}'}, expected 0x00"
        )
    for a, was in before.items():
        now = await read_reg(master, a)
        assert now == was, (
            f"an out-of-range write aliased onto addr 0x{a:02X}: 0x{was:02X} -> "
            f"{'XX' if now is None else f'0x{now:02X}'}"
        )


@cocotb.test(timeout_time=500, timeout_unit="us")
async def test_unknown_opcode_does_not_write(dut):
    """Only 0x02 may write. Any other opcode with a write-shaped frame is inert."""
    master = await reset_dut(dut)

    addr = 0x00
    original = await read_reg(master, addr)

    for opcode in (0x99, 0x03, 0x00, 0xFF):
        await master.transfer([opcode, addr, original ^ 0xBE])
        got = await read_reg(master, addr)
        assert got == original, (
            f"opcode 0x{opcode:02X} changed addr 0x{addr:02X} from 0x{original:02X} "
            f"to {'XX' if got is None else f'0x{got:02X}'} -- only CMD_WRITE may "
            "reach reg_wr_en"
        )


@cocotb.test(timeout_time=500, timeout_unit="us")
async def test_truncated_write_does_not_commit(dut):
    """CS rising part-way through the data byte must leave the register untouched.

    The write strobe hangs off rx_byte_valid, which never fires for a byte that
    was not clocked all the way in -- so a master that gives up mid-byte cannot
    leave a half-formed value behind.
    """
    master = await reset_dut(dut)

    addr = 0x00
    original = await read_reg(master, addr)

    # Opcode and address complete, then only 4 bits of the data byte.
    await master.select()
    await master.xfer_byte(CMD_WRITE)
    await master.xfer_byte(addr)
    for bit in [1, 0, 1, 1]:
        await master.xfer_bit(bit)
    await master.deselect()

    await ClockCycles(dut.clk, 20)
    got = await read_reg(master, addr)
    assert got == original, (
        f"a write truncated mid-byte changed addr 0x{addr:02X} from "
        f"0x{original:02X} to {'XX' if got is None else f'0x{got:02X}'}"
    )


@cocotb.test(timeout_time=2, timeout_unit="ms")
async def test_write_read_alternating_sequence(dut):
    """Alternating commands: no frame may inherit state from the one before it."""
    master = await reset_dut(dut)
    n = await reg_count(master)

    # Two passes over every register, second pass with different values, so a
    # write that lands on the previous frame's address shows up as a mismatch.
    expected = {}
    for pass_no in (0, 1):
        for addr in range(n):
            value = (0x11 * (addr + 1) + 0x40 * pass_no) & 0xFF
            await write_reg(master, addr, value)
            expected[addr] = value
            got = await read_reg(master, addr)
            assert got == value, (
                f"pass {pass_no}: wrote 0x{value:02X} to addr 0x{addr:02X}, read back "
                f"{'XX' if got is None else f'0x{got:02X}'}"
            )

    for addr, value in expected.items():
        got = await read_reg(master, addr)
        assert got == value, (
            f"addr 0x{addr:02X} held 0x{value:02X} but later read "
            f"{'XX' if got is None else f'0x{got:02X}'}"
        )


# ----------------------------------------------------------------------
# Pad-level protocol legality
# ----------------------------------------------------------------------


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_miso_only_moves_while_sck_low(dut):
    """MISO must never change during the SCK high phase (mode 0)."""
    master = await reset_dut(dut)
    mon = PadMonitor(dut)

    await master.transfer(read_frame(0x00))
    await master.transfer(read_frame(0x01))
    mon.stop()

    bad = mon.illegal_miso_changes()
    assert bad == [], (
        f"MISO changed while SCK was high at {bad[:8]} "
        f"({len(bad)} total) -- the master samples on the rising edge, so this "
        "is a setup/hold violation on its side"
    )


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_no_x_driven_onto_pad(dut):
    """The slave must never drive an unknown value while spi_miso_oe is high."""
    master = await reset_dut(dut)
    mon = PadMonitor(dut)

    await master.transfer(read_frame(0x00))
    mon.stop()

    bad = mon.x_on_pads()
    assert bad == [], (
        f"spi_miso was X while driven, at {len(bad)} cycles starting {bad[:8]} -- "
        "an X here reaches a real pad in silicon"
    )


@cocotb.test(timeout_time=200, timeout_unit="us")
async def test_unseeded_register_reads_known_value(dut):
    """Addresses the initial block does not seed must still read a defined value.

    registers.v seeds mem[0..2] only, so mem[3] powers up unknown. An X here is
    not a simulation artefact: on silicon those flops come up at whatever the
    process gives them, and the value reaches the pad.

    The frame runs long and checks every data-phase byte rather than byte 2
    alone, so the test stays honest whichever byte the read latency puts the
    payload on.
    """
    master = await reset_dut(dut)

    mosi = [CMD_READ, 0x03] + [0x00] * 3
    miso = await master.transfer(mosi)
    dut._log.info(describe(mosi, miso, "unseeded addr 0x03:"))

    unknown = [k for k, b in enumerate(miso) if b is None]
    assert unknown == [], (
        f"reading an unseeded register drove X onto MISO at byte(s) {unknown} -- "
        "reg_file needs a reset, not just an initial block\n" + describe(mosi, miso)
    )


@cocotb.test(timeout_time=400, timeout_unit="us")
async def test_slow_master(dut):
    """A much slower SCK must decode identically -- nothing may depend on SCK rate."""
    master = await reset_dut(dut)
    master.half = 20            # ~208 kHz

    mosi = read_frame(0x02)
    miso = await master.transfer(mosi)
    assert miso[DATA_BYTE] == SEEDED[0x02], (
        f"slow master returned "
        f"{'XX' if miso[DATA_BYTE] is None else f'0x{miso[DATA_BYTE]:02X}'}, "
        f"expected 0x{SEEDED[0x02]:02X}\n" + describe(mosi, miso)
    )
