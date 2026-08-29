# Testing the Kinematic Wave Engine on a Cmod A7-35T

Everything in `fpga/` is **FPGA-only**. None of it is taped out. Do not add
any of these files to `info.yaml` or `test/Makefile` — the only thing that
goes to the shuttle is `src/`, and `kwe_fpga_top.v` instantiates
`tt_um_nimelli_kinematic_wave_engine` completely unmodified.

## What this actually proves

Simulation has already shown the design is *logically* correct. What it
cannot show is whether it behaves correctly in real time, at real pins, for
hours, across all 16 speed settings. That is what the board is for.

| Validated here | Not validated here |
|---|---|
| Pulse contract at physical pins | The IHP netlist or GDS |
| Frame rate and timebase against a real oscillator | Timing at 130 nm |
| All 16 speed-ROM entries, measured | Cell-level power |
| Every amp / spread / mirror / reverse combination, interactively | Anything the gate-level sim covers |
| 3.3 V signalling into real wiring | The tapeout's reset behaviour (FPGA flops initialise; ASIC flops do not) |
| Eventually: the mechanical hypothesis, once servos exist | |

The tapeout still rests on simulation. This is additional evidence, not a
substitute — but it is the only evidence that runs at wall-clock speed.

## The clock problem, and why there is an MMCM

The Cmod A7 has a **12 MHz** oscillator. The design assumes **10 MHz**, and
every constant in `kwe_timebase` is derived from it. Run the RTL straight
off 12 MHz and every pulse comes out at 10/12 of its intended width —
0.83 ms to 1.67 ms — which drives an SG90 hard into both mechanical stops.

`kwe_fpga_top` therefore synthesises exactly 10 MHz:

```
12 MHz × 50 = 600 MHz VCO,   600 MHz / 60 = 10.000 MHz
```

600 MHz is the bottom of the -1 speed grade MMCM VCO range. A PLL cannot be
used at all: the 7-series PLL has a 19 MHz minimum input frequency (Cmod A7
reference manual §5). The taped-out RTL is untouched — the fix lives
entirely in the wrapper.

## Step 0 — run the simulation first (no Vivado needed)

The harness has its own logic: a pulse monitor, an ASCII formatter, two
UARTs. If those were wrong you would be debugging them on a board with no
scope, which is miserable. So verify them first.

```bash
source ~/oss-cad-suite/environment
cd test/fpga && make
```

Six tests, about six minutes (`test_wave_moves` simulates 840 ms to get past
the 500 ms startup hold). All six must pass before you spend time in Vivado.

`KWE_SIM` bypasses the MMCM here — Xilinx primitives would need the unisim
libraries and `glbl`. The MMCM is the one part only hardware can confirm,
and it confirms itself: if it were wrong, every measured width would be off
by 20% and nothing would match.

## Step 1 — install Vivado

The free **ML Standard** edition covers XC7A35T; you do not need a licence.

During the installer's device-selection page, tick **Artix-7 only**. The
default selects every 7-series and UltraScale family and turns a ~30 GB
install into well over 100 GB for no benefit here.

```bash
source ~/Xilinx/2025.1/Vivado/settings64.sh   # adjust to your install path
vivado -version
```

## Step 2 — build the bitstream

```bash
cd fpga
vivado -mode batch -source build.tcl
```

Non-project mode: no `.xpr`, no IP cache, nothing to accidentally commit.
Everything lands in `fpga/build/`, which `.gitignore` excludes. Expect
3–5 minutes.

The script fails the build if timing is not met. At 10 MHz on an Artix-7
this design has enormous margin, so a violation does not mean "tighten the
constraints" — it means something structural is wrong, most likely that the
MMCM was not inferred. Check `build/timing_summary.rpt` before anything else.

Sanity check on `build/utilization.rpt`: the design is a few hundred flops
and one MMCM, so anything above ~2% LUT usage means something unexpected got
synthesised.

## Step 3 — program the board

Plug in the micro-USB cable. Two ways:

```bash
# Terminal, no GUI (openFPGALoader ships with oss-cad-suite)
source ~/oss-cad-suite/environment
openFPGALoader -b cmoda7_35t build/kwe_fpga_top.bit

# Or make it survive a power cycle, by writing the SPI flash
openFPGALoader -b cmoda7_35t -f build/kwe_fpga_top.bit
```

`-f` writes the Quad-SPI flash instead of the FPGA, so the board comes up
running the wave on any power-up with no PC attached. It takes noticeably
longer than a JTAG load. `build.tcl` also emits a `.bin` for Vivado's
Hardware Manager, which wants that format for indirect flash programming.

Vivado's Hardware Manager works too, but the command line is faster and
scriptable.

If you get a permissions error, install openFPGALoader's udev rules (they
ship with it as `99-openfpgaloader.rules`) into `/etc/udev/rules.d/`, then
`sudo udevadm control --reload`.

## Step 4 — verify it, with nothing attached

Two LEDs tell you most of what you need before you open any software.

- **LD2** (`led[1]`) blinks at about **1.6 Hz** — one toggle per 16 frames.
  This is the single most important indicator on the board. If it blinks at
  that rate, the frame period is right, which means the MMCM is right, which
  means the timebase is right.
- **LD1** (`led[0]`) brightness follows channel 0's pulse width, so the wave
  is visible as a slow breathing glow.

**For the first 500 ms after reset, LD1 sits at a constant brightness.** That
is the startup centre-hold in `kwe_phase_gen`, commanding every rod flat so
servos can assemble before any motion. It is not a fault.

Now the real view. The board reports every pulse it produces over the same
USB cable:

```bash
python3 fpga/kwe_monitor.py --port /dev/ttyUSB1
```

The Cmod A7 presents two serial devices — the FTDI's channel A is JTAG,
channel B is the UART. The UART is normally the **higher-numbered** one. If
`/dev/ttyUSB1` is silent, try `/dev/ttyUSB0`, or check `dmesg | tail` after
plugging in.

You get a live gauge of all eight rods:

```
  speed  8   amp 3   spread 0   mirror 0   reverse 0
  frames 412       period  2.00s (nominal 2.00s)   violations 0

  ch0   1701.4 us  pos  180.3  |---------------------+------O------|
  ch1   1810.2 us  pos  208.2  |---------------------+---------O---|
  ...
  s/S speed  a/A amp  p spread  m mirror  r/R reverse  x reset  q quit
```

Those keys drive the design live — no rebuild to try a different setting.
This is the whole parameter space, explorable in a couple of minutes.

## Step 5 — the two checks worth running

**Pulse contract.** Validates every frame against the spec and gives a
verdict:

```bash
python3 fpga/kwe_monitor.py --port /dev/ttyUSB1 --check 200
```

The sharpest test it applies: a pulse is `(256 + pos) × 39` clocks, so every
measured width must be an exact multiple of 39. A wrong clock or an off-by-one
in the measurement breaks that immediately.

**Speed ROM.** This is BUILD-PLAN stage 6 done on hardware — it measures the
real wave period at each of the 16 settings and compares against
`kwe_speed_rom`:

```bash
python3 fpga/kwe_monitor.py --port /dev/ttyUSB1 --sweep --max-wait 45
```

The slowest setting has a 20.1 s period and needs several cycles, so a full
sweep takes a few minutes; with a smaller `--max-wait` the slow settings come
back unmeasured rather than wrong.

Periods are counted in **frames**, not host time — the FTDI latency timer
batches arrivals into 16 ms chunks, which would be useless for timing
anything. The FPGA emits exactly one line per 19.968 ms frame, which makes
the line count a perfect clock.

One caveat the tool handles: `kwe_phase_gen` reverses direction every 2–6
cycles, and the interval spanning a reversal is not a period. The sweep sets
reversal to its longest setting and takes the median, so those intervals show
up as outliers rather than skewing the answer.

## Step 6 — attaching servos, when you have them

Read this section before wiring anything.

**Never power servos from the Cmod A7.** The board's 3.3 V rail is rated
0.6 A *total* and is shared with the SRAM, the USB controller, the LEDs and
every FPGA I/O. One SG90 draws around 700 mA stalled. Eight of them will
brown out the FPGA, and a brown-out mid-JTAG can corrupt the flash.

Wiring:

| Servo wire | Goes to |
|---|---|
| Signal (orange) | DIP pins 26–33, one per channel |
| V+ (red) | External 5 V supply, **not** the board |
| GND (brown) | Supply ground, **and** DIP pin 25 (GND) |

- Use an external **5 V supply rated 3 A or more**, with 470–1000 µF of bulk
  capacitance close to the servos.
- The supply ground and the board ground must be joined, or the servos see
  no valid signal. DIP pin 25 is GND and sits right next to the servo
  signal pins — that adjacency is why those pins were chosen.
- **Do not feed the external supply into VU (DIP pin 24) while USB is
  connected.** The reference manual warns about this explicitly: with a USB
  host attached, the board drives VU to ~5 V itself, and back-feeding it can
  damage your supply.
- SG90s are 5 V parts driven here by 3.3 V logic. Their input threshold is
  usually low enough that this just works. If a servo jitters or ignores the
  signal, that is the first thing to suspect — a level shifter fixes it.
- **Start with one servo on channel 0**, confirm it centres and sweeps, then
  add the rest. Eight unknown servos at once is eight times the debugging.

The 500 ms startup hold exists precisely for this moment: on power-up every
rod is commanded flat first, so the horns can be fitted without the array
slamming from arbitrary positions.

## Pinout

Board pin data from Digilent's `Cmod-A7-Master.xdc` (rev. B) and the Cmod A7
reference manual.

| Signal | DIP pin | Package pin |
|---|---|---|
| `servo[0]` | 26 | R3 |
| `servo[1]` | 27 | T3 |
| `servo[2]` | 28 | R2 |
| `servo[3]` | 29 | T1 |
| `servo[4]` | 30 | T2 |
| `servo[5]` | 31 | U1 |
| `servo[6]` | 32 | W2 |
| `servo[7]` | 33 | V2 |
| **GND** | **25** | — |
| **VU (5 V)** | **24** | — |
| `spi_cs` | 1 | M3 |
| `spi_sck` | 2 | L3 |
| `spi_mosi` | 3 | A16 |
| `spi_miso` | 4 | K3 |

On-board: `sysclk` L17, `btn[0]` A18 (reset), `btn[1]` B18 (speed++),
`led[0]` A17, `led[1]` C16, UART J18/J17.

## How the controls map to the chip

The wrapper has no DIP switches to offer, so the control pins are driven by
registers you can nudge over serial or with BTN1. They sit in the power-on
domain, not the DUT's reset domain — resetting the design keeps your
settings, exactly as physical switches would. Only unplugging the board
restores the defaults (speed 8, amp 3, spread 0, mirror 0, reverse 0).

| Setting | Chip pin | Key |
|---|---|---|
| `speed_sel` | `ui[4:1]` | `s` / `S`, or BTN1 |
| `amp_sel` | `ui[6:5]` | `a` / `A` |
| `spread_sel` | `ui[7]` | `p` |
| `mirror_sel` | `uio[4]` | `m` |
| `reverse_sel` | `uio[6:5]` | `r` / `R` |
| `MODE_SW` | `ui[0]` | `n` |

`MODE_SW` powers up **low** — auto mode, parameters from the pins, the SPI
registers inert. `n` toggles it. The safe state is the one you get by doing
nothing. `uio[3:0]` are the SPI slave — see Step 7.

## Step 7 — the SPI slave, driven by an RP2040

`uio[3:0]` carry the P1 SPI slave and are wired straight out to DIP pins 1–4.
The wrapper does not touch them — no synchroniser, no debounce, no test
fixture. `spis_synchro` inside the DUT is what makes an asynchronous SCK safe,
so what the RP2040 talks to is exactly what the chip will present.

### Wiring

Pin 1 is the one marked with a triangle on the silkscreen.

| Cmod DIP | Package | Signal | RP2040 side |
|---|---|---|---|
| 1 | M3 | `SPI_CS` | any GPIO, driven high when idle |
| 2 | L3 | `SPI_SCK` | SPI SCK |
| 3 | A16 | `SPI_MOSI` | SPI TX |
| 4 | K3 | `SPI_MISO` | SPI RX |
| 25 | — | **GND** | **GND** |

Run the ground wire. Both boards being on the same USB host is not a
substitute — that is a long shared return path, and SCK edges belong on a
short one.

`SPI_CS` has a PULLUP and `SPI_SCK` a PULLDOWN in the XDC. With the RP2040
unplugged or in reset those lines float, and a floating active-low CS reads as
*selected*: the slave would drive MISO and clock on noise.

### Two constraints the master must respect

**SCK ≤ 1.25 MHz — use 1 MHz.** Two limits apply and the tighter one binds.
Edge detection needs SCK high and low for ≥2 system clocks (2.5 MHz), but the
*MISO turnaround* is what actually caps the bus: a SCK falling edge takes 3
system clocks to reach the pad through the synchroniser, plus one more for the
asynchronous alignment, and the master samples half an SCK period later. That
gives clk/8 = 1.25 MHz. Measured against the RTL, a read is correct at clk/5
and returns data shifted one bit late at clk/3. Derivation in
`src/spis_synchro.v`.

At 1 MHz you have comfortable margin, and there is nothing to gain from going
faster — the whole map is two bytes.

**~400 ns between CS falling and the first SCK edge.** The synchroniser delays
CS by 2–3 clocks, and the MSB has to reach the pad before the master samples
it — in mode 0 there is no falling edge before that first sample. This is the
ordinary SPI CS-setup time, just larger than on a natively-clocked slave.
MicroPython's per-call overhead covers it by accident; the explicit sleep
below covers it on purpose.

### The register map

| addr | name | access | reset | contents |
|---|---|---|---|---|
| `0x00` | WAVE0 | RW | `0x38` | `[3:0]` SPEED, `[5:4]` AMP, `[6]` SPREAD, `[7]` MIRROR |
| `0x01` | WAVE1 | RW | `0x00` | `[1:0]` REVERSE, `[7:2]` reserved |
| `0x7F` | ID | RO | `0xA5` | constant — reads neither `0x00` (dead MISO) nor `0xFF` (floating) |

The registers only reach the wave engine when **`ui[0]` MODE_SW is high**. With
MODE_SW low the chip is in auto mode and the SPI side is a spectator: writing
the registers changes nothing. That is deliberate, and it is what makes the SPI
block safe to have on a chip whose priority is auto-mode.

WAVE0's reset value is speed=8, amp=3 — a wave worth shipping, because it is
what the chip runs if MODE_SW is ever stuck high with no master attached.

### The protocol

Mode 0 (CPOL=0, CPHA=0), MSB first.

```
READ  0x03:  [0x03] [ADDR] [dummy] [dummy]   register data on the 4th byte
WRITE 0x02:  [0x02] [ADDR] [DATA]            MISO is 0x00 throughout
```

The READ's dummy byte is not padding. `spis_phy` grabs the byte it will shift
out during byte N+1 at the *end* of byte N, one cycle before the protocol
layer has latched byte N — so byte 3 is the first byte whose content can
depend on the address. `src/spis_app.v` explains it in full.

### RP2040, MicroPython

```python
import time
from machine import Pin, SPI

spi = SPI(0, baudrate=1_000_000, polarity=0, phase=0,
          sck=Pin(18), mosi=Pin(19), miso=Pin(20))
cs = Pin(25, Pin.OUT, value=1)

def xfer(data):
    buf = bytearray(len(data))
    cs(0)
    time.sleep_us(5)              # CS setup: the slave needs ~400 ns
    spi.write_readinto(bytes(data), buf)
    time.sleep_us(5)
    cs(1)
    return buf

def read_reg(addr):        return xfer([0x03, addr, 0x00, 0x00])[3]
def write_reg(addr, val):  xfer([0x02, addr, val])

print(hex(read_reg(0x7F)))                     # 0xa5 -- the link is alive
print(hex(read_reg(0x00)))                     # 0x38 -- speed 8, amp 3

# Drive the wave from SPI. Requires ui[0] MODE_SW high.
def set_wave(speed=8, amp=3, spread=0, mirror=0, reverse=0):
    write_reg(0x00, (speed & 0xF) | ((amp & 3) << 4)
                    | ((spread & 1) << 6) | ((mirror & 1) << 7))
    write_reg(0x01, reverse & 3)

set_wave(speed=12, amp=3, spread=1)
```

`0xa5` from the ID register on the first try means the whole chain is alive:
pads, CDC, PHY, protocol decode, register file, and MISO back out.

**Nothing you write moves the servos until MODE_SW (`ui[0]`) is high.** Press
`n` on the serial port to toggle it. It powers up low, so the chip is in auto
mode until you deliberately hand control over.

### If it does not work

| Symptom | Likely cause |
|---|---|
| Every address reads `0x00` | MISO not getting back — check the DIP 4 wire, and that `uio_oe[3]` is driven by `spi_miso_oe` rather than tied off |
| Every address reads `0xFF` | The slave is never driving; you are reading the RP2040's own pull-up. CS not actually reaching DIP 1, or CS never goes low |
| Reads are shifted by one byte | The dummy byte is missing — a READ is 4 bytes, not 3 |
| Correct on the first byte, garbage after | SCK too fast. Drop to 500 kHz and confirm |
| Intermittent, worse with long wires | No ground wire, or the RP2040 is not grounded to DIP 25 |
| Reads work, writes never stick | Address out of range: `reg_file` has `N_REGS` entries and drops writes past it |

Before suspecting the board, run the slave in simulation — it is the same RTL:

```bash
cd test/unit && make MOD=spis_top     # 21 tests, seconds
```

## Troubleshooting

**Neither LED does anything.** The bitstream is not loaded, or the FPGA is
booting an old image from flash. Reprogram over JTAG and watch for
openFPGALoader's success message.

**LD2 blinks, but at the wrong rate.** If it is around 1.9 Hz rather than
1.6 Hz, the MMCM is not being used and the design is running at 12 MHz —
check that `build.tcl` picked up the non-`KWE_SIM` path and that the MMCM
appears in `build/clock_util.rpt`.

**No serial output.** Wrong device node — try the other `/dev/ttyUSB*`.
Confirm with `python3 fpga/kwe_monitor.py --raw --port ...`, which prints
lines verbatim.

**openFPGALoader says the device is busy.** The kernel's `ftdi_sio` driver
has claimed the JTAG interface. openFPGALoader can usually detach it; if
not, unbind interface 0 manually or unplug and retry.

**`--check` reports "not a whole number of 39-clock ticks".** The clock is
not 10 MHz. This is the MMCM, not the design.

**Everything reads 1497.6 µs and never changes.** That is the centre
position on every channel — the design is stuck in the startup hold, which
means `phase_tick` is not arriving. Worth a waveform in simulation, not on
the board.
