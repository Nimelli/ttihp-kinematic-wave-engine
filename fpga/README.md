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

`ui[0]` (MODE_SW) and `uio[3:0]` are the reserved P1 SPI pins and are tied
low, matching what the DUT expects today.

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
