#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Host-side viewer for the Kinematic Wave Engine running on a Cmod A7-35T.

*** NOT PART OF THE TAPEOUT. ***

The FPGA emits one line per 19.968 ms frame over the same USB cable that
programs it:

    W cccc wwww wwww wwww wwww wwww wwww wwww wwww

field 0     control word (what the DUT is being told to do)
field 1..8  channel 0..7 pulse width, in 10 MHz clock cycles, hex

This turns that into a live picture of the eight rods, and checks the
numbers against the pulse contract. Without servos or a scope, this is the
only thing that can tell you the design is actually working on silicon-like
hardware rather than merely fitting in it.

Modes:
    (default)  live bar display, keys change the wave
    --check N  validate N frames, print a verdict, exit 0 or 1
    --sweep    measure the real wave period at each speed setting
    --raw      dump lines as they arrive

Usage:
    python3 kwe_monitor.py --port /dev/ttyUSB1
    python3 kwe_monitor.py --port /dev/ttyUSB1 --check 200
    python3 kwe_monitor.py --port /dev/ttyUSB1 --sweep --max-wait 12
"""

import argparse
import select
import sys
import termios
import time
import tty

try:
    import serial
except ImportError:
    sys.exit("pyserial is missing:  pip install -r fpga/requirements.txt")


CLK_HZ = 10_000_000
TICK_CLKS = 39  # kwe_timebase prescaler
PULSE_BASE = 256  # ticks at pos = 0

MIN_CLKS = (PULSE_BASE + 0) * TICK_CLKS  # 9984
CENTRE_CLKS = (PULSE_BASE + 128) * TICK_CLKS  # 14976
MAX_CLKS = (PULSE_BASE + 255) * TICK_CLKS  # 19929

FRAME_HZ = CLK_HZ / (8 * 640 * TICK_CLKS)  # 50.08...

# Phase increments from src/kwe_speed_rom.v. Kept as the increments rather
# than as precomputed periods so this list can be diffed against the RTL
# directly -- the period is derived the same way kwe_phase_gen derives it.
SPEED_INC = [65, 87, 119, 156, 208, 278, 374, 503,
             654, 872, 1190, 1577, 2111, 2784, 3739, 5033]


def nominal_period_s(speed_sel):
    """Wave period at a speed setting: 65536 accumulator units at `inc`/frame."""
    return 65536 / (SPEED_INC[speed_sel] * FRAME_HZ)

KEYS = "sSaApmrRx"
KEY_HELP = (
    "s/S speed  a/A amp  p spread  m mirror  r/R reverse  x reset  q quit"
)


def clocks_to_us(clocks):
    return clocks * 1e6 / CLK_HZ


def clocks_to_pos(clocks):
    """Exact inverse of the pulse contract: pulse = (256 + pos) ticks."""
    return clocks / TICK_CLKS - PULSE_BASE


def unpack_ctrl(ctrl):
    return {
        "speed": (ctrl >> 3) & 0xF,
        "amp": (ctrl >> 7) & 0x3,
        "spread": (ctrl >> 9) & 0x1,
        "mirror": (ctrl >> 2) & 0x1,
        "reverse": ctrl & 0x3,
    }


def parse(line):
    """'W cccc wwww ...' -> (ctrl, [w0..w7]), or None if malformed."""
    parts = line.split()
    if len(parts) != 10 or parts[0] != "W":
        return None
    try:
        values = [int(p, 16) for p in parts[1:]]
    except ValueError:
        return None
    return values[0], values[1:]


def check_widths(widths):
    """Return a list of contract violations, empty if the frame is good."""
    problems = []
    for channel, width in enumerate(widths):
        if not MIN_CLKS <= width <= MAX_CLKS:
            problems.append(
                f"ch{channel} {width} clk ({clocks_to_us(width):.1f} us) is "
                f"outside {MIN_CLKS}..{MAX_CLKS} -- an SG90 would be driven "
                f"past its stop"
            )
        elif width % TICK_CLKS != 0:
            # width = (256 + pos) * 39 exactly, so anything else means the
            # clock is not 10 MHz or the measurement is off by a cycle. This
            # is the sharpest single check available.
            problems.append(
                f"ch{channel} {width} clk is not a whole number of "
                f"{TICK_CLKS}-clock ticks (remainder "
                f"{width % TICK_CLKS}) -- check the MMCM"
            )
    return problems


def bar(pos, width=41):
    """One rod as a text gauge, centre marked."""
    pos = max(0.0, min(255.0, pos))
    slot = int(round(pos / 255.0 * (width - 1)))
    cells = ["-"] * width
    cells[width // 2] = "+"
    cells[slot] = "O"
    return "".join(cells)


class PeriodTracker:
    """Measure the wave period from channel 0 crossing its centre upward.

    Time is counted in FRAMES, not host clock. The FPGA emits exactly one
    line per 19.968 ms frame, so the frame index is a perfect clock, whereas
    host arrival times are useless for this: the FTDI latency timer batches
    bytes into 16 ms chunks, which is most of a frame period.

    The crossing instant is interpolated between the two straddling samples,
    which matters at the fast settings where a whole period is only a dozen
    frames.

    Caveat: kwe_phase_gen reverses direction every 2..6 wave cycles, and the
    interval spanning a reversal is not a period. That is why this keeps a
    list and reports the MEDIAN -- the reversal intervals are outliers, not
    a bias to be averaged in.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.prev = None
        self.last_cross = None
        self.intervals = []

    def update(self, pos0, frame_index):
        if self.prev is not None:
            prev_pos, prev_index = self.prev
            if prev_pos < 128 <= pos0:
                span = pos0 - prev_pos
                frac = (128 - prev_pos) / span if span else 0.0
                cross = prev_index + frac * (frame_index - prev_index)
                if self.last_cross is not None:
                    self.intervals.append(cross - self.last_cross)
                    del self.intervals[:-16]
                self.last_cross = cross
        self.prev = (pos0, frame_index)

    @property
    def count(self):
        return len(self.intervals)

    @property
    def period(self):
        """Median interval, in seconds. None until at least one is seen."""
        if not self.intervals:
            return None
        ordered = sorted(self.intervals)
        middle = len(ordered) // 2
        if len(ordered) % 2:
            frames = ordered[middle]
        else:
            frames = (ordered[middle - 1] + ordered[middle]) / 2
        return frames / FRAME_HZ


def open_port(args):
    return serial.Serial(args.port, args.baud, timeout=args.timeout)


def read_frame(port):
    """Next well-formed frame, or None on timeout / garbage."""
    raw = port.readline()
    if not raw:
        return None
    return parse(raw.decode("ascii", errors="replace").strip())


def run_raw(args):
    with open_port(args) as port:
        while True:
            raw = port.readline()
            if raw:
                sys.stdout.write(raw.decode("ascii", errors="replace"))
                sys.stdout.flush()


def run_check(args):
    """Validate N frames against the contract. Exit status is the verdict."""
    problems = []
    frames = 0
    bad_lines = 0
    started = None
    ctrl = None

    with open_port(args) as port:
        port.reset_input_buffer()
        port.readline()  # discard a possibly-partial first line

        while frames < args.check:
            frame = read_frame(port)
            if frame is None:
                bad_lines += 1
                if bad_lines > args.check:
                    sys.exit(
                        "no valid frames -- wrong port, wrong baud, or the "
                        "bitstream is not loaded"
                    )
                continue

            ctrl, widths = frame
            if started is None:
                started = time.monotonic()
            frames += 1
            for problem in check_widths(widths):
                problems.append(f"frame {frames}: {problem}")

    elapsed = time.monotonic() - started
    measured_hz = (frames - 1) / elapsed if elapsed > 0 else 0.0
    hz_error = abs(measured_hz - FRAME_HZ) / FRAME_HZ * 100

    print(f"frames checked   : {frames}")
    print(f"malformed lines  : {bad_lines}")
    print(f"controls         : {unpack_ctrl(ctrl)}")
    print(f"frame rate       : {measured_hz:.3f} Hz "
          f"(expected {FRAME_HZ:.3f}, error {hz_error:.2f}%)")

    # 2% covers host scheduling jitter over a few seconds. A wrong clock
    # would show up as 20% (12/10), not 2%.
    rate_ok = hz_error < 2.0
    if not rate_ok:
        print("  frame rate is wrong -- the MMCM is not producing 10 MHz")

    if problems:
        print(f"\n{len(problems)} contract violations:")
        for problem in problems[:20]:
            print(f"  {problem}")
        if len(problems) > 20:
            print(f"  ... and {len(problems) - 20} more")

    if problems or not rate_ok:
        print("\nFAIL")
        return 1
    print("\nPASS -- pulse contract holds and the timebase is correct")
    return 0


def run_sweep(args):
    """Step through all 16 speed settings and time the real wave period.

    This is the hardware version of BUILD-PLAN stage 6: it proves every entry
    of kwe_speed_rom produces the period it claims, on real silicon-adjacent
    hardware rather than in simulation.
    """
    print("Measuring the wave period at each speed setting.")
    print("Slow settings need several cycles, so raise --max-wait if the top "
          "of the table comes back unmeasured.\n")
    print(f"{'speed':>5}  {'measured':>10}  {'nominal':>10}  {'error':>8}  "
          f"{'samples':>7}")
    print("-" * 50)

    failures = 0
    unmeasured = 0

    with open_port(args) as port:
        port.reset_input_buffer()

        frame = None
        while frame is None:
            frame = read_frame(port)
        current = unpack_ctrl(frame[0])

        # Reversal every 6 cycles instead of the default 2: the interval that
        # spans a reversal is not a period, so fewer reversals means more
        # clean samples. The median rejects the rest.
        for _ in range((3 - current["reverse"]) % 4):
            port.write(b"r")
            time.sleep(0.05)

        # Walk to speed 0 so the sweep starts from a known setting.
        for _ in range((0 - current["speed"]) % 16):
            port.write(b"s")
            time.sleep(0.05)

        for setting in range(16):
            if setting > 0:
                port.write(b"s")
                time.sleep(0.05)

            port.write(b"x")  # reset, so every setting starts from centre
            time.sleep(0.7)  # ride out the 500 ms startup hold
            port.reset_input_buffer()

            tracker = PeriodTracker()
            deadline = time.monotonic() + args.max_wait
            frame_index = 0
            actual = setting

            while time.monotonic() < deadline and tracker.count < 5:
                frame = read_frame(port)
                if frame is None:
                    continue
                ctrl, widths = frame
                actual = unpack_ctrl(ctrl)["speed"]
                tracker.update(clocks_to_pos(widths[0]), frame_index)
                frame_index += 1

            nominal = nominal_period_s(actual)
            measured = tracker.period

            if measured is None:
                unmeasured += 1
                print(f"{actual:>5}  {'--':>10}  {nominal:>9.2f}s  "
                      f"{'--':>8}  {0:>7}   not seen in {args.max_wait:.0f}s")
                continue

            error = (measured - nominal) / nominal * 100
            # One or two samples cannot outvote a reversal artefact, so those
            # are reported but not counted as failures.
            confident = tracker.count >= 3
            off = abs(error) >= args.tolerance
            if off and confident:
                failures += 1
            flag = "  <-- off" if off else ("" if confident else "  (few)")
            print(f"{actual:>5}  {measured:>9.2f}s  {nominal:>9.2f}s  "
                  f"{error:>7.1f}%  {tracker.count:>7}{flag}")

    print()
    if failures:
        print(f"FAIL -- {failures} settings outside +/-{args.tolerance:.0f}% "
              f"of nominal")
        return 1
    if unmeasured:
        print(f"{unmeasured} settings too slow to measure in "
              f"{args.max_wait:.0f}s -- rerun with a larger --max-wait")
    print("PASS -- every measured period matches kwe_speed_rom")
    return 0


def run_live(args):
    tracker = PeriodTracker()
    interactive = sys.stdin.isatty()
    old_termios = None

    port = open_port(args)
    try:
        if interactive:
            old_termios = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())

        print("\x1b[2J", end="")  # clear once; frames redraw in place
        violations = 0
        frames = 0
        last_speed = None

        while True:
            if interactive and select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key == "q":
                    break
                if key in KEYS:
                    port.write(key.encode())
                    if key == "x":
                        tracker.reset()

            frame = read_frame(port)
            if frame is None:
                continue
            ctrl, widths = frame
            frames += 1
            settings = unpack_ctrl(ctrl)
            positions = [clocks_to_pos(w) for w in widths]

            # Intervals measured at the old speed say nothing about the new one.
            if settings["speed"] != last_speed:
                tracker.reset()
                last_speed = settings["speed"]

            tracker.update(positions[0], frames)
            violations += len(check_widths(widths))

            period = tracker.period
            nominal = nominal_period_s(settings["speed"])
            period_text = (
                f"{period:5.2f}s (nominal {nominal:.2f}s)"
                if period else f"    -- (nominal {nominal:.2f}s)"
            )

            out = ["\x1b[H"]  # home, then overwrite
            out.append(
                f"  speed {settings['speed']:>2}   amp {settings['amp']}   "
                f"spread {settings['spread']}   mirror {settings['mirror']}   "
                f"reverse {settings['reverse']}\x1b[K\n"
            )
            out.append(
                f"  frames {frames:<8}  period {period_text}   "
                f"violations {violations}\x1b[K\n\x1b[K\n"
            )
            for channel, (width, pos) in enumerate(zip(widths, positions)):
                out.append(
                    f"  ch{channel}  {clocks_to_us(width):7.1f} us  "
                    f"pos {pos:6.1f}  |{bar(pos)}|\x1b[K\n"
                )
            out.append(f"\x1b[K\n  {KEY_HELP}\x1b[K\n")
            sys.stdout.write("".join(out))
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if old_termios is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_termios)
        port.close()
        print()
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Live view and validation for the Kinematic Wave Engine "
                    "on a Cmod A7-35T."
    )
    parser.add_argument("--port", default="/dev/ttyUSB1",
                        help="serial device (default: %(default)s)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=1.0,
                        help="serial read timeout in seconds")
    parser.add_argument("--raw", action="store_true",
                        help="dump lines verbatim")
    parser.add_argument("--check", type=int, metavar="N",
                        help="validate N frames and exit with a verdict")
    parser.add_argument("--sweep", action="store_true",
                        help="measure the wave period at all 16 speeds")
    parser.add_argument("--max-wait", type=float, default=12.0,
                        help="seconds to wait per speed setting during "
                             "--sweep (default: %(default)s)")
    parser.add_argument("--tolerance", type=float, default=10.0,
                        help="percent error allowed in --sweep "
                             "(default: %(default)s)")
    args = parser.parse_args()

    if args.raw:
        return run_raw(args)
    if args.check:
        return run_check(args)
    if args.sweep:
        return run_sweep(args)
    return run_live(args)


if __name__ == "__main__":
    sys.exit(main())
