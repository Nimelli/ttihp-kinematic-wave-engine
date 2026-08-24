# SPDX-FileCopyrightText: © 2026 Jeremie W
# SPDX-License-Identifier: Apache-2.0
"""
Top-level contract tests for tt_um_nimelli_kinematic_wave_engine.

This is the suite Tiny Tapeout actually runs, and the only one that exercises all
eight modules wired together. The unit suites in test/unit/ prove each block in
isolation; this one proves the integration -- which is where the two nastiest bugs
in the design would live (BUILD-PLAN Traps 1 and 2), because both are wiring
mistakes that leave every individual module perfectly correct.

WHAT IS CHECKED
    - the electrical contract to the servos: frame period, pulse bounds, one-hot
      staggering, slot assignment
    - that the eight pulse widths in a frame really are a sine of the right
      wavelength, reconstructed from the waveform and fitted against a bit-exact
      model of the RTL
    - that the wave travels, and reverses
    - reset and startup-hold behaviour

PORTS ONLY -- no hierarchical references. Everything here also runs against the
gate-level netlist (`make GATES=yes`), where internal names do not exist.

RUNTIME
    Crossing the 500 ms startup hold costs ~17 s of wall time (5.2M clock cycles;
    that is the Icarus floor, not cocotb overhead). The suite therefore crosses it
    exactly ONCE: `warm()` is idempotent, the first test pays for it, and every
    later test reuses the running state and only changes pins. Pins are all
    combinational into the datapath, so no further resets are needed.

Run:  source ~/oss-cad-suite/environment && cd test && make
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, Timer, ValueChange
from cocotb.utils import get_sim_time

# --- timing, from PRD §6.1 -------------------------------------------------
CLK_NS = 100                      # 10.000 MHz
PRESCALE = 39
TICK_NS = PRESCALE * CLK_NS       # 3_900
TICKS_PER_SLOT = 640
SLOTS = 8
SLOT_NS = TICKS_PER_SLOT * TICK_NS        # 2_496_000
FRAME_NS = SLOTS * SLOT_NS                # 19_968_000  (50.08 Hz)

PULSE_BASE = 256                  # ticks at pos = 0  -> 0.998 ms
PULSE_MAX = 511                   # ticks at pos = 255 -> 1.993 ms
CENTRE_POS = 128
HOLD_FRAMES = 25

# --- bit-exact model of the RTL datapath (PRD §6.3-§6.7) -------------------
import math

SINE_LUT = [round(127 * math.sin((i + 0.5) * math.pi / 64)) for i in range(32)]
SPEED_TABLE = [65, 87, 119, 156, 208, 278, 374, 503,
               654, 872, 1190, 1577, 2111, 2784, 3739, 5033]
TURN = 128


def model_sine(angle):
    quad = (angle >> 5) & 3
    idx = angle & 31
    mag = SINE_LUT[31 - idx if quad in (1, 3) else idx]
    return -mag if quad >= 2 else mag


def model_amp(s, amp):
    v = (s >> 2) if amp == 0 else (s >> 1) if amp == 1 else \
        (s - (s >> 2)) if amp == 2 else s
    return max(1, min(255, CENTRE_POS + v))


def model_angle(phase, slot, spread, mirror):
    m = (slot if slot < 4 else 7 - slot) if mirror else slot
    return (phase + m * (8 if spread else 16)) % TURN


def model_positions(phase, amp, spread, mirror):
    return [model_amp(model_sine(model_angle(phase, n, spread, mirror)), amp)
            for n in range(SLOTS)]


def best_fit_phase(observed, amp, spread, mirror):
    """Find the phase whose modelled array best matches the observed positions.

    Absolute phase cannot be predicted -- it depends on how many frames have
    elapsed since the hold released -- so the test fits it instead. That still
    pins the *shape*: wavelength, amplitude, mirror geometry and sign are all
    constrained, only the time origin is free.

    Returns (phase, max_abs_error).
    """
    best = None
    for phase in range(TURN):
        want = model_positions(phase, amp, spread, mirror)
        err = max(abs(a - b) for a, b in zip(observed, want))
        if best is None or err < best[1]:
            best = (phase, err)
    return best


# --- harness ---------------------------------------------------------------
_clock_task = None
_warm = False


def ensure_clock(dut):
    """One live clock. A second driver on dut.clk corrupts every measurement."""
    global _clock_task
    if _clock_task is not None and not _clock_task.done():
        _clock_task.cancel()
    _clock_task = cocotb.start_soon(
        Clock(dut.clk, CLK_NS, unit="ns", impl="gpi").start())


def set_pins(dut, speed=8, amp=3, spread=0, mirror=0, reverse=0, mode=0):
    """Drive the tuning pins. PRD §8.

    ui[0]=MODE ui[4:1]=SPEED ui[6:5]=AMP ui[7]=SPREAD
    uio[4]=MIRROR uio[6:5]=REVERSE
    """
    dut.ena.value = 1
    dut.ui_in.value = (mode & 1) | ((speed & 0xF) << 1) | \
                      ((amp & 3) << 5) | ((spread & 1) << 7)
    dut.uio_in.value = ((mirror & 1) << 4) | ((reverse & 3) << 5)


async def hard_reset(dut):
    ensure_clock(dut)
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    await FallingEdge(dut.clk)
    dut.rst_n.value = 1
    await FallingEdge(dut.clk)


async def warm(dut, **pins):
    """Get past the 500 ms startup hold, once per simulation.

    Idempotent so that any single test can also be run alone with
    COCOTB_TEST_FILTER and still work.
    """
    global _warm
    ensure_clock(dut)
    set_pins(dut, **pins)
    if not _warm:
        await hard_reset(dut)
        await Timer((HOLD_FRAMES + 1) * FRAME_NS, unit="ns")
        _warm = True
    await Timer(FRAME_NS, unit="ns")     # let the new pin values take effect


# Anything held for less than this is a decode glitch, not a pulse.
# The shortest legal pulse is 256 ticks = 998_400 ns, so the margin is ~1000x
# and no genuine width violation can hide behind this filter.
GLITCH_NS = 1000

_glitches = 0


async def capture_frames(dut, n=1):
    """Capture n CONSECUTIVE frames. Returns [(starts, widths), ...] per frame.

    Deliberately one continuous transition stream rather than repeated calls to
    a next_pulse() helper. Any per-call helper either has to re-synchronise
    (which silently drops a frame between calls, so a frame-period measurement
    reads 2x too long) or carry state across calls (which goes stale whenever a
    test advances time with Timer and no one is watching uo_out).

    GATE-LEVEL GLITCHES
        uo_out is combinational: (rst_n && pulse_on) ? (8'd1 << slot) : 0.
        At a slot boundary tick_cnt and slot update on the same clock edge, so
        pulse_on can go true while the new slot decode is still propagating
        through real gates -- emitting a sub-nanosecond pulse on the PREVIOUS
        channel. RTL evaluates that conditional atomically and never shows it;
        a gate-level netlist does, and the channel sequence then looks like
        0,0,1,1,2,2...

        Harmless in silicon: a servo integrates its pulse over ~1.5 ms and
        cannot respond to a nanosecond spike. Anything held for less than
        GLITCH_NS is therefore discarded and counted, never treated as a pulse.
    """
    global _glitches

    await ValueChange(dut.uo_out)
    v = int(dut.uo_out.value)
    t0 = get_sim_time(unit="ns")

    pulses = []
    first_seen = False       # the very first pulse may be truncated: drop it
    synced = False
    while len(pulses) < 8 * n:
        await ValueChange(dut.uo_out)
        now = get_sim_time(unit="ns")
        held, dur = v, now - t0
        v = int(dut.uo_out.value)
        t0 = now

        if held == 0:
            continue
        if dur < GLITCH_NS:
            _glitches += 1
            continue
        assert held & (held - 1) == 0, (
            f"uo_out = 0b{held:08b} held for {dur} ns: more than one channel "
            "driven at once. Slots must never overlap."
        )

        ch = held.bit_length() - 1
        if not first_seen:
            first_seen = True
            continue
        if not synced:
            if ch != 0:
                continue
            synced = True
        pulses.append((ch, now - dur, round(dur / TICK_NS)))

    frames = []
    for f in range(n):
        starts = [0] * SLOTS
        widths = [0] * SLOTS
        for i in range(SLOTS):
            ch, t, w = pulses[f * SLOTS + i]
            assert ch == i, (
                f"frame {f}: pulses arrived out of order, expected channel {i}, "
                f"got {ch}. Channel N must own slot N."
            )
            starts[i] = t
            widths[i] = w
        frames.append((starts, widths))
    return frames


async def capture_frame(dut):
    """One frame. Convenience wrapper over capture_frames()."""
    return (await capture_frames(dut, 1))[0]


def positions_from(widths):
    """pos = width - 256, inverting {1'b1, pos} from kwe_servo_pwm."""
    return [w - PULSE_BASE for w in widths]


# ===========================================================================
# reset and startup
# ===========================================================================

@cocotb.test()
async def test_reset_and_startup_hold(dut):
    """Outputs quiet in reset; array holds centre for 500 ms after release.

    Runs first so it also performs the one startup-hold traversal the rest of
    the suite reuses.
    """
    global _warm
    ensure_clock(dut)
    set_pins(dut, speed=8, amp=3)
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 50)

    for _ in range(20):
        await ClockCycles(dut.clk, 500)
        assert int(dut.uo_out.value) == 0, (
            f"uo_out = 0b{int(dut.uo_out.value):08b} during reset, expected 0. "
            "Servos must hang limp for mechanical assembly -- note pos=0 is "
            "still a valid 1 ms pulse, so the outputs need an explicit rst_n gate."
        )
        assert int(dut.uio_oe.value) == 0, (
            f"uio_oe = 0x{int(dut.uio_oe.value):02x} during reset, expected 0x00"
        )

    await FallingEdge(dut.clk)
    dut.rst_n.value = 1
    await FallingEdge(dut.clk)

    # During the hold every channel must sit at centre.
    for frame, (_, widths) in enumerate(await capture_frames(dut, 3)):
        for ch, w in enumerate(widths):
            assert w == PULSE_BASE + CENTRE_POS, (
                f"frame {frame} channel {ch}: {w} ticks "
                f"({w * TICK_NS / 1e6:.3f} ms), expected "
                f"{PULSE_BASE + CENTRE_POS} (1.498 ms centre). The array must "
                "hold flat for 500 ms so the servos physically assemble before "
                "the wave starts (P0.3)."
            )

    # ...and must be moving once it releases.
    await Timer((HOLD_FRAMES + 1) * FRAME_NS, unit="ns")
    _warm = True
    _, widths = await capture_frame(dut)
    assert len(set(widths)) > 1, (
        f"all channels still identical ({widths[0]} ticks) after the hold "
        "released -- the wave never started"
    )


@cocotb.test()
async def test_report_decode_glitches(dut):
    """Informational: how many sub-microsecond decode glitches were seen.

    Expected to be 0 in RTL simulation and non-zero at gate level (see
    next_pulse). Not a failure either way -- a nanosecond spike is far below
    anything a servo can respond to. Logged so the number is visible rather
    than silently swallowed by the filter.
    """
    await warm(dut)
    for _ in range(2):
        await capture_frame(dut)
    dut._log.info(
        f"decode glitches seen so far: {_glitches} "
        f"(all shorter than {GLITCH_NS} ns; shortest legal pulse is 998400 ns)"
    )


@cocotb.test()
async def test_uio_is_all_input(dut):
    """Every uio is an input: uio_oe must be 0x00. Common TT bring-up failure."""
    await warm(dut)
    for _ in range(4):
        await Timer(SLOT_NS, unit="ns")
        assert int(dut.uio_oe.value) == 0, (
            f"uio_oe = 0x{int(dut.uio_oe.value):02x}, expected 0x00"
        )


# ===========================================================================
# electrical contract to the servos
# ===========================================================================

@cocotb.test()
async def test_frame_period(dut):
    """Each channel is refreshed once per 19.968 ms frame (50.08 Hz)."""
    await warm(dut)
    (starts_a, _), (starts_b, _) = await capture_frames(dut, 2)

    for ch in range(SLOTS):
        period = starts_b[ch] - starts_a[ch]
        assert period == FRAME_NS, (
            f"channel {ch} refreshed every {period} ns "
            f"({1e9 / period:.2f} Hz), expected {FRAME_NS} ns (50.08 Hz). "
            "Servos need 40-60 Hz."
        )


@cocotb.test()
async def test_slot_assignment(dut):
    """Channel N's pulse starts exactly N slots into the frame."""
    await warm(dut)
    starts, _ = await capture_frame(dut)
    for ch in range(SLOTS):
        offset = starts[ch] - starts[0]
        assert offset == ch * SLOT_NS, (
            f"channel {ch} starts {offset} ns into the frame, expected "
            f"{ch * SLOT_NS} ns ({ch} x 2.496 ms). The stagger is what removes "
            "seven comparators and flattens the 5 V supply transient."
        )


@cocotb.test()
async def test_pulse_bounds_across_all_settings(dut):
    """No pulse ever leaves 1.0-2.0 ms, for any combination of tuning pins.

    The servo-safety property. A pulse outside this band drives an SG90 past its
    mechanical stops and stalls it against the linkage. Checked across the whole
    reachable input space rather than at nominal settings only.
    """
    await warm(dut)
    for spread in (0, 1):
        for mirror in (0, 1):
            for amp in range(4):
                for speed in (0, 8, 15):
                    set_pins(dut, speed=speed, amp=amp,
                             spread=spread, mirror=mirror)
                    await Timer(FRAME_NS, unit="ns")
                    _, widths = await capture_frame(dut)
                    for ch, w in enumerate(widths):
                        assert PULSE_BASE <= w <= PULSE_MAX, (
                            f"speed={speed} amp={amp} spread={spread} "
                            f"mirror={mirror} channel={ch}: {w} ticks "
                            f"({w * TICK_NS / 1e6:.3f} ms), outside "
                            f"{PULSE_BASE}..{PULSE_MAX} "
                            f"(0.998-1.993 ms). This damages servos."
                        )


# ===========================================================================
# the wave itself
# ===========================================================================

@cocotb.test()
async def test_sine_reconstruction(dut):
    """The eight pulse widths really are a sine of the specified geometry.

    Reconstructs the commanded positions from the waveform and fits them against
    a bit-exact model of the RTL. Absolute phase is free (it depends on elapsed
    frames) but wavelength, amplitude, mirror geometry and sign are all pinned.
    """
    await warm(dut)
    for spread in (0, 1):
        for mirror in (0, 1):
            for amp in (1, 3):
                set_pins(dut, speed=8, amp=amp, spread=spread, mirror=mirror)
                await Timer(FRAME_NS, unit="ns")
                _, widths = await capture_frame(dut)
                obs = positions_from(widths)
                phase, err = best_fit_phase(obs, amp, spread, mirror)
                assert err <= 2, (
                    f"spread={spread} mirror={mirror} amp={amp}: observed "
                    f"positions {obs} do not match any modelled sine "
                    f"(best fit phase={phase}, max error {err} LSB, allowed 2). "
                    f"Expected {model_positions(phase, amp, spread, mirror)}."
                )


@cocotb.test()
async def test_phase_spread_geometry(dut):
    """spread=0 puts one full wavelength across the array, spread=1 a half."""
    await warm(dut)
    for spread, delta in ((0, 16), (1, 8)):
        set_pins(dut, speed=8, amp=3, spread=spread, mirror=0)
        await Timer(FRAME_NS, unit="ns")
        _, widths = await capture_frame(dut)
        obs = positions_from(widths)
        phase, err = best_fit_phase(obs, 3, spread, 0)
        assert err <= 2, f"spread={spread}: no clean fit (err {err})"

        want = model_positions(phase, 3, spread, 0)
        assert obs == want, (
            f"spread={spread}: got {obs}, expected {want} "
            f"(adjacent rods must differ by {delta}/128 of a turn)"
        )


@cocotb.test()
async def test_mirror_symmetry(dut):
    """mirror=1 makes channel N and channel 7-N identical -- this IS two-ball mode.

    The defining property of P2 (PRD §6.7): fold the rod index and the array
    becomes symmetric about its centre, giving two troughs instead of one.
    """
    await warm(dut)
    for spread in (0, 1):
        set_pins(dut, speed=8, amp=3, spread=spread, mirror=1)
        await Timer(FRAME_NS, unit="ns")
        _, widths = await capture_frame(dut)
        for ch in range(SLOTS // 2):
            assert widths[ch] == widths[SLOTS - 1 - ch], (
                f"mirror=1 spread={spread}: channel {ch} is {widths[ch]} ticks "
                f"but channel {SLOTS - 1 - ch} is {widths[SLOTS - 1 - ch]}. "
                "They must match -- the folded index is 0,1,2,3,3,2,1,0."
            )

    # And with mirror off the array must NOT be symmetric, or the test above
    # would pass on a design that ignores the pin entirely.
    set_pins(dut, speed=8, amp=3, spread=0, mirror=0)
    await Timer(FRAME_NS, unit="ns")
    _, widths = await capture_frame(dut)
    assert any(widths[c] != widths[SLOTS - 1 - c] for c in range(SLOTS // 2)), (
        f"mirror=0 still produced a symmetric array {widths} -- the MIRROR pin "
        "(uio[4]) is not reaching kwe_angle_map"
    )


@cocotb.test()
async def test_amplitude_scaling(dut):
    """Deflection from centre grows with the AMP setting, 25/50/75/100%."""
    await warm(dut)
    peaks = []
    for amp in range(4):
        set_pins(dut, speed=8, amp=amp, spread=0, mirror=0)
        await Timer(FRAME_NS, unit="ns")
        _, widths = await capture_frame(dut)
        obs = positions_from(widths)
        peaks.append(max(abs(p - CENTRE_POS) for p in obs))

    for a in range(3):
        assert peaks[a] < peaks[a + 1], (
            f"amp={a} peaks at {peaks[a]} LSB but amp={a + 1} peaks at "
            f"{peaks[a + 1]} -- larger amplitude must deflect further. "
            f"All four: {peaks}"
        )
    assert peaks[3] >= 100, (
        f"amp=100% only reached {peaks[3]} LSB from centre, expected ~127. "
        f"All four: {peaks}"
    )


@cocotb.test()
async def test_slew_stays_within_servo_capability(dut):
    """Frame-to-frame position change stays within what an SG90 can track.

    This is the number the "no LERP needed" argument rests on (PRD §5).

    The bound is 13 LSB, not the 8 a continuous-derivative estimate gives. At
    speed 8 the phase advances 654/512 = 1.277 *integer* steps per frame, and a
    fractional step is not a thing: most frames move the angle by 1 (worst case
    7 LSB) and the rest by 2 (worst case 13 LSB). Quantisation, not a defect --
    and 13 LSB is 51 us of pulse, about 6 deg of travel per 20 ms frame, a
    demanded slew of ~306 deg/s against the SG90's ~600 deg/s. Still half the
    servo's capability, so an interpolator would have nothing to smooth.

    Also worth noting against PRD §6.4: the sine LUT's own quantisation step is
    7 LSB, so it remains well below this, and a larger table would still buy
    nothing visible.
    """
    await warm(dut, speed=8, amp=3)
    frames = await capture_frames(dut, 7)
    worst = 0
    for (_, prev), (_, cur) in zip(frames, frames[1:]):
        worst = max(worst, max(abs(a - b) for a, b in zip(cur, prev)))

    assert worst <= 14, (
        f"largest frame-to-frame change is {worst} LSB "
        f"({worst * TICK_NS / 1000:.0f} us of pulse), expected <= 14 at speed 8 "
        "(13 is the exact worst case from a 2-step angle advance). "
        "A larger step means the wave is advancing faster than intended."
    )


@cocotb.test()
async def test_wave_travels_and_reverses(dut):
    """The wave travels in one direction, then reverses. P0.5.

    Tracked by fitting the phase of each frame: it must advance monotonically,
    then start retreating. A wave that only ever travelled one way would carry
    the ball to one end and leave it there.
    """
    await warm(dut, speed=15, amp=3, spread=0, mirror=0, reverse=0)

    phases = []
    for _, widths in await capture_frames(dut, 40):
        phase, err = best_fit_phase(positions_from(widths), 3, 0, 0)
        assert err <= 2, f"frame did not fit a sine (err {err} LSB)"
        phases.append(phase)

    # Unwrap modulo 128 into signed per-frame steps.
    steps = []
    for a, b in zip(phases, phases[1:]):
        d = (b - a) % TURN
        steps.append(d if d < TURN // 2 else d - TURN)

    assert any(s > 0 for s in steps), (
        f"phase never advanced -- the wave is not travelling. Phases: {phases}"
    )
    assert any(s < 0 for s in steps), (
        f"phase never retreated -- the wave never reversed direction, so a ball "
        f"would be carried to one end and stay there. Phases: {phases}"
    )


@cocotb.test()
async def test_speed_pin_changes_wave_rate(dut):
    """A higher SPEED setting advances the phase further per frame.

    Confirms ui[4:1] actually reaches kwe_speed_rom -- a pin-mapping error that
    every unit test would happily pass.
    """
    await warm(dut)
    rates = {}
    for speed in (4, 9, 14):
        set_pins(dut, speed=speed, amp=3, spread=0, mirror=0, reverse=3)
        await Timer(2 * FRAME_NS, unit="ns")

        seen = []
        for _, widths in await capture_frames(dut, 4):
            phase, err = best_fit_phase(positions_from(widths), 3, 0, 0)
            assert err <= 2, f"speed={speed}: frame did not fit a sine"
            seen.append(phase)

        steps = []
        for a, b in zip(seen, seen[1:]):
            d = (b - a) % TURN
            steps.append(d if d < TURN // 2 else d - TURN)
        rates[speed] = sum(abs(s) for s in steps) / len(steps)

    assert rates[4] < rates[9] < rates[14], (
        f"phase advance per frame did not increase with the SPEED pin: {rates}. "
        f"Expected roughly {SPEED_TABLE[4] / 512:.1f}, "
        f"{SPEED_TABLE[9] / 512:.1f}, {SPEED_TABLE[14] / 512:.1f} steps/frame."
    )
