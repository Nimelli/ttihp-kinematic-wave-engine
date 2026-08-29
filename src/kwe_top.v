/*
======================================================================
Copyright (c) 2026 Jeremie W
SPDX-License-Identifier: Apache-2.0

File Name:  kwe_top.v
Author:     Jeremie W (willab.ch)
Brief:      Kinematic Wave Engine -- Tiny Tapeout top level
======================================================================

Almost pure wiring. The chip is two halves joined by kwe_timebase:

    TIME                     WAVE DATAPATH                  OUTPUT
    kwe_timebase  --slot_nxt--> angle_map -> sine -> amp -->  mux --> servo_pwm
                  --phase_tick-> phase_gen -^                  ^
                  --tick_cnt/slot/slot_start ------------------|-------^
                                              hold ------------|

The datapath from `phase` all the way to `pos` is one long combinational
chain -- kwe_phase_gen holds the only register in it. It is not pipelined
because it does not need to be: it is only *sampled* once per slot, when
kwe_servo_pwm latches pos_next on slot_start.

CAREFUL: "sampled once per slot" is a functional statement, not a timing one.
Static timing analysis assumes single-cycle paths by default, so this chain
must still settle within ONE CLOCK PERIOD (signed off at 20 ns, config.json
CLOCK_PERIOD) -- not within a slot. It has roughly 40 levels of logic, a few
ns at 130 nm, so it meets that with a wide margin and needs no multicycle
constraint. But do not read the slot period as available slack.

Sharing one datapath across all 8 channels is possible for the same reason:
only one channel is ever active at a time (PRD 6.2).
*/

`default_nettype none

module tt_um_nimelli_kinematic_wave_engine (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered
    input  wire       clk,      // 10.000 MHz
    input  wire       rst_n     // active-low reset
);

    // ---------------------------------------------------------------
    // Parameter source: pins (auto) or SPI registers
    //
    //   MODE_SW = 0   AUTO -- parameters come from the pins, exactly as they
    //                 did before the SPI link existed. This is the default:
    //                 an undriven ui_in reads 0 on the TT harness.
    //   MODE_SW = 1   SPI  -- parameters come from the register file.
    //
    // Three properties make this safe to add to a chip whose priority is
    // auto-mode:
    //
    // 1. The select is a PAD, not a register. Nothing reachable over SPI
    //    touches it, so with MODE_SW low the whole SPI block is a spectator:
    //    its registers can hold any value at all and the wave engine behaves
    //    exactly as before. test_spi_registers_do_not_affect_auto_mode checks
    //    exactly that: it writes junk into every register and requires the
    //    array to keep both the SHAPE (amp/spread/mirror, fitted against a
    //    model of the pin settings) and the RATE (speed, from how far the
    //    phase advances per frame) that the pins asked for.
    //
    // 2. Every SPI-sourced field is a bit-slice of the SAME WIDTH as the pin
    //    it replaces, so the two sources span an identical value space. There
    //    is no setting reachable over SPI that the pins could not already
    //    reach -- kwe_speed_rom decodes all 16 speed codes, amp all 4, and so
    //    on. test_pulse_bounds_across_all_settings already sweeps that space.
    //
    // 3. The reset state of the register file is a wave worth shipping
    //    (speed=8, amp=3), so even a MODE_SW stuck high with no master
    //    attached leaves the chip running a good-looking pattern rather than
    //    a degenerate one.
    //
    // MODE_SW is used unsynchronised, like every other parameter pin here. A
    // slow or bouncing edge can only mix pin-sourced and SPI-sourced fields
    // for a cycle, and by (2) every such mixture is itself a legal setting.
    // ---------------------------------------------------------------
    wire       mode_spi = ui_in[0];         // MODE_SW

    wire [7:0] spi_wave0;                   // reg 0x00, see registers.v
    wire [1:0] spi_wave1;                   // reg 0x01, REVERSE only

    wire [3:0] speed_sel   = mode_spi ? spi_wave0[3:0] : ui_in[4:1];   // 16 wave periods, 20.1 s .. 0.26 s
    wire [1:0] amp_sel     = mode_spi ? spi_wave0[5:4] : ui_in[6:5];   // 25 / 50 / 75 / 100 %
    wire       spread_sel  = mode_spi ? spi_wave0[6]   : ui_in[7];     // 0 = full wavelength, 1 = half
    wire       mirror_sel  = mode_spi ? spi_wave0[7]   : uio_in[4];    // 1 = two-ball mirrored mode
    wire [1:0] reverse_sel = mode_spi ? spi_wave1      : uio_in[6:5];  // reverse after 2 / 3 / 4 / 6 cycles


    // ---------------------------------------------------------------
    // Timebase -- the only sequencer in the design
    // ---------------------------------------------------------------
    wire       tick_en;
    wire [9:0] tick_cnt;
    wire [2:0] slot;
    wire [2:0] slot_nxt;
    wire       slot_start;
    wire       phase_tick;

    kwe_timebase u_timebase (
        .clk        (clk),
        .rst_n      (rst_n),
        .tick_en    (tick_en),
        .tick_cnt   (tick_cnt),
        .slot       (slot),
        .slot_nxt   (slot_nxt),
        .slot_start (slot_start),
        .phase_tick (phase_tick)
    );


    // ---------------------------------------------------------------
    // Wave datapath
    // ---------------------------------------------------------------
    wire [6:0]        phase;
    wire              hold;
    wire [6:0]        angle;
    wire signed [8:0] sine;
    wire [7:0]        pos_wave;

    kwe_phase_gen u_phase_gen (
        .clk         (clk),
        .rst_n       (rst_n),
        .phase_tick  (phase_tick),
        .speed_sel   (speed_sel),
        .reverse_sel (reverse_sel),
        .phase       (phase),
        .hold        (hold)
    );

    // NOTE: driven by slot_nxt, NOT slot.
    //
    // kwe_servo_pwm latches this position on slot_start, and a register
    // captures the value present BEFORE the clock edge, at which point
    // `slot` still reads the slot that is ending. Feeding `slot` here would
    // make every channel drive its neighbour's rod.
    kwe_angle_map u_angle_map (
        .phase  (phase),
        .slot   (slot_nxt),
        .spread (spread_sel),
        .mirror (mirror_sel),
        .angle  (angle)
    );

    kwe_sine u_sine (
        .angle (angle),
        .sine  (sine)
    );

    kwe_amp_scale u_amp_scale (
        .sine (sine),
        .amp  (amp_sel),
        .pos  (pos_wave)
    );

    // Startup centre-hold: for the first 500 ms after reset the array is
    // commanded flat (1.5 ms pulse) so the servos physically assemble before
    // any wave motion starts. kwe_phase_gen freezes the accumulator over the
    // same window, so the wave has not silently advanced by the time it
    // releases.
    wire [7:0] pos_next = hold ? 8'd128 : pos_wave;


    // ---------------------------------------------------------------
    // Output stage
    // ---------------------------------------------------------------
    wire [7:0] servo_pwm;

    kwe_servo_pwm u_servo_pwm (
        .clk        (clk),
        .rst_n      (rst_n),
        .slot_start (slot_start),
        .pos_next   (pos_next),
        .tick_cnt   (tick_cnt),
        .slot       (slot),
        .servo_pwm  (servo_pwm)
    );

    assign uo_out = servo_pwm;


    // ---------------------------------------------------------------
    // SPI slave (P1)
    //
    //   uio[0]  SPI_CS    in
    //   uio[1]  SPI_SCK   in
    //   uio[2]  SPI_MOSI  in
    //   uio[3]  SPI_MISO  out -- the only uio this design ever drives
    //
    // Mode 0, and SCK is oversampled rather than used as a clock, so the
    // master must stay at or below clk/4 = 2.5 MHz. See spis_synchro.v.
    //
    // The register file drives the wave parameters only when MODE_SW is high;
    // see the parameter mux at the top of this file.
    // ---------------------------------------------------------------
    wire spi_miso;
    wire spi_miso_oe;

    spis_top u_spis (
        .clk         (clk),
        .rst_n       (rst_n),

        .spi_clk     (uio_in[1]),
        .spi_mosi    (uio_in[2]),
        .spi_cs      (uio_in[0]),

        .spi_miso    (spi_miso),
        .spi_miso_oe (spi_miso_oe),

        .wave0       (spi_wave0),
        .wave1       (spi_wave1)
    );


    // ---------------------------------------------------------------
    // Bidirectional pins
    //
    // uio[3] is an output while the slave is selected, Hi-Z otherwise, so a
    // master can share the bus. Every other uio stays an input.
    // ---------------------------------------------------------------
    assign uio_out = {4'b0000, spi_miso,    3'b000};
    assign uio_oe  = {4'b0000, spi_miso_oe, 3'b000};


    // Unused inputs, listed to keep the linter quiet.
    //   ena       - always 1 while powered
    //   uio_in[3] - MISO is an output; the input leg of the pad is unused
    //   uio_in[7] - spare
    //   tick_en   - kwe_servo_pwm derives what it needs from tick_cnt
    wire _unused = &{ena, uio_in[3], uio_in[7], tick_en, 1'b0};

endmodule
