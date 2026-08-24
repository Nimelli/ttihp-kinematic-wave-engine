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
chain -- kwe_phase_gen holds the only register in it. Nothing is pipelined
because a whole slot (2.496 ms, ~24000 clocks) is available to settle in.
That is also why all 8 channels can share one datapath instead of eight
copies of it.
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

    // Pin map
    // Those are all tuning parameters to be able to somehow change them slightly
    // instead of hardcoded params
    wire [3:0] speed_sel   = ui_in[4:1];    // 16 wave periods, 20.1 s .. 0.26 s
    wire [1:0] amp_sel     = ui_in[6:5];    // 25 / 50 / 75 / 100 %
    wire       spread_sel  = ui_in[7];      // 0 = full wavelength, 1 = half
    wire       mirror_sel  = uio_in[4];     // 1 = two-ball mirrored mode
    wire [1:0] reverse_sel = uio_in[6:5];   // reverse after 2 / 3 / 4 / 6 cycles

    // ui_in[0] is MODE_SW, reserved for the P1 SPI override. Unused for now.


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
    // Bidirectional pins
    //
    // Every uio is an INPUT in this design
    // ---------------------------------------------------------------
    assign uio_out = 8'h00;
    assign uio_oe  = 8'h00;


    // Unused inputs, listed to keep the linter quiet.
    //   ena         - always 1 while powered
    //   ui_in[0]    - MODE_SW, reserved for P1 SPI
    //   uio_in[3:0] - reserved for P1 SPI (CS/MOSI/SCK)
    //   uio_in[7]   - spare
    //   tick_en     - kwe_servo_pwm derives what it needs from tick_cnt
    wire _unused = &{ena, ui_in[0], uio_in[3:0], uio_in[7], tick_en, 1'b0};

endmodule
