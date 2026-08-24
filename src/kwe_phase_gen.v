/*
======================================================================
Copyright (c) 2026 Jeremie W
SPDX-License-Identifier: Apache-2.0

File Name:  kwe_phase_gen.v
Author:     Jeremie W (willab.ch)
Brief:      Produce the time sine wave --> phase 0...127
======================================================================

A counter would not give enough time resolution. Need to use a 16b accumulator
and take only the 7 MSB.

The low 9 bits are FRACTIONAL phase. They never leave this module, but they are
what buys fine speed control: one whole step of `phase` costs 512 accumulator
units, so an increment smaller than 512 advances `phase` only every few frames.

    wave period = 65536 / inc  frames        (frame = 19.968 ms)

    inc =   65  -> 1008 frames -> 20.1 s   (phase holds ~8 frames per step)
    inc =  654  ->  100 frames ->  2.0 s   (phase steps by 1 or 2)
    inc = 5033  ->   13 frames -> 0.26 s   (phase jumps ~10 per frame)

This is a phase accumulator (an NCO): the standard way to build a
frequency-controlled oscillator in digital logic.

Three behaviours live here:
  1. accumulate  -- advance the wave once per frame
  2. reverse     -- flip direction after N complete cycles  (PRD 6.5)
  3. hold        -- freeze everything for 500 ms after reset  (PRD P0.3)
*/

`default_nettype none

module kwe_phase_gen (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        phase_tick,   // advance the accumulator, once per frame
    input  wire  [3:0] speed_sel,    // ui[4:1]  -> kwe_speed_rom
    input  wire  [1:0] reverse_sel,  // uio[6:5] -> 2 / 3 / 4 / 6 wave cycles
    output wire  [6:0] phase,        // acc[15:9]
    output wire        hold          // 1 during the 500 ms startup centre-hold
);

    // 25 frames x 19.968 ms = 499.2 ms
    localparam [4:0] HOLD_FRAMES = 5'd25;

    wire [12:0] inc;

    kwe_speed_rom u_speed_rom (
        .sel(speed_sel),
        .inc(inc)
    );


    // ---------------------------------------------------------------
    // Startup hold
    //
    // On power-up the servo horns are wherever they were last left. If the
    // wave started immediately, eight servos would slam from arbitrary
    // positions at once. `hold` tells the top level to command centre instead,
    // and freezes the accumulator so the wave does not silently advance while
    // the array is still assembling.
    // ---------------------------------------------------------------
    reg [4:0] startup_cnt;

    assign hold = (startup_cnt != HOLD_FRAMES);


    // ---------------------------------------------------------------
    // Reversal period, from the REVERSE[1:0] pins
    //
    // 2 cycles is the reset default: mechanical simulation showed the ball
    // crosses the track in about 2 wave cycles, so anything larger leaves it
    // parked at the end doing nothing.
    // ---------------------------------------------------------------
    reg [2:0] rev_limit;

    always @(*) begin
        case (reverse_sel)
            2'b00:   rev_limit = 3'd2;   // default
            2'b01:   rev_limit = 3'd3;
            2'b10:   rev_limit = 3'd4;
            default: rev_limit = 3'd6;
        endcase
    end


    // ---------------------------------------------------------------
    // The accumulator
    //
    // Computed in 17 bits so that bit 16 is the carry-out when going forward,
    // and the borrow when going backward. Either way it is set exactly when
    // the 16-bit accumulator completed a full turn, so one spare bit gives us
    // cycle detection for free -- no comparator needed.
    // ---------------------------------------------------------------
    reg  [15:0] acc;
    reg         fwd;      // 1 = wave travels one way, 0 = the other
    reg   [2:0] cycles;   // complete turns since the last reversal

    wire [16:0] acc_next = fwd ? ({1'b0, acc} + {4'b0, inc})
                               : ({1'b0, acc} - {4'b0, inc});

    wire wrapped = acc_next[16];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            startup_cnt <= 5'd0;
            acc         <= 16'd0;
            fwd         <= 1'b1;
            cycles      <= 3'd0;
        end else if (phase_tick) begin
            if (hold) begin
                // Freeze the wave; just count off the startup frames.
                startup_cnt <= startup_cnt + 5'd1;
            end else begin
                acc <= acc_next[15:0];

                if (wrapped) begin
                    if (cycles + 3'd1 >= rev_limit) begin
                        cycles <= 3'd0;
                        // Negate the increment -- do NOT reset acc. Phase stays
                        // continuous through the reversal, so no rod jumps; only
                        // the direction of travel changes.
                        fwd    <= ~fwd;
                    end else begin
                        cycles <= cycles + 3'd1;
                    end
                end
            end
        end
    end

    // The bottom 9 bits are fractional and stay internal.
    assign phase = acc[15:9];

endmodule
