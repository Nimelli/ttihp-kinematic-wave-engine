/*
======================================================================
Copyright (c) 2026 Jeremie W
SPDX-License-Identifier: Apache-2.0

File Name:  kwe_servo_pwm.v
Author:     Jeremie W (willab.ch)
Brief:      Servo pulse output stage
======================================================================

 *   pos_r <= pos_next            on slot_start
 *   servo_pwm = (tick_cnt < 256 + pos_r) ? (1 << slot) : 0
 *
 * Pulse length is 256 + pos ticks, one channel at a time:
 *   pos =   0 -> 256 ticks -> 0.998 ms   (one end of servo travel)
 *   pos = 128 -> 384 ticks -> 1.498 ms   (centre)
 *   pos = 255 -> 511 ticks -> 1.993 ms   (other end)
 *
 * A slot is 640 ticks (2.496 ms), so even the longest pulse leaves about 0.5 ms 
 * of gap before the next channel's turn.
 */

`default_nettype none

module kwe_servo_pwm (
    input  wire         clk,
    input  wire         rst_n,
    input  wire         slot_start,   // high on the last tick of the previous slot
    input  wire [7:0]   pos_next,     // position computed for the slot about to begin
    input  wire [9:0]   tick_cnt,     // 0..639, position within the current slot
    input  wire [2:0]   slot,         // 0..7, channel owning the current slot
    output wire [7:0]   servo_pwm     // one-hot pulse train to the 8 servos
);

    // The only state in this module: the position for the slot currently running.
    // Captured once at the slot boundary and held, so the pulse width cannot
    // change midway through a pulse the servo is still measuring.
    reg [7:0] pos_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos_r <= 8'd128;            // centre, matches the startup hold at top level
        end else if (slot_start) begin
            pos_r <= pos_next;
        end
    end

    // {1'b1, pos_r} *is* 256 + pos_r
    // The extra 1'b0 is to match tick_cnt len of 10b
    wire pulse_on = (tick_cnt < {1'b0, 1'b1, pos_r});

    // Drive only the channel owning this slot; every other output stays low.
    assign servo_pwm = (rst_n && pulse_on) ? (8'd1 << slot) : 8'd0;

endmodule
