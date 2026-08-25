/*
======================================================================
Copyright (c) 2026 Jeremie W
SPDX-License-Identifier: Apache-2.0

File Name:  kwe_pulse_monitor.v
Author:     Jeremie W (willab.ch)
Brief:      Measure the 8 servo pulse widths -- FPGA TEST HARNESS ONLY
======================================================================

*** NOT PART OF THE TAPEOUT. Do not add to info.yaml or test/Makefile. ***

This is the substitute for an oscilloscope. It watches uo_out and times how
long each channel is high, in 10 MHz clock cycles.

Expected values -- pulse = (256 + pos) ticks, 1 tick = 39 clk:

    pos =   0  ->  256 * 39 =  9984 clk =  998.4 us
    pos = 128  ->  384 * 39 = 14976 clk = 1497.6 us   (centre / startup hold)
    pos = 255  ->  511 * 39 = 19929 clk = 1992.9 us

so the position can be recovered exactly:  pos = width/39 - 256.

The measurement leans on two guarantees from kwe_servo_pwm: the outputs are
strictly one-hot, and every pulse is separated from the next by at least
129 ticks (503 us) of all-low. So a single shared counter is enough -- there
is never more than one pulse in flight.

The one-clock output register added to kwe_servo_pwm delays both edges
equally, so the measured width is unaffected by it.
*/

`default_nettype none

module kwe_pulse_monitor (
    input  wire         clk,
    input  wire         rst_n,
    input  wire  [7:0]  pwm,          // uo_out from the DUT
    output reg  [127:0] widths,       // 8 x 16 bit, channel N at [N*16 +: 16]
    output reg          frame_done    // 1-cycle pulse after channel 7 is measured
);

    reg [15:0] cnt;      // clocks elapsed in the pulse currently being measured
    reg [2:0]  idx;      // which channel that pulse belongs to
    reg        any_r;    // `any` delayed one clock, to find the falling edge

    wire any = |pwm;

    // One-hot to binary. `default` cannot occur while `any` is high given the
    // one-hot guarantee, and idx is only used when it was high.
    reg [2:0] idx_c;
    always @* begin
        case (pwm)
            8'b0000_0001: idx_c = 3'd0;
            8'b0000_0010: idx_c = 3'd1;
            8'b0000_0100: idx_c = 3'd2;
            8'b0000_1000: idx_c = 3'd3;
            8'b0001_0000: idx_c = 3'd4;
            8'b0010_0000: idx_c = 3'd5;
            8'b0100_0000: idx_c = 3'd6;
            8'b1000_0000: idx_c = 3'd7;
            default:      idx_c = 3'd0;
        endcase
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt        <= 16'd0;
            idx        <= 3'd0;
            any_r      <= 1'b0;
            widths     <= 128'd0;
            frame_done <= 1'b0;
        end else begin
            any_r      <= any;
            frame_done <= 1'b0;

            if (any) begin
                cnt <= cnt + 16'd1;
                idx <= idx_c;
            end else begin
                cnt <= 16'd0;
            end

            // Falling edge: `cnt` still holds the full width, because the
            // non-blocking assignment above has not taken effect yet.
            if (any_r && !any) begin
                widths[idx*16 +: 16] <= cnt;
                if (idx == 3'd7) begin
                    frame_done <= 1'b1;
                end
            end
        end
    end

endmodule

`default_nettype wire
