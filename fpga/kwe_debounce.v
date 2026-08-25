/*
======================================================================
Copyright (c) 2026 Jeremie W
SPDX-License-Identifier: Apache-2.0

File Name:  kwe_debounce.v
Author:     Jeremie W (willab.ch)
Brief:      Button synchroniser + debouncer -- FPGA TEST HARNESS ONLY
======================================================================

*** NOT PART OF THE TAPEOUT. Do not add to info.yaml or test/Makefile. ***

The Cmod A7 buttons are mechanical and bounce for a few milliseconds. The
input is resynchronised, then a candidate value must hold steady for
STABLE_CLKS before it is accepted.

100_000 clocks at 10 MHz = 10 ms, longer than typical tactile switch bounce.
*/

`default_nettype none

module kwe_debounce #(
    parameter integer STABLE_CLKS = 100000
) (
    input  wire clk,
    input  wire rst_n,
    input  wire noisy,
    output reg  level,   // debounced level
    output reg  rise     // 1-cycle pulse on a clean 0 -> 1 transition
);

    reg [1:0]  sync;
    reg [19:0] cnt;

    wire in = sync[1];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync  <= 2'b00;
            cnt   <= 20'd0;
            level <= 1'b0;
            rise  <= 1'b0;
        end else begin
            sync <= {sync[0], noisy};
            rise <= 1'b0;

            if (in == level) begin
                cnt <= 20'd0;                 // agrees with the accepted value
            end else if (cnt == STABLE_CLKS - 1) begin
                cnt   <= 20'd0;
                level <= in;
                if (in) begin
                    rise <= 1'b1;
                end
            end else begin
                cnt <= cnt + 20'd1;
            end
        end
    end

endmodule

`default_nettype wire
