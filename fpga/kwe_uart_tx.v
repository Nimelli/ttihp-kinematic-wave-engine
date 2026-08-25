/*
======================================================================
Copyright (c) 2026 Jeremie W
SPDX-License-Identifier: Apache-2.0

File Name:  kwe_uart_tx.v
Author:     Jeremie W (willab.ch)
Brief:      8N1 UART transmitter -- FPGA TEST HARNESS ONLY
======================================================================

*** NOT PART OF THE TAPEOUT. Do not add to info.yaml or test/Makefile. ***

Plain 8N1, no parity, no flow control. `send` is a one-cycle pulse that is
only honoured while `busy` is low; `data` must be stable on that cycle.

CLKS_PER_BIT = clk_hz / baud.  At 10 MHz / 115200 that is 86.8, so 87 is
used -- 114943 baud, -0.22% error. A UART tolerates roughly +/-2% per bit
over 10 bit times, so this has ample margin.
*/

`default_nettype none

module kwe_uart_tx #(
    parameter integer CLKS_PER_BIT = 87
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       send,      // 1-cycle request, ignored while busy
    input  wire [7:0] data,      // sampled on the send cycle
    output reg        tx,        // idles high
    output wire       busy
);

    localparam [1:0] S_IDLE = 2'd0,
                     S_STRT = 2'd1,
                     S_DATA = 2'd2,
                     S_STOP = 2'd3;

    reg [1:0]  state;
    reg [15:0] clk_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  shreg;

    assign busy = (state != S_IDLE);

    // One bit period has elapsed. Compared against CLKS_PER_BIT-1 because the
    // counter starts at 0, so the period is inclusive of both endpoints.
    wire bit_done = (clk_cnt == CLKS_PER_BIT - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            clk_cnt <= 16'd0;
            bit_idx <= 3'd0;
            shreg   <= 8'd0;
            tx      <= 1'b1;          // line idles high
        end else begin
            case (state)
                S_IDLE: begin
                    tx      <= 1'b1;
                    clk_cnt <= 16'd0;
                    bit_idx <= 3'd0;
                    if (send) begin
                        shreg <= data;
                        state <= S_STRT;
                    end
                end

                S_STRT: begin
                    tx <= 1'b0;       // start bit
                    if (bit_done) begin
                        clk_cnt <= 16'd0;
                        state   <= S_DATA;
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                S_DATA: begin
                    tx <= shreg[bit_idx];   // LSB first
                    if (bit_done) begin
                        clk_cnt <= 16'd0;
                        if (bit_idx == 3'd7) begin
                            bit_idx <= 3'd0;
                            state   <= S_STOP;
                        end else begin
                            bit_idx <= bit_idx + 3'd1;
                        end
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                S_STOP: begin
                    tx <= 1'b1;       // stop bit
                    if (bit_done) begin
                        clk_cnt <= 16'd0;
                        state   <= S_IDLE;
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
