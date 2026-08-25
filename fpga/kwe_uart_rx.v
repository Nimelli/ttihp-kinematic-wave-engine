/*
======================================================================
Copyright (c) 2026 Jeremie W
SPDX-License-Identifier: Apache-2.0

File Name:  kwe_uart_rx.v
Author:     Jeremie W (willab.ch)
Brief:      8N1 UART receiver -- FPGA TEST HARNESS ONLY
======================================================================

*** NOT PART OF THE TAPEOUT. Do not add to info.yaml or test/Makefile. ***

Receives single-byte commands from the PC so the wave parameters can be
changed without rebuilding the bitstream (see kwe_fpga_top).

Mid-bit sampling: on a falling edge the counter is armed to HALF a bit
period, which lands the first sample in the middle of the start bit; every
sample after that is a full bit period later, so all eight data bits are
sampled at their centres. That gives the maximum margin against baud
mismatch and edge jitter.

No oversampling filter -- the input comes from an on-board FTDI a few
centimetres away, not a long cable.
*/

`default_nettype none

module kwe_uart_rx #(
    parameter integer CLKS_PER_BIT = 87
) (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,         // asynchronous serial input
    output reg [7:0]  data,
    output reg        valid       // 1-cycle pulse when `data` is fresh
);

    localparam [1:0] S_IDLE = 2'd0,
                     S_STRT = 2'd1,
                     S_DATA = 2'd2,
                     S_STOP = 2'd3;

    // Two-flop synchroniser: `rx` is asynchronous to clk, so it must be
    // resynchronised before any logic looks at it, or metastability can
    // propagate into the state machine.
    reg [1:0] rx_sync;
    wire      rx_s = rx_sync[1];

    reg [1:0]  state;
    reg [15:0] clk_cnt;
    reg [2:0]  bit_idx;
    reg [7:0]  shreg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_sync <= 2'b11;         // idle high, so no phantom start bit at reset
            state   <= S_IDLE;
            clk_cnt <= 16'd0;
            bit_idx <= 3'd0;
            shreg   <= 8'd0;
            data    <= 8'd0;
            valid   <= 1'b0;
        end else begin
            rx_sync <= {rx_sync[0], rx};
            valid   <= 1'b0;

            case (state)
                S_IDLE: begin
                    clk_cnt <= 16'd0;
                    bit_idx <= 3'd0;
                    if (!rx_s) begin              // start bit edge
                        state <= S_STRT;
                    end
                end

                S_STRT: begin
                    // Wait half a bit, then check the line is still low. A
                    // glitch that has already returned high is rejected here
                    // rather than being shifted in as a bogus byte.
                    if (clk_cnt == (CLKS_PER_BIT / 2) - 1) begin
                        clk_cnt <= 16'd0;
                        state   <= rx_s ? S_IDLE : S_DATA;
                    end else begin
                        clk_cnt <= clk_cnt + 16'd1;
                    end
                end

                S_DATA: begin
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt        <= 16'd0;
                        shreg[bit_idx] <= rx_s;   // LSB first
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
                    if (clk_cnt == CLKS_PER_BIT - 1) begin
                        clk_cnt <= 16'd0;
                        state   <= S_IDLE;
                        // Accept the byte only on a valid (high) stop bit.
                        if (rx_s) begin
                            data  <= shreg;
                            valid <= 1'b1;
                        end
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
