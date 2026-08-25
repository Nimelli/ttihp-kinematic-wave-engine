/*
======================================================================
Copyright (c) 2026 Jeremie W
SPDX-License-Identifier: Apache-2.0

File Name:  kwe_report.v
Author:     Jeremie W (willab.ch)
Brief:      Format measurements as an ASCII line -- FPGA TEST HARNESS ONLY
======================================================================

*** NOT PART OF THE TAPEOUT. Do not add to info.yaml or test/Makefile. ***

Emits one line per trigger:

    W cccc wwww wwww wwww wwww wwww wwww wwww wwww\r\n

field 0  = control word (see kwe_fpga_top for the packing)
field 1..8 = channel 0..7 pulse width in 10 MHz clocks, uppercase hex

49 bytes at 115200 baud is 4.25 ms, comfortably inside the 19.968 ms frame,
so a line per frame never backs up.

Hex rather than decimal deliberately: decimal would need a binary-to-BCD
conversion for no benefit, since the receiving script parses it either way.
*/

`default_nettype none

module kwe_report #(
    parameter integer NFIELDS = 9
) (
    input  wire                    clk,
    input  wire                    rst_n,
    input  wire                    trigger,   // 1-cycle: snapshot and send
    input  wire [NFIELDS*16-1:0]   fields,    // field N at [N*16 +: 16]

    output reg                     tx_send,
    output reg  [7:0]              tx_data,
    input  wire                    tx_busy,
    output wire                    busy
);

    localparam [2:0] S_IDLE = 3'd0,
                     S_HDR  = 3'd1,
                     S_SP   = 3'd2,
                     S_FLD  = 3'd3,
                     S_CR   = 3'd4,
                     S_LF   = 3'd5;

    reg [2:0]                  state;
    reg [3:0]                  fld;     // 0 .. NFIELDS-1
    reg [2:0]                  sub;     // 0..3 nibbles, 4 = separator
    reg [NFIELDS*16-1:0]       snap;    // frozen copy, so `fields` may move

    assign busy = (state != S_IDLE);

    // Nibble currently being emitted, most significant first. A case rather
    // than a variable shift: `sub` reaches 4 on the separator cycle, and a
    // shift of (3 - 4) would go negative before the value is discarded.
    wire [15:0] word = snap[fld*16 +: 16];

    reg [3:0] nib;
    always @* begin
        case (sub[1:0])
            2'd0:    nib = word[15:12];
            2'd1:    nib = word[11:8];
            2'd2:    nib = word[7:4];
            default: nib = word[3:0];
        endcase
    end

    wire [7:0] nib_ch = (nib < 4'd10) ? (8'd48 + {4'd0, nib})          // '0'..'9'
                                      : (8'd65 + {4'd0, nib} - 8'd10); // 'A'..'F'

    // A byte is handed over only when the UART is free. tx_send is also
    // checked so the one cycle where send is asserted but busy has not yet
    // risen cannot issue a second byte.
    wire can_send = !tx_busy && !tx_send;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            fld     <= 4'd0;
            sub     <= 3'd0;
            snap    <= {(NFIELDS*16){1'b0}};
            tx_send <= 1'b0;
            tx_data <= 8'd0;
        end else begin
            tx_send <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (trigger) begin
                        snap  <= fields;
                        fld   <= 4'd0;
                        sub   <= 3'd0;
                        state <= S_HDR;
                    end
                end

                S_HDR: if (can_send) begin
                    tx_data <= 8'd87;            // 'W'
                    tx_send <= 1'b1;
                    state   <= S_SP;
                end

                S_SP: if (can_send) begin
                    tx_data <= 8'd32;            // ' '
                    tx_send <= 1'b1;
                    state   <= S_FLD;
                end

                S_FLD: if (can_send) begin
                    tx_send <= 1'b1;
                    if (sub == 3'd4) begin
                        // Separator after each field: space, except the last
                        // field which runs straight into the CR/LF.
                        tx_data <= 8'd32;        // ' '
                        sub     <= 3'd0;
                        if (fld == NFIELDS - 1) begin
                            state <= S_CR;
                        end else begin
                            fld <= fld + 4'd1;
                        end
                    end else begin
                        tx_data <= nib_ch;
                        sub     <= sub + 3'd1;
                    end
                end

                S_CR: if (can_send) begin
                    tx_data <= 8'd13;            // '\r'
                    tx_send <= 1'b1;
                    state   <= S_LF;
                end

                S_LF: if (can_send) begin
                    tx_data <= 8'd10;            // '\n'
                    tx_send <= 1'b1;
                    state   <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire
