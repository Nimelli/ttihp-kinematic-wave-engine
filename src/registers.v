/*
======================================================================
Copyright (c) 2026 Jeremie W
SPDX-License-Identifier: Apache-2.0

File Name:  registers.v
Author:     Jeremie W (willab.ch)
Brief:      SPI Slave - register file
======================================================================

Flop-based register file: N_REGS bytes, combinational read, synchronous write.

Two things this has to get right that a testbench-only register file does not:

1. Reset, not `initial`. An initial block sets the array up in Icarus and is
   silently dropped or only partially honoured on the way to silicon. Untouched
   flops then power up at whatever the process gives them, and rdata goes
   straight to a pad through spis_phy. Every entry gets a reset value here.

2. A range guard. addr is 8 bits but only N_REGS entries exist, so mem[addr]
   for addr >= N_REGS is an out-of-bounds read: X in simulation, and an
   unpredictable mux output in synthesis. Out-of-range reads return 0x00 and
   out-of-range writes are dropped.
*/

`default_nettype none

module reg_file (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] addr,
    output wire [7:0] rdata,
    input  wire [7:0] wdata,
    input  wire       wr_en
);

    localparam N_REGS = 3;         // mem[0] ..
    localparam ADDR_W = 4;         // bits needed to index N_REGS entries

    // Power-up / reset values. Anything not listed defaults to 0x00.
    localparam [7:0] DEFAULT_00 = 8'hA5;
    localparam [7:0] DEFAULT_01 = 8'h3C;
    localparam [7:0] DEFAULT_02 = 8'hFF;

    reg [7:0] mem [0:N_REGS-1];
    integer   i;

    wire addr_in_range = (addr < N_REGS);

    // ---- read: combinational, guarded ---------------------------------
    // The guard makes the out-of-range case return a defined 0x00 instead of
    // letting an unbound index reach the output.
    assign rdata = addr_in_range ? mem[addr[ADDR_W-1:0]] : 8'h00;

    // ---- write: synchronous, guarded ----------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Blanket-clear the array, then override the few entries that have
            // a non-zero default. Two non-blocking assignments to the same
            // element in one always block are resolved last-one-wins, which is
            // what makes this legal and deterministic -- and it synthesises to
            // a plain reset value per flop, not to any kind of priority logic.
            for (i = 0; i < N_REGS; i = i + 1) begin
                mem[i] <= 8'h00;
            end
            mem[0] <= DEFAULT_00;
            mem[1] <= DEFAULT_01;
            mem[2] <= DEFAULT_02;
        end else if (wr_en && addr_in_range) begin
            mem[addr[ADDR_W-1:0]] <= wdata;
        end
    end

endmodule
