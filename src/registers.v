/*
======================================================================
Copyright (c) 2026 Jeremie W
SPDX-License-Identifier: Apache-2.0

File Name:  registers.v
Author:     Jeremie W (willab.ch)
Brief:      SPI Slave - register file
======================================================================

Flop-based register file: two writable bytes plus a read-only ID.

    addr  name    access  reset   contents
    ----  ------  ------  -----   ----------------------------------------
    0x00  WAVE0   RW      0x38    [3:0] SPEED  [5:4] AMP  [6] SPREAD
                                  [7] MIRROR
    0x01  WAVE1   RW      0x00    [1:0] REVERSE   [7:2] reserved, ignored
    0x7F  ID      RO      0xA5    constant; costs no flops, and gives bring-up
                                  a value that is neither 0x00 (a dead MISO)
                                  nor 0xFF (a floating one). Parked high so the
                                  writable block stays contiguous from 0x00 and
                                  can grow without moving it.

WAVE0's reset value is speed=8, amp=3, spread=0, mirror=0 -- the same
"obviously alive" defaults the FPGA wrapper powers up with. That choice is
load-bearing: it is what the chip runs on if MODE_SW is ever stuck high, so
the reset state of this file has to be a wave you would be happy to ship.

Reserved bits are stored, not masked. Keeping the array uniform keeps this
module trivial, and a byte nothing reads cannot do harm; it just reads back
whatever was written.

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
    input  wire       wr_en,

    // Live contents, for the parameter mux in kwe_top.
    output wire [7:0] wave0,
    output wire [7:0] wave1
);

    localparam N_REGS = 2;              // writable entries, addresses 0..1
    localparam ADDR_W = 1;              // bits needed to index them

    localparam [7:0] ADDR_ID = 8'h7F;
    localparam [7:0] CHIP_ID = 8'hA5;

    // Reset values. speed=8, amp=3, spread=0, mirror=0, reverse=0.
    localparam [7:0] RESET_WAVE0 = 8'h38;
    localparam [7:0] RESET_WAVE1 = 8'h00;

    reg [7:0] mem [0:N_REGS-1];
    integer   i;

    wire addr_in_range = (addr < N_REGS);

    assign wave0 = mem[0];
    assign wave1 = mem[1];

    // ---- read: combinational, guarded ---------------------------------
    // The guard makes the out-of-range case return a defined 0x00 instead of
    // letting an unbound index reach the output. ID is decoded ahead of it and
    // is outside the array, so it is inherently read-only: a write to 0x02
    // fails addr_in_range and is dropped like any other out-of-range write.
    assign rdata = (addr == ADDR_ID) ? CHIP_ID
                 : addr_in_range     ? mem[addr[ADDR_W-1:0]]
                                     : 8'h00;

    // ---- write: synchronous, guarded ----------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Blanket-clear the array, then override the entries that have a
            // non-zero default. Two non-blocking assignments to the same
            // element in one always block are resolved last-one-wins, which is
            // what makes this legal and deterministic -- and it synthesises to
            // a plain reset value per flop, not to any kind of priority logic.
            for (i = 0; i < N_REGS; i = i + 1) begin
                mem[i] <= 8'h00;
            end
            mem[0] <= RESET_WAVE0;
            mem[1] <= RESET_WAVE1;
        end else if (wr_en && addr_in_range) begin
            mem[addr[ADDR_W-1:0]] <= wdata;
        end
    end

endmodule
