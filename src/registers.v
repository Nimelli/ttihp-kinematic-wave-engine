/*
======================================================================
Copyright (c) 2026 Jeremie W
SPDX-License-Identifier: Apache-2.0

File Name:  registers.v
Author:     Jeremie W (willab.ch)
Brief:      SPI Slave - register file
======================================================================

Two writable registers plus a read-only ID.

    addr  name    access  reset   contents
    ----  ------  ------  -----   ----------------------------------------
    0x00  WAVE0   RW      0x38    [3:0] SPEED  [5:4] AMP  [6] SPREAD
                                  [7] MIRROR
    0x01  WAVE1   RW      0x00    [1:0] REVERSE   [7:2] reserved, read 0
    0x7F  ID      RO      0xA5    constant; costs no flops, and gives bring-up
                                  a value that is neither 0x00 (a dead MISO)
                                  nor 0xFF (a floating one). Parked high so the
                                  writable block stays contiguous from 0x00 and
                                  can grow without moving it.

WAVE0's reset value is speed=8, amp=3, spread=0, mirror=0 -- the same
"obviously alive" defaults the FPGA wrapper powers up with. That choice is
load-bearing: it is what the chip runs if MODE_SW is ever stuck high, so the
reset state of this file has to be a wave you would be happy to ship.

NAMED REGISTERS, NOT AN ARRAY

The obvious shape here is `reg [7:0] mem [0:1]` indexed by addr. It was written
that way first, and it cost six flops: WAVE1 only has two real bits, but an
array is uniform, so [7:2] became storage that is clocked and reset every cycle
and read by nothing.

Masking the write (`mem[i] <= wdata & mask`) does not fix it. Synthesis cannot
prove those bits are constant, because the write index is computed at runtime --
it has no way to know the mask is the narrow one exactly when the index is 1.
The flops survive and the mask logic is added on top.

Declaring each register at its true width is what actually removes them. It also
drops the out-of-range index guard the array needed, makes reserved bits read
back 0 structurally rather than by convention, and stops yosys having to report
"Replacing memory \mem with list of registers" at all.

Reset, not `initial`: an initial block sets flops up in Icarus and is silently
dropped or only partially honoured on the way to silicon. Untouched flops then
power up at whatever the process gives them, and rdata goes straight to a pad
through spis_phy.
*/

`default_nettype none

module reg_file (
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] addr,
    output wire [7:0] rdata,
    input  wire [7:0] wdata,
    input  wire       wr_en,

    // Live contents, for the parameter mux in kwe_top. Declared at the width
    // that is actually used, so no consumer has to ignore padding.
    output wire [7:0] wave0,
    output wire [1:0] wave1
);

    localparam [7:0] ADDR_WAVE0 = 8'h00;
    localparam [7:0] ADDR_WAVE1 = 8'h01;
    localparam [7:0] ADDR_ID    = 8'h7F;

    localparam [7:0] CHIP_ID = 8'hA5;

    // speed=8, amp=3, spread=0, mirror=0, reverse=0
    localparam [7:0] RESET_WAVE0 = 8'h38;
    localparam [1:0] RESET_WAVE1 = 2'b00;

    reg [7:0] wave0_r;
    reg [1:0] wave1_r;

    assign wave0 = wave0_r;
    assign wave1 = wave1_r;

    // ---- read: combinational --------------------------------------------
    // Every address is decoded explicitly, so anything unmapped returns a
    // defined 0x00 and reserved bits read back 0 rather than echoing whatever
    // was written. ID is decoded here and has no storage, which is what makes
    // it inherently read-only.
    assign rdata = (addr == ADDR_WAVE0) ? wave0_r
                 : (addr == ADDR_WAVE1) ? {6'b000000, wave1_r}
                 : (addr == ADDR_ID)    ? CHIP_ID
                                        : 8'h00;

    // ---- write: synchronous ---------------------------------------------
    // Unmapped addresses fall through `default` and are dropped, so a stray
    // address cannot alias onto a real register.
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wave0_r <= RESET_WAVE0;
            wave1_r <= RESET_WAVE1;
        end else if (wr_en) begin
            case (addr)
                ADDR_WAVE0: wave0_r <= wdata;
                ADDR_WAVE1: wave1_r <= wdata[1:0];
                default:    ;
            endcase
        end
    end

endmodule
