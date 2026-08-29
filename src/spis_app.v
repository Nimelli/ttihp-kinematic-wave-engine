/*
======================================================================
Copyright (c) 2026 Jeremie W
SPDX-License-Identifier: Apache-2.0

File Name:  spis_app.v
Author:     Jeremie W (willab.ch)
Brief:      SPI Slave - Application layer - register manipulation
======================================================================

Simple protocol. Every frame is CS low, opcode, address, payload, CS high.

READ  (0x03), 4 bytes:
    Byte 0: 0x03        READ opcode
    Byte 1: ADDR[7:0]   register address
    Byte 2: dummy from master, slave drives 0x00
    Byte 3: dummy from master, slave drives the register data on MISO

WRITE (0x02), 3 bytes:
    Byte 0: 0x02        WRITE opcode
    Byte 1: ADDR[7:0]   register address
    Byte 2: DATA[7:0]   value to store; slave drives 0x00 on MISO throughout

Why READ needs a dummy byte and WRITE does not:

WRITE only travels inbound. The data byte lands in rx_data with rx_byte_valid,
addr_reg has been stable for a whole byte already, so the write strobe fires in
that same cycle -- nothing to wait for.

READ has to turn the bus around, and spis_phy captures the byte it will shift
out during byte N+1 at the END of byte N (spis_phy.v:118), one cycle BEFORE this
layer has latched what byte N contained. So the earliest byte whose content can
depend on the address is byte 3, not byte 2. Byte 2 pays for that one-cycle
offset with a whole byte time, which is the point: addr_reg is stable for all 8
SCK periods of byte 2, so addr_reg -> reg_file mux -> tx_data -> tx_shift is a
plain single-cycle combinational path, unrelated to the SCK rate. This is the
same reason SPI flash parts insert dummy cycles before read data.
*/

`default_nettype none

module spis_app (
    input  wire     clk,
    input  wire     rst_n,

    // Spi framing
    input wire      spi_cs_falling,     // 1 cc, new frame

    // Receiving byte
    input wire      rx_byte_valid,
    input wire      [7:0] rx_data,

    // Transmitting byte. spis_phy also exposes tx_load, a strobe marking the
    // cycle it captured tx_data; this layer does not need it, because tx_data
    // is driven combinationally from `phase` and is always correct at the
    // moment the PHY looks. A burst or FIFO source would want it.
    output wire     [7:0] tx_data,

    // register file interface
    output wire [7:0]    reg_addr,
    input  wire [7:0]    reg_rdata,
    output wire [7:0]    reg_wdata,
    output wire          reg_wr_en    // 1 cc strobe, writes reg_wdata to reg_addr

);

    // The phase is really just "which byte of the frame is this", saturating at
    // the last one. Byte 2 carries different things for the two opcodes, hence
    // the neutral names.
    localparam PH_CMD  = 2'd0;      // byte 0: opcode
    localparam PH_ADDR = 2'd1;      // byte 1: register address
    localparam PH_DAT0 = 2'd2;      // byte 2: WRITE data in / READ dummy out
    localparam PH_DAT1 = 2'd3;      // byte 3: READ data out / unused by WRITE

    localparam CMD_READ  = 8'h03;
    localparam CMD_WRITE = 8'h02;

    reg [1:0] phase;
    reg [7:0] cmd_reg;
    reg [7:0] addr_reg;

    // framing and command decode
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            phase    <= PH_CMD;
            cmd_reg  <= 8'd0;
            addr_reg <= 8'd0;
        end else if (spi_cs_falling) begin
            // Start of a new transaction. cmd_reg and addr_reg are cleared too,
            // not just phase: a frame that is cut short before its ADDR byte
            // would otherwise decode against the previous frame's address.
            phase    <= PH_CMD;
            cmd_reg  <= 8'd0;
            addr_reg <= 8'd0;
        end else if (rx_byte_valid) begin
            case (phase)
                PH_CMD: begin
                    cmd_reg <= rx_data;
                    phase   <= PH_ADDR;
                end
                PH_ADDR: begin
                    addr_reg <= rx_data;
                    phase    <= PH_DAT0;
                end
                PH_DAT0: begin
                    // WRITE commits this cycle, combinationally, via reg_wr_en
                    // below -- there is nothing to latch here. READ has already
                    // handed reg_rdata to spis_phy on this same cycle.
                    phase <= PH_DAT1;
                end
                default: begin
                    // PH_DAT1: the phase sticks, so extra clocked bytes are
                    // harmless. A burst would act here (addr_reg <= addr_reg + 1).
                    phase <= phase;
                end
            endcase
        end
    end

    // ---- opcode decode -------------------------------------------------
    wire is_read_cmd  = (cmd_reg == CMD_READ);
    wire is_write_cmd = (cmd_reg == CMD_WRITE);

    // ---- address forwarding --------------------------------------------
    // Straight out of the register, no combinational bypass off rx_data. The
    // dummy byte buys enough time that the bypass is not needed, and dropping
    // it keeps MOSI out of the reg_file address path.
    assign reg_addr = addr_reg;

    // ---- write path ----------------------------------------------------
    // rx_data holds the complete byte for the one cycle rx_byte_valid is high,
    // and addr_reg has been stable since the end of byte 1, so a plain
    // combinational strobe is all that is needed. reg_file registers it.
    //
    // The write lands as soon as byte 2 completes rather than waiting for CS to
    // rise. A frame truncated part-way through byte 2 therefore writes nothing,
    // because rx_byte_valid never fires for it. (Commit-on-CS-rise would also be
    // valid and is what flash parts do; it would need one more holding register
    // and buys nothing here.)
    //
    // The !spi_cs_falling term is the same guard as on tx_data below: on the
    // cs_falling cycle, phase and cmd_reg still hold the PREVIOUS frame's
    // values, so without it a new frame could inherit a write strobe.
    assign reg_wdata = rx_data;
    assign reg_wr_en = !spi_cs_falling && is_write_cmd
                       && (phase == PH_DAT0) && rx_byte_valid;

    // ---- read path -----------------------------------------------------
    // Driven only during PH_DAT0, because that is the byte at whose end
    // spis_phy captures the value it will shift out during byte 3. Everything
    // else -- every byte of a WRITE frame, and an unrecognised opcode -- reads
    // back 0x00 rather than leaking register contents.
    assign tx_data = (!spi_cs_falling && phase == PH_DAT0 && is_read_cmd)
                     ? reg_rdata : 8'h00;

endmodule
