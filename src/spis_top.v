/*
======================================================================
Copyright (c) 2026 Jeremie W
SPDX-License-Identifier: Apache-2.0

File Name:  spis_top.v
Author:     Jeremie W (willab.ch)
Brief:      SPI Slave - Top level module
======================================================================

Stitches the four SPI slave layers together:

    pads --> spis_synchro --> spis_phy --> spis_app --> reg_file
             (CDC + edge      (bit <-> byte)  (protocol)   (storage)
              detect)

Everything runs on the system clock. SCK is never used as a clock, only
oversampled: see spis_synchro.v for the resulting SCK <= clk/4 limit.
*/

`default_nettype none

module spis_top (
    input  wire clk,
    input  wire rst_n,

    // Raw SPI pads, asynchronous to clk
    input  wire spi_clk,
    input  wire spi_mosi,
    input  wire spi_cs,

    output wire spi_miso,
    output wire spi_miso_oe     // 1 = drive, 0 = Hi-Z (uio_oe is a per-bit enable)
);

    // ---- spis_synchro -> spis_phy ------------------------------------
    wire spi_clk_sync;          // unused by the logic, kept for waveform reading
    wire spi_mosi_sync;
    wire spi_cs_sync;
    wire spi_clk_rising;        // 1 cc, SCK low to high
    wire spi_clk_falling;       // 1 cc, SCK high to low
    wire spi_cs_falling;        // 1 cc, CS high to low

    // ---- spis_phy <-> spis_app ---------------------------------------
    wire [7:0] rx_data;
    wire       rx_byte_valid;
    wire [7:0] tx_data;
    wire       tx_load;

    // ---- spis_app <-> reg_file ---------------------------------------
    wire [7:0] reg_addr;
    wire [7:0] reg_rdata;
    wire [7:0] reg_wdata;
    wire       reg_wr_en;

    spis_synchro u_synchro (
        .clk                (clk),
        .rst_n              (rst_n),

        .spi_clk_async      (spi_clk),
        .spi_mosi_async     (spi_mosi),
        .spi_cs_async       (spi_cs),

        .spi_clk_sync       (spi_clk_sync),
        .spi_mosi_sync      (spi_mosi_sync),
        .spi_cs_sync        (spi_cs_sync),

        .spi_clk_rising     (spi_clk_rising),
        .spi_clk_falling    (spi_clk_falling),
        .spi_cs_falling     (spi_cs_falling)
    );

    spis_phy u_phy (
        .clk                (clk),
        .rst_n              (rst_n),

        .spi_mosi_sync      (spi_mosi_sync),
        .spi_cs_sync        (spi_cs_sync),

        .spi_clk_rising     (spi_clk_rising),
        .spi_clk_falling    (spi_clk_falling),
        .spi_cs_falling     (spi_cs_falling),

        .rx_data            (rx_data),
        .rx_byte_valid      (rx_byte_valid),

        .tx_data            (tx_data),
        .tx_load            (tx_load),
        .spi_miso           (spi_miso),
        .spi_miso_oe        (spi_miso_oe)
    );

    spis_app u_app (
        .clk                (clk),
        .rst_n              (rst_n),

        .spi_cs_falling     (spi_cs_falling),

        .rx_byte_valid      (rx_byte_valid),
        .rx_data            (rx_data),

        .tx_load            (tx_load),
        .tx_data            (tx_data),

        .reg_addr           (reg_addr),
        .reg_rdata          (reg_rdata),
        .reg_wdata          (reg_wdata),
        .reg_wr_en          (reg_wr_en)
    );

    reg_file u_regs (
        .clk                (clk),
        .rst_n              (rst_n),
        .addr               (reg_addr),
        .rdata              (reg_rdata),
        .wdata              (reg_wdata),
        .wr_en              (reg_wr_en)
    );

endmodule
