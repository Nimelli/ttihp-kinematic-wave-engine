/*
======================================================================
Copyright (c) 2026 Jeremie W
SPDX-License-Identifier: Apache-2.0

File Name:  kwe_fpga_top.v
Author:     Jeremie W (willab.ch)
Brief:      Cmod A7-35T wrapper for the Kinematic Wave Engine
======================================================================

*** NOT PART OF THE TAPEOUT. Do not add to info.yaml or test/Makefile. ***

The taped-out module tt_um_nimelli_kinematic_wave_engine is instantiated
here completely unmodified. Everything else in fpga/ exists only to feed it
a correct clock and to make its outputs observable without a scope.

Three problems this wrapper solves:

1. CLOCK. The Cmod A7 has a 12 MHz oscillator; the design's timing is signed
   off at 10 MHz and every constant in kwe_timebase assumes it. Running the
   RTL straight off 12 MHz gives 0.83-1.67 ms pulses, which drives an SG90
   past both of its mechanical stops. An MMCM synthesises exactly 10 MHz:

       12 MHz x 50 = 600 MHz VCO,  600 / 60 = 10.000 MHz

   600 MHz is the bottom of the -1 speed grade MMCM VCO range, and 12 MHz is
   inside the MMCM PFD range. A PLL cannot be used at all here -- the 7-series
   PLL has a 19 MHz minimum input frequency.

2. CONTROL. There are no DIP switches on the board. The wave parameters live
   in registers that can be nudged by BTN1 or by single-character commands
   over the USB serial port, so the whole parameter space can be explored
   without rebuilding the bitstream.

3. OBSERVABILITY. kwe_pulse_monitor times every servo pulse and the result is
   streamed out as ASCII once per frame. That is the substitute for the
   oscilloscope and the eight servos that do not exist yet.

4. SPI ACCESS. The DUT's P1 SPI slave lives on uio[3:0]. Those four pins are
   brought straight out to DIP pins 1-4 so an external master -- an RP2040 --
   can drive them. Nothing in this wrapper touches them: it is a wire, not a
   test fixture, so what the RP2040 talks to is exactly what the chip will
   present.

Serial commands, 115200 8N1 (see fpga/kwe_monitor.py):

    s / S   speed_sel  +1 / -1        (BTN1 also does +1)
    a / A   amp_sel    +1 / -1
    p       toggle spread
    m       toggle mirror
    r / R   reverse_sel +1 / -1
    x       reset the DUT (BTN0 also does this)
*/

`default_nettype none

module kwe_fpga_top #(
    // 10 ms at 10 MHz. Simulation overrides this to keep button tests short:
    //     COMPILE_ARGS += -Pkwe_fpga_top.DEBOUNCE_CLKS=50
    parameter integer DEBOUNCE_CLKS = 100000
) (
    input  wire       sysclk,        // 12 MHz oscillator, pin L17
    input  wire [1:0] btn,           // btn[0] = reset, btn[1] = speed++
    output wire [1:0] led,
    output wire [7:0] servo,         // DIP pins 26..33
    output wire       uart_rxd_out,  // FPGA -> FTDI -> PC
    input  wire       uart_txd_in,   // PC  -> FTDI -> FPGA

    // P1 SPI slave, DIP pins 1-4. Driven by an external master (RP2040).
    input  wire       spi_cs,        // active low
    input  wire       spi_sck,
    input  wire       spi_mosi,
    output wire       spi_miso       // Hi-Z unless the slave is selected
);

    // Power-on defaults. Chosen to be obviously alive: mid speed, full
    // amplitude, one full wavelength across the array, no mirroring.
    localparam [3:0] DEF_SPEED   = 4'd8;
    localparam [1:0] DEF_AMP     = 2'd3;
    localparam       DEF_SPREAD  = 1'b0;
    localparam       DEF_MIRROR  = 1'b0;
    localparam [1:0] DEF_REVERSE = 2'd0;

    localparam integer CLKS_PER_BIT = 87;   // 10 MHz / 115200


    // ---------------------------------------------------------------
    // 1. Clock: 12 MHz -> 10.000 MHz
    // ---------------------------------------------------------------
    wire clk;
    wire mmcm_locked;

`ifdef KWE_SIM
    // Simulation drives sysclk at 10 MHz directly. Xilinx primitives need
    // the unisim libraries and glbl, which is not worth it just to verify
    // the harness logic -- the MMCM is validated on hardware by the fact
    // that the measured pulse widths come out right.
    assign clk         = sysclk;
    assign mmcm_locked = 1'b1;
`else
    wire clk_unbuf;
    wire fb_out;
    wire fb_in;

    MMCME2_BASE #(
        .BANDWIDTH          ("OPTIMIZED"),
        .CLKIN1_PERIOD      (83.333),   // 12 MHz
        .DIVCLK_DIVIDE      (1),
        .CLKFBOUT_MULT_F    (50.000),   // 12 x 50 = 600 MHz VCO
        .CLKFBOUT_PHASE     (0.000),
        .CLKOUT0_DIVIDE_F   (60.000),   // 600 / 60 = 10.000 MHz
        .CLKOUT0_DUTY_CYCLE (0.500),
        .CLKOUT0_PHASE      (0.000),
        .REF_JITTER1        (0.010),
        .STARTUP_WAIT       ("FALSE")
    ) u_mmcm (
        .CLKOUT0  (clk_unbuf),
        .CLKOUT0B (),
        .CLKOUT1  (), .CLKOUT1B (),
        .CLKOUT2  (), .CLKOUT2B (),
        .CLKOUT3  (), .CLKOUT3B (),
        .CLKOUT4  (), .CLKOUT5 (), .CLKOUT6 (),
        .CLKFBOUT (fb_out),
        .CLKFBOUTB(),
        .LOCKED   (mmcm_locked),
        .CLKIN1   (sysclk),
        .PWRDWN   (1'b0),
        .RST      (1'b0),
        .CLKFBIN  (fb_in)
    );

    BUFG u_bufg_fb  (.I(fb_out),    .O(fb_in));
    BUFG u_bufg_clk (.I(clk_unbuf), .O(clk));
`endif


    // ---------------------------------------------------------------
    // 2. Resets
    //
    // por_n  -- power-on / MMCM lock. Holds the control registers and the
    //           command path, which must survive a DUT reset.
    // rst_n  -- the DUT's reset. Asserted by por_n, by BTN0, or by the 'x'
    //           command. Async assert, synchronous release.
    // ---------------------------------------------------------------
    // Initialised, not just reset: on Artix-7 the bitstream sets flop initial
    // values, so the board comes out of configuration held in reset. It also
    // makes simulation deterministic -- without it por_sync starts X, the
    // async-reset branch is never taken, and every downstream reset stays X.
    reg [2:0] por_sync = 3'b000;
    always @(posedge clk or negedge mmcm_locked) begin
        if (!mmcm_locked) begin
            por_sync <= 3'b000;
        end else begin
            por_sync <= {por_sync[1:0], 1'b1};
        end
    end
    wire por_n = por_sync[2];

    wire btn_rst_level;
    wire btn_rst_rise_unused;
    wire btn_speed_level_unused;
    wire btn_speed_rise;

    kwe_debounce #(.STABLE_CLKS (DEBOUNCE_CLKS)) u_db_rst (
        .clk (clk), .rst_n (por_n), .noisy (btn[0]),
        .level (btn_rst_level), .rise (btn_rst_rise_unused)
    );

    kwe_debounce #(.STABLE_CLKS (DEBOUNCE_CLKS)) u_db_speed (
        .clk (clk), .rst_n (por_n), .noisy (btn[1]),
        .level (btn_speed_level_unused), .rise (btn_speed_rise)
    );

    // Soft reset from the 'x' command. Lives in the por_n domain so it can
    // request a reset without resetting itself.
    reg [7:0] soft_rst_cnt;
    wire      cmd_reset;
    always @(posedge clk or negedge por_n) begin
        if (!por_n) begin
            soft_rst_cnt <= 8'd0;
        end else if (cmd_reset) begin
            soft_rst_cnt <= 8'hFF;
        end else if (soft_rst_cnt != 8'd0) begin
            soft_rst_cnt <= soft_rst_cnt - 8'd1;
        end
    end
    wire rst_req = btn_rst_level | (soft_rst_cnt != 8'd0);

    reg [2:0] rst_sync = 3'b000;
    always @(posedge clk or negedge por_n) begin
        if (!por_n) begin
            rst_sync <= 3'b000;
        end else if (rst_req) begin
            rst_sync <= 3'b000;
        end else begin
            rst_sync <= {rst_sync[1:0], 1'b1};
        end
    end
    wire rst_n = rst_sync[2];


    // ---------------------------------------------------------------
    // 3. Control registers -- stand-ins for the switches on the real chip
    //
    // Deliberately in the por_n domain, not rst_n: physical switches would
    // not snap back to their defaults just because the chip was reset, and
    // neither should these. Only a power cycle restores the defaults.
    // ---------------------------------------------------------------
    reg [3:0] speed_sel;
    reg [1:0] amp_sel;
    reg       spread_sel;
    reg       mirror_sel;
    reg [1:0] reverse_sel;

    wire [7:0] cmd_data;
    wire       cmd_valid;

    kwe_uart_rx #(.CLKS_PER_BIT (CLKS_PER_BIT)) u_uart_rx (
        .clk (clk), .rst_n (por_n), .rx (uart_txd_in),
        .data (cmd_data), .valid (cmd_valid)
    );

    assign cmd_reset = cmd_valid && (cmd_data == 8'h78);   // 'x'

    always @(posedge clk or negedge por_n) begin
        if (!por_n) begin
            speed_sel   <= DEF_SPEED;
            amp_sel     <= DEF_AMP;
            spread_sel  <= DEF_SPREAD;
            mirror_sel  <= DEF_MIRROR;
            reverse_sel <= DEF_REVERSE;
        end else begin
            if (btn_speed_rise) begin
                speed_sel <= speed_sel + 4'd1;
            end

            if (cmd_valid) begin
                case (cmd_data)
                    8'h73: speed_sel   <= speed_sel   + 4'd1;   // 's'
                    8'h53: speed_sel   <= speed_sel   - 4'd1;   // 'S'
                    8'h61: amp_sel     <= amp_sel     + 2'd1;   // 'a'
                    8'h41: amp_sel     <= amp_sel     - 2'd1;   // 'A'
                    8'h70: spread_sel  <= ~spread_sel;          // 'p'
                    8'h6D: mirror_sel  <= ~mirror_sel;          // 'm'
                    8'h72: reverse_sel <= reverse_sel + 2'd1;   // 'r'
                    8'h52: reverse_sel <= reverse_sel - 2'd1;   // 'R'
                    default: ;                                  // ignore
                endcase
            end
        end
    end

    // Pin mapping per info.yaml.
    //   ui_in[0]   MODE_SW, reserved for the P1 SPI override
    //   ui_in[4:1] SPEED    ui_in[6:5] AMP    ui_in[7] SPREAD
    //   uio_in[0]  SPI_CS   uio_in[1] SPI_SCK   uio_in[2] SPI_MOSI
    //   uio_in[3]  SPI_MISO is an OUTPUT of the DUT; its input leg is unused
    //   uio_in[4]  MIRROR   uio_in[6:5] REVERSE
    //
    // The SPI pins pass through untouched -- no synchroniser, no debounce.
    // spis_synchro inside the DUT is what makes them safe, and putting
    // anything else in the path would mean the RP2040 is not talking to the
    // same logic the chip will have.
    wire [7:0] ui_in  = {spread_sel, amp_sel, speed_sel, 1'b0};
    wire [7:0] uio_in = {1'b0, reverse_sel, mirror_sel,
                         1'b0, spi_mosi, spi_sck, spi_cs};


    // ---------------------------------------------------------------
    // 4. The design under test, exactly as taped out
    // ---------------------------------------------------------------
    wire [7:0] uo_out;
    wire [7:0] uio_out;
    wire [7:0] uio_oe;

    tt_um_nimelli_kinematic_wave_engine u_dut (
        .ui_in   (ui_in),
        .uo_out  (uo_out),
        .uio_in  (uio_in),
        .uio_out (uio_out),
        .uio_oe  (uio_oe),
        .ena     (1'b1),
        .clk     (clk),
        .rst_n   (rst_n)
    );

    assign servo = uo_out;

    // MISO. uio_oe[3] is the DUT's own output enable, so the FPGA pin goes
    // high-Z at exactly the moments the real chip's pad would -- the RP2040
    // sees the same bus behaviour either way, and nothing fights it if it
    // ever drives that line by mistake.
    assign spi_miso = uio_oe[3] ? uio_out[3] : 1'bz;

    // uio[3] is consumed above; every other uio_out/uio_oe bit is constant
    // zero. Kept alive so synthesis does not warn about dangling outputs.
    wire _unused_dut = &{uio_out[7:4], uio_out[2:0], uio_oe[7:4], uio_oe[2:0],
                         btn_rst_rise_unused, btn_speed_level_unused, 1'b0};


    // ---------------------------------------------------------------
    // 5. Measurement and reporting
    // ---------------------------------------------------------------
    wire [127:0] widths;
    wire         frame_done;

    kwe_pulse_monitor u_monitor (
        .clk (clk), .rst_n (rst_n), .pwm (uo_out),
        .widths (widths), .frame_done (frame_done)
    );

    // Field 0 of the report line: what the DUT is actually being told to do.
    wire [15:0] ctrl_word = {6'd0, spread_sel, amp_sel, speed_sel,
                             mirror_sel, reverse_sel};

    wire       tx_send;
    wire [7:0] tx_data;
    wire       tx_busy;
    wire       report_busy_unused;

    kwe_report #(.NFIELDS (9)) u_report (
        .clk (clk), .rst_n (rst_n),
        .trigger (frame_done),
        .fields  ({widths, ctrl_word}),
        .tx_send (tx_send), .tx_data (tx_data), .tx_busy (tx_busy),
        .busy    (report_busy_unused)
    );

    kwe_uart_tx #(.CLKS_PER_BIT (CLKS_PER_BIT)) u_uart_tx (
        .clk (clk), .rst_n (rst_n),
        .send (tx_send), .data (tx_data),
        .tx (uart_rxd_out), .busy (tx_busy)
    );

    wire _unused_rep = &{report_busy_unused, 1'b0};


    // ---------------------------------------------------------------
    // 6. LEDs -- the zero-instrument sanity check
    //
    // led[0] brightness tracks channel 0's pulse width, so the wave is
    //        visible as a slow breathing glow with nothing attached.
    //        Width spans 9984..19929 clocks; (width-9984)>>6 maps that onto
    //        0..155 of the 0..255 PWM range.
    // led[1] blinks at ~1.6 Hz, one toggle per 16 frames. If it blinks at
    //        that rate the frame period is right, which is the single most
    //        important thing to confirm.
    // ---------------------------------------------------------------
    wire [15:0] w0 = widths[15:0];
    wire [15:0] w0_off = (w0 > 16'd9984) ? (w0 - 16'd9984) : 16'd0;
    wire [7:0]  bright = (w0_off[15:6] > 10'd255) ? 8'd255 : w0_off[13:6];

    reg [7:0] pwm_cnt;
    reg [7:0] frame_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pwm_cnt   <= 8'd0;
            frame_cnt <= 8'd0;
        end else begin
            pwm_cnt <= pwm_cnt + 8'd1;        // 10 MHz / 256 = 39 kHz PWM
            if (frame_done) begin
                frame_cnt <= frame_cnt + 8'd1;
            end
        end
    end

    assign led[0] = (pwm_cnt < bright);
    assign led[1] = frame_cnt[4];

endmodule

`default_nettype wire
