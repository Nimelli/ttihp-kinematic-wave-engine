## ====================================================================
## Kinematic Wave Engine -- Digilent Cmod A7-35T constraints
## Copyright (c) 2026 Jeremie W
## SPDX-License-Identifier: Apache-2.0
##
## *** NOT PART OF THE TAPEOUT. ***
##
## Pin assignments taken from Digilent's Cmod-A7-Master.xdc (rev. B).
## Board: XC7A35T-1CPG236C, part xc7a35tcpg236-1.
## ====================================================================

## --------------------------------------------------------------------
## Clock -- 12 MHz oscillator. An MMCM in kwe_fpga_top turns this into
## the 10.000 MHz the design is designed and timed for.
## --------------------------------------------------------------------
set_property -dict { PACKAGE_PIN L17 IOSTANDARD LVCMOS33 } [get_ports { sysclk }]
create_clock -add -name sys_clk_pin -period 83.33 -waveform {0 41.66} [get_ports { sysclk }]

## --------------------------------------------------------------------
## Buttons -- active high (pressed = 1)
##   btn[0]  reset the DUT
##   btn[1]  speed_sel + 1
## --------------------------------------------------------------------
set_property -dict { PACKAGE_PIN A18 IOSTANDARD LVCMOS33 } [get_ports { btn[0] }]
set_property -dict { PACKAGE_PIN B18 IOSTANDARD LVCMOS33 } [get_ports { btn[1] }]

## --------------------------------------------------------------------
## LEDs -- LD1/LD2, driven through a 330R into the anode, so active high
##   led[0]  brightness follows channel 0's pulse width
##   led[1]  ~1.6 Hz heartbeat, one toggle per 16 frames
## --------------------------------------------------------------------
set_property -dict { PACKAGE_PIN A17 IOSTANDARD LVCMOS33 } [get_ports { led[0] }]
set_property -dict { PACKAGE_PIN C16 IOSTANDARD LVCMOS33 } [get_ports { led[1] }]

## --------------------------------------------------------------------
## UART, via the on-board FTDI. Digilent's names are from the bridge's
## point of view, so uart_rxd_out is the FPGA's TRANSMIT pin.
## --------------------------------------------------------------------
set_property -dict { PACKAGE_PIN J18 IOSTANDARD LVCMOS33 } [get_ports { uart_rxd_out }]
set_property -dict { PACKAGE_PIN J17 IOSTANDARD LVCMOS33 } [get_ports { uart_txd_in  }]

## --------------------------------------------------------------------
## Servo outputs -- DIP pins 26..33 (pio26..pio33)
##
## Chosen because they are physically adjacent to DIP pin 25, which is
## GND: the servo ground reference is then one pin away from the signals
## instead of at the far end of the board.
##
## Reduced drive and slow slew: these run to a breadboard, and a fast
## 3.3 V edge into an unterminated flying lead rings badly. Nothing here
## is timing-critical -- the pulse is 1 ms long.
## --------------------------------------------------------------------
set_property -dict { PACKAGE_PIN R3 IOSTANDARD LVCMOS33 DRIVE 8 SLEW SLOW } [get_ports { servo[0] }]
set_property -dict { PACKAGE_PIN T3 IOSTANDARD LVCMOS33 DRIVE 8 SLEW SLOW } [get_ports { servo[1] }]
set_property -dict { PACKAGE_PIN R2 IOSTANDARD LVCMOS33 DRIVE 8 SLEW SLOW } [get_ports { servo[2] }]
set_property -dict { PACKAGE_PIN T1 IOSTANDARD LVCMOS33 DRIVE 8 SLEW SLOW } [get_ports { servo[3] }]
set_property -dict { PACKAGE_PIN T2 IOSTANDARD LVCMOS33 DRIVE 8 SLEW SLOW } [get_ports { servo[4] }]
set_property -dict { PACKAGE_PIN U1 IOSTANDARD LVCMOS33 DRIVE 8 SLEW SLOW } [get_ports { servo[5] }]
set_property -dict { PACKAGE_PIN W2 IOSTANDARD LVCMOS33 DRIVE 8 SLEW SLOW } [get_ports { servo[6] }]
set_property -dict { PACKAGE_PIN V2 IOSTANDARD LVCMOS33 DRIVE 8 SLEW SLOW } [get_ports { servo[7] }]

## --------------------------------------------------------------------
## Configuration
##
## CFGBVS/CONFIG_VOLTAGE describe bank 0 and are required on 7-series --
## without them write_bitstream raises a critical warning.
##
## UNUSEDPIN PULLNONE rather than the PULLDOWN default: every unused pin
## on this board is a through-hole pin in a breadboard, and a pull-down
## on something wired externally is a surprise waiting to happen.
## --------------------------------------------------------------------
set_property CFGBVS VCCO        [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.CONFIG.UNUSEDPIN PULLNONE [current_design]

## Quad-SPI flash settings, so the same bitstream can be written to flash
## and boot standalone (see build.tcl / openFPGALoader -f).
set_property CONFIG_MODE SPIx4                    [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4      [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 33       [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE      [current_design]
