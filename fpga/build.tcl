# ======================================================================
# Copyright (c) 2026 Jeremie W
# SPDX-License-Identifier: Apache-2.0
#
# File Name:  build.tcl
# Brief:      Non-project Vivado build for the Cmod A7-35T
# ======================================================================
#
# *** NOT PART OF THE TAPEOUT. ***
#
#   source ~/Xilinx/2025.1/Vivado/settings64.sh
#   cd fpga && vivado -mode batch -source build.tcl
#
# Non-project mode on purpose: no .xpr, no generated IP directory, nothing
# to accidentally commit, and the whole flow is one reviewable file. Takes
# roughly 3-5 minutes on a laptop.
#
# Everything lands in fpga/build/, which .gitignore excludes.

set script_dir [file normalize [file dirname [info script]]]
set root_dir   [file normalize $script_dir/..]
set out_dir    $script_dir/build

file mkdir $out_dir

set part      xc7a35tcpg236-1
set top       kwe_fpga_top

# The taped-out design, unmodified. Kept in the same order as info.yaml so
# the two lists can be diffed by eye.
set tapeout_sources [list \
    $root_dir/src/kwe_top.v       \
    $root_dir/src/kwe_timebase.v  \
    $root_dir/src/kwe_servo_pwm.v \
    $root_dir/src/kwe_sine_lut.v  \
    $root_dir/src/kwe_sine.v      \
    $root_dir/src/kwe_angle_map.v \
    $root_dir/src/kwe_amp_scale.v \
    $root_dir/src/kwe_speed_rom.v \
    $root_dir/src/kwe_phase_gen.v \
    $root_dir/src/spis_top.v      \
    $root_dir/src/spis_synchro.v  \
    $root_dir/src/spis_phy.v      \
    $root_dir/src/spis_app.v      \
    $root_dir/src/registers.v     \
]

# FPGA-only test harness.
set fpga_sources [list \
    $script_dir/kwe_debounce.v      \
    $script_dir/kwe_uart_tx.v       \
    $script_dir/kwe_uart_rx.v       \
    $script_dir/kwe_pulse_monitor.v \
    $script_dir/kwe_report.v        \
    $script_dir/kwe_fpga_top.v      \
]

read_verilog [concat $tapeout_sources $fpga_sources]
read_xdc     $script_dir/kwe_cmod_a7.xdc

synth_design -top $top -part $part
write_checkpoint -force $out_dir/post_synth.dcp

opt_design
place_design
phys_opt_design
route_design
write_checkpoint -force $out_dir/post_route.dcp

report_utilization      -file $out_dir/utilization.rpt
report_timing_summary   -file $out_dir/timing_summary.rpt
report_clock_utilization -file $out_dir/clock_util.rpt

write_bitstream -force $out_dir/$top.bit

# .bin as well, because openFPGALoader wants it for SPI flash writes.
write_cfgmem -force -format bin -interface spix4 -size 4 \
    -loadbit "up 0x0 $out_dir/$top.bit" \
    -file $out_dir/$top.bin

# Timing is the one thing worth failing the build over. A design this small
# at 10 MHz has enormous margin, so a violation means something structural
# went wrong -- a missing constraint, or the MMCM not being inferred.
set wns [get_property SLACK [get_timing_paths -delay_type max]]
set whs [get_property SLACK [get_timing_paths -delay_type min]]
puts "=========================================================="
puts " setup slack (WNS): $wns ns"
puts " hold slack  (WHS): $whs ns"
puts " bitstream: $out_dir/$top.bit"
puts "=========================================================="

if {$wns < 0 || $whs < 0} {
    puts "ERROR: timing not met -- do not program this bitstream"
    exit 1
}

exit 0
