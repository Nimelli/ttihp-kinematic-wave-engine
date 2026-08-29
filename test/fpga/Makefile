# Simulation harness for the Cmod A7 FPGA wrapper (fpga/kwe_fpga_top.v).
#
#   source ~/oss-cad-suite/environment
#   make                                       # run every test
#   make COCOTB_TEST_FILTER=test_hold_widths    # run one test
#   make WAVES=1                               # also dump a waveform
#   make clean
#
# This verifies the TEST HARNESS, not the tapeout. It exists so that the
# pulse monitor, the ASCII formatter and the UART are known-good before any
# time is spent in Vivado -- debugging them on the board, with no scope,
# would be miserable.
#
# KWE_SIM bypasses the MMCM: sysclk is driven at 10 MHz directly, because
# Xilinx primitives would need the unisim libraries and glbl. The MMCM is
# the one part of the wrapper that only hardware can confirm.

SIM           ?= icarus
TOPLEVEL_LANG ?= verilog
SRC_DIR        = $(CURDIR)/../../src
FPGA_DIR       = $(CURDIR)/../../fpga

VERILOG_SOURCES = \
	$(SRC_DIR)/kwe_top.v         \
	$(SRC_DIR)/kwe_timebase.v    \
	$(SRC_DIR)/kwe_servo_pwm.v   \
	$(SRC_DIR)/kwe_sine_lut.v    \
	$(SRC_DIR)/kwe_sine.v        \
	$(SRC_DIR)/kwe_angle_map.v   \
	$(SRC_DIR)/kwe_amp_scale.v   \
	$(SRC_DIR)/kwe_speed_rom.v   \
	$(SRC_DIR)/kwe_phase_gen.v   \
	$(SRC_DIR)/spis_top.v        \
	$(SRC_DIR)/spis_synchro.v    \
	$(SRC_DIR)/spis_phy.v        \
	$(SRC_DIR)/spis_app.v        \
	$(SRC_DIR)/registers.v       \
	$(FPGA_DIR)/kwe_debounce.v      \
	$(FPGA_DIR)/kwe_uart_tx.v       \
	$(FPGA_DIR)/kwe_uart_rx.v       \
	$(FPGA_DIR)/kwe_pulse_monitor.v \
	$(FPGA_DIR)/kwe_report.v        \
	$(FPGA_DIR)/kwe_fpga_top.v

TOPLEVEL            = kwe_fpga_top
COCOTB_TEST_MODULES = test_kwe_fpga_top
SIM_BUILD           = sim_build

COMPILE_ARGS += -DKWE_SIM -I$(SRC_DIR)

# Shrink the button debounce window from 10 ms to 5 us so test_button_speed
# does not have to simulate 200_000 clocks per press.
COMPILE_ARGS += -Pkwe_fpga_top.DEBOUNCE_CLKS=50

include $(shell cocotb-config --makefiles)/Makefile.sim

clean::
	@rm -rf sim_build results.xml __pycache__
