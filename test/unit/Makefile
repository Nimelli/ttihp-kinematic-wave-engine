# Unit-test harness for individual kwe_* modules.
#
#   source ~/oss-cad-suite/environment
#   make MOD=kwe_timebase                          # run every test for one module
#   make MOD=kwe_timebase COCOTB_TEST_FILTER=test_frame_period    # run one test
#   make MOD=kwe_timebase WAVES=1                  # also dump a waveform
#   make clean
#
# WAVES=1 is cocotb's own variable, handled by its Makefile.icarus. The dump lands
# in sim_build/$(MOD)/$(MOD).fst -- view it with `surfer` or `gtkwave`.
#
# The top-level Tiny Tapeout test lives in ../Makefile and is unaffected by this.

SIM           ?= icarus
TOPLEVEL_LANG ?= verilog
SRC_DIR        = $(CURDIR)/../../src

MOD ?= kwe_timebase

# Submodule dependencies, per module under test.
DEPS_kwe_sine      = kwe_sine_lut.v
DEPS_kwe_phase_gen = kwe_speed_rom.v

VERILOG_SOURCES = $(addprefix $(SRC_DIR)/,$(MOD).v $(DEPS_$(MOD)))

TOPLEVEL            = $(MOD)
COCOTB_TEST_MODULES = test_$(MOD)
SIM_BUILD           = sim_build/$(MOD)

COMPILE_ARGS += -I$(SRC_DIR)

include $(shell cocotb-config --makefiles)/Makefile.sim

clean::
	@rm -rf sim_build results.xml __pycache__
