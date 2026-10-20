# GPU-style memory interconnect - build / verify / analyse
#
#   make sim                 build Verilator model (BUG=BUG_xxx injects a defect)
#   make test                directed + randomized stress regression
#   make formal              SymbiYosys proofs (FIFO, arbiter, full fabric)
#   make bughunt             inject every known bug, show what catches it
#   make perf                contention sweep + crowd-trace replay -> docs/img
#   make all                 everything

RTL      := rtl/sync_fifo.sv rtl/qos_arbiter.sv rtl/axi_interconnect.sv rtl/axi_mem.sv rtl/mem_subsystem_top.sv
TOP      := mem_subsystem_top
BUG      ?=
OBJ      := build/sim$(if $(BUG),_$(BUG),)
SIM      := $(OBJ)/V$(TOP)
CROWD    := build/crowd_sim
PY       ?= python3
WAIVE    := -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-PINCONNECTEMPTY
VFLAGS   := -O3 --x-assign fast --x-initial fast --trace $(WAIVE) $(if $(BUG),+define+$(BUG))

.PHONY: all sim lint test formal bughunt perf crowd clean

all: lint test formal bughunt perf

sim: $(SIM)

$(SIM): $(RTL) tb/sim_main.cpp
	@mkdir -p $(OBJ)
	verilator --cc --exe --build -j 0 $(VFLAGS) --top-module $(TOP) \
	  -Mdir $(OBJ) -o V$(TOP) $(RTL) tb/sim_main.cpp -CFLAGS "-std=c++17 -O2"

lint:
	verilator --lint-only -Wall $(WAIVE) --top-module $(TOP) $(RTL)

test: $(SIM)
	$(PY) scripts/run_tests.py --sim $(SIM)

formal:
	cd formal && sby -f fifo.sby prove && sby -f fifo.sby cover
	cd formal && sby -f arbiter.sby prove && sby -f arbiter.sby cover
	cd formal && sby -f interconnect.sby bmc && sby -f interconnect.sby cover

bughunt:
	$(PY) scripts/bug_hunt.py

$(CROWD): crowd/crowd_sim.cpp
	@mkdir -p build
	c++ -std=c++17 -O2 -o $@ $<

crowd: $(CROWD)

perf: $(SIM) $(CROWD)
	$(PY) scripts/perf_sweep.py --sim $(SIM) --crowd $(CROWD)

clean:
	rm -rf build traces/*.trace formal/*_prove formal/*_cover formal/*_bmc formal/*_bug_*
