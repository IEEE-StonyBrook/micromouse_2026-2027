# Host tests: the platform-neutral firmware, built against hal/host with no
# Pico SDK, plus the doctest unit tests that run on it. Run with `make -C tests`.
CXX      ?= g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Werror -I../firmware -isystem ../third_party -I.
OUT      := build-host
LIB      := $(OUT)/libfirmware_host.a
# The same suite rebuilt at the STM32's 1 kHz loop rate (config/board.h).
OUT_1K   := build-host-1k
LIB_1K   := $(OUT_1K)/libfirmware_host.a
DEFS_1K  := -DHOST_LOOP_FREQUENCY_HZ=1000.0f
TRAJ_500 := build-tests/traj_500.txt

# One binary holds every test case; main.cpp supplies doctest's main().
TEST_SRCS     := $(wildcard *_test.cpp)
FIRMWARE_SRCS := ../firmware/maze/maze_setting.cpp ../firmware/hal/console.cpp \
                 ../firmware/drivers/lsm6dsr.cpp \
                 ../firmware/maze/maze.cpp ../firmware/maze/maze_mouse.cpp \
                 ../firmware/maze/route_proof.cpp \
                 ../firmware/navigation/path_converter.cpp \
                 ../firmware/navigation/route_planner.cpp ../firmware/navigation/run_time.cpp
FAKE_SRCS     := $(wildcard fakes/*.cpp)
# The physics sim library; sim/physim_main.cpp is its CLI, with its own main().
SIM_SRCS      := $(filter-out sim/physim_main.cpp,$(wildcard sim/*.cpp))

.PHONY: test
test:
	CXX="$(CXX)" sh host_compile.sh
	$(MAKE) host_tests
	RATE_TRAJ_OUT=$(TRAJ_500) ./host_tests
	CXX="$(CXX)" OUT=$(OUT_1K) HOST_DEFS="$(DEFS_1K)" sh host_compile.sh
	$(MAKE) host_tests_1k
	RATE_TRAJ_REF=$(TRAJ_500) ./host_tests_1k
	@if $(CXX) $(CXXFLAGS) -fsyntax-only range_assert_fires.cpp 2>&1 | grep -q "encoder history"; \
	then echo "PASS: range static_assert fires past the history range"; \
	else echo "FAIL: range static_assert did not fire"; exit 1; fi

# doctest's main is slow to compile and never changes, so it gets its own object
# outside build-host, which host_compile.sh empties on every run.
build-tests/doctest_main.o: main.cpp
	mkdir -p build-tests
	$(CXX) $(CXXFLAGS) -c $< -o $@

host_tests: build-tests/doctest_main.o $(TEST_SRCS) $(FAKE_SRCS) $(SIM_SRCS) $(FIRMWARE_SRCS) $(LIB)
	$(CXX) $(CXXFLAGS) build-tests/doctest_main.o $(TEST_SRCS) $(FAKE_SRCS) $(SIM_SRCS) $(FIRMWARE_SRCS) $(LIB) -o $@

host_tests_1k: build-tests/doctest_main.o $(TEST_SRCS) $(FAKE_SRCS) $(SIM_SRCS) $(FIRMWARE_SRCS) $(LIB_1K)
	$(CXX) $(CXXFLAGS) $(DEFS_1K) build-tests/doctest_main.o $(TEST_SRCS) $(FAKE_SRCS) $(SIM_SRCS) $(FIRMWARE_SRCS) $(LIB_1K) -o $@

# The physics sim CLI at the default (Pico) loop rate: prints a CSV trace.
physim: sim/physim_main.cpp $(SIM_SRCS) sim/physim.h $(FAKE_SRCS) $(LIB)
	$(CXX) $(CXXFLAGS) sim/physim_main.cpp $(SIM_SRCS) $(FAKE_SRCS) $(LIB) -o $@

# make test builds the library first; this covers `make physim` on its own.
$(LIB):
	CXX="$(CXX)" sh host_compile.sh

.PHONY: clean
clean:
	rm -f host_tests host_tests_1k physim
	rm -rf $(OUT) $(OUT_1K) build-tests
