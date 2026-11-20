# Host tests: the platform-neutral firmware, built against hal/host with no
# Pico SDK, plus the doctest unit tests that run on it. Run with `make -C tests`.
CXX      ?= g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Werror -I../firmware -isystem ../third_party -I.
OUT      := build-host
LIB      := $(OUT)/libfirmware_host.a

# One binary holds every test case; main.cpp supplies doctest's main().
TEST_SRCS     := $(wildcard *_test.cpp)
FIRMWARE_SRCS := ../firmware/maze/maze_setting.cpp ../firmware/hal/console.cpp \
                 ../firmware/drivers/lsm6dsr.cpp \
                 ../firmware/maze/maze.cpp ../firmware/maze/maze_mouse.cpp \
                 ../firmware/navigation/path_converter.cpp \
                 ../firmware/navigation/route_planner.cpp ../firmware/navigation/run_time.cpp
FAKE_SRCS     := $(wildcard fakes/*.cpp)

.PHONY: test
test:
	CXX="$(CXX)" sh host_compile.sh
	$(MAKE) host_tests
	./host_tests
	@if $(CXX) $(CXXFLAGS) -fsyntax-only range_assert_fires.cpp 2>&1 | grep -q "encoder history"; \
	then echo "PASS: range static_assert fires past the history range"; \
	else echo "FAIL: range static_assert did not fire"; exit 1; fi

# doctest's main is slow to compile and never changes, so it gets its own object
# outside build-host, which host_compile.sh empties on every run.
build-tests/doctest_main.o: main.cpp
	mkdir -p build-tests
	$(CXX) $(CXXFLAGS) -c $< -o $@

host_tests: build-tests/doctest_main.o $(TEST_SRCS) $(FAKE_SRCS) $(FIRMWARE_SRCS) $(LIB)
	$(CXX) $(CXXFLAGS) build-tests/doctest_main.o $(TEST_SRCS) $(FAKE_SRCS) $(FIRMWARE_SRCS) $(LIB) -o $@

.PHONY: clean
clean:
	rm -f host_tests
	rm -rf $(OUT) build-tests
