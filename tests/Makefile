# Host unit tests for pure firmware logic (no Pico SDK). Run: make -C tests
CXX      = g++
CXXFLAGS = -Wall -Wextra -Werror -std=c++17 -I../firmware -g -O0
OUTDIR   = bin

TESTS = $(OUTDIR)/maze_setting_test

all: test

$(OUTDIR):
	mkdir -p $(OUTDIR)

$(OUTDIR)/maze_setting_test: maze_setting_test.cpp ../firmware/maze/maze_setting.cpp ../firmware/maze/maze_setting.h | $(OUTDIR)
	$(CXX) $(CXXFLAGS) maze_setting_test.cpp ../firmware/maze/maze_setting.cpp -o $@

test: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done

clean:
	rm -rf $(OUTDIR)

.PHONY: all test clean
