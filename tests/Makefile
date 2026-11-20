# Host-side checks for firmware code that needs no Pico SDK.
# Run with `make -C tests`. Ticket #26 replaces this with the host test target.
CXX      ?= g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Werror -I../firmware

.PHONY: test
test: encoder_history_test maze_setting_test run_time_test route_planner_test
	./encoder_history_test
	./maze_setting_test
	./run_time_test
	./route_planner_test
	CXX="$(CXX)" sh host_compile.sh
	@if $(CXX) $(CXXFLAGS) -fsyntax-only range_assert_fires.cpp 2>&1 | grep -q "encoder history"; \
	then echo "PASS: range static_assert fires past the history range"; \
	else echo "FAIL: range static_assert did not fire"; exit 1; fi

encoder_history_test: encoder_history_test.cpp ../firmware/control/encoder_history.h \
		../firmware/config/geometry.h ../firmware/config/motion.h
	$(CXX) $(CXXFLAGS) $< -o $@

maze_setting_test: maze_setting_test.cpp ../firmware/maze/maze_setting.cpp \
		../firmware/maze/maze_setting.h
	$(CXX) $(CXXFLAGS) maze_setting_test.cpp ../firmware/maze/maze_setting.cpp -o $@

# Navigation code needs the maze model and logging; maze/log/breadcrumb link
# against hal/host.
NAV_SRCS := ../firmware/navigation/run_time.cpp ../firmware/navigation/path_converter.cpp \
		../firmware/maze/maze.cpp ../firmware/maze/maze_mouse.cpp \
		../firmware/common/log.cpp ../firmware/common/breadcrumb.cpp \
		$(wildcard ../firmware/hal/host/*.cpp)

run_time_test: run_time_test.cpp $(NAV_SRCS) ../firmware/navigation/run_time.h
	$(CXX) $(CXXFLAGS) run_time_test.cpp $(NAV_SRCS) -o $@

route_planner_test: route_planner_test.cpp ../firmware/navigation/route_planner.cpp $(NAV_SRCS) \
		../firmware/navigation/route_planner.h ../firmware/navigation/run_time.h
	$(CXX) $(CXXFLAGS) route_planner_test.cpp ../firmware/navigation/route_planner.cpp \
		$(NAV_SRCS) -o $@

.PHONY: clean
clean:
	rm -f encoder_history_test maze_setting_test run_time_test route_planner_test route_planner_test
	rm -rf build-host
