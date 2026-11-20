# Host-side checks for firmware code that needs no Pico SDK.
# Run with `make -C tests`. Ticket #26 replaces this with the host test target.
CXX      ?= g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Werror -I../firmware

.PHONY: test
test: encoder_history_test maze_setting_test route_proof_test
	./encoder_history_test
	./maze_setting_test
	./route_proof_test
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

ROUTE_PROOF_SRCS := ../firmware/maze/route_proof.cpp ../firmware/maze/maze.cpp \
		../firmware/maze/maze_mouse.cpp
route_proof_test: route_proof_test.cpp $(ROUTE_PROOF_SRCS) ../firmware/maze/route_proof.h \
		../firmware/maze/maze.h ../firmware/maze/maze_mouse.h
	$(CXX) $(CXXFLAGS) route_proof_test.cpp $(ROUTE_PROOF_SRCS) -o $@

.PHONY: clean
clean:
	rm -f encoder_history_test maze_setting_test route_proof_test
	rm -rf build-host
