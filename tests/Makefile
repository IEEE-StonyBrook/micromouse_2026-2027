# Host-side checks for firmware headers that need no Pico SDK.
# Run with `make -C tests`. Ticket #26 replaces this with the host test target.
CXX      ?= g++
CXXFLAGS := -std=c++17 -Wall -Wextra -Werror -I../firmware

.PHONY: test
test: encoder_history_test
	./encoder_history_test
	@if $(CXX) $(CXXFLAGS) -fsyntax-only range_assert_fires.cpp 2>&1 | grep -q "encoder history"; \
	then echo "PASS: range static_assert fires past the history range"; \
	else echo "FAIL: range static_assert did not fire"; exit 1; fi

encoder_history_test: encoder_history_test.cpp ../firmware/control/encoder_history.h \
		../firmware/config/geometry.h ../firmware/config/motion.h
	$(CXX) $(CXXFLAGS) $< -o $@

.PHONY: clean
clean:
	rm -f encoder_history_test
