# Host-side acceptance tests for the proximity engine (v2.1 Phase 1).
# No hardware, no framework: `make` builds and runs everything.
#
#   make          build + run
#   make clean

CXX      ?= g++
CXXFLAGS ?= -std=c++11 -O1 -g -Wall -Wextra -Wno-unused-parameter \
            -DPROXIMITY_ROLE_WATCH -I../src

BIN := ./run_tests

.PHONY: all run clean
all: run

$(BIN): test_prox_v2.cpp ../src/proximity.cpp ../src/proximity.h ../src/prox_luts.h
	$(CXX) $(CXXFLAGS) -o $(BIN) test_prox_v2.cpp ../src/proximity.cpp -lm

run: $(BIN)
	$(BIN)

clean:
	rm -f $(BIN)
