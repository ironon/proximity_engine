# Host-side acceptance tests for the proximity engine (v2.1 Phase 1).
# No hardware, no framework: `make` builds and runs everything.
#
#   make          build + run both roles
#   make watch    watch-role tests only
#   make anchor   anchor-role tests only
#   make clean

CXX      ?= g++
BASE     ?= -std=c++11 -O1 -g -Wall -Wextra -Wno-unused-parameter -I../src

WATCH_BIN  := ./run_tests
ANCHOR_BIN := ./run_tests_anchor

.PHONY: all run watch anchor clean
all: run
run: watch anchor

$(WATCH_BIN): test_prox_v2.cpp ../src/proximity.cpp ../src/proximity.h ../src/prox_luts.h
	$(CXX) $(BASE) -DPROXIMITY_ROLE_WATCH -o $(WATCH_BIN) test_prox_v2.cpp ../src/proximity.cpp -lm

$(ANCHOR_BIN): test_prox_anchor.cpp ../src/proximity.cpp ../src/proximity.h ../src/prox_luts.h
	$(CXX) $(BASE) -DPROXIMITY_ROLE_ANCHOR -o $(ANCHOR_BIN) test_prox_anchor.cpp ../src/proximity.cpp -lm

watch: $(WATCH_BIN)
	$(WATCH_BIN)

anchor: $(ANCHOR_BIN)
	$(ANCHOR_BIN)

clean:
	rm -f $(WATCH_BIN) $(ANCHOR_BIN)
