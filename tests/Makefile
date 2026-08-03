# Host-side acceptance tests for the proximity engine (v2.1 Phase 1).
# No hardware, no framework: `make` builds and runs everything.
#
#   make          build + run both roles
#   make watch    watch-role tests only
#   make anchor   anchor-role tests only
#   make schema   assert the Python decoder matches the C capture structs
#   make corpus   replay every captured session against the CURRENT engine
#   make clean
#
# `corpus` is the one that grows over time. The synthetic tests above prove the
# arithmetic; the corpus proves the arithmetic describes a real bedroom. See
# ../README.md ("Capturing real data") for how sessions get there.

CXX      ?= g++
BASE     ?= -std=c++11 -O1 -g -Wall -Wextra -Wno-unused-parameter -I../src

WATCH_BIN  := ./run_tests
ANCHOR_BIN := ./run_tests_anchor
REPLAY_BIN := ./run_replay

CORPUS_DIR ?= ../corpus
TOOLS      := ../tools
# Sessions are archived as JSONL; .replay is a derived, regenerable artifact.
SESSIONS   := $(wildcard $(CORPUS_DIR)/*.jsonl)
REPLAYS    := $(SESSIONS:.jsonl=.replay)

.PHONY: all run watch anchor schema corpus corpus-drift clean
all: run
run: watch anchor schema

$(WATCH_BIN): test_prox_v2.cpp ../src/proximity.cpp ../src/proximity.h ../src/prox_luts.h
	$(CXX) $(BASE) -DPROXIMITY_ROLE_WATCH -o $(WATCH_BIN) test_prox_v2.cpp ../src/proximity.cpp -lm

$(ANCHOR_BIN): test_prox_anchor.cpp ../src/proximity.cpp ../src/proximity.h ../src/prox_luts.h
	$(CXX) $(BASE) -DPROXIMITY_ROLE_ANCHOR -o $(ANCHOR_BIN) test_prox_anchor.cpp ../src/proximity.cpp -lm

$(REPLAY_BIN): replay.cpp ../src/proximity.cpp ../src/proximity.h ../src/prox_luts.h
	$(CXX) $(BASE) -DPROXIMITY_ROLE_ANCHOR -o $(REPLAY_BIN) replay.cpp ../src/proximity.cpp -lm

watch: $(WATCH_BIN)
	$(WATCH_BIN)

anchor: $(ANCHOR_BIN)
	$(ANCHOR_BIN)

# The capture wire format is declared twice — once in C for the devices, once in
# Python for the ingest tool. Nothing but this target stops them drifting apart,
# and drift would silently mis-decode every session recorded afterwards.
schema:
	@python3 $(TOOLS)/schema_check.py

%.replay: %.jsonl
	@python3 $(TOOLS)/capture_ingest.py replay-export $< --out $@

# Replay every session. Fails if today's engine no longer reproduces what the
# devices recorded, which is exactly what you want to hear about by accident.
corpus: $(REPLAY_BIN) $(REPLAYS)
	@if [ -z "$(REPLAYS)" ]; then \
	  echo "no sessions in $(CORPUS_DIR) — see ../README.md, 'Capturing real data'"; \
	else \
	  $(REPLAY_BIN) $(REPLAYS); \
	fi

# Same, but for after a deliberate scoring change: reports how far the engine
# moved instead of treating movement as a failure.
corpus-drift: $(REPLAY_BIN) $(REPLAYS)
	@if [ -z "$(REPLAYS)" ]; then \
	  echo "no sessions in $(CORPUS_DIR)"; \
	else \
	  $(REPLAY_BIN) --expect-drift -v $(REPLAYS); \
	fi

clean:
	rm -f $(WATCH_BIN) $(ANCHOR_BIN) $(REPLAY_BIN) $(CORPUS_DIR)/*.replay
