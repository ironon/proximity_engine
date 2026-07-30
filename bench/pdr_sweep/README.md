# `pdr_sweep` — Spike S1a / S2 bench

Two roles from one source tree, for measuring the things about the beacon
schedule that source reading cannot settle. Results live in
`../../tests/prox_v2_spikes.md`; this file is how to reproduce them.

Standalone on purpose: no provisioning, no pairing, no enforcement state
machine, no light sleep. It *does* link the real engine, so
`prox_beacon_minor_encode/decode` and `prox_beacon_slot_is_lo` are the shipped
implementations rather than reimplementations.

```
~/.platformio/penv/bin/pio run -e tx -t upload --upload-port /dev/ttyACM1
~/.platformio/penv/bin/pio run -e rx -t upload --upload-port /dev/ttyACM2
```

Any two ESP32-C3 boards; the roles are interchangeable.

## Serial commands

**TX** — `probe` (walk `setPower()` over −30…+21 dBm and read back what the stack
accepted), `sweep on|off`, `fixed <dbm>|auto`, `restart on|off` (whether the slot
change brackets itself in `pAdv->stop()`/`start()`), `stat`.

**RX** — `reset`, `report`, `scan active|passive`, `connect <n> [settle_ms]`
(Spike S2), `stat`.

## How the measurement avoids trusting the transmitter

The TX runs a 2-slot cycle: a +9 dBm **reference** slot (`slot_id` 0) and a
**swept** slot (`slot_id` 3+level, so `prox_beacon_slot_is_lo()` still classifies
it correctly). The level in force is a pure function of `cycle_seq`, computed
identically on both sides.

The receiver therefore derives the **denominator arithmetically** from the range
of `cycle_seq` values it observed, rather than from any count the transmitter
reports. A level whose swept slot was never heard *at all* still has a valid
denominator, taken from the reference slots bracketing it — which is exactly the
regime the cliff lives in. The decoded `slot_id` is cross-checked against the
arithmetic level on every packet and reported as `mism`; it was 0 across every
run.

The reference slot doubles as the experiment's control: it never changes power,
so if its hit rate or RSSI moves during a run, the geometry drifted and the run
is void.

## Reproducing

Driver scripts (written to the session scratchpad, reproduced here in outline):

- **S1a sweep** — open both ports in one process (reopening a USB-JTAG CDC port
  can reset the board and silently zero the counters), then
  `rx: scan passive`, `rx: reset`, `tx: sweep on`, wait 255 s, `rx: report`.
  One full pass is 12 levels × 40 cycles × 500 ms.
- **S2** — `tx: sweep on`, then `rx: connect 60`. Compare against
  `tx: sweep off` as the control; the schedule condition should not be worse.

## Gotchas found the hard way

- **`upload_speed = 115200`** is pinned in `platformio.ini`. The native USB-JTAG
  CDC link is not a real UART, and esptool's mid-flash switch to 460800 drops the
  connection outright.
- A **chip-erased** C3 boot-loops on `invalid header: 0xffffffff`, re-enumerating
  USB each time, which makes flashing flaky. Retry, or flash with esptool
  directly — and remember the app image belongs at **0x10000**, not 0x0.
- **Keep the receiver passive** unless you are specifically measuring the active
  penalty. Active scanning cost 40–70 % of advertisement receptions here.
