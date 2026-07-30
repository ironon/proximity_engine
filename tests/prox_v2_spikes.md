# Proximity v2.1 — Phase S0 spike results

Write-up location mandated by FIRMWARE_SPEC v0.9 amendment §10-A Phase S0.
One section per spike. A spike is only "PASS" when its stated pass criterion has
actually been met, not when it looks likely.

| Spike | Question | Status |
|---|---|---|
| S1a | Which TX levels does the C3 emit, and where does the near-band link close but not the far one? | **PASS — 2026-07-30, both positions swept; BEACON_TX_LO_DBM = -6** |
| S1b | Can advertising be restricted to one channel per slot on this NimBLE? | **RESOLVED in source; on-air confirmation outstanding** |
| S2 | Does a 250 ms adv stop/reconfig/start cycle keep the anchor connectable and its scan tasks fed? | **PARTIAL — reconfiguration shown not to degrade connectability; absolute criterion needs the real firmware pair** |
| S3 | LIS3DH burst thresholds on real wrists | **PARTIAL — resting floor measured, motion classes outstanding** |
| S4, S5 | CSI / FTM (pre-P4 only) | not started; not needed before P4 |

---

## S1b — per-slot channel restriction

**Question (amendment §10-A):** can advertising be restricted to a single
channel per slot on this NimBLE host, via param / ext-adv / vendor-HCI?

**Pass criterion:** slot-tagged adverts observed on exactly one channel.

**Status: the capability exists — two independent routes, both in-tree. The
pass criterion itself is an on-air observation and has NOT been made.**

### What the amendment assumed

Engine spec §3.4 states the fallback exists because "the legacy
`ble_gap_adv_params` does not expose a channel map". **That premise is wrong for
the version in this tree.** Verified against
`AnchorFirmware/.pio/libdeps/esp32-c3-devkitc-02/NimBLE-Arduino/` (h2zero
NimBLE-Arduino ^2.0.0):

```
nimble/nimble/host/include/host/ble_gap.h
    struct ble_gap_adv_params {
        ...
        /** Advertising channel map , if 0 stack use sane defaults */
        uint8_t channel_map;
```

and it is plumbed all the way to the HCI command, for both the legacy and the
extended path:

```
nimble/nimble/host/src/ble_gap.c:3420   if (adv_params->channel_map == 0) { ...default... }
nimble/nimble/host/src/ble_gap.c:3423       cmd.chan_map = adv_params->channel_map;
nimble/nimble/host/src/ble_gap.c:3804       cmd->pri_chan_map = params->channel_map;
```

So the host stack supports it. The obstacle is one layer up.

### Route 1 — legacy params via the raw host API

`NimBLEAdvertising` owns `ble_gap_adv_params m_advParams` as a **private**
member (`NimBLEAdvertising.h:100`) and exposes no setter for `channel_map` —
only interval, connectable/discoverable mode, filter policy and payload. So the
C++ wrapper cannot express it, but `ble_gap_adv_start()` can, and it is
callable directly.

**Cost:** bypassing the wrapper means also owning `ble_gap_adv_set_data()` and,
critically, the GAP event callback. `NimBLEAdvertising::handleGapEvent` is a
private static that routes connect/disconnect into `NimBLEServer`; taking over
advertising means re-implementing that routing, and getting it wrong breaks
connectability — which §4.12 item 1 makes a hard requirement.

### Route 2 — extended-advertising API (preferred)

`NimBLEExtAdvertisement` exposes it directly:

```
NimBLEExtAdvertising.h:87   void setPrimaryChannels(bool ch37, bool ch38, bool ch39);
NimBLEExtAdvertising.h:76   void setLegacyAdvertising(bool enable);
```

`setLegacyAdvertising(true)` is the important companion: it emits **legacy PDUs**
(so existing scanners, including the watch's Major-`0x4A0F` filter and the phone
app, keep seeing an ordinary iBeacon) while still going through the extended
HCI parameter set, which carries `Primary_Advertising_Channel_Map`. That is
exactly the combination the schedule needs.

**Cost:** this requires `CONFIG_BT_NIMBLE_EXT_ADV` at build time, and in this
library legacy and extended advertising are **mutually exclusive** — the whole
`NimBLEAdvertising` class is compiled out:

```
NimBLEAdvertising.h:108
  #endif // (CONFIG_BT_ENABLED && CONFIG_BT_NIMBLE_ROLE_BROADCASTER && !CONFIG_BT_NIMBLE_EXT_ADV)
```

So enabling it is not a local change to the beacon schedule; it replaces the
advertising API for every advertiser in the build.

### What is still unknown (and why this is not yet PASS)

Source inspection proves the *API* exists. It does not prove any of the things
the pass criterion actually asks about:

1. **That the adverts really land on one channel.** Needs a sniffer or a
   channel-selective scanner. This is the literal pass criterion.
2. **That the anchor stays connectable in every slot** (§4.12 item 1) under
   whichever route is chosen — and under Route 1, that hand-rolled GAP event
   routing did not break the server.
3. **That legacy-PDU-over-ext-adv is still seen by the watch's scanner and by
   iOS/Android.** This is the biggest risk in Route 2 and it is pure hardware.
4. **That the C3's controller honours a single-channel map at all**, rather than
   silently falling back to all three.

**Recommendation:** implement the schedule against the
`prox_platform_set_beacon_slot(channel_map, ...)` seam that the engine spec
already defines, with the channel argument honoured or ignored behind one
build-time switch. Ship the power-only 2-slot schedule as the default until
items 1–4 are confirmed on air, then flip the switch. PDR — the feature that
carries most of P2's value — never needed channel attribution at all (§3.4), so
the fallback is not a crippled mode.

---

## S1a — TX_LO level table

**PASS — the pass criterion was met on 2026-07-30 with a full 12-level sweep at
both positions of a real install: `BEACON_TX_LO_DBM = -6` gives 97 % slot-hit
INSIDE and 0 % at EDGE (criterion: >=90 % / <=30 %). Working and analysis in
`spike_s1a_txlo.md` "result 2 part 2".**

The bench characterisation below (2026-07-29, two tethered C3s) established the
level ladder, the cliff shape and the active-scan hazard; the install sweep that
closes the criterion is summarised at the end of this section.

Measured with `bench/pdr_sweep/` (two roles from one source tree, linking the
shipped `prox_beacon_minor_encode/decode`). Method: a 2-slot cycle — a +9 dBm
reference slot and a swept slot — 250 ms per slot, 50 ms advertising interval,
40 cycles per level, all 12 levels, 4 min per run. The receiver derives the
**denominator arithmetically** from the observed `cycle_seq` range rather than
trusting anything the transmitter claims, so a level whose swept slot was never
heard at all still has a valid denominator from the reference slots bracketing it.

### Finding 1 — the level ladder is real, and −24 dBm is the floor

`NimBLEDevice::setPower(dbm)` computes `esp_power_level_t(dbm/3 + ESP_PWR_LVL_N0)`
with `ESP_PWR_LVL_N0 == 8` on this IDF (4.4.7, esp32c3 `esp_bt.h`). Probed by
calling it for every dBm in −30…+21 and reading `getPower()` back:

- **−27 and below are REJECTED** (`setPower` returns false; index would be −1).
  **−24 dBm is the hard floor.**
- Everything −26…+21 is accepted and quantised to exactly 3 dB.
- **Rounding is toward HIGHER power**: −26 and −25 both become −24; −23 and −22
  both become −21. Requesting a non-multiple of 3 silently buys you *more* power
  than you asked for, never less. `BEACON_TX_LO_DBM` must be a multiple of 3.

On air the commanded level tracks 1:1 over the usable range — this is the trap
("setPower silently rounds or ignores") closed: +9→−81, +6→−84, +3→−87, 0→−90 dBm
received, exactly 3 dB per step. Below about −90 dBm received the curve flattens
(−3→−92, −6→−95, −9→−95, −12→−98), which is floor truncation plus survivor bias,
not a power-setting failure: only the strongest packets get through, so the mean
of what arrives is biased upward.

### Finding 2 — the cliff, and the number that generalises

Passive scan, 100 % duty. `cycles` is the arithmetic denominator; `hi` is the
+9 dBm reference slot, `lo` the swept slot.

| swept dBm | lo slot-hit | lo PDU/ref PDU | lo RSSI | hi slot-hit | hi RSSI |
|---|---|---|---|---|---|
| −24 | 0 % | 0/349 | — | 100 % | −81 |
| −21 | 0 % | 0/184 | — | 100 % | −81 |
| −18 | 0 % | 0/192 | — | 100 % | −81 |
| −15 | **15 %** | 6/185 | −102 | 100 % | −81 |
| −12 | **82 %** | 46/189 | −98 | 100 % | −81 |
| −9 | **95 %** | 65/187 | −95 | 100 % | −81 |
| −6 | 100 % | 160/186 | −95 | 100 % | −81 |
| −3 | 100 % | 183/183 | −92 | 100 % | −81 |
| 0 | 100 % | 175/186 | −90 | 100 % | −81 |
| +3 | 100 % | 191/189 | −87 | 100 % | −81 |
| +6 | 100 % | 188/191 | −84 | 100 % | −81 |
| +9 | 100 % | 182/189 | −81 | 100 % | −81 |

Controls that make this table trustworthy: the reference slot held **100 % hit
and −81 dBm at every one of the 12 levels** across the whole 4 minutes, so the
geometry did not drift and the denominator is solid; at +9 dBm the swept slot
reads −81 too, identical to the reference, which is the internal consistency
check that the two slots differ only in power; and the transmitted `slot_id`
agreed with the arithmetic cycle→level map on **every single packet**
(`mism=0`), so slot attribution via the Minor field is exact.

**The number that transfers between installations is the receiver threshold, not
the TX level.** Slot-hit as a function of *received* power: 100 % at ≥ −95 dBm,
≈50 % at −97, 0 % by −101. That is a ~6 dB transition and it is a property of the
C3's receiver, so it holds in any room. TX level and path loss are per-install.

Hence the **calibration rule**, which needs no sweep — only the reference slot's
RSSI at the two positions, which the watch already measures:

```
BEACON_TX_LO_DBM  ~=  -91 - RSSI_HI_at_EDGE      (rounded DOWN to a multiple of 3)
usable only if     RSSI_HI_at_INSIDE - RSSI_HI_at_EDGE  >=  ~6 dB
```

Derivation: `RSSI_HI = 9 - PL`, so `PL = 9 - RSSI_HI`; wanting the LO slot to
land near −100 dBm at the edge gives `TX_LO = -100 + PL = -91 - RSSI_HI_edge`.

### Finding 3 — the placeholder −21 dBm is almost certainly unusable

This bench's path loss is 90 dB (+9 dBm out, −81 dBm in). At −21 dBm the link
needs `PL <= 74 dB` to stay above the −95 dBm reliable-hit threshold — **16 dB
less loss than this bench had**. Any install with comparable path loss would
read **0 % PDR everywhere, including inside the near-zone**: the feature would
assert AWAY permanently. Per the rule above, this geometry wants ≈ **−12 dBm**.

`BEACON_TX_LO_DBM` is therefore moved from −21 to **−12** as the shipping
default. This is a better-founded default, not a calibrated value: it is correct
for one measured geometry, and §8's `PROX_FLAG_TXLO_MISCAL` path (near-leg
PDR < 0.8 ⇒ suggest another level) remains the mechanism that fixes it per install.

> **Caveat, stated plainly.** 90 dB at desk range is much higher than free-space
> would predict (~30–40 dB at 30 cm). The two boards are USB-tethered beside a
> laptop with cables across the PCB antennas, so absolute path loss here is not
> representative of a bedroom. The *receiver threshold*, the *3 dB-per-step
> linearity*, the *cliff width* and the *slot-vs-PDU margin* are radio properties
> and do transfer. The absolute dBm recommendation does not, which is exactly why
> it is expressed as a rule keyed to measured RSSI.

### Finding 4 — slot-hit really is the right statistic

The spec's choice of "did ≥1 PDU arrive in this slot" over raw PDU delivery ratio
is vindicated with ~2 slots of margin to spare: at −12 dBm only **24 %** of PDUs
arrived, yet **82 %** of slots were hit; at −15 dBm, 3 % of PDUs still yielded
15 % of slots. Roughly 5 PDUs per slot is what buys that, and the reference slot
measured 4.6–4.9 PDU/slot — matching §3.1's prediction.

### Finding 5 — ACTIVE scanning breaks the measurement (and probably the vector)

The identical sweep with the receiver in **active** scan mode, everything else
held constant:

| | reference slot-hit | reference PDUs / 40 cycles | reference RSSI |
|---|---|---|---|
| passive | **100 % at all 12 levels** | ~186 | −81 |
| active | **30–95 %, erratic** | 13–114 | −81 |

RSSI is unchanged, so this is not link budget — the receiver is missing
advertisements because it is transmitting SCAN_REQs and awaiting SCAN_RSPs
instead of listening, while the advertiser is interrupting its own schedule to
answer them. It also moves the apparent cliff by ~9 dB (−12 dBm reads 12 %
active vs 82 % passive), which would mis-set `TX_LO` by three whole levels.

**PDR must be measured on a passive window.** The consequence is larger than PDR,
because `prox_aligned_active_scan()` sets `setActiveScan(true)` for the same
window the *proximity vector* is built from — and losing 40–70 % of
advertisements would degrade the device census too. The watch's parse path reads
the anchor UUID out of the ADV_IND manufacturer data, and every BLE advertiser is
visible to a passive scan (scan responses add payload, never new devices), so
there is no known reason the window needs to be active.

That last step is an **inference, not a measurement**: it was measured for one
advertiser, not for a multi-device census. Flipping the window to passive is
recorded as the recommendation, and left for a field run to confirm against real
vector sizes — it is a one-line change and a plausible contributor to the
`n = 8..31` vector-size churn seen in the field.

### Finding 6 — the two-position result (2026-07-30) closes the spike

Full 12-level sweep at both positions of a real install. Reference slot: INSIDE
median −80 dBm, EDGE median −92 dBm — **12 dB of separation**, path loss 89 and
101 dB. `hi_pct` 97–100 % everywhere, `mism = 0`.

| TX_LO | INSIDE hit | EDGE hit | |
|---|---|---|---|
| −15 | 27 % | 0 % | |
| −12 | 40 % | 0 % | |
| −9 | 62 % | 0 % | |
| **−6** | **97 %** | **0 %** | **PASS — chosen** |
| −3 | 100 % | 15 % | passes, but see below |
| 0 | 100 % | 50 % | edge fails |
| +3…+9 | 100 % | 87–97 % | edge fails |

Two levels meet the criterion. **−6 dBm is chosen because it centres the level on
the point where PDR's log-LR changes sign** — 42 % hit rate (`385/917`), reached
at ≈ −101 dBm received — leaving a symmetric ±6 dB margin. Both positions showed
±3 dB of RSSI wander, and −3 dBm sits only 3 dB from that crossover at the edge,
where a 3 dB gain would tip PDR into asserting NEAR outside the zone.

**Per-install rule, corrected.** Finding 2's `-91 - RSSI_HI_at_EDGE` was wrong: it
used only the edge, and it treated `lo_rssi` as unbiased when it is conditioned on
reception (so it reads several dB strong at low hit rates). Applied here it gives
+1.5 dBm, where the edge measures 50 %. Use instead:

```
BEACON_TX_LO_DBM = -92 - (RSSI_HI_inside + RSSI_HI_edge)/2   (round to a multiple of 3)
feasible only if   RSSI_HI_inside - RSSI_HI_edge >= ~7 dB    (>=10 dB for comfort)
```

**And a retraction.** Finding 2 claimed the received-power threshold generalises
between installs. Within one install it does — the two positions above trace a
single curve. Across installs it shifts: the bench read 82 % at −102 dBm where
this install reads ~40 %, i.e. the bench sat ~3–4 dB more sensitive, almost
certainly a quieter noise floor. Hit rate is set by SNR, not signal alone. The
curve's *shape* transfers and the rule is a sound starting point, but the sweep
is the calibration.

## S2 — 250 ms reconfiguration stability

**PARTIAL — measured 2026-07-29. The risk this spike was gating is not present:
reconfiguring the advertiser every 250 ms does not degrade connectability. The
absolute "100 consecutive connects" criterion is not met by this harness, but it
is not met by the control either, so the harness is the limit, not the anchor.**

Method: the `bench/pdr_sweep` transmitter also runs a GATT server (one service,
one readable characteristic). The receiver repeatedly stops scanning, connects,
discovers the service, reads the characteristic, verifies its value, disconnects
and resumes scanning — the same single-radio hand-off the watch performs. Four
conditions, otherwise identical:

| condition | connects OK | connect fail | read fail | t_mean | t_max |
|---|---|---|---|---|---|
| schedule, restart on (**shipping config**) | **53/60 (88.3 %)** | 7 | **0** | 543 ms | 1350 ms |
| schedule, restart off | **53/60 (88.3 %)** | 7 | **0** | 506 ms | 1200 ms |
| **no schedule at all (control)** | **45/60 (75.0 %)** | 15 | **0** | 456 ms | 1100 ms |
| schedule, restart on, 1200 ms settle | 32/40 (80.0 %) | 8 | **0** | 610 ms | 1300 ms |

### What this establishes

**The schedule does not hurt connectability.** Both scheduled conditions scored
88.3 %, the *unscheduled control scored worse at 75 %*, and quadrupling the settle
time between attempts did not help (80 %). The residual failure rate is therefore
uncorrelated with advertiser reconfiguration, with `stop()`/`start()`, and with
inter-attempt spacing — it is a property of the harness's bare
create/connect/read/delete loop, which has no retry and no connection-parameter
tuning, unlike the shipping watch (which holds a persistent GATT session via
`prox_session_begin()` and retries).

That the control is *worse* is explainable rather than noise: with the schedule
running, `pAdv->start()` is re-issued every 250 ms, which re-arms the advertiser
after every disconnect race. Without it the advertiser depends on a single
post-disconnect `start()`.

**Once a link is up, the schedule never disturbs it.** `read_fail = 0` in all
four conditions — **183/183 established connections** completed full service
discovery and a characteristic read while the advertiser was being stopped,
re-powered and restarted underneath them. That is the substantive half of §3.1's
"the anchor remains connectable in every slot".

**The stop/start is not needed at all** (from S1a's no-restart sweep): the Minor
payload updates live and the TX power change applies to subsequent advertising
events without an advertiser restart. `cycle_seq` advanced normally through a
135 s run with `stop()`/`start()` removed, and the PDR cliff landed in the same
place within binomial noise. Dropping the restart is therefore available as a
simplification, though S2 shows it is not required for connectability.

### Not established

- The stated **100-consecutive-connect** criterion. Needs the real anchor and the
  real watch, whose session/retry behaviour is what the criterion implicitly
  assumes; this harness cannot demonstrate it in any condition, including with
  the schedule disabled.
- **Scan staleness** is not cleanly measured here: the receiver stops scanning for
  every connect attempt, so the observed 1.8–5.1 s max beacon gaps are dominated
  by that, not by scan starvation on the anchor.

The fallback (`BEACON_SLOT_MS = 500`, `PROX_OBSERVE_WINDOW_MS` → 3600) remains a
constants-only change and is not currently needed.

## S3 — LIS3DH burst thresholds

**PARTIAL.** Measured 2026-07-28 on the tethered watch (see
`prox_v2_p1_notes.md` §4): a **resting device reads 9–33 mg²**, against the
`IMU_STILL_VAR` placeholder of 400 mg². STILL is therefore declared robustly,
with better than 10× headroom, and the constant is not too tight.

Outstanding, and requiring a real wrist in motion: the STILL/FIDGET/LOCOMOTION
confusion matrix (desk work, typing, walking, sleeping), which is what sets
`IMU_LOCO_VAR` and validates `IMU_CADENCE_*`. Pass criterion: STILL
false-LOCOMOTION < 20 %, LOCOMOTION false-STILL ≈ 0. Misclassification toward
LOCOMOTION is safe by design, so the current conservative defaults fail in the
safe direction meanwhile.
