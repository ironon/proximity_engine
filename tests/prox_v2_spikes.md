# Proximity v2.1 — Phase S0 spike results

Write-up location mandated by FIRMWARE_SPEC v0.9 amendment §10-A Phase S0.
One section per spike. A spike is only "PASS" when its stated pass criterion has
actually been met, not when it looks likely.

| Spike | Question | Status |
|---|---|---|
| S1a | Which TX levels does the C3 emit, and where does the near-band link close but not the far one? | **BLOCKED — needs hardware** |
| S1b | Can advertising be restricted to one channel per slot on this NimBLE? | **RESOLVED in source; on-air confirmation outstanding** |
| S2 | Does a 250 ms adv stop/reconfig/start cycle keep the anchor connectable and its scan tasks fed? | **BLOCKED — needs hardware** |
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

**BLOCKED.** Requires measuring, at real distances, which advertising TX power
level closes the link at 1–4 ft but not at ~10 ft (pass: ≥90 % slot-hit at 3 ft,
≤30 % at 10 ft LOS). No amount of source reading substitutes for this. Until it
is run, `BEACON_TX_LO_DBM` keeps the spec's placeholder of −21 dBm, and the PDR
distributions must be treated as untrained.

Note the ESP32-C3's advertised TX levels are quantised (`ESP_PWR_LVL_N24`
… `ESP_PWR_LVL_P9`); the spike should record the *measured* level per enum
value, since the nominal and actual differ.

## S2 — 250 ms reconfiguration stability

**BLOCKED.** Requires the anchor powered and a watch making repeated proximity
connects (pass: 100 consecutive connects succeed while the schedule runs, scan
staleness unchanged). Its fallback — `BEACON_SLOT_MS = 500` with
`PROX_OBSERVE_WINDOW_MS` scaled to 3600 — is a constant change only, so P2 code
does not need restructuring if S2 fails.

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
