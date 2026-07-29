# Impulse Proximity Engine — Design Specification

**Version:** 2.1 (supersedes 2.0 and 1.0)
**Scope:** All RF-proximity logic for the Impulse watch and anchor system.
**Module:** Unchanged — all algorithms live in `proximity.cpp` / `proximity.h`, gated by `PROXIMITY_ROLE_WATCH` / `PROXIMITY_ROLE_ANCHOR`. New platform seams (IMU, beacon schedule, FTM) are injected exactly like the v1 scan/NVS/clock seams.
**Companion:** FIRMWARE_SPEC v0.9 amendment (same delivery) carries every wire-format, GATT, power-state, and build-order change. Where the two disagree, the firmware amendment wins — it is the one reconciled against the code.

---

## 0. What changed since v2.0, and why v2.1 exists

v2.0 defined the right physics but was written against an idealized platform. v2.1 is the same design **re-grounded against the shipped firmware** (FIRMWARE_SPEC v0.8): the frozen IDF 4.4.7 toolchain and its NimBLE concurrency crash ("Option A"), the watch's power-state machine (no IMU interrupt in DORMANT since v0.7, light sleep between enforcement polls), the iBeacon advertising format with its Major filter, the lockstep-flash fleet policy, and the LIS3DH's actual capabilities. Every v2.0 feature is either (a) specified to implementable precision here, (b) given a degraded fallback behind a feasibility spike, or (c) explicitly deferred with the blocking dependency named.

**Summary of grounding decisions (normative):**

| v2.0 idea | v2.1 disposition |
|---|---|
| IMU motion channel @ 4 Hz continuous | **Rescoped.** No continuous sampling. Motion state is derived from the LIS3DH IA1 interrupt (already the ENFORCEMENT wake source) plus one short accel burst per poll, sampled *concurrently with* the observation scan (§4). Engine v2 features run only during ENFORCEMENT and calibration bursts; DORMANT keeps its v0.7 zero-IMU-interrupt guarantee. |
| Motion-conditioned HMM | **Kept in full.** Watch-side, in `proximity.cpp`, replacing the threshold interpretation of the score (Mode A) and the hysteresis machine (Modes B/C). Ships first — watch-only, zero wire change (Phase 1). |
| Per-(channel, power) beacon schedule with payload slot tags | **Kept**, encoded in the iBeacon **Minor** field (currently constant `0x0000`) so the Major-filter and scan-response format are untouched. Per-slot **TX power** control is confirmed platform API; per-slot **channel-map** control is Spike S1 with a defined power-only fallback (§3.4). |
| Stepped-power PDR | **Kept**, as slot-hit-ratio over TX_LO slots measured inside a full-duty observation window (§3.3), integrated across polls as Beta counts. |
| Cross-channel spread $s_{ch}$ feature | **Kept** (requires S1 channel control; absent under the fallback). |
| Per-channel fingerprints for all own-anchor entries | **Rescoped** to a small anchor-side per-peer-anchor side table (≤ `ANCHOR_PROX_MAX_PEER_ANCHORS`), fed by the vector trailer, rather than tripling the main registry (§5.3). |
| $N_{\text{eff}}$-honest integration | **Kept in full** (§4). |
| Wi-Fi CSI | **Kept via link reversal, Phase 4.** Watch-side CSI capture is dead on the frozen toolchain: CSI is a compile-time IDF option (`CONFIG_ESP_WIFI_CSI_ENABLED`, default off) absent from Arduino 2.0.17's precompiled WiFi libs, and RX-side load lands in the known-fragile 4.4.7 coexistence regime. Instead the **watch transmits** an ESP-NOW ping burst (stock TX works on 2.0.17) and the **anchor captures CSI** and folds channel-shape features into the score it already returns — riding the anchor's already-planned pioarduino/IDF 5.x migration. §3.5; Spike S4. |
| Wi-Fi FTM tie-breaker | **Deferred to Phase 4**, feasibility-gated: the anchor must run APSTA (SoftAP FTM responder + STA + BLE) on 4.4.7. Spike S5. |
| Fusion: watch sums many log-LRs | **Split.** The **anchor** folds trailer features (PDR, $s_{ch}$, per-channel RSSI) into its returned score using its location-trained distributions — it owns per-location statistics. The **watch** converts that score to a log-LR, adds watch-local evidence (connect-failure, later FTM), and runs the HMM — it owns temporal/motion state. One new wire field pair (`neff` out, trailer in), not ten (§6). |
| *(not in v2.0)* | **Added in v2.1a: occlusion & tamper resistance (§13, Phase 5).** v2.0 and v2.1 both assume the RF environment is *indifferent*, not *adversarial*. Where a criterion rewards the user for appearing far (sunrise lock's `getAway`), deliberate body/bedding attenuation is a cheap and effective AWAY spoof, and three paths in the shipped design convert it into a confident AWAY. §13 specifies the common-mode/differential discriminant that separates attenuation from displacement, and the abstention rules that make the spoof unprofitable. |

---

## 1. Shared Foundations

### 1.1 Log-distance path loss (unchanged from v1)

$$\text{RSSI}(d) = A - 10\,n \log_{10}(d) + X_\sigma$$

Near-field gradient $\frac{d(\text{RSSI})}{dd} \propto \frac{1}{d}$; ratio-invariant separations $\Delta\text{RSSI} = 10\,n\log_{10}(d_2/d_1)$.

### 1.2 Small-scale fading model (the v2 cornerstone)

$X_\sigma = X_s + X_f$: slow shadowing (body, furniture) plus fast multipath fading (the standing-wave pattern). Two decorrelation facts:

- **Spatial:** $X_f$ decorrelates over $\lambda/2 \approx 6.2$ cm at 2.44 GHz. Each ~6 cm of antenna displacement is one fresh, approximately independent fading draw.
- **Spectral:** coherence bandwidth $B_c \approx 1/(5\tau_{rms}) \approx 4\text{–}20$ MHz for indoor delay spreads, so BLE channels 37 (2402 MHz), 38 (2426 MHz), 39 (2480 MHz) are largely independent draws; a BLE *connection* hops 37 data channels and is inherently frequency-averaged. Corollary for Wi-Fi: a single 20 MHz channel spans only ~1–5 coherence bandwidths (~1–5 independent draws), so CSI subcarriers are heavily correlated — the schedule's three channels, spread across 78 MHz, already capture most of the available frequency diversity, and CSI's marginal value is channel *shape* (§3.5), not draw count.

**Normative consequences:**

1. A stationary watch yields **one** independent $X_f$ sample per frequency regardless of averaging time. Time-averaging is only evidence-accumulating while the IMU indicates motion. This is the root cause of the v1 "RSSI barely varies across the room" failure: a still wrist parked in a fade (or a hot spot) averages confidently to the wrong answer.
2. Frequency diversity is the only diversity available while stationary — hence the beacon schedule.
3. Near an anchor the channel is Rician/flat (per-channel readings agree); far, it is selective (large cross-channel spread). $s_{ch}$ is a range feature independent of absolute amplitude.

### 1.3 Effective sample count

Every integrating estimator tracks

$$N_{\text{eff}} = N_{\text{freq}} \cdot \left(1 + \frac{s_{\text{IMU}}}{\lambda/2}\right)$$

with $N_{\text{freq}}$ = distinct frequency bins observed in the window (3 under the schedule, 1 under the fallback, "many" for connection-RSSI) and $s_{\text{IMU}}$ = the coarse IMU path-length proxy (§4.2 — accuracy within ~2× suffices). Reported confidence scales as $\sigma_{\text{eff}} = \sigma/\sqrt{N_{\text{eff}}}$; a still wrist has frozen $N_{\text{eff}}$, and the engine says so rather than pretending its long average is informative.

Implementation: integer math. $N_{\text{eff}}$ stored as `uint16` in Q4 fixed point; the $\sqrt{}$ appears only inside precomputed LR lookup tables (§7), never at runtime.

---

## 2. Operating Envelope (new — read first)

The v2 engine is **not** an always-on subsystem. It activates in exactly three contexts, all of which the watch already spends radio/CPU in:

1. **ENFORCEMENT of an anchor-based or `phoneAway` event** — the poll/motion-check loop of FIRMWARE_SPEC §5.4.1. This is where the HMM, integrators, observation windows, and IMU bursts live.
2. **Calibration bursts** (§4.10.7 of the firmware spec) — same machinery at high query rate, plus the new away-leg (§8).
3. **Anchor side, continuously** — the beacon schedule task and the background scan/fingerprint tasks (anchors are mains/USB-powered; continuous operation is fine).

Outside these, nothing changes: DORMANT keeps its v0.7 always-on clock with **zero IMU interrupts**, DORMANT scans stay discovery-only, and the engine holds no state that must survive between enforcement windows except NVS-persisted calibration/fingerprint data. On ENFORCEMENT entry for a relevant event, the engine cold-starts: HMM at flat prior biased to the criterion-satisfying state, integrators empty, motion state UNKNOWN until the first burst.

---

## 3. Beacon Schedule, PDR, and Channel Features

### 3.1 Slot schedule (anchor)

The anchor's advertiser rotates through a fixed cycle of `(channel_set, tx_power)` slots:

```
slot 0: ch37 @ TX_HI     slot 3: ch37 @ TX_LO
slot 1: ch38 @ TX_HI     slot 4: ch38 @ TX_LO
slot 2: ch39 @ TX_HI     slot 5: ch39 @ TX_LO
```

- `BEACON_SLOT_MS = 250`, advertising interval `BEACON_ADV_INTERVAL_MS = 50` → ~5 PDUs per slot, full cycle `BEACON_CYCLE_MS = 1500`.
- TX_HI = `PROX_QUERY_TX_POWER_DBM` (+9, as today). TX_LO default **−21 dBm** (`ESP_PWR_LVL_N21`) — **calibrated by Spike S1**, which measures the level at which the near-band link budget closes at 1–4 ft but not across a room. Power is set per-slot via the platform TX-power seam (confirmed API on this toolchain).
- The anchor remains **connectable in every slot** — proximity queries and setup must keep working. Spike S2 verifies that per-slot advertising reconfiguration (stop/param/start every 250 ms on the NimBLE host task) neither destabilizes connectability nor starves the scan tasks; its fallback is a slower schedule (`BEACON_SLOT_MS = 500`, window scaled to match).

### 3.2 Slot tagging in the iBeacon Minor field

The iBeacon **Major** stays `0x4A0F` (the Impulse filter is untouched). The **Minor** — constant `0x0000` since v1 — becomes:

$$\text{Minor} = (\text{slot\_id} \ll 12)\;|\;(\text{cycle\_seq} \,\&\, \text{0x0FFF})$$

`slot_id` ∈ 0–5; `cycle_seq` increments once per full cycle. The watch attributes every received advertisement to an exact frequency and TX power **with zero controller support on the scan side**, and `cycle_seq` lets it detect window truncation. Legacy behavior (`Minor = 0`, no schedule) is selected by `BEACON_SCHEDULE_ENABLE = 0`; a watch receiving all-zero Minors simply gets no per-slot features and the engine degrades to Phase-1 behavior. Because the fleet flashes in lockstep (FIRMWARE_SPEC §10.1), no mixed-version protocol is needed beyond this.

### 3.3 Observation window & PDR (watch)

Per-slot features require observing the advertiser at **full scan duty** for at least one full cycle. The pre-query aligned scan therefore becomes the **observation window**: full-duty, `PROX_OBSERVE_WINDOW_MS = 1800` ms (≥ cycle + slot + margin, so every slot is covered at least once regardless of phase). It replaces `ENFORCEMENT_QUERY_SCAN_DURATION_MS` for anchor-based queries; DORMANT discovery scanning is unchanged. The power cost and its compensation are handled in §9.

From one window, per target (and rival) anchor:

- $r_{37}, r_{38}, r_{39}$: strongest RSSI seen in each channel's TX_HI slot (0 = unseen).
- $\hat{r} = \text{mean of present per-channel values}$ — the frequency-averaged range estimate that feeds the integrator.
- $s_{ch} = \max - \min$ over present per-channel values (dB; valid only when ≥ 2 channels present) — the selectivity feature.
- **LO-slot hits**: for each of the 3 TX_LO slots covered, whether ≥ 1 PDU was received. With ~5 PDUs per slot at full duty, a closed link yields a hit with near-certainty, so the slot-hit ratio is a clean Bernoulli-per-slot statistic:

$$\text{PDR}_{\text{LO}} = \frac{\text{hits}}{\text{LO slots covered}} \in \{0, \tfrac{1}{3}, \tfrac{2}{3}, 1\} \text{ per window}$$

Hits/covered counts (not the ratio) accumulate in the integrator as Beta-posterior counts across polls, decayed on motion-state reset. The likelihood ratio $\text{LR}_{\text{pdr}}$ uses Beta-binomial tails from calibrated (or default) near/away parameters.

PDR generalizes v1's connect-failure fail-safe (advertisement-receivable-but-unconnectable ⇒ far) into a graded, always-on feature; the connect-failure rule itself is retained as watch-local evidence (§6.3).

### 3.4 Spike S1 fallback — power-only schedule

If per-slot channel restriction proves unavailable on this NimBLE (the legacy `ble_gap_adv_params` does not expose a channel map; extended-advertising or vendor-HCI paths are what S1 evaluates), the schedule degrades to **2 slots (TX_HI / TX_LO) on all three channels**:

- PDR survives intact (it never needed channel attribution).
- $r_{37/38/39}$ and $s_{ch}$ are dropped; the trailer marks them unknown; anchors skip those distributions.
- $N_{\text{freq}} = 1$ for advertisement-based estimates (connection-RSSI paths keep their hopping average).

The engine remains a strict improvement over v1 under the fallback (HMM + $N_{\text{eff}}$ honesty + PDR); S1 decides only how much frequency diversity rides on top.

### 3.5 Reversed-link CSI (Phase 4; spike-gated — do not build before S4)

v2.0 placed CSI capture on the watch (anchor transmits, watch captures). That direction is unimplementable as shipped: CSI is a compile-time IDF option that Arduino-ESP32 2.0.17's **precompiled** WiFi libraries almost certainly exclude, so enabling it means custom library rebuilds for a dead toolchain line — plus WiFi RX load in exactly the 4.4.7 BLE-coexistence regime the toolchain spike documented as fragile, on the battery-constrained device. v2.1 therefore **reverses the link**:

- The **watch transmits** a short ESP-NOW ping burst (`CSI_PING_FRAMES = 4`, carrying its MAC + a nonce) at the start of each observation window. Plain ESP-NOW TX works on stock 2.0.17; the burst is milliseconds of radio time.
- The **anchor captures CSI on receive**, filters the callback on the watch's MAC, reduces each frame *in the callback* (raw CSI never buffered) to three fixed-point features — $\mu_{sc}$ (mean subcarrier amplitude via $\alpha$-max-$\beta$-min, no sqrt), $\text{FS} = \text{IQR}/\mu_{sc}$ (spectral flatness: flat ⇒ near/LOS, notched ⇒ far/diffuse), and a notch count — and folds them into `prox_compute_score2` as two LR terms with §5.3-style trained near/away distributions. Same "anchor computes, watch queries" pattern as everything else in Mode A, on the mains-powered device that already owns per-location statistics.

**Reciprocity caveat:** the physical channel is reciprocal, but TX/RX chains differ, so absolute amplitudes shift between directions. The shape features (FS, notches) are channel properties, and the anchor trains its own distributions, so the per-direction offset is absorbed automatically.

**Channel coordination:** ESP-NOW requires both radios on one channel. When both are associated to the same AP this is automatic; otherwise the watch reads the anchor's current channel from the WiFi Status characteristic (`…000E` gains a channel byte in the P4 lockstep batch, FIRMWARE_SPEC amendment §10-A P4) and hops for the burst.

**Dependency chain, in order:** (1) the anchor's already-planned pioarduino/IDF 5.x migration (FIRMWARE_SPEC §10.1) with a CSI-enabled build — likely Arduino-as-IDF-component so the sdkconfig is owned; (2) Spike S4 (whose *first step* is grepping the target framework's sdkconfig for `CONFIG_ESP_WIFI_CSI_ENABLED`); (3) implementation.

**Expectation-setting:** per §1.2, one 20 MHz frame contributes only ~1–5 independent fading draws, so CSI adds little raw diversity beyond the schedule's three well-separated channels. Its unique contribution is fine channel *shape* — a sharpening of $s_{ch}$ into a much finer selectivity measurement — which is why it is Phase 4 headroom, not load-bearing: every acceptance target in this spec must be met without it.

---

## 4. IMU Motion Channel & Motion-Gated Integration

### 4.1 Motion classification without continuous sampling

The LIS3DH already provides a high-pass-filtered motion interrupt (IA1) that is a wake source **during ENFORCEMENT only** (FIRMWARE_SPEC §8.4) — this is precisely the "did the wrist move" detector the engine needs, at zero added idle cost. The motion channel combines three inputs:

1. **Sleep-interval verdict.** On every wake from enforcement light sleep, the firmware reports whether IA1 fired during the sleep interval (`prox_note_sleep_interval(ms, motion_woke)`). `motion_woke = false` over an interval ⇒ the wrist was **STILL** for that whole interval — the strongest, cheapest STILL evidence available, and it is exactly the case where v1 was most wrong.
2. **Awake interrupt notes.** Each IA1 firing while awake is reported (`prox_note_motion_interrupt()`).
3. **Per-poll accel burst.** Concurrently with each observation window (the CPU is idle while the radio scans), the firmware samples the accelerometer: `IMU_BURST_SAMPLES = 32` at `IMU_BURST_HZ = 50` (~640 ms), delivered via `prox_ingest_imu_burst()`. The engine computes gravity-removed variance and a zero-crossing/step-cadence check in integer math.

Classification per decision tick:

| State | Rule |
|---|---|
| `STILL` | last sleep interval had no IA1, and burst variance < `IMU_STILL_VAR` |
| `LOCOMOTION` | burst shows step cadence (0.5–3 Hz periodicity) or sustained variance > `IMU_LOCO_VAR`, or ≥ `IMU_LOCO_MIN_INTS` IA1 firings since last tick |
| `FIDGET` | anything between |
| `UNKNOWN` | no burst yet this window (cold start) — treated as `LOCOMOTION` for HMM purposes (fail toward v1 behavior, never toward a frozen decision) |

Path proxy $s_{\text{IMU}}$: `LOCOMOTION` credits `NEFF_LOCO_PER_S` (default 8) independent draws per second of the interval since the last tick; `FIDGET` credits `NEFF_FIDGET_PER_S` (default 2); `STILL` credits 0. Crude and sufficient (§1.3 needs ~2× accuracy).

### 4.2 Motion-gated integrator (replaces flat EWMA everywhere)

All ranged quantities — $\hat{r}$ per anchor, $R_{wp}$, $\delta = R_P - R_D$, PDR counts — pass through one common integrator:

- **`FIDGET`/`LOCOMOTION`:** full-weight accumulation; $N_{\text{eff}}$ advances per §1.3 and §4.1.
- **`STILL`:** samples update the mean with weight `INTEG_STILL_WEIGHT` (0.10 in Q8) — enough to track genuine slow shadowing, too little for a frozen fade to masquerade as accumulating evidence. $N_{\text{eff}}$ frozen (floor 3 with the schedule, 1 under the S1 fallback). Reported variance relaxes back toward single-draw $\sigma^2$ with time constant `INTEG_STILL_RELAX_S`.
- **`STILL → LOCOMOTION`:** window restart — position may now be changing; stale evidence must not outvote fresh.

### 4.3 Fingerprint-training qualification (Mode A)

Self-supervised training samples (v1 §4.10.4 gate) additionally require `motion_state ∈ {FIDGET, LOCOMOTION}` reported in the vector trailer, and training weight becomes

$$w_n = \frac{\text{score}}{255}\cdot\min\!\left(1, \frac{N_{\text{eff}}}{N_{\text{train}}}\right)$$

This closes the failure where a user asleep in a fade pumps hundreds of identical frozen samples into the fingerprint and collapses $\sigma^2$ around the wrong mean. Calibration-burst samples always qualify (the user is instructed to walk).

---

## 5. Mode A Upgrades (fingerprinting)

All v1 Mode A machinery stands: scan vector, registry, background scans, Pearson + fingerprint blend, weighted Welford, transport, connect-failure fail-safe. Deltas:

### 5.1 Vector format v2 with trailer

The proximity vector gains a leading version byte and a trailer carrying watch-measured features (exact wire format: FIRMWARE_SPEC amendment §6.3.1): `motion_state`, `neff`, and per-anchor `{pdr_hits, pdr_slots, s_ch, r37, r38, r39}` for the target plus up to 3 rival anchors. Lockstep-flashed; no mixed-version parsing.

### 5.2 Score computation v2 (anchor)

`prox_compute_score2()` extends the v1 blend with trailer-feature Naive-Bayes terms:

$$\text{score} = \text{blend}_{v1}(\rho, L) \;\oplus\; \ell_{\text{pdr}} \oplus \ell_{s_{ch}} \oplus \ell_{ch}$$

where $\oplus$ denotes summation in log-odds domain before the final logistic → `uint8`, and each $\ell$ abstains (contributes 0) when its feature is marked unknown or its distributions are untrained. $\ell_{ch}$ is the per-channel Gaussian log-LR against the peer-anchor side table (§5.3), which is sharper than pooled stats because the standing-wave pattern at a fixed location is stable in time *per frequency*. The result is `ProxScoreResult2 {score, flags, neff}` — `neff` echoing (and capping to) the trailer's claim so the watch's ambiguity band can widen when evidence was thin.

### 5.3 Per-peer-anchor side table + symmetric training (anchor)

A small NVS table, ≤ `ANCHOR_PROX_MAX_PEER_ANCHORS = 8` entries keyed by anchor MAC, each holding per-channel Welford stats $(\mu_{ch}, \sigma^2_{ch}, W_{ch})$ for near **and** away, plus Beta counts for PDR and Gaussian stats for $s_{ch}$, near and away. Trained by:

- **Near samples:** vectors passing the (motion-qualified) v1 training gate.
- **Away samples:** vectors with $\text{score}/255 \le$ `PROX_TRAIN_AWAY_THRESHOLD` (0.25) **and** target-anchor raw RSSI ≤ `PROX_FAR_RSSI_THRESHOLD_DBM` — a conservative, unambiguous-far gate, the mirror image of the near gate.
- **Calibration legs** (§8) at high weight.

The main 128-entry registry is untouched — external emitters stay channel-pooled exactly as v1 (their channels are unattributable anyway).

### 5.4 Watch-side interpretation → HMM

Step 8 of the enforcement query (v1 threshold mapping NEAR/AWAY/AMBIGUOUS) is replaced by the HMM of §6. The three-way output and the criterion-dependent fail-safe resolution of AMBIGUOUS are preserved as the *decision layer on top of the posterior*, so every downstream consumer (enforcement, `phoneAway` fusion, tolerance timers) sees the same interface as before.

---

## 6. Inference: Motion-Conditioned HMM (watch)

Two-state forward filter over $\{\text{NEAR}, \text{AWAY}\}$, run in `proximity.cpp` on each decision tick (each completed query, plus each motion-interrupt check).

### 6.1 Emission

$$\ell_t = \underbrace{\text{logit}\!\left(\frac{\text{score}+0.5}{256}\right)}_{\text{anchor's fused verdict}} \cdot g(N_{\text{eff}}) \;+\; \ell_{\text{local}}$$

$g(N_{\text{eff}}) \in [0,1]$ discounts thin evidence (LUT). $\ell_{\text{local}}$ is watch-only evidence: the connect-failure rule (failed connect + recent advert ≤ `PROX_FAR_RSSI_THRESHOLD_DBM` ⇒ strong AWAY log-LR, `LL_CONNFAIL_AWAY`), and in Phase 4, FTM.

### 6.2 Motion-conditioned transitions

$$P(\text{flip per tick}) = \begin{cases} \text{HMM\_PFLIP\_STILL} = 10^{-4} & \text{STILL} \\ \text{HMM\_PFLIP\_FIDGET} = 10^{-3} & \text{FIDGET} \\ \text{HMM\_PFLIP\_MOVE} = 0.15 & \text{LOCOMOTION / UNKNOWN} \end{cases}$$

scaled by elapsed time since the previous tick (ticks are irregular). Forward update in log domain — two states, four multiply-adds, `logsumexp` via max + LUT; no transcendentals on the hot path (§7).

**Effect:** an RSSI excursion while the wrist is provably still — the signature of multipath, interference, or body-shadow shift — is structurally incapable of flipping the decision. A genuine approach necessarily involves LOCOMOTION, which simultaneously unlocks transitions and (via §4.2) floods the integrator with spatially fresh samples. `PFLIP_STILL` is small, not zero: sustained contradictory evidence still wins eventually (IMU fault, watch removed). If the IMU seam goes stale (`IMU_STALE_MS`), transitions revert to `PFLIP_MOVE` — fail toward v1 behavior.

### 6.3 Decision layer

Posterior $P(\text{NEAR})$ maps to the v1 three-way interface: NEAR if $P \ge$ `HMM_TAU_NEAR` (0.80), AWAY if $P \le$ `HMM_TAU_AWAY` (0.20), else AMBIGUOUS resolved per the existing criterion-dependent fail-safe (`stayNear` → NEAR, `getAway` → AWAY, `phoneAway` → compliant). Cold start on ENFORCEMENT entry: prior set to the criterion-satisfying state at $P = 0.65$.

### 6.4 Modes B/C

The coloc pipeline keeps its factor structure ($\text{LR}_{\text{range}}, \text{LR}_{\text{env}}, \text{LR}_\delta, \text{LR}_{\text{var}}$) but: (a) all inputs ride the motion-gated integrator; (b) $\delta$ is computed per channel then averaged when the schedule is active, with the two anchors' schedules phase-offset by `BEACON_SCHEDULE_EPOCH_OFFSET_MS` so LO slots don't collide; (c) the summed log-LR becomes the HMM emission and the two-threshold hysteresis machine is **deleted** — the HMM subsumes it. The Phase-3 **coupling detector** adds $\rho_{\text{cpl}} = \text{corr}(e(t), |R_{wp}(t)-R_{wp}(t-1)|)$ over `CPL_WINDOW_S`, abstaining when motion bins < `CPL_MIN_MOTION_BINS`: high-mean/low-variance/motion-decoupled $R_{wp}$ ⇒ phone carried with the watch; motion-coupled churn + low mean ⇒ phone parked.

---

## 7. ESP32-C3 Performance Discipline

No-FPU rules carry over from v1 and tighten:

- All per-tick math is integer/Q-format. The only float remaining on the hot path is the v1 Pearson/Welford core (anchor side, already shipped and budgeted).
- LUTs, generated at build time into flash: `logit8[256]` (score → Q8 log-odds), `neff_gain[64]`, `logsumexp_corr[64]`, Beta-binomial tail tables for PDR (2 × small).
- Per-slot beacon reconfig is timer-driven on the anchor; CPU negligible; TX_LO slots *reduce* mean radiated power.
- IMU burst: 32 int16-triples, variance + zero-crossings — a few thousand cycles, executed while the radio scans.
- Everything remains $O(\text{devices})$ per tick, sub-millisecond.

---

## 8. Calibration Additions

The v0.8 calibration burst (near leg) already exercises the exact query path; v2.1 adds:

- **Leg byte on START** (near = 0x01 default, away = 0x02): the away leg has the user stand across the room; every query is fed to the anchor's away-training gate at high weight, populating the away distributions of §5.3 in ~1 minute instead of days of conservative self-supervision.
- Burst windows always include LO slots (they use the same observation window), so PDR and $s_{ch}$ distributions train at burst speed too.
- Two-point coloc calibration (v1 Mode C §4.5) is unchanged and now also seeds `TX_LO`-band verification: if the near leg shows $\text{PDR}_{\text{LO}} < 0.8$, the engine flags `PROX_FLAG_TXLO_MISCAL` so the app can suggest a different `TX_LO` level (S1 provides the level table).

---

## 9. Power Reconciliation (watch)

Two opposing deltas, netting neutral-to-positive for the dominant usage pattern:

- **Cost:** the observation window is 1800 ms full-duty vs the v0.8 700 ms pre-query scan — ~2.6× the per-query cost of the largest BLE consumer (FIRMWARE_SPEC §8.1 rank 3).
- **Compensation:** the HMM makes still-and-compliant polling nearly worthless to repeat, so a third poll tier is added: when condition met **and** HMM confident **and** motion state STILL, the poll interval backs off to `ENFORCEMENT_POLL_INTERVAL_STILL_S = 600` (vs 180 met / 60 not-met). The IA1 interrupt still forces an immediate re-check on any motion, so responsiveness is untouched — a still wrist cannot change proximity class (that is the whole HMM premise), so not polling it is *correct*, not just cheap. At 600 s vs 180 s this is 3.3× fewer queries for the still-compliant bulk of a window, more than offsetting the 2.6× per-query growth.
- Violation periods (not-met, 60 s polls, full windows) cost ~2.6× rank-3 during the violation only; violations are short by product design.
- WiFi AP scan caching, the association gate, and all v0.8 §8.2 optimizations are unchanged. The IMU adds no idle cost (no new interrupts; bursts ride awake windows).

Anchor: mains/USB-powered; schedule overhead is negligible and mean TX power drops (LO slots).

---

## 10. Public API Delta (`proximity.h`)

```c
// ── New platform → engine seams (watch role) ─────────────────────────────
void prox_ingest_imu_burst(const int16_t (*xyz)[3], uint16_t n, uint16_t hz);
void prox_note_motion_interrupt(void);                 // IA1 fired while awake
void prox_note_sleep_interval(uint32_t slept_ms, bool motion_woke);
void prox_note_connect_failure(int8_t last_advert_rssi_dbm, bool have_advert);

// ── New engine → platform seams ──────────────────────────────────────────
// (anchor role) called by the beacon-schedule timer; platform performs the
// adv stop/reconfig/start. channel_map==0x07 under the S1 fallback.
void prox_platform_set_beacon_slot(uint8_t channel_map, int8_t tx_power_dbm,
                                   uint16_t minor);

// ── Extended results & state (watch role) ────────────────────────────────
typedef struct { uint8_t score; uint8_t flags; uint8_t neff; } ProxScoreResult2;
typedef enum { PROX_HMM_NEAR, PROX_HMM_AWAY, PROX_HMM_AMBIGUOUS } ProxDecision;

void         prox_hmm_reset(uint8_t criterion);        // ENFORCEMENT entry
ProxDecision prox_hmm_tick(const ProxScoreResult2* r); // after each query
ProxDecision prox_hmm_decision(void);                  // current, no new evidence
uint8_t      prox_motion_state(void);                  // STILL/FIDGET/LOCO/UNKNOWN

// ── Anchor role ──────────────────────────────────────────────────────────
ProxScoreResult2 prox_compute_score2(const ProxScanVector2* v);
// prox_compute_score() (v1 signature) wraps score2 and drops neff.

// ── Phase 4 seams (provisional — do NOT implement before S4/S5 pass) ─────
void prox_platform_espnow_ping(uint8_t wifi_channel, uint8_t n_frames);    // watch → platform (§3.5)
void prox_ingest_csi(const int8_t* csi, uint16_t len,
                     const uint8_t src_mac[6]);                            // anchor CSI callback (§3.5)
void prox_ingest_ftm(uint16_t dist_cm, uint16_t sigma_cm, bool ok);        // watch, tie-breaker

// ── Phase 5 — occlusion & tamper resistance (§13; provisional until S6) ──
typedef struct {
    int8_t   delta_db;      // common-mode offset vs reference; 0x80 = unknown
    uint8_t  mad_db;        // differential dispersion (MAD of residuals); 0xFF = unknown
    uint8_t  censor_u8;     // rank-monotonicity of dropouts, 255=perfect; 0xFF = unknown
    uint8_t  chflat_db;     // per-channel spread of the offset; 0xFF = unknown
    uint8_t  shared;        // |S|, devices shared with the reference vector
    uint8_t  dropped;       // |D|, reference devices now absent
} ProxOcclusion;

// (watch) Recompute the occlusion statistics for the vector about to be sent
// and update the reference. Called once per observation window, before the
// vector is written. Returns the Q8 log-LR of occlusion-vs-displacement.
int16_t  prox_occlusion_eval(const ProxScanVector2* v, ProxOcclusion* out);
int      prox_is_occluded(void);          // ℓ_occ ≥ OCC_LOGLR_TAU_Q8, latched per §13.5
uint32_t prox_occlusion_sustained_ms(void); // for the §13.6 reportable event

// (anchor) Common-mode offset of a vector against the trained fingerprint,
// in dB; ±0x7F when undeterminable. Used to make Signal B affine-invariant
// (§13.4-R2) and exposed so the level term can be gated separately.
int8_t   prox_fingerprint_offset_db(const ProxScanVector2* v);
```

Wire formats for `ProxScanVector2`, the 3-byte score payload, the Minor encoding, and the calibration leg byte are normative in the FIRMWARE_SPEC v0.9 amendment §6.3 / §5.6.

---

## 11. Constants (delta; authoritative copy in FIRMWARE_SPEC §7 amendment)

```
// Beacon schedule (anchor)
BEACON_SCHEDULE_ENABLE            = 1
BEACON_SLOT_MS                    = 250     // S2 fallback: 500
BEACON_ADV_INTERVAL_MS            = 50
BEACON_CYCLE_MS                   = 1500    // derived: 6 × SLOT
BEACON_TX_LO_DBM                  = -21     // (S1-calibrated)
BEACON_SCHEDULE_EPOCH_OFFSET_MS   = 750     // anchor-pair phase offset (Mode C)

// Observation & PDR (watch)
PROX_OBSERVE_WINDOW_MS            = 1800    // full-duty; ≥ cycle + slot + margin
PDR_NEAR_ALPHA_BETA               = {8, 2}
PDR_AWAY_ALPHA_BETA               = {1, 9}
PROX_TRAIN_AWAY_THRESHOLD         = 0.25
ANCHOR_PROX_MAX_PEER_ANCHORS      = 8

// IMU / integration (watch)
IMU_BURST_SAMPLES                 = 32
IMU_BURST_HZ                      = 50
IMU_STILL_VAR                     = (per-unit, S3-tuned)
IMU_LOCO_VAR                      = (per-unit, S3-tuned)
IMU_LOCO_MIN_INTS                 = 2
IMU_STALE_MS                      = 5000
NEFF_LOCO_PER_S                   = 8       // Q4 credit rate
NEFF_FIDGET_PER_S                 = 2
NEFF_TRAIN_MIN                    = 8
INTEG_STILL_WEIGHT                = 26      // ≈0.10 in Q8
INTEG_STILL_RELAX_S               = 30

// HMM (watch)
HMM_PFLIP_STILL                   = 1e-4    // per-tick basis, time-scaled
HMM_PFLIP_FIDGET                  = 1e-3
HMM_PFLIP_MOVE                    = 0.15
HMM_TAU_NEAR                      = 0.80
HMM_TAU_AWAY                      = 0.20
LL_CONNFAIL_AWAY                  = -3.0    // log-LR (stored as Q8)

// Poll tiers (watch; §9)
ENFORCEMENT_POLL_INTERVAL_STILL_S = 600

// Coupling detector (Phase 3)
CPL_WINDOW_S                      = 12
CPL_BIN_MS                        = 250
CPL_MIN_MOTION_BINS               = 8

// Occlusion & tamper resistance (Phase 5 — provisional; S6 sets the real numbers)
OCC_ENABLE                        = 1       // 0 ⇒ byte-identical to P4
OCC_REF_MAX_AGE_S                 = 900     // reference vector expiry
OCC_MIN_SHARED                    = 6       // |S| below this ⇒ δ/MAD abstain
OCC_MIN_DROPOUTS                  = 3       // |D| below this ⇒ censoring term abstains
OCC_DELTA_DB                      = -10     // common-mode drop that makes occlusion plausible
OCC_MAD_DB                        = 4       // residual dispersion below this ⇒ homogeneous
OCC_CENSOR_MIN_U8                 = 204     // 0.80 rank-monotonicity
OCC_CHFLAT_DB                     = 5       // per-channel offset spread below this ⇒ absorptive
OCC_LOGLR_TAU_Q8                  = 512     // +2.0 nats ⇒ declare OCCLUDED
OCC_CLEAR_TAU_Q8                  = 128     // +0.5 nats ⇒ clear the latch (hysteresis)
OCC_SUSTAIN_S                     = 120     // OCCLUDED this long ⇒ reportable event (§13.6)
LL_LEVEL_MAX_Q8                   = 384     // cap on the separated absolute-level term (§13.4-R2)
PROX_STARVED_SCORE                = 128     // criterion-neutral; replaces the score-50 branch
```

---

## 12. Phase Map (details and acceptance tests: FIRMWARE_SPEC amendment §10)

| Phase | Contents | Wire change | Gate |
|---|---|---|---|
| **S0** | Spikes S1–S3 (throwaway branch) | — | — |
| **P1** | IMU channel, motion-gated integrator, HMM, connect-failure LR, STILL poll tier | **None** — watch-only | none |
| **P2** | Beacon schedule + Minor tagging, vector v2 + trailer, score2 + neff, PDR + $s_{ch}$ + per-channel side table, motion-qualified & away training, calibration leg byte | Lockstep batch | S1, S2 |
| **P3** | Mode C per-channel $\delta$, coupling detector, TX_LO miscal flag | none beyond P2 | P2 shipped |
| **P4** | Reversed-link CSI (§3.5); FTM tie-breaker | `…000E` + channel byte (CSI); FTM TBD | S4, S5; anchor IDF 5.x migration for CSI |
| **P5** | **Occlusion & tamper resistance (§13):** common-mode/differential discriminant, offset-invariant Signal B, abstention hardening, sustained-occlusion event | Lockstep batch (trailer + score flag) | S6; P2 shipped |

Spikes: **S1** TX_LO level table + channel-map availability on this NimBLE (fallback §3.4). **S2** per-slot adv reconfig stability while connectable (fallback slower slots). **S3** LIS3DH burst thresholds on real wrists (STILL/FIDGET/LOCO confusion matrix). **S4** reversed-link CSI: sdkconfig `CONFIG_ESP_WIFI_CSI_ENABLED` check first, then anchor-side capture + BLE + STA stability on IDF 5.x, plus watch ESP-NOW TX beside NimBLE on 4.4.7 (§3.5). **S5** anchor APSTA + FTM responder under 4.4.7 coexistence. **S6** occlusion statistics on real bedding and real bodies (§13.7) — the only spike that gathers *distributions* rather than answering a yes/no feasibility question.

---

## 13. Occlusion & Tamper Resistance (Phase 5)

### 13.0 Why this section exists

Every model in §1–§12 assumes the RF environment is *indifferent*: noisy, multipath-ridden, sometimes cruel, but not steered by someone with an interest in the answer. That assumption is false for any criterion that **rewards the user for appearing far from an anchor** — `getAway`, and specifically sunrise lock, where the user must leave the bed anchor to dismiss the alarm and would very much like not to.

The attack is not exotic. A watch worn on a wrist tucked under a sleeping body, face-down in a mattress, is attenuated by 15–25 dB. That is indistinguishable from a 6–17× increase in free-space distance *by amplitude alone*, costs nothing, requires no equipment, and can happen by accident. A design that cannot separate it from displacement will be defeated in the first week of real use, and — worse — will be defeated *unintentionally*, generating false AWAY for compliant sleeping users.

**The governing asymmetry, normative:** a passive attenuating adversary can fabricate **AWAY but not NEAR**. Attenuation only ever removes signal. Producing a false NEAR requires active gain (a repeater or relay), which is out of scope for this phase and would be attacked differently (§13.8). Every abstention, fallback, and unknown path introduced anywhere in this engine must therefore be biased so that concluding **far requires more evidence than concluding near**. Phase 5 is largely the systematic application of that one rule.

### 13.1 What soft material actually does at 2.4 GHz

λ ≈ 12.5 cm, so thin low-water-content fabrics are nearly transparent; the loss budget is dominated by water and by near-field antenna detuning, not by the number of layers.

| Obstruction | Typical one-way loss @ 2.44 GHz |
|---|---|
| Cotton sheet, per layer | < 0.5 dB |
| Duvet / pillow (predominantly air) | 1–3 dB |
| Memory-foam mattress, ~15 cm | 3–10 dB |
| **Human torso (~70 % water, ~25 cm)** | **15–30 dB** |
| Near-field detune (chip antenna pressed into a dense dielectric) | 6–15 dB, additive |

**Consequence 1 — bedding alone is not the threat.** Sheets and pillows total perhaps 3–5 dB, inside the normal small-scale fading swing of §1.2 and already absorbed by the integrator. The threat is the *body*, or the antenna pressed against dense material. This matters for setting thresholds: `OCC_DELTA_DB` must sit above the bedding-only regime or the detector fires on every sleeping user.

**Consequence 2 — and this is the whole discriminant — the loss is at the watch antenna, so it is *common-mode*.** It applies to every path the watch has, to every emitter it hears, by nearly the same amount. Displacement is *differential*: moving to another room changes each device's path geometry by a different amount, and severs some paths entirely while leaving others intact.

| | Deliberate/accidental occlusion | Genuine displacement |
|---|---|---|
| Per-device ΔRSSI | tightly clustered around one offset | broadly dispersed |
| Which devices drop out | the weakest ones, in rank order | whichever ones a wall now blocks |
| Cross-channel behaviour | flat (absorption is broadband) | frequency-selective (§1.2) |
| IMU | no locomotion required | locomotion necessarily present |
| Reversibility | instant on uncovering | requires walking back |

The right question is therefore never "is the signal weak?" — it is **"is the *shape* of the environment preserved?"** Shape is what displacement destroys and attenuation leaves alone.

### 13.2 What the shipped design already gets right

`signal_correlation()` (Signal A) is Pearson, which is **invariant under any affine transform of either vector**. A uniform −20 dB blanket leaves ρ mathematically unchanged. This is not luck — correlating *patterns* rather than comparing *levels* is exactly the correct response to a common-mode attacker, and it means the Mode A Signal-A path needs no defending.

The motion-conditioned transition prior (§6.2) is the second existing defence and the strongest: at `HMM_PFLIP_STILL`, a wrist that is provably still essentially cannot transition NEAR→AWAY, no matter what the amplitude does. You cannot get an order of magnitude farther from an anchor without walking.

Phase 5 exists because **evidence can still reach the posterior through paths that bypass both of these**, and because one Phase-2 feature actively reintroduces the vulnerability.

### 13.3 The three leaks (all present in the shipped code)

**L1 — the starved-vector fallback is a hard AWAY.** `prox_compute_score()` short-circuits below `PROX_MIN_DEVICE_COUNT` (8) to a raw-RSSI comparison returning score 200 or **50**. Attenuation censors the weakest emitters first, so a 15 dB blanket takes a typical 12–29-device vector under the floor; the entire correlation architecture is then bypassed in favour of the one statistic the attack directly defeats. **The attack does not have to beat Signal A — it only has to starve it.** This is the primary exploit.

**L2 — connect-failure evidence has no motion witness.** `prox_note_connect_failure()` contributes `LL_CONNFAIL_AWAY_Q8` (−3 nats) whenever the advertisement is receivable at ≤ `PROX_FAR_RSSI_THRESHOLD_DBM` but the link will not close — precisely what occlusion produces at 30 cm. The P1 still-window budget caps the damage at one draw's worth, so this is bounded rather than open, but it is not zero and it is pure attacker-controlled input.

**L3 — Signal B is not offset-invariant, and P2 makes it dominant.** `signal_fingerprint()` evaluates a Gaussian Naive-Bayes likelihood on **absolute** dBm (`dx = x − μ`), and imputes censored devices at `PROX_MISSING_RSSI_DBM` (−100) rather than at their attenuated expectation. A uniform −20 dB shift moves every residual by 20 dB and collapses `L`. Because `alpha = exp(−W_total/PROX_ALPHA_W0)`, a *well-trained* anchor drives α→0 and the score becomes almost purely `L` — **weighting out the one branch that was immune.** Training fingerprints improves accuracy and simultaneously opens this hole; the two must land together.

### 13.4 Countermeasures

Numbered R1–R5, in dependency order. R1 and R4 are pure hardening with no wire cost and no new statistics; R2, R3 and R5 need the P5 batch.

**R1 — truncation is not distance (fixes L1).** Delete the score-50 branch. Below `PROX_MIN_DEVICE_COUNT` the anchor returns `score = PROX_STARVED_SCORE` (128, criterion-neutral, lands inside the HMM dead zone), `neff = 0`, and `PROX_FLAG_LOW_DEVICE_COUNT`. The watch's emission is then structurally zero — a starved vector says *nothing*, which is the truth.

The score-200 branch (own-anchor raw RSSI ≥ `ANCHOR_NEAR_RSSI_THRESHOLD_DBM`) is **retained**, deliberately asymmetric: a strong own-anchor RSSI cannot be manufactured by an attenuating adversary, so near-evidence from a starved vector remains trustworthy while far-evidence does not. This is §13.0's rule in its simplest form.

**R2 — offset-invariant Signal B, with the level information split out (fixes L3).** Two changes to `signal_fingerprint()`:

1. Estimate the common-mode offset against the fingerprint over **present** devices only:
   $$\hat{\delta} = \operatorname*{median}_{i \in \text{fp-active} \,\cap\, V}\left(x_i - \mu_i\right)$$
   Evaluate the shape term at $x_i - \hat{\delta}$, and impute censored fingerprint devices at $\mu_i + \hat{\delta}$ (their attenuated expectation) instead of −100. The shape term is now affine-invariant like Signal A. This is a strict improvement irrespective of anyone cheating — it also fixes the mundane cases: a coat sleeve, a duvet, a watch worn under a cuff.
2. The absolute level is genuinely informative about distance and must not simply be discarded. Re-admit it as its own explicit, capped, separately-gated term $\ell_{\text{level}} = \text{clamp}(f(\hat\delta), \pm\texttt{LL\_LEVEL\_MAX\_Q8})$, which **abstains to 0 whenever the occlusion classifier fires**. Split the signal; gate only the half that is attackable.

**R3 — the occlusion classifier (watch side).** The watch retains a **reference vector** $V_{\text{ref}}$: the most recent vector captured while the HMM was confident *and* motion permitted a position update, expiring after `OCC_REF_MAX_AGE_S`. Against it, per observation window, with $S$ = shared devices, $D$ = reference devices now absent, $R$ = survivors:

| Statistic | Definition | Integer method |
|---|---|---|
| $\delta$ (common-mode) | $\operatorname{median}_{i\in S}\left(r_i(t) - r_i(\text{ref})\right)$ | 121-bin histogram over −60…+60 dB, median by prefix count — $O(|S|)$, no sort |
| $\text{MAD}$ (differential) | $\operatorname{median}_{i\in S}\left|\Delta_i - \delta\right|$ | second pass over the same histogram |
| $C$ (censor monotonicity) | $\dfrac{\#\{(d,r) \in D\times R : r_d(\text{ref}) < r_r(\text{ref})\}}{|D|\cdot|R|}$ | bucket $R$ by int8 RSSI into 128 bins, prefix-sum, one pass over $D$ — $O(|S| + 128)$ |
| $F$ (channel flatness) | $\max_c \Delta_c - \min_c \Delta_c$ over channels 37/38/39 for the target anchor | requires S1b channel control; abstains under the §3.4 fallback |
| motion witness | LOCOMOTION observed since $V_{\text{ref}}$? | already tracked (§4.1) |

$C$ is a normalized Mann–Whitney statistic: $C = 1$ means every dropout was weaker than every survivor (perfect monotone censoring, the absorptive signature); $C \approx 0.5$ means dropouts were unrelated to previous strength (the displacement signature). It abstains when $|D| < \texttt{OCC\_MIN\_DROPOUTS}$ or $|R| < \texttt{OCC\_MIN\_DROPOUTS}$; $\delta$ and MAD abstain when $|S| < \texttt{OCC\_MIN\_SHARED}$.

These combine in log-odds domain exactly as §5.2's features do — **not** as a hard AND — each term abstaining to 0 when undefined:

$$\ell_{\text{occ}} = \ell_\delta + \ell_{\text{MAD}} + \ell_C + \ell_F + \ell_{\text{motion}}$$

with Gaussian/Beta near-away distributions trained per §5.3 from S6 data and defaulted from the `OCC_*` constants until then. OCCLUDED is latched when $\ell_{\text{occ}} \ge \texttt{OCC\_LOGLR\_TAU\_Q8}$ and cleared below `OCC_CLEAR_TAU_Q8` (hysteresis, so a marginal case does not chatter). $\ell_{\text{motion}}$ carries the most weight by design: a large common-mode drop with *no locomotion since the reference* has no physical displacement explanation at all.

**R4 — connect-failure requires a motion witness (fixes L2).** `prox_note_connect_failure()` contributes `LL_CONNFAIL_AWAY_Q8` **only if LOCOMOTION has been observed since the last successful connect to that anchor**; otherwise exactly 0. The justification is physical rather than statistical: with no locomotion, the distance cannot have changed, so a failed connect is a statement about the channel, not about geometry. Zero, not down-weighted.

**R5 — what OCCLUDED does to the posterior: suppress, never invert.** When OCCLUDED is latched:

- the anchor score's emission is forced to `neff = 0` (contributes nothing);
- $\ell_{\text{level}}$ (R2) abstains;
- the connect-failure term is suppressed regardless of R4;
- the **transition prior still runs**, so the posterior decays toward the prior over hours, but no *observation* moves it.

Absence of signal is absence of evidence. Note that this deliberately does **not** push toward NEAR either — inverting the evidence would create the mirror exploit (deliberately occluding to force a false NEAR), and §13.0's asymmetry is about which conclusion needs *more* evidence, not about manufacturing one.

### 13.5 Reference-vector lifecycle (the part that goes wrong if rushed)

$V_{\text{ref}}$ is the entire basis of R3, so its maintenance is normative:

- **Set** on any observation window where the HMM is confident (NEAR or AWAY, not AMBIGUOUS) *and* `neff ≥ NEFF_TRAIN_MIN`.
- **Re-set** — not merely refreshed — after any sustained LOCOMOTION segment, because the user is legitimately somewhere else and every statistic in R3 is relative.
- **Expire** at `OCC_REF_MAX_AGE_S`; with no valid reference, $\ell_{\text{occ}} = 0$ and the engine reverts to P4 behaviour. Fail toward the previous phase, never toward a decision.
- **Discard** on ENFORCEMENT entry (the engine cold-starts per §2).

The failure mode to test for: a user who legitimately moves to another room, and whose stale bedroom reference then makes every subsequent window read as "occluded". The re-set-on-locomotion rule is what prevents it, and the S6 replay corpus must contain this case.

### 13.6 Detection is not prevention — the product question this spec will not answer silently

The engine can refuse to be **fooled**. It cannot refuse to be **jammed**: a user who stays buried simply produces no usable evidence, and R5 correctly freezes the posterior. For `stayNear` that is the right and safe outcome. For `getAway` — sunrise lock — it is a denial of service on the enforcement: the alarm never satisfies, but neither does it correctly refuse to satisfy.

Phase 5 therefore surfaces the condition rather than resolving it:

- `PROX_FLAG_OCCLUDED` in the score characteristic's flags byte;
- after `OCC_SUSTAIN_S` continuously occluded during an enforcement window, the watch raises an `occlusionSustained` event to the app.

**Recommendation, requiring an explicit product decision before P5 ships:** for criteria where the user *benefits* from appearing far (`getAway`, sunrise lock), sustained occlusion should be treated as **non-compliant** — the criterion is not satisfied, and the app says why. For `stayNear`, where occlusion offers the user nothing, the frozen posterior of R5 is sufficient and no user-visible action is warranted. This asymmetry is a policy choice with UX consequences (a sleeping user who rolls onto their watch is not cheating), and it must be made by a person, not inherited from a default.

### 13.7 Spike S6 — the missing distributions

Every threshold in §11's `OCC_*` block is currently an estimate from first principles. S6 gathers the real distributions. Unlike S1–S5 it answers no feasibility question; it is a measurement campaign, and it is the gate for P5.

Capture the full vector plus per-channel RSSI and motion state at ~1 Hz for ≥ 3 minutes per condition, on ≥ 2 wrists in ≥ 2 homes:

| # | Condition | Expected signature |
|---|---|---|
| a | Watch under a duvet, arm free | small \|δ\| (~2–4 dB), tiny MAD — **must not** trip the detector |
| b | Watch under a pillow | \|δ\| ~3–6 dB, tiny MAD — must not trip |
| c | Arm tucked under torso, face-down in mattress | \|δ\| ≥ 12 dB, small MAD, high $C$, no locomotion — **must** trip |
| d | Watch removed and buried in bedding | same RF signature as (c); separable only by IMU (§13.8) |
| e | Genuine walk to the next room | large \|δ\| **and** large MAD, $C \approx 0.5$, locomotion present — must **not** trip |
| f | Genuine walk within the same room | moderate δ, large MAD — must not trip |
| g | Rolling over in bed (accidental, repeated) | δ oscillating; tests latch hysteresis |

**Pass criterion:** thresholds exist that give ≥ 90 % detection on (c) and ≤ 5 % false-positive across (a), (b), (e), (f). **If no such separation exists**, fall back to R1/R2/R4 alone — which close the exploits without ever needing to classify — and ship R3/R5 disabled (`OCC_ENABLE = 0`). R1/R2/R4 are the load-bearing part of this phase; R3/R5 are the sharpening.

### 13.8 Open questions (do not guess these — they need S6 data)

- **Occlusion vs. watch-removal.** Conditions (c) and (d) are RF-identical. The only separator is the IMU: a worn watch is never perfectly still — respiration and pulse produce micro-motion measurable at the LIS3DH's noise floor at high ODR, and a watch on a nightstand does not. Whether that separation survives at `IMU_BURST_HZ = 50` with the existing HP filter is unknown, and off-wrist detection is arguably its own feature rather than part of this phase. S6 should log raw burst variance in both conditions so the question can be settled with data.
- **Channel flatness under the S1b fallback.** $\ell_F$ is the cleanest physical discriminant in R3 (absorption is broadband; fading is frequency-selective) and it is unavailable while `BEACON_CHANNEL_CONTROL = 0`. This is an independent reason to revisit S1b via the `NimBLEExtAdvertisement::setPrimaryChannels()` route noted in `tests/prox_v2_spikes.md`.
- **Active relay attacks** (a repeater manufacturing false NEAR) are out of scope. They require equipment, they are the opposite polarity to everything here, and the countermeasure is different in kind — round-trip timing (FTM, §3.5/P4) rather than amplitude statistics.
- **Interaction with the P3 coupling detector.** A phone and watch both buried together will show correlated occlusion; whether $\rho_{\text{cpl}}$ degrades gracefully under R5's evidence suppression is untested.

### 13.9 Pull-forward subset

The user-facing schedule places P5 after P4. **L1 and L2 are exploitable in the shipped firmware today**, and their fixes — **R1 and R4** — are watch/anchor-local, need no wire change, no new statistics, and no S6 data. If sunrise lock ships before P5, pull R1 and R4 forward into whichever phase is current; they are individually small and independently testable. R2 must land no later than the first fleet-wide fingerprint training, since that is the point at which L3 becomes the dominant path.
