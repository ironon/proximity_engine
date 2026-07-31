# Impulse Proximity Engine

A portable C++ module that answers one question, reliably, on a $2 radio:

> **Is the watch inside this anchor's zone, right now?**

It runs on both sides of the link — on the battery-powered watch and on the mains-powered anchor — from a single source file pair, with all platform I/O injected through seams. No RTOS dependency, no Arduino dependency, host-compilable, and covered by ~1900 assertions that run without hardware.

The full normative design is in **[proximity_engine_spec_v2.1.md](proximity_engine_spec_v2.1.md)**. This README is the map, not the territory: it explains *why* the engine looks the way it does. Where the two disagree, the spec wins.

---

## 1. What it's for

Impulse is a wrist-worn behavioural enforcement device. The user commits to a rule in advance and the watch holds them to it. Anchors are small ESP32 pucks stuck to a wall or a desk that define places:

| Criterion | Meaning | Example |
|---|---|---|
| `stayNear` | Remain in an anchor's zone | Study Time — stay at the desk for 90 minutes |
| `getAway` | Leave an anchor's zone | Sunrise Lock — get out of bed to dismiss the alarm |
| `phoneAway` | Watch and phone are not together | Phone-Free — dock the phone and walk away |

Everything downstream — timers, alarms, lockouts, the whole product — is built on a boolean the engine produces. If the engine says NEAR when the user is in the next room, the alarm never fires. If it says AWAY when the user hasn't moved, the user is punished for compliance. **There is no graceful degradation at the product level**, which is why the engine spends so much of its complexity on *knowing when it doesn't know*.

Note what is **not** required: a distance in metres, a position on a floor plan, or a fix in an unmapped building. The engine does zone classification, not localisation. That distinction is what makes the whole thing tractable, and section 4 is essentially a list of consequences of it.

---

## 2. Why not UWB

UWB is the technically correct answer to indoor ranging. Time-of-flight beats amplitude at everything: it gives you 10–30 cm accuracy, it is nearly immune to multipath, and it resists the relay and attenuation attacks that amplitude-based methods have to be engineered around. We are not using it, for three practical reasons that have nothing to do with the physics:

**Cost.** A UWB transceiver is an order of magnitude more expensive than the BLE radio, and it is an *additional* part — the watch still needs BLE for phone connectivity, app pairing, and OTA. It goes on top of the existing BOM, not instead of it. Multiply by every anchor, since ranging needs a UWB radio at both ends, and a multi-anchor install stops being an impulse purchase.

**Sourcing.** The UWB market consolidated around automotive digital-key applications. Most of the vendors with mature parts sell into that channel and are not meaningfully interested in low-volume consumer orders; getting supply, documentation, and support at our volume ranges from painful to impossible. That is a business risk on the critical path of the whole product, not just an engineering inconvenience.

**Board area.** UWB needs its own antenna, its own matching network, and keep-out around both. On a watch-sized four-layer board already carrying an ESP32-C3, an IMU, a charger, a PMIC, and a display, there is no room to spend, and the mechanical envelope is fixed by the fact that people have to want to wear it.

Meanwhile the ESP32-C3 already has a BLE radio, and it's already on. **BLE proximity costs zero dollars, zero square millimetres, and zero new suppliers.** The whole engine is the argument that this is enough — that the gap between BLE's raw capability and the product's requirement can be closed in software, given the right constraints.

---

## 3. Why BLE RSSI ranging is normally a bad idea

The naive approach — read RSSI, apply the log-distance path-loss model, compare to a threshold — is a well-known failure, and it deserves to be. The reasons compound:

**Multipath fading dominates.** The received signal is the vector sum of many reflected copies. At 2.44 GHz the wavelength is ~12.5 cm, so constructive and destructive interference form a standing-wave pattern that decorrelates over **λ/2 ≈ 6.2 cm**. Moving your wrist by the width of two fingers can swing RSSI by 20 dB. Under the path-loss model, 20 dB is a factor of ~10 in apparent distance.

**Shadowing is comparable to the signal.** A human torso is ~70% water and attenuates 2.4 GHz by 15–30 dB. Turning around costs more dB than walking to the next room.

**The transmitter is uncalibrated.** RSSI depends on TX power, antenna gain, antenna orientation, and the receiver's own RSSI reporting accuracy — none of which are specified tightly enough to invert the path-loss model. The `A` in `RSSI = A − 10n·log₁₀(d)` is not a constant you can look up.

**And the killer: averaging doesn't fix it.** This is the part that catches people. Fading is a *spatial* phenomenon — the standing-wave pattern is a property of the room, and it is static while you are. A stationary receiver parked in a null gets **one** independent sample of the fading distribution, no matter how long you average. All averaging does is drive down the estimator's variance around the wrong number. You converge, confidently, to an answer that is 20 dB off. This was the single largest failure mode of the v1 engine: a still wrist averaged its way to a rock-solid wrong verdict, and the more samples it took the more certain it became.

So the standard advice — "BLE RSSI is good for coarse presence, ±3–5 m at best, don't build anything load-bearing on it" — is correct **for the problem it's usually applied to**: estimating an unknown distance to an uncharacterised transmitter in an unknown room.

That is not our problem.

---

## 4. Why it works here

Six structural advantages. None of them is a clever algorithm; each is a way of changing the question until it becomes answerable.

### 4.1 We recognise a place, we don't measure a distance

The dominant signal isn't RSSI to the anchor at all. It's **RF fingerprinting**: the watch takes a census of every BLE and WiFi emitter it can hear — neighbours' access points, TVs, laptops, other anchors, in practice 12–31 devices — and sends that whole vector to the anchor. The anchor, which has been continuously scanning its own environment, compares it against its own view.

Two signals are computed and blended:

- **Signal A — Pearson correlation** between the two vectors over their shared devices. This asks *"does the watch see the world in the same shape I do?"* It is a comparison of **patterns**, not levels, and it is **affine-invariant**: scale and offset the whole vector and ρ is mathematically unchanged.
- **Signal B — a per-device Gaussian fingerprint.** Over time the anchor learns, for each emitter, the mean and variance of the RSSI a watch reports *while in the zone*. A new vector is scored as a likelihood against those learned distributions. Signal B is sharper than Signal A but needs training, so the blend weight `α = exp(−W_total/W₀)` hands authority from A to B as evidence accumulates.

The point of both is that a fingerprint over dozens of emitters is a far higher-dimensional observation than one RSSI. Any individual device may be in a fade; they are not all in a fade simultaneously, because they are at different places in the room and therefore have different standing-wave geometries. **Multipath is per-path, so a census of paths averages it out in a way that a single path never can.** The reading that would have wrecked a threshold comparison becomes one noisy coordinate out of thirty.

### 4.2 The anchor is stationary, mains-powered, and permanently installed

This asymmetry is doing more work than any algorithm in the repo.

Generic BLE localisation has to work in an arbitrary space it has never seen. Ours doesn't. The anchor sits in one place for months. It can scan continuously, at full duty, without a power budget. It can accumulate per-device statistics over weeks. It can hold a 128-device fingerprint in NVS. **It doesn't need a model of RF propagation in general; it needs a model of one room, and it has unlimited time and power to measure it.**

So the division of labour is: the **anchor owns per-location statistics** (fingerprints, learned thresholds, calibration), and the **watch owns temporal and motion state** (the HMM, the integrator, the IMU). Each side owns what it is physically in a position to know. The watch sends a vector; the anchor returns a score; the watch decides what to believe.

### 4.3 The user demonstrates the boundary — we never guess it

There is no universal "near" threshold, because there is no universal room. Calibration has the user walk the inside of the zone for a while, then stand at its edge, while the anchor collects scored samples from both legs. It then measures where the two score distributions separate and sets that anchor's cutoff there.

Crucially it also **measures whether they separate at all**, and reports the arithmetic — per-leg mean and standard deviation, the percentiles the acceptance rule compares, Cohen's *d′*, the error-minimising cutoff and its misclassification rate. A calibration that cannot separate the two positions is **refused**, and the failure is attributed (too few samples in a leg / distributions overlap) rather than silently producing a threshold that doesn't work. That refusal is a feature: an anchor that admits it can't tell is infinitely more useful than one that guesses.

One subtlety worth knowing, because it cost real debugging time: both legs are **buffered and scored after the fact**. The INSIDE leg trains the fingerprint it is simultaneously being scored by, so `α` collapses mid-leg and the score silently changes scale. Measured on hardware: samples scored against a fingerprint of <10 devices averaged 213, and against ≥20 devices averaged 158 — a 55-point drift *within one leg*, larger than the INSIDE-vs-EDGE separation the calibration exists to measure. A percentile over a mixture of two estimators is not a percentile of anything. FINALIZE now trains once, then re-scores every buffered sample against the final fingerprint, so both legs sit on one scale.

### 4.4 We manufacture a clean binary observation: stepped-power PDR

Instead of measuring amplitude and comparing it to a threshold, the anchor **puts the threshold in the transmitter**.

The advertiser rotates through a slot schedule, tagging each slot in the iBeacon `Minor` field (previously a constant `0x0000`, so this is free — the Major filter and scan-response format are untouched). Some slots transmit at full power; some transmit at a deliberately crippled `BEACON_TX_LO_DBM`, chosen per install so the link budget *just barely* closes inside the zone and fails outside it. The watch then asks one binary question per low-power slot: **did anything arrive?**

This converts an analogue measurement into a Bernoulli trial. It sidesteps RSSI reporting accuracy entirely — you don't need to know what −87 dBm means on this chip, you only need to know whether a packet made it. With ~5 PDUs per slot, a closed link yields a hit with near-certainty, so the slot-hit ratio is clean. Measured at a real install: **97% hit rate inside, 0% at the edge.**

Two details that turned out to matter:

- The window must be **passive-scanned**. Measured, everything else held constant, active scanning delivered 30–95% of the advertisements that passive scanning delivered 100% of, and shifted the apparent power cliff by ~9 dB — enough to mis-set `TX_LO` by three levels. The receiver misses advertisements while transmitting SCAN_REQs, and the advertiser interrupts its own schedule to answer them.
- "Covered" is defined by reception, not by clock arithmetic: a cycle counts when **either** of its slots was heard. That alone proves the cycle happened and that the window spanned it, with no need for the watch to know the anchor's schedule phase. As the anchor recedes far enough that even full-power slots are lost, the *denominator* shrinks and the channel abstains rather than manufacturing misses.

### 4.5 Frequency diversity is the only diversity a stationary receiver can get

Coherence bandwidth indoors is roughly 4–20 MHz, and the three BLE advertising channels span 78 MHz (2402 / 2426 / 2480 MHz). They are therefore **largely independent fading draws of the same path**. A schedule that rotates channels gives a motionless watch three samples where it would otherwise have one.

(This is also why WiFi CSI is *headroom* rather than a solution: a single 20 MHz channel spans only ~1–5 coherence bandwidths, so its subcarriers are heavily correlated. Its unique contribution is fine channel *shape*, not extra draws.)

### 4.6 The IMU tells us when evidence is real — this is the keystone

Section 3 established that averaging a frozen fade produces confidence without information. The corollary is that **the engine must know whether it is accumulating evidence or accumulating noise**, and only motion can tell it.

Every integrating estimator therefore tracks an effective sample count:

```
N_eff = N_freq · (1 + s_IMU / (λ/2))
```

Walking at ~1 m/s sweeps ~16 fresh fading draws per second. Sitting still sweeps zero, forever. So a still wrist's `N_eff` is **frozen**, and the engine reports low confidence — not because the readings are noisy, but because they are all the *same* reading. The LIS3DH interrupt that already wakes the watch during enforcement is repurposed as the motion detector, plus one short accelerometer burst taken concurrently with each scan (the CPU is idle while the radio listens). Classes separate cleanly on burst variance alone, measured on a real wrist:

```
desk, untouched        9 –     34 mg²
worn, sitting still   95 –    338
typing               756 –   9278
walking            19645 – 415024
```

The verdict then comes from a **two-state HMM over {NEAR, AWAY}** whose transition probability is conditioned on that motion state:

| Motion state | P(flip per minute) |
|---|---|
| STILL | 10⁻⁴ |
| FIDGET | 10⁻³ |
| LOCOMOTION / UNKNOWN | 0.15 |

This is the structural fix for the failure in §3, and it's worth stating plainly: **you cannot get farther from an anchor without walking.** An RSSI excursion on a provably-still wrist is, by physics, multipath or interference or a body-shadow shift — never displacement — and the transition prior makes it structurally incapable of flipping the decision. A genuine approach necessarily involves locomotion, which simultaneously unlocks transitions *and* floods the integrator with spatially fresh samples. The two things the engine needs to happen together, happen together, for free, because they're the same physical event.

`P(flip | STILL)` is small but not zero, so sustained contradiction still wins eventually (a stuck IMU, a removed watch) — currently on a ~12-hour timescale, so a motionless night cannot be flipped by a frozen fade while a genuinely broken sensor still yields within half a day.

This also *pays for itself in power*: a still-and-compliant wrist can't change class, so polling it is pointless. The poll interval backs off from 180 s to 600 s in that state, which more than covers the cost of the longer 1800 ms observation window. The motion interrupt still forces an immediate re-check, so responsiveness is untouched.

---

## 5. How a single decision actually happens

```
   WATCH                                            ANCHOR
   ─────                                            ──────
1  1800 ms passive full-duty scan  ◄────  beacon schedule: HI/LO slots,
   · every advertisement's Minor              slot tagged in iBeacon Minor
     attributed to a slot
   · IMU burst sampled concurrently
                                            (continuously, in background:
2  scan cache: median of per-window           scan own environment, maintain
   maxima over 40 s → suppresses               128-device registry)
   per-window sampling noise

3  build vector (≤60 devices,
   persistent ones preferred)
                          ──── GATT write ────►
                                            4  score the vector:
                                               · Signal A: Pearson vs own view
                                               · Signal B: Gaussian fingerprint
                                               · self-RSSI level term (capped)
                                               · blend by α = exp(−W/W₀)
                          ◄─── score, flags, ──
                               near_thr
5  fold in watch-local evidence:
   · PDR log-LR from the LO slots
   · connect-failure evidence
6  HMM tick: emission is a bounded
   linear discriminant about the
   anchor's own calibrated cutoff,
   transitions gated by motion state
7  posterior → NEAR / AWAY / AMBIGUOUS
   (AMBIGUOUS resolves to whichever
    side is fail-safe for the criterion)
```

Steps 1–3 and 5–7 are the watch; step 4 is the anchor. Modes B/C (`phoneAway`) reuse the same integrator and HMM but replace the anchor score with a co-location log-likelihood built from the direct watch↔phone RSSI, the shared-environment statistic, and — with two anchors — a differential ranging term.

---

## 6. Design decisions worth knowing

**Everything abstains rather than guessing.** Pearson below 6 shared devices is arithmetic, not measurement (at k=2 the correlation is ±1 whatever you measured — observed on hardware as a still watch beside a still anchor emitting `0, 0, 255, 0, 255`). Below the floor the engine returns a criterion-neutral 128, *never* 0. An absent measurement is not evidence of distance, and 0 reads as a confident AWAY. The same rule appears everywhere: PDR abstains below 2 covered cycles, the self-RSSI term abstains below 6 dB of demonstrated span, the motion channel goes UNKNOWN when stale and UNKNOWN is treated as LOCOMOTION (fail toward the more permissive model, never toward a frozen decision).

**Evidence is capped asymmetrically, on purpose.** A passive attenuating adversary — or a sleeping user who rolls onto their wrist — can fabricate **AWAY but never NEAR**. Attenuation only ever removes signal; producing a false NEAR would require active gain. So *concluding far requires more evidence than concluding near*, systematically. PDR hits count at +3.0 nats and misses at −1.5. The self-RSSI term is worth 48 score points upward and 24 downward. This isn't tuning; it's one invariant applied consistently, and it is what stops "shove your wrist under the duvet" from being a working exploit against Sunrise Lock. The full threat model is [§13 of the spec](proximity_engine_spec_v2.1.md).

**Integer math on the hot path.** No FPU on the ESP32-C3. Everything per-tick is Q-format fixed point; log-odds are Q8; `logsumexp` is max plus a lookup table; square roots exist only inside precomputed tables. The only float left is the Pearson/Welford core, which runs on the mains-powered anchor.

**One module, two roles, injected platform.** `proximity.cpp` has no `#include <Arduino.h>` and no RTOS calls. The platform provides a clock, NVS load/save, scan results, an IMU burst, and a beacon-slot setter; the engine provides everything else. `PROXIMITY_ROLE_WATCH` / `PROXIMITY_ROLE_ANCHOR` select the surface. This is why the entire engine compiles and tests on a laptop in about a second, and why the hard bugs were found there rather than over a serial cable.

**The score is a similarity metric, not a posterior.** The HMM emission is deliberately *not* `logit(score/256)`. That treats a live Pearson correlation as calibrated odds, claims a score of 0 is 500:1, and is nearly flat across the mid-range where most real readings live. On hardware it made the filter easy to push to NEAR and nearly impossible to bring back — a walk away produced scores of 157 and 103 that contributed literally nothing. It's now a bounded linear discriminant about the anchor's *own* calibrated cutoff: symmetric, proportionate in the middle, capped so no single observation can claim more than it knows, with a dead zone around the cutoff so noise at the decision point can't accumulate into certainty.

**One scan window is a random subset, not a census.** Advertisers are asynchronous with 100 ms–10 s intervals, so a 700 ms window catches a 2 s advertiser only ~35% of the time. Measured with a still watch and a still anchor, vector size swung 21–31 devices and the resulting score swung 26–159 — pure estimator noise, with a frozen channel behind it. The scan cache keeps 40 s of per-window maxima and reports the median. Note this does **not** violate the frozen-fade rule: the fading draw is frozen when the wrist is still, but the *subset each scan happens to catch* is re-randomised every window regardless of motion. The cache averages only the second, and `N_eff` accounting is untouched.

---

## 7. Layout

| Path | What's in it |
|---|---|
| [src/proximity.h](src/proximity.h) | Public API, all tunables, and the reasoning behind each constant. The comments here are load-bearing — most tunables carry the measurement that set them. |
| [src/proximity.cpp](src/proximity.cpp) | Every algorithm. Role-gated, no platform dependencies. |
| [src/prox_luts.h](src/prox_luts.h) | Generated lookup tables (see [tools/gen_prox_luts.py](tools/gen_prox_luts.py)). |
| [proximity_engine_spec_v2.1.md](proximity_engine_spec_v2.1.md) | **The normative design spec.** Physics, phase map, wire formats, threat model. |
| [tests/](tests/) | Host-compiled acceptance tests, both roles, no hardware needed. |
| [tests/prox_v2_spikes.md](tests/prox_v2_spikes.md) | Feasibility spikes S1–S6 and their results. |
| [tests/spike_s1a_txlo.md](tests/spike_s1a_txlo.md) | The TX-power sweep that set `BEACON_TX_LO_DBM`. |
| [bench/pdr_sweep/](bench/pdr_sweep/) | Two-role bench firmware for measuring PDR against TX power on real hardware. |

Consumers: `WatchFirmware` and `AnchorFirmware` include this as a PlatformIO library and supply the platform seams.

## 8. Build and test

```sh
cd tests && make          # builds and runs both roles
make watch                # watch-role only
make anchor               # anchor-role only
```

No hardware, no framework, no network. Currently 1850 watch checks and 70 anchor checks.

---

## 9. Status

Phase 1 (motion channel, integrator, HMM) and the watch-local parts of Phase 2 (beacon schedule, stepped-power PDR, calibration-v2) are implemented and validated on hardware. Known gaps, all deliberate and recorded in the spec:

- **`PROX_V2_AUTHORITATIVE` is still 0.** The v2 decision is computed and logged beside the shipped v0.8 threshold decision, which remains authoritative. Shadow-then-flip.
- **`BEACON_CHANNEL_CONTROL` is 0.** Spike S1b found per-slot channel restriction *is* available on this NimBLE, but its on-air pass criterion hasn't been checked. Until it is, the power-only 2-slot schedule ships: PDR never needed channel attribution, so this is a reduced feature set rather than a broken one. The cross-channel spread feature and the channel-flatness occlusion discriminant are unavailable meanwhile.
- **Phase 4 (reversed-link WiFi CSI, FTM) and Phase 5 (the full occlusion classifier) are specified but not built.** Phase 5's load-bearing hardening — the starved-vector fix and the motion witness on connect-failure evidence — is pulled forward; the classifier that sharpens it is gated on a measurement campaign that hasn't run.
