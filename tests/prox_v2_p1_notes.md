# Proximity v2.1 — Phase 1 implementation notes

Companion to `firmware_spec_v0.9_amendment.md` §10-A Phase P1 and
`proximity_engine_spec_v2.1.md` §4/§6. Records what was decided where the specs
left a choice open, and what a reader should check before Phase 2.

Run the acceptance suite with `make` in this directory (host-side, no hardware).

---

## 1. Deviations from the amendment

### 1.1 `ProxScoreResult2` carries a 4th field, `near_thr`

The amendment defines the HMM emission as

```
l_t = logit((score + 0.5)/256) * g(N_eff) + l_local
```

which is centred on score 128 — it treats 128 as the score at which the anchor's
evidence is neutral. On this branch that is not true. Calibration-v2 (already
shipped, `PROX_CALIB_*` / `prox_calib_finalize`) has each anchor *demonstrate*
where its near zone ends and return that cutoff in the score characteristic's
3rd byte. For an anchor whose demonstrated cutoff is 210, a score of 180 is
evidence of being **outside** the zone; the spec's emission would read it as
weak evidence of being inside.

The emission is therefore centred on the anchor's own decision point:

```
raw = logit8[score] - logit8[near_thr ? near_thr : PROX_CONFIDENCE_THRESHOLD_U8]
```

Score == cutoff contributes exactly zero, which is the correct meaning of a
demonstrated boundary. Uncalibrated anchors fall back to the global default, so
behavior with no calibration is what the amendment describes.

**Action required in P2.** Amendment Part 13 grows the score characteristic to 3
bytes `[score][flags][neff]` — but byte 3 is already `near_thr` on this branch.
The payload must become **4 bytes `[score][flags][neff][near_thr]`**, and the app
parser flagged accordingly in the same lockstep batch. This is a wire decision,
not an implementation detail; do not paper over it by dropping either field.

### 1.2 `NEFF_LOCO_PER_S` / `NEFF_FIDGET_PER_S` are draws per second

The amendment annotates these as "Q4 credit rate", which reads as *the constant
is pre-scaled Q4* — i.e. `NEFF_LOCO_PER_S = 8` means 0.5 draws per second of
walking. That contradicts the engine spec's own §1.3: fast fading decorrelates
every λ/2 = 6.2 cm, so a wrist moving at ~1 m/s sweeps ~16 fresh draws per
second. 0.5/s is 32× low, and with it the walk-approach acceptance test cannot
pass at any tick rate — a genuine approach never accumulates enough evidence.

Implemented as **whole draws per second** (8/s walking, 2/s fidgeting — still a
conservative half of the physical rate). N_eff remains *stored* in Q4; that is
what the "Q4" in the annotation describes.

### 1.3 New constant: `HMM_STILL_DRAW_PERIOD_S` (12 h)

The amendment asserts two things about the still state that do not follow from
`HMM_PFLIP_STILL` alone:

- §6.2: an RSSI excursion while the wrist is provably still is "structurally
  incapable of flipping the decision";
- §6.2: "PFLIP_STILL is small, not zero: sustained contradictory evidence still
  wins eventually".

A transition prior cannot deliver the first — a repeated emission always
overwhelms a prior given enough repetitions, and the transition merely bounds
how confident the filter may become. The mechanism that *does* deliver it is
N_eff honesty: a still wrist earns no fresh draws, so its emission gain is zero
and re-measuring a frozen fade a thousand times is worth exactly one
measurement.

That would make the still-lock absolute, contradicting the second assertion. So
`HMM_STILL_DRAW_PERIOD_S` states the residual rate as a physical quantity: how
long a motionless wrist must sit before the engine concedes one fresh
independent draw (the world moves even when the wrist does not — people, doors,
body drift below the 48 mg IA1 threshold).

**This single number sets the entire still-lock strength.** At 12 h:

- an 8-hour night of a wrist parked in a null cannot flip the decision (the
  frozen-fade acceptance test);
- a silently stuck IMU — one reporting STILL while the user moves, the residual
  risk that `IMU_STALE_MS` does not cover — still yields to sustained
  contradiction within half a day. Note the failure direction: every window
  cold-starts on the criterion-satisfying side, so a stuck filter errs toward
  *not* alarming.

If field data says the lock is too strong or too weak, this is the knob.

---

## 1.4 Watch-local evidence rides the draw gate (found on hardware)

Amendment §6.1 places `l_local` **outside** the `g(N_eff)` discount:

```
l_t = logit((score + 0.5)/256) * g(N_eff) + l_local
```

Implemented literally, and the bench run immediately showed why it cannot stand.
A stationary watch out of comfortable range re-fails its GATT connect on every
poll, and each failure added a full `LL_CONNFAIL_AWAY_Q8`:

```
[PROXv2] connect-failed: v0.8=AWAY v2=AWAY p_near=22 motion=STILL lam=-610
[PROXv2] connect-failed: v0.8=AWAY v2=AWAY p_near=1  motion=STILL lam=-1378
[PROXv2] connect-failed: v0.8=AWAY v2=AWAY p_near=0  motion=STILL lam=-2146
[PROXv2] score:          v0.8=AWAY v2=AWAY p_near=0  motion=STILL lam=-2769
[PROXv2] connect-failed: v0.8=AWAY v2=AWAY p_near=0  motion=STILL lam=-3473
```

That is the frozen-fade failure the engine exists to prevent, arriving through a
different channel: the posterior marching to saturation on one observation
repeated. A re-failed connect with nothing moved is no more independent than a
re-measured fade. Watch-local evidence is therefore summed with the anchor's
verdict **before** the draw gate.

Regression test: "connect-failure evidence does not accumulate while STILL".

## 1.5 A still window's evidence is a level, not an increment

Gating `l_local` alone produced the opposite failure, also visible on hardware:
whichever observation arrived first in a still window spent the window's single
draw and silently locked out every later one. A weak connect-failure hint
(`raw = -768`) landed first and pinned the posterior at `lam = -115` —
AMBIGUOUS — while the anchor's actual score (`raw = -1941`, unambiguously far)
was suppressed as "the same draw", forever.

Within a still window the evidence is now tracked as a **level** worth at most
one observation (`g_hmm_still_credited_q8`): each tick credits only the change
since the window's last reading, and only in the direction the observation
points (never clawing back what stronger draws established). Both required
properties hold:

- re-reading the same thing adds nothing (no accumulation);
- a materially *different* reading still lands (no first-observation lockout).

Confirmed on hardware — the level converges and then pins:

```
[PROXv2] score:          v0.8=AWAY  v2=AMBIG lam=-16
[PROXv2] connect-failed: v0.8=AWAY  v2=AMBIG lam=-115
[PROXv2] score:          v0.8=AWAY  v2=AWAY  lam=-473
[PROXv2] connect-failed: v0.8=AWAY  v2=AWAY  lam=-473   (x17, unchanged)
```

---

## 2. Choices the specs left open

- **Emission weighting is per *new draws since the last emission*, not per tick.**
  `hmm_earn_draws()` accrues fractional draws in Q12 across ticks and spends
  whole draws; any tick taken while the wrist is moving is worth at least one
  draw, so motion-triggered re-checks stay fully responsive. A zero log-LR (a
  failed query, or a score exactly on the cutoff) spends nothing.
- **`prox_hmm_decision()` is a pure read.** The poll-tier logic consults it from
  the main loop many times a second, so it must not advance the transition
  model. A poll that produced no query ticks with a `NULL` result instead —
  that advances time and folds pending watch-local evidence.
- **`HMM_TICK_REF_S = 60`** — the flip probabilities are per-minute and linearly
  time-scaled to the actual (irregular) tick spacing, clamped at
  `HMM_PFLIP_MAX_PPM` (0.5 = complete mixing, i.e. a gap so long that nothing is
  remembered).
- **IMU burst units are milli-g.** At ±2 g high-resolution the LIS3DH is
  1 mg/LSB, so `IMU_STILL_VAR` / `IMU_LOCO_VAR` in mg² and in "LSB² after HP
  filter" (the amendment's wording) are the same number. Gravity is removed by
  the burst's own per-axis mean rather than the sensor's high-pass filter,
  because `FDS = 0` leaves the data registers unfiltered — only the interrupt
  generator sees the HPF.
- **`IMU_STILL_VAR` / `IMU_LOCO_VAR` are conservative placeholders** pending
  Spike S3 (400 mg² / 40000 mg², i.e. 20 / 200 mg RMS). STILL is deliberately
  hard to declare, and every misclassification lands on LOCOMOTION, which fails
  toward v0.8 behavior rather than toward a frozen decision.
- **The HMM is a single filter instance**, shared by Mode A and the Modes B/C
  co-location pipeline (`prox_hmm_tick_loglr_q8`). Only one criterion is
  enforced at a time, so this is sufficient; it would need to become per-event
  state if that ever changes. Note that the co-location pipeline is currently
  unreferenced by the watch firmware — `phoneAway` uses the anchor query plus
  dock status — so its port is compile-and-test verified, not field verified.

---

## 3. What is deliberately NOT claimed

The frozen-fade guarantee covers *accumulation*, not the first look. The first
observation of an enforcement window still moves the posterior, fade or not — a
single fresh look is genuine evidence and there is nothing yet to contradict it.
What cannot happen is a still wrist turning one frozen look into hours of
mounting confidence.

---

## 4. Hardware verification — what was and was not covered

Run on a tethered watch (`/dev/ttyACM0`, blank NVS, anchor `84a6a9e8…` at
≈ −79 dBm) using the `BENCH_NO_SLEEP` bench harness in `main.cpp`, which
synthesises a `stayNear` event against whichever anchor the scan finds and calls
the real `is_enforcement_condition_met()` path — so the code under test is the
shipping code, not a mock.

**Covered:**

| Check | Result |
|---|---|
| Boot, no crash/WDT/panic with v2 code active | clean |
| IMU burst plumbing (SPI read inside the scan wait loop → `prox_ingest_imu_burst`) | works |
| Motion classifier on real sensor noise | STILL, stable, no flapping |
| Real burst variance, watch at rest on a desk | **9–33 mg²** (≈ 3–5.7 mg RMS) |
| Full query path: scan → 24–31 device vector → connect → score → HMM → shadow log | works |
| Shadow logging with both decisions | works; divergences are v0.8 flip-flop, v2 stable |
| Draw gating under repeated identical evidence | posterior pins, no drift (§1.4, §1.5) |
| Heap over ~10 min of continuous query load | flat, 33.6–34.1 KB, oscillating with the GATT connection, no trend |
| 4 min soak: posterior drift under unchanging still evidence | **zero** — 48/48 decisions at lam = −473 exactly (one distinct value) |
| 4 min soak: panics / WDT / resets | none |

**On `IMU_STILL_VAR`.** The measured resting floor is 9–33 mg², against a
placeholder threshold of 400 mg² — better than 10× headroom, so STILL is declared
robustly on a resting device and the constant is *not* too tight. Whether it is
too *loose* (a worn-but-quiet wrist reading STILL when it should read FIDGET)
cannot be answered from a watch on a desk. That is Spike S3's job.

**NOT covered — requires physically moving the watch:**

- FIDGET and LOCOMOTION classification, and the cadence detector, on a real
  wrist. Only STILL was exercised, so `IMU_LOCO_VAR` and
  `IMU_CADENCE_*` are still purely theoretical.
- The IA1 interrupt seam (`prox_note_motion_interrupt`) — never fired.
- The walk-approach behavior end to end (the acceptance test covers it only in
  simulation).
- **The sleep-interval verdict** (`prox_note_sleep_interval`) — the bench build
  suppresses light sleep by construction, so this path never ran. It needs a
  sleep-enabled build.
- The STILL poll tier, which is gated behind `PROX_V2_AUTHORITATIVE`.

## 5. Before Phase 2

1. **Finish the shadow-logging check with a sleep-enabled build and real
   motion** — walk toward and away from an anchor, and let the watch light-sleep
   between polls. Then flip `PROX_V2_AUTHORITATIVE` to 1. The bench build is
   deliberately left flashed so the board stays re-flashable.
2. **Spikes S1a, S1b, S2, S3** — all require the physical devices. S1/S2 gate P2
   entirely; S3 replaces the placeholder IMU thresholds above.
3. Resolve the score-characteristic collision in §1.1 before writing any P2 wire
   code.

## 6. The bench harness

`-DBENCH_NO_SLEEP=1` (via `PLATFORMIO_BUILD_FLAGS`, never in `platformio.ini`)
compiles in `bench_tick()` and suppresses both light-sleep entry points. Light
sleep powers down the USB-serial peripheral, after which the board cannot be
re-flashed without a physical reset — which makes iterating on a tethered watch
impossible.

```
cd WatchFIrmware
PLATFORMIO_BUILD_FLAGS="-DBENCH_NO_SLEEP=1" ~/.platformio/penv/bin/pio run -t upload
```

Never define it for a shipping build: the watch would stay awake permanently.
A normal `pio run` leaves every bench path out of the binary.
