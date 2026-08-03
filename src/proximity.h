// proximity.h
//
// Impulse Proximity Engine — shared declarations.
//
// All proximity ALGORITHM logic lives in proximity.cpp. Other firmware files
// include this header and call only the public API below. Platform glue (BLE/
// WiFi scanning, NVS, clock) is injected via the "platform seam" functions that
// proximity.cpp declares `extern` and the platform layer defines.
//
// Role selection: define exactly one of PROXIMITY_ROLE_ANCHOR / PROXIMITY_ROLE_WATCH
// per build. Co-location (Modes B/C) runs on the watch. Mode A scoring/training
// runs on the anchor; the watch builds and submits scan vectors.

#ifndef PROXIMITY_H
#define PROXIMITY_H

#include <stdint.h>
#include <stddef.h>

// ============================================================================
// Constants & tunables  (see design spec §7)
// ============================================================================

// ── Proximity engine v2.1 — Phase 1 (watch-side inference upgrade) ──────────
// Engine spec proximity_engine_spec_v2.1.md §4/§6/§11; FIRMWARE_SPEC v0.9
// amendment Parts 9, 10, 14. P1 is watch-only and changes no wire format.
//
// Shadow-then-flip: while PROX_V2_AUTHORITATIVE is 0 the v2 decision is computed
// and logged beside the shipped v0.8 threshold decision, which stays
// authoritative. Flip to 1 after the on-hardware shadow-logging check (§10-A P1).
//
// FLIPPED 2026-07-31 for field trial, after two accepted per-anchor calibrations
// and with these prerequisites landed together (they are not optional — see the
// commit that flipped this):
//   * §13.4-R1 — the starved-vector fallback no longer returns a confident AWAY.
//     Attenuation censors the weakest emitters first, so smothering the watch
//     drove count under PROX_MIN_DEVICE_COUNT and bypassed the whole correlation
//     architecture. On getAway (sunrise lock) that reads as "criterion met".
//   * §13.4-R4 — connect-failure AWAY evidence now requires a motion witness.
//   * The donning grace no longer discards its queries: it is the only
//     LOCOMOTION most windows ever see, and the draw gate makes evidence earned
//     while still nearly worthless, so throwing it away left the filter unable
//     to accumulate for the rest of the window.
//
//   * §13.4-R6 (added 2026-08-03) — an AWAY verdict on getAway must be
//     corroborated by witnessed locomotion. See PROX_AWAY_ARM_HITS below.
//
// The limitation previously recorded here — that getAway cold-starts on the
// criterion-satisfying side and resolved AMBIGUOUS to compliant, so a window
// opening on a motionless wrist read as met — is closed by R6 rather than by
// changing §6.3. The cold start still leans toward AWAY, and AMBIGUOUS still
// does not assert NEAR; what changed is that neither is admissible on its own
// any more. A motionless wrist cannot satisfy getAway however the RF reads.
//
// STILL OPEN, and the reason R6 is explicitly interim: §13.4-R2. Signal B
// scores absolute dBm, so uniform attenuation reads as displacement, and the
// blend hands B more authority as training matures — a better calibration makes
// the occlusion attack easier, not harder. R6 defends the verdict; R2 would fix
// the measurement. Sunrise lock still wants a backup alarm until R2 lands.
#define PROX_V2_AUTHORITATIVE                1

// A failed GATT connect plus a recent advertisement at or below this level is
// strong evidence the watch is FAR (connection establishment fails ~-88 dBm,
// well before advertisement reception does). Was watch-local in
// watch_prox_transport.h; the engine now owns it because the rule became a
// log-LR fed through prox_note_connect_failure() (amendment Part 9, step 8).
#define PROX_FAR_RSSI_THRESHOLD_DBM          (-85)

// IMU motion channel (§5.4.5 / engine §4.1). Burst samples are milli-g with
// gravity removed by the burst's own mean; at +/-2 g high-resolution the LIS3DH
// is 1 mg/LSB, so mg^2 and LSB^2 are the same number here.
#define IMU_BURST_SAMPLES                    32
#define IMU_BURST_HZ                         50
// Measured on a real wrist, 2026-07-28 (Spike S3, first pass). The classes
// separate cleanly on variance alone, with a wide empty gap between them:
//
//     desk, untouched      9 -    34 mg^2
//     worn, sitting still 95 -   338
//     typing             756 -  9278
//     walking          19645 - 415024
//
// So STILL sits well under 400 and the typing/walking gap is 9.3k..19.6k.
// IMU_LOCO_VAR is placed in that gap rather than at the original guess of
// 40000, which sat above much of real walking.
#define IMU_STILL_VAR                        400      // (20 mg RMS)^2
#define IMU_LOCO_VAR                         15000    // (~122 mg RMS)^2, S3-measured gap
#define IMU_LOCO_MIN_INTS                    2
// Raised from 5000 after field measurement: when the anchor is marginal the GATT
// connect blocks for its full 5 s timeout, so the gap from the burst (taken
// during the pre-query scan) to the tick reaches ~5.7 s and the motion channel
// went UNKNOWN exactly when queries were slowest. UNKNOWN fails toward
// LOCOMOTION, so this was safe rather than dangerous, but it blinded the channel
// precisely when the link was struggling. 10 s still detects a genuinely dead
// seam promptly.
#define IMU_STALE_MS                         10000
// Step-cadence detection inside one burst. RETIRED AS A CLASSIFIER INPUT: the
// same S3 pass showed it flipping on noise at typing amplitudes — bursts of
// var 9278 and 9788, physically indistinguishable, were separated only by this
// flag and landed in different classes, which is what made ~50% of typing read
// as LOCOMOTION. Variance alone separates the classes with a 2x gap, so cadence
// bought nothing and cost accuracy. Still computed and exposed via
// prox_motion_burst_cadence() as raw data for the full S3 confusion matrix, but
// it no longer decides anything.
#define IMU_CADENCE_MIN_CROSSINGS            2
#define IMU_CADENCE_MAX_CROSSINGS            8
#define IMU_CADENCE_MIN_VAR                  900      // (30 mg RMS)^2

// Effective sample count (§1.3). N_eff is *stored* Q4; these rates are whole
// draws per second. Sanity-check against the physics the spec derives them from:
// fast fading decorrelates every lambda/2 = 6.2 cm, so a wrist walking at ~1 m/s
// sweeps ~16 fresh draws per second — 8/s is a deliberately conservative half of
// that, and 2/s for fidgeting is the same discount applied to a hand that moves
// a few cm at a time. (Reading these as pre-scaled Q4 would credit 0.5 draws per
// second of walking, 32x below the physical rate, and no walk-in would ever
// accumulate enough evidence to flip the decision promptly.)
#define NEFF_LOCO_PER_S                      8
#define NEFF_FIDGET_PER_S                    2
#define NEFF_TRAIN_MIN                       8
#define NEFF_FLOOR_NOSCHED_Q4                16       // 1 draw: P1 / S1-fallback
#define NEFF_FLOOR_SCHED_Q4                  48       // 3 draws: beacon schedule (P2)
#define NEFF_MAX_Q4                          4080     // integer 255 cap

// Motion-gated integrator (§4.2). Weights are Q8 EWMA gains.
#define INTEG_STILL_WEIGHT                   26       // ~0.10
#define INTEG_MOVE_WEIGHT                    77       // ~0.30 (v1 COLOC_EWMA_ALPHA)
#define INTEG_STILL_RELAX_S                  30
#define INTEG_SINGLE_DRAW_VAR                25       // dB^2, one fading draw

// HMM (§6). Flip probabilities are per HMM_TICK_REF_S seconds, in ppm, and are
// linearly time-scaled to the actual (irregular) tick spacing.
#define HMM_PFLIP_STILL_PPM                  100      // 1e-4
#define HMM_PFLIP_FIDGET_PPM                 1000     // 1e-3
#define HMM_PFLIP_MOVE_PPM                   150000   // 0.15
#define HMM_TICK_REF_S                       60
#define HMM_PFLIP_MAX_PPM                    500000   // total mixing; no information
#define HMM_TAU_NEAR_Q8                      355      // logit(0.80) in Q8
#define HMM_TAU_AWAY_Q8                      (-355)   // logit(0.20) in Q8
#define HMM_PRIOR_Q8                         158      // logit(0.65) in Q8
#define HMM_LAMBDA_MAX_Q8                    4096     // +/-16 nats saturation
#define LL_CONNFAIL_AWAY_Q8                  (-768)   // -3.0 log-LR in Q8

// Score -> log-LR mapping. NOT logit(score/256): that treats the anchor's score
// as a calibrated posterior, which it is not. With no fingerprint yet trained
// the score is a live Pearson correlation over whatever devices the watch and
// anchor happened to share (in the field, 12-29 of them), and logit() claims a
// score of 0 is 500:1 odds while being almost flat across the mid-range where
// most real readings live. On hardware that made the filter easy to push to
// NEAR and nearly impossible to bring back: walking away produced scores of 157
// and 103 that contributed literally nothing, and a 79 that contributed -0.1
// nats, so a stale NEAR belief survived the whole walk.
//
// A bounded linear discriminant is the honest reading of a similarity metric:
// symmetric in both directions, proportionate in the middle, and capped so no
// single noisy observation can claim more than it knows.
#define HMM_EMIT_SLOPE_Q8                    8        // log-odds per score count (~0.03 nats)
#define HMM_EMIT_MAX_Q8                      768      // +/-3 nats cap per observation,
                                                      // same weight as LL_CONNFAIL_AWAY_Q8: an
                                                      // unambiguous score and a refused connect
                                                      // are comparably strong evidence
// Scores this close to the anchor's cutoff carry no usable information and are
// dropped, so noise around the decision point cannot accumulate into certainty
// (the 2 cm / score-153-167 case). Deliberately much narrower than v0.8's
// ambiguous band: that band is a *decision* rule needing hysteresis, not a claim
// that the score is uninformative there. The HMM accumulates and lets the
// posterior decide, so it should use the mid-range evidence v0.8 discards.
#define HMM_EMIT_DEADZONE_U8                 PROX_NEAR_HYST_U8
// How long a provably-still wrist must sit before the engine concedes it has
// received ONE fresh independent fading draw. Not zero — the world moves even
// when the wrist does not (people, doors, slow body drift below the 48 mg IA1
// threshold) — but very slow, because spatial decorrelation needs ~6 cm of
// antenna displacement (§1.2) and IA1 would have fired for that.
//
// This single number sets the whole still-lock strength. At 12 h, a wrist that
// is motionless for an entire night cannot have its decision flipped by a
// frozen fade (the failure this engine exists to fix), while a silently stuck
// IMU still yields to sustained contradiction within half a day. Under a stuck
// IMU the held state is whatever was last believed — and since every window
// cold-starts on the criterion-satisfying side, that errs toward not alarming.
#define HMM_STILL_DRAW_PERIOD_S              43200

// Poll tiers (§5.4.1 / §9): met + HMM-confident + STILL backs the enforcement
// poll interval off this far. The IA1 interrupt still forces an immediate
// re-check, so responsiveness is unchanged.
#define ENFORCEMENT_POLL_INTERVAL_STILL_S    600

// ── Phase 2: beacon schedule (anchor) ───────────────────────────────────────
// Engine spec §3.1-§3.2; amendment Parts 3, 4, 14.
//
// The anchor rotates through (channel x TX power) slots and tags each one in the
// iBeacon Minor field, constant 0x0000 since v1. That gives the watch
// per-channel RSSI and a stepped-power packet-delivery ratio with zero
// controller support on the scan side — the only two forms of diversity a
// stationary receiver can actually get (§1.2).
#define BEACON_SCHEDULE_ENABLE               1
#define BEACON_SLOT_MS                       250      // S2 fallback: 500
#define BEACON_ADV_INTERVAL_MS               50
// Spike S1a (tests/prox_v2_spikes.md + spike_s1a_txlo.md). MUST be a multiple of
// 3: NimBLE maps dBm -> esp_power_level_t as dbm/3 and rounds toward HIGHER
// power, so -22 silently transmits at -21. -24 is the hard floor (-27 is rejected
// outright); +9 is the top of the ladder the anchor uses.
//
// The spec's -21 placeholder was measured to be unusable — at 90 dB of path loss
// it delivered 0% PDR everywhere INCLUDING inside the near-zone, i.e. a permanent
// false AWAY. Choose per install from the reference slot's RSSI:
//     BEACON_TX_LO_DBM ~= -91 - RSSI_HI_at_EDGE, rounded DOWN to a multiple of 3
// which needs >= ~6 dB of RSSI separation between the two positions to work.
//
// -6 dBm is MEASURED, not inferred: a full 12-level sweep at both positions of a
// real install (2026-07-30, spike_s1a_txlo.md "result 2 part 2") put it at
// 97% slot-hit INSIDE and 0% at EDGE, meeting the >=90/<=30 pass criterion.
//
// It is chosen to centre the level on the point where PDR's evidence CHANGES
// SIGN, which matters more than the pass criterion itself. With hits worth
// LL_PDR_HIT_Q8 and misses LL_PDR_MISS_Q8, the log-LR passes through zero at a
// hit rate of 385/917 = 42%, which that install reached at about -101 dBm
// received. Centring on it:
//     BEACON_TX_LO_DBM = -101 + (PL_inside + PL_edge)/2
//                      = -92 - (RSSI_HI_inside + RSSI_HI_edge)/2
// giving -92 - (-80 + -92)/2 = -6. That leaves +6 dB of margin at INSIDE and
// -6 dB at EDGE, symmetric, which is what survives the +/-3 dB of RSSI wander
// both positions actually showed. -3 dBm also passes the raw criterion but sits
// only 3 dB from the crossover at EDGE, and flips sign there under a 3 dB gain.
//
// Feasibility gate: RSSI_HI_inside - RSSI_HI_edge >= ~7 dB (>=10 dB for comfort).
// That install measured 12 dB.
#define BEACON_TX_LO_DBM                     (-6)
#define BEACON_SCHEDULE_EPOCH_OFFSET_MS      750      // Mode C anchor-pair phase offset
#define BEACON_CYCLE_MS                      (BEACON_SLOT_MS * BEACON_SLOT_COUNT)

// Per-slot channel restriction. Spike S1b found the capability IS available on
// this NimBLE (contrary to the amendment's premise) by two routes, but its pass
// criterion — adverts observed on exactly one channel, anchor still connectable,
// legacy scanners unaffected — is an on-air check that has not been made. Until
// it has, ship the power-only 2-slot schedule: PDR never needed channel
// attribution (§3.4), so this is a reduced feature set, not a broken one.
// See tests/prox_v2_spikes.md before changing this to 1.
#define BEACON_CHANNEL_CONTROL               0

#if BEACON_CHANNEL_CONTROL
  #define BEACON_SLOT_COUNT                  6
#else
  #define BEACON_SLOT_COUNT                  2        // slot_id 0 (HI) and 3 (LO)
#endif

// Channel-map bits as the BLE HCI layer expects them (ch37 = bit0).
#define BEACON_CH37_BIT                      0x01u
#define BEACON_CH38_BIT                      0x02u
#define BEACON_CH39_BIT                      0x04u
#define BEACON_CH_ALL                        0x07u

// ── Phase 2: observation window & stepped-power PDR (watch) ──────────────────
// Engine spec §3.3. Within one full-duty observation window the watch attributes
// every received advertisement to a slot via its Minor tag, then asks one binary
// question per reduced-power slot: did anything arrive?
//
// WHAT COUNTS AS "COVERED" (spec §3.3 does not say; this is the rule that works).
// Not a timing calculation. A cycle is covered when EITHER of its slots was
// heard — that alone proves the cycle occurred and that the window spanned it,
// with no need for the watch to know the anchor's schedule phase or clock. A
// cycle whose HI slot arrived but whose LO slot did not is a genuine miss; a
// cycle the window merely clipped contributes nothing to either count. This is
// what makes the statistic self-calibrating: as the anchor gets far enough that
// even HI slots are lost, the denominator shrinks and PDR abstains rather than
// manufacturing misses. Measured on hardware (tests/prox_v2_spikes.md §S1a):
// HI slots land 100% of the time at 4.8 PDU/slot, so the denominator is solid
// wherever the anchor is audible at all.
#define PDR_OBS_MAX_CYCLES                   8        // ring depth; 1800 ms / 500 ms ~= 4
#define PDR_MIN_COVERED                      2        // below this, abstain entirely

// Beta parameters from spec §11: near {8,2}, away {1,9}. Used as their point
// means (p_near = 0.8, p_away = 0.1) rather than full Beta-binomial tails: the
// accumulated counts are already motion-decayed fractions, for which a
// Beta-binomial in integer hits is not defined, and the HMM's own N_eff draw
// gate is what bounds accumulated confidence. Per-slot Bernoulli log-LRs:
//   hit  -> ln(0.8 / 0.1) = +2.079 nats
//   miss -> ln(0.2 / 0.9) = -1.504 nats
#define LL_PDR_HIT_Q8                        532
#define LL_PDR_MISS_Q8                       (-385)

// Asymmetric caps. This is NOT tuning — it is the §13.0 governing invariant
// ("an attenuating adversary can fabricate AWAY but never NEAR") applied to the
// one feature most exposed to it. PDR is a bare amplitude threshold on a single
// emitter, so a watch pressed into a mattress produces exactly the same LO-slot
// misses as a watch carried out of the room. A *hit* at reduced power cannot be
// forged by attenuation — nothing a user drapes over the radio makes a marginal
// link close — so hits are trusted at full strength while misses are deliberately
// capped at half. PDR may therefore corroborate AWAY but never carry it alone.
#define LL_PDR_NEAR_MAX_Q8                   768      // +3.0 nats, == HMM_EMIT_MAX_Q8
#define LL_PDR_AWAY_MAX_Q8                   (-384)   // -1.5 nats, deliberately half

// Accumulator ceiling, in covered slots. Bounds how much standing confidence the
// PDR channel can hold; both counts are rescaled together on overflow so the
// estimated rate survives untouched.
#define PDR_MAX_EFF_SLOTS                    8

// Motion states — same encoding as the P2 vector trailer's motion_state byte.
#define PROX_MOTION_STILL       0u
#define PROX_MOTION_FIDGET      1u
#define PROX_MOTION_LOCOMOTION  2u
#define PROX_MOTION_UNKNOWN     3u

// Enforcement criterion, for prox_hmm_reset()'s criterion-satisfying cold start.
#define PROX_CRIT_STAY_NEAR     0u
#define PROX_CRIT_GET_AWAY      1u
#define PROX_CRIT_PHONE_AWAY    2u

// ── Mode A: fingerprinting ──────────────────────────────────────────────────
#define PROX_MAX_DEVICES                     60
#define PROX_MIN_DEVICE_COUNT                8
// Minimum devices shared between the watch vector and the anchor's fresh cache
// before Pearson's r means anything. At k=2 the correlation is mathematically
// +/-1 whatever was measured (two points always lie on a line), so the score
// saturates to 0 or 255 -- observed on hardware as a still watch beside a still
// anchor emitting 0, 0, 255, 0, 255. Standard error is roughly 1/sqrt(k-3), so
// 6 is the point where the estimate carries more signal than arithmetic.
#define PROX_MIN_SHARED_DEVICES              6
// Criterion-neutral score returned when no signal has enough data. NOT 0: an
// absent measurement is not evidence of distance, and 0 reads as confident AWAY.
#define PROX_STARVED_SCORE                   128

// ── §13.4-R6: locomotion corroboration for AWAY (the "you can't teleport" gate)
//
// Interim defence for sunrise lock, standing in for R2 until Signal B is made
// offset-invariant. The attack it answers: a pillow or duvet over the wrist
// attenuates every device in the vector by roughly the same amount, which
// Signal A (Pearson, affine-invariant) shrugs off but Signal B (a Gaussian on
// ABSOLUTE dBm) reads as "somewhere else" — and the blend hands authority to B
// as the fingerprint matures. A better calibration therefore makes this attack
// EASIER, which is why no amount of recalibrating has fixed it.
//
// The gate sidesteps the RF argument entirely. getAway asserts "the user left
// the room", and the physical precondition for that is walking: you cannot be
// away from where you were sleeping without having moved there, and a person
// who genuinely got up keeps producing locomotion. A wrist under a pillow
// produces none. So the verdict is corroborated against the body rather than
// against the radio, and no occlusion detector is needed.
//
// Deliberately one-directional (§13.0): it can only make AWAY harder to
// conclude, never NEAR. It can never silence an alarm, only keep one ringing.
//
// ARMING. AWAY is inadmissible until PROX_AWAY_ARM_HITS locomotion sightings
// have landed, each at least PROX_AWAY_ARM_SPACING_S apart, all inside
// PROX_AWAY_ARM_WINDOW_S. The spacing is what makes the sightings independent —
// a single roll-over is one event and cannot supply three. The window is what
// makes them a WALK rather than a scattering: a restless sleeper accumulates
// isolated sightings across a long window, but never three inside a minute.
// With MINIMUM_BLE_DELAY_ENFORCEMENT at 3 s the interrupt path re-checks every
// ~3 s while walking, so genuinely getting up arms this in ~15 s.
#define PROX_AWAY_ARM_HITS                   3
#define PROX_AWAY_ARM_SPACING_S              5
#define PROX_AWAY_ARM_WINDOW_S               60

// LIVENESS. Once armed, a continuous stretch of STILL this long revokes it.
// This closes what arming alone leaves open: get up, walk out, come back to bed
// under the covers with the engine still reporting AWAY. Deliberately NOT keyed
// on locomotion — someone who got up and sat down at a desk produces no
// locomotion for half an hour and is entirely compliant, so the test is
// non-STILL (FIDGET counts), which an awake body produces constantly and a
// sleeping one does not. Five minutes rather than the one minute first proposed:
// a minute of stillness is ordinary for an awake person, and the cost of getting
// it wrong is an alarm restarting on someone who obeyed.
#define PROX_AWAY_LIVENESS_S                 300
#define PROX_MIN_MTU_BYTES                   256
#define PROX_CONFIDENCE_THRESHOLD_U8         170
#define PROX_MISSING_RSSI_DBM                (-100)
#define PROX_MIN_FINGERPRINT_WEIGHT          5.0f
#define PROX_MIN_VARIANCE                    4.0f      // dBm^2 floor (~2 dB sigma)
#define PROX_ALPHA_W0                        2000.0f

// ── Self-RSSI discriminant (the anchor's own beacon level) ──────────────────
// WHY THIS EXISTS. Signals A and B are both blind to the single most
// discriminative measurement in the vector. Pearson is affine-invariant, so a
// uniform level shift cancels EXACTLY; the fingerprint is a shape match over
// third-party emitters. Meanwhile the anchor's own beacon measured 12 dB apart
// between a real install's INSIDE and EDGE positions (-80 vs -92 dBm, S1a),
// against a per-position spread of only ~2-3 dB — a feature with d' ~ 4-6 on its
// own, where the whole rest of the score managed d' = 1.7.
//
// THE TRADE-OFF, STATED. Pearson's affine invariance is exactly what makes it
// resistant to the occlusion attack (§13): smothering the watch attenuates
// everything at once and cancels out. Re-introducing an absolute level term
// re-introduces that attack surface. It is therefore capped ASYMMETRICALLY, on
// the §13.0 invariant "an attenuating adversary can fabricate AWAY but never
// NEAR": a strong self-RSSI cannot be forged by attenuation and is trusted at
// full weight, while a weak one — which is what both distance AND bedding
// produce — is worth half as much. The AWAY direction is thus bounded to a
// modest fraction of the score range and can corroborate but never carry.
#define PROX_SELF_UP_MAX_U8                  48       // strong self-RSSI: full trust
#define PROX_SELF_DOWN_MAX_U8                24       // weak self-RSSI: deliberately half
// Below this demonstrated INSIDE-vs-EDGE span the two positions are not
// distinguishable by level at all, and the term abstains rather than amplify
// noise into a decision.
#define PROX_SELF_MIN_SPAN_DB                6
#define PROX_STRINGIFY_(x) #x
#define PROX_STRINGIFY(x)  PROX_STRINGIFY_(x)
// Maps signal_fingerprint()'s MEAN STANDARDISED RESIDUAL (0 for a perfect match,
// -0.5 at 1 sigma, -2 at 2 sigma, -4.5 at 3 sigma) onto [0,1]. Retuned when the
// Gaussian normalisation term was removed from that average — see the comment in
// signal_fingerprint() for why it had to go. Resulting curve:
//     perfect  -> 0.90     1 sigma -> 0.82
//     2 sigma  -> 0.32     3 sigma -> 0.01
// so a well-matched fingerprint can now assert NEAR instead of only ever pulling
// the score down toward the middle.
#define PROX_LL_CENTER                       (-1.5f)
#define PROX_LL_SCALE                        1.5f
#define PROX_COLLECT_SCORE_THRESHOLD         0.75f
#define PROX_COLLECT_AMBIGUITY_MARGIN_DBM    10
#define PROX_NVS_PERSIST_INTERVAL_S          300
#define ANCHOR_PROX_BLE_SCAN_INTERVAL_MS     2000
#define ANCHOR_PROX_BLE_SCAN_DURATION_MS     500
#define ANCHOR_PROX_WIFI_SCAN_INTERVAL_S     250
// esp_wifi_scan_start() leaks ~1.5 KB/call under BLE coexistence on the C3 (an
// ESP-IDF/coex issue we can't free from app code). The anchor is stationary and
// its WiFi contribution to Signal A is stale-gated (ANCHOR_PROX_DEVICE_STALE_MS
// below) far shorter than the scan interval, so once the local AP environment is
// captured, rescanning adds little. Cap total WiFi scans then end the task so the
// leak is a bounded one-time cost and the heap stays flat. 0 = unlimited (the
// old leaky behavior).
#define ANCHOR_PROX_WIFI_MAX_SCANS           8
#define ANCHOR_PROX_DEVICE_STALE_MS          10000
#define ANCHOR_PROX_MAX_FINGERPRINT_DEVICES  128
// Passive (phase==NONE) self-supervised training gate only. Calibration-v2 no
// longer bootstraps NEAR labels from RSSI (phases are app-demonstrated); this
// strict value keeps passive refinement from ever widening a demonstrated zone.
#define ANCHOR_NEAR_RSSI_THRESHOLD_DBM       (-68)     // raw-RSSI fallback only (passive)
#define PROX_MAX_PEER_ANCHORS                16

// ── Watch scan cache: multi-window sampling-noise suppression ────────────────
// A single scan window is a *random subset* of the BLE population, not a census.
// Advertisers are asynchronous with 100 ms–10 s intervals, so a 700 ms window
// catches a 2 s advertiser only ~35% of the time. Field measurement (still watch,
// still anchor) showed the vector swinging 21–31 devices query-to-query and the
// resulting Pearson score swinging 26–159 — with a frozen channel, i.e. entirely
// estimator noise with no physical signal behind it.
//
// The fix is to stop treating one window as one observation. The cache keeps the
// last PROX_CACHE_TTL_MS of per-window maxima per device and reports the median, so:
//   * membership stabilises — over a 40 s window a 2 s advertiser is caught
//     with near-certainty instead of ~35% per 700 ms scan;
//   * per-reading RSSI noise averages down;
//   * a hit count becomes available, so the MTU-capped device slots can be spent
//     on persistent emitters rather than whichever ephemerals happened to appear.
//
// NOTE this does NOT violate the frozen-fade rule (§1.2). There are two distinct
// noise sources and only one of them is frozen when the wrist is still:
//   * the fading draw is frozen — averaging it adds confidence, not information,
//     which is exactly what N_eff exists to refuse; and
//   * the *subset* drawn by each scan is re-randomised every window regardless of
//     motion, so averaging it removes real error.
// This cache averages the second only. N_eff accounting is untouched, so a still
// wrist still earns no positional evidence — it just stops being fed noise.
// The cache is bounded by WALL TIME, not by a window count. A count alone ties
// coverage to the query cadence, and that broke as soon as queries got faster:
// holding one GATT link open across a calibration burst cut the per-query cost
// from a multi-second connect to a write plus a read, so queries went from ~10 s
// apart to ~2 s. Four windows then spanned ~8 s of advertising opportunity
// instead of ~40 s, and vector sizes collapsed to 8-13 devices on some queries
// (measured range 8..31). More frequent sampling with less coverage per sample is
// the opposite of the intent.
//
// PROX_CACHE_TTL_MS is the real knob: a device is carried for this long after it
// was last heard, whatever the query rate. PROX_CACHE_SAMPLES only caps how many
// per-window maxima are retained for the median.
#define PROX_CACHE_SAMPLES                   8
#define PROX_CACHE_TTL_MS                    40000
#define PROX_RSSI_ABSENT                     (-128)    // sentinel: not seen in that window

// ── Calibration-v2: per-anchor demonstrated near-zone threshold ──────────────
// Score-distribution collectors (INSIDE vs EDGE) fed during calibration; a
// 32-bucket histogram over 0..255 (8 counts/bucket) gives cheap p10/p90.
#define PROX_CALIB_HIST_BUCKETS              32
#define PROX_CALIB_BUCKET_WIDTH              (256 / PROX_CALIB_HIST_BUCKETS)   // 8
// Cutoff sits just above the top of the EDGE scores (edge_p90 + MARGIN), but is
// only accepted if it stays below the INSIDE floor (inside_p10) — else the two
// distributions overlap (bad demo) and we fall back to the global default and
// flag low confidence (decision 3).
#define PROX_CALIB_THRESHOLD_MARGIN_U8       8
#define PROX_CALIB_MIN_SAMPLES               5
// Deferred scoring (see prox_calib_collect / prox_calib_finalize).
//
// The INSIDE leg trains the fingerprint it is simultaneously being scored by, so
// the score changed meaning *during* the run: because
// alpha = exp(-W_total/PROX_ALPHA_W0), early samples were scored by Signal A
// alone and late ones by Signal B. Field measurement, one continuous INSIDE leg
// with the anchor's flash freshly erased:
//
//     scored with fp < 10 devices    n= 6   mean 213
//     scored with fp >= 20 devices   n=13   mean 158
//
// A 55-point drift inside one leg — larger than the INSIDE-vs-EDGE separation
// the calibration exists to measure, and it made the pooled INSIDE p10 collapse
// into the EDGE p90. Percentiles over a mixture of two estimators are not
// percentiles of anything.
//
// Fix: buffer the demonstrated vectors, then at FINALIZE train once and re-score
// every buffered sample against the *final* fingerprint — the estimator that
// will actually be deployed. Both legs then sit on one scale.
#define PROX_CALIB_BUF_SAMPLES               16   // retained per leg (reservoir-sampled)
#define PROX_CALIB_BUF_DEVICES               24   // strongest devices kept per sample
// Hysteresis band below a calibrated per-anchor threshold: scores in
// [near_threshold - HYST, near_threshold) read AMBIGUOUS (resolved to the
// fail-safe-compliant/AWAY side by callers). Decision 4.
#define PROX_NEAR_HYST_U8                    20

// ── Modes B/C: co-location ───────────────────────────────────────────────────
// RETIRED by v2.1 §6.4 (kept for reference / external callers): the EWMA window
// is replaced by the motion-gated integrator, and the two-threshold hysteresis
// machine plus its debounce counter are subsumed by the HMM posterior. The live
// equivalents are INTEG_* and HMM_TAU_* above.
#define COLOC_EWMA_ALPHA                     0.30f    // superseded by INTEG_MOVE_WEIGHT
#define COLOC_WINDOW_SAMPLES                 30       // unused: the integrator has no window
#define COLOC_DECIDE_INTERVAL_S              2
#define COLOC_DEBOUNCE_SAMPLES               3        // unused: HMM replaces the debounce
#define COLOC_TAU_HIGH                       0.80f    // unused: see HMM_TAU_NEAR_Q8
#define COLOC_TAU_LOW                        0.45f    // unused: see HMM_TAU_AWAY_Q8
#define COLOC_PRIOR_NEAR                     0.5f
#define COLOC_ENV_MIN_SHARED_DEVICES         4
#define COLOC_ENV_RSSI_FLOOR_DBM             (-85)

// Default (uncalibrated) distributions; overwritten by calibration.
#define COLOC_DEF_RWP_NEAR_MU                (-55.0f)
#define COLOC_DEF_RWP_NEAR_SIGMA             5.0f
#define COLOC_DEF_RWP_AWAY_MU                (-72.0f)
#define COLOC_DEF_RWP_AWAY_SIGMA             6.0f
#define COLOC_DEF_S_NEAR_MU                  2.0f
#define COLOC_DEF_S_NEAR_SIGMA               1.5f
#define COLOC_DEF_S_AWAY_MU                  7.0f
#define COLOC_DEF_S_AWAY_SIGMA               3.0f
#define COLOC_DEF_DELTA_NEAR_MU              8.0f
#define COLOC_DEF_DELTA_NEAR_SIGMA           5.0f
#define COLOC_DEF_DELTA_AWAY_MU              (-8.0f)
#define COLOC_DEF_DELTA_AWAY_SIGMA           5.0f
#define COLOC_VAR_NEAR_MU                    2.0f
#define COLOC_VAR_NEAR_SIGMA                 2.0f
#define COLOC_VAR_AWAY_MU                    6.0f
#define COLOC_VAR_AWAY_SIGMA                 4.0f

// Device type tags.
#define PROX_TYPE_BLE   0u
#define PROX_TYPE_WIFI  1u

// ProxScoreResult.flags bits.
#define PROX_FLAG_FINGERPRINT_ACTIVE  0x01u
#define PROX_FLAG_LOW_DEVICE_COUNT    0x02u
// Set when the just-scored vector passed the self-supervised training gate and
// was folded into the fingerprint (§4.10.4). Lets a calibration burst count
// samples that actually taught the anchor, rather than mere elapsed time.
#define PROX_FLAG_SAMPLE_ACCEPTED     0x04u

// ============================================================================
// Shared data structures
// ============================================================================

// One observed RF emitter.
typedef struct {
    uint8_t mac[6];   // BLE MAC or WiFi BSSID
    uint8_t type;     // PROX_TYPE_BLE | PROX_TYPE_WIFI
    int8_t  rssi;     // dBm (signed)
} ProxDevice;

// A complete RF-environment snapshot, assembled by the watch.
typedef struct {
    ProxDevice devices[PROX_MAX_DEVICES];
    uint8_t    count;
} ProxScanVector;

// Result of scoring a watch vector against an anchor.
typedef struct {
    uint8_t score;    // 0 = away, 255 = here
    uint8_t flags;    // PROX_FLAG_*
} ProxScoreResult;

// Extended score result consumed by the HMM (engine spec §10). In P1 this is an
// in-memory type only — nothing on the wire changes. `neff` is filled from the
// watch's own integrator; from P2 the anchor echoes its own capped value.
//
// DEVIATION from engine spec §10 (recorded, not accidental): the struct carries
// a 4th field, `near_thr`. This branch already ships calibration-v2, where each
// anchor learns a demonstrated near-zone cutoff in score space and returns it in
// the 3rd byte of the score characteristic. The spec's emission
// logit((score+0.5)/256) is centred on score 128, which would silently discard
// that calibration (a score of 180 would read NEAR-ish even for an anchor whose
// demonstrated cutoff is 210). The emission is therefore centred on the anchor's
// decision point instead — see prox_hmm_tick(). P2 must reconcile the score
// characteristic's 3rd byte, which the amendment (Part 13) reassigns to `neff`:
// the payload needs to be 4 bytes [score][flags][neff][near_thr].
typedef struct {
    uint8_t score;     // 0 = away, 255 = here
    uint8_t flags;     // PROX_FLAG_*
    uint8_t neff;      // effective sample count, integer floor of Q4 (0 = unknown)
    uint8_t near_thr;  // per-anchor calibrated cutoff (0 = uncalibrated)
} ProxScoreResult2;

// Watch-side three-way proximity verdict from the HMM decision layer (§6.3).
// Identical interface to the v0.8 threshold mapping it replaces.
typedef enum {
    PROX_HMM_NEAR = 0,
    PROX_HMM_AWAY = 1,
    PROX_HMM_AMBIGUOUS = 2,
} ProxDecision;

// Motion-gated integrator (§4.2). One instance per integrated quantity; all
// ranged quantities (R_wp, delta, and from P2 the per-anchor r-hat and PDR
// counts) run through it instead of a flat EWMA.
typedef struct {
    int32_t  mean_q8;      // running mean in the source unit, Q8
    int32_t  var_q8;       // reported variance, Q8
    uint16_t neff_q4;      // effective sample count, Q4
    uint8_t  primed;       // 0 until the first sample
    uint8_t  last_motion;  // motion state at the previous update
    uint32_t last_ms;      // clock at the previous update
} ProxIntegrator;

// Co-location detector hypotheses / output.
typedef enum {
    COLOC_AWAY = 0,   // user is away from the phone (compliant)
    COLOC_NEAR = 1,   // user is within ~1-3 ft of the phone
} ColocState;

typedef struct {
    float      p_near;     // fused posterior P(near)
    ColocState state;      // hysteresis-stabilized decision
    uint8_t    holding;    // 1 if the "high mean + low variance" hold signature is present
    uint8_t    used_factors; // bitmask of which factors contributed this tick
} ColocDecision;

// used_factors bits
#define COLOC_F_RANGE  0x01u  // Factor 1: direct watch<->phone RSSI
#define COLOC_F_ENV    0x02u  // Factor 2: RF environment std(delta)
#define COLOC_F_DIFF   0x04u  // Factor delta: two-anchor differential ranging
#define COLOC_F_VAR    0x08u  // Factor V: Var(R_wp)

// One Gaussian hypothesis pair (near vs away) for a scalar feature.
typedef struct {
    float near_mu, near_sigma;
    float away_mu, away_sigma;
    // cached at finalize: log-normalization and 1/(2 sigma^2)
    float near_lognorm, near_inv2var;
    float away_lognorm, away_inv2var;
    uint8_t enabled;
} ColocFeature;

// Full co-location configuration (the calibratable model).
typedef struct {
    ColocFeature rwp;    // direct watch<->phone RSSI       (Factor 1)
    ColocFeature s;      // std(delta) environment statistic (Factor 2)
    ColocFeature delta;  // R_P - R_D differential ranging   (Factor delta)
    ColocFeature var;    // Var(R_wp) hold signature         (Factor V)
    float prior_near;
} ColocConfig;

// Calibration accumulator (Welford) for one feature/class.
typedef struct {
    double mean;
    double m2;
    uint32_t n;
} ColocAccum;

// ============================================================================
// Platform seam — DEFINED BY THE PLATFORM LAYER, called by proximity.cpp.
// ============================================================================
#ifdef __cplusplus
extern "C" {
#endif

uint32_t prox_platform_now_ms(void);
int      prox_platform_nvs_load(const char* key, void* buf, size_t cap, size_t* out_len);
int      prox_platform_nvs_save(const char* key, const void* buf, size_t len);

#ifdef __cplusplus
}
#endif

// ============================================================================
// Public API
// ============================================================================
#ifdef __cplusplus
extern "C" {
#endif

// ---- common ----
void prox_init(void);

// ---- Beacon-schedule slot tagging (both roles; engine spec §3.2) ----
// Minor = (slot_id << 12) | (cycle_seq & 0x0FFF). Transmitted big-endian on air
// per iBeacon convention, so slot_id lands in the high nibble of the FIRST Minor
// byte — see the round-trip test before touching the byte order.
//
// cycle_seq deliberately never takes the value 0: an all-zero Minor is what a
// v0.8 anchor emits, and a watch must be able to read it as "no schedule" rather
// than as slot 0 of cycle 0. decode() returns 0 for that case and the engine
// degrades to Phase-1 behavior.
uint16_t prox_beacon_minor_encode(uint8_t slot_id, uint16_t cycle_seq);
int      prox_beacon_minor_decode(uint16_t minor, uint8_t* out_slot, uint16_t* out_cycle);

// Slot -> physical meaning. channel() gives 37/38/39, or 0 when the slot covers
// all three (the S1b fallback); channel_map() gives the HCI bitmap; is_lo() is 1
// for the reduced-power slots that PDR is measured over.
uint8_t  prox_beacon_slot_channel(uint8_t slot_id);
uint8_t  prox_beacon_slot_channel_map(uint8_t slot_id);
int      prox_beacon_slot_is_lo(uint8_t slot_id);


// ---- Mode A: anchor side ----
#ifdef PROXIMITY_ROLE_ANCHOR
// Called by the platform BLE/WiFi scan callbacks to update the live cache.
void            prox_ingest_scan_result(const uint8_t mac[6], uint8_t type, int8_t rssi);
// Score a submitted watch vector against this anchor (Signal A + Signal B).
ProxScoreResult prox_compute_score(const ProxScanVector* watch_vec);
// Conditionally fold the vector into the fingerprint (self-supervised).
// Returns 1 if the sample passed the training gate and was accepted, else 0.
int             prox_maybe_update_fingerprint(const ProxScanVector* watch_vec,
                                              ProxScoreResult result);
// Diagnostics for the last prox_maybe_update_fingerprint() decision:
// reason — 0 accepted · 1 low-device-count · 2 score-below-threshold ·
//          3 self-MAC-not-in-vector · 4 self-too-far (rssi < near threshold) ·
//          5 ambiguous (a peer anchor was comparably close).
// self_rssi — the anchor's own RSSI as found in that vector (0 = not found).
int             prox_last_train_reason(void);
// Sample sizes behind the last score: devices shared with the watch (Signal A's
// k) and fingerprinted devices actually present in the vector (Signal B's nb).
// Log these next to any score — a score without its sample size cannot be
// debugged, as the 0/255 saturation episode demonstrated.
int             prox_last_shared_count(void);
int             prox_last_fp_seen(void);
int8_t          prox_last_self_rssi(void);
// The anchor's own beacon level within a specific vector (0 = absent from it).
// Prefer this for logging: prox_last_self_rssi() is only populated by the
// passive-training path and reads 0 during calibration.
int8_t          prox_vector_self_rssi(const ProxScanVector* v);
// Replace fingerprint + registry from an app-provided blob.
int             prox_load_fingerprint(const uint8_t* blob, size_t len);
// Deserialize a watch scan vector from the wire format the watch produces with
// prox_serialize_vector() (spec §6.3.1). Returns 1 on success, 0 if malformed.
int             prox_deserialize_vector(const uint8_t* buf, size_t len, ProxScanVector* out);
// Persist fingerprint to NVS (rate-limited internally).
void            prox_persist_if_due(void);
// Set this anchor's own BLE MAC so it can be recognised inside watch vectors.
void            prox_set_self_mac(const uint8_t mac[6]);
// Optional: tell this anchor the MACs of peer anchors, so the self-supervised
// training gate can reject boundary samples that are ambiguous between anchors.
void            prox_set_peer_anchor_macs(const uint8_t macs[][6], int count);

// ---- Phase 2: beacon schedule (anchor side) ----
// Engine -> platform seam (engine spec §10). Called by the engine on every slot
// boundary; the platform performs the advertising stop -> reconfigure (channel
// map, TX power, Minor) -> start sequence. channel_map is BEACON_CH_ALL under
// the S1b fallback, and the platform is free to ignore it in that case.
void     prox_platform_set_beacon_slot(uint8_t channel_map, int8_t tx_power_dbm,
                                       uint16_t minor);

// Start the schedule. epoch_offset_bit comes from the anchor UUID's low bit, so
// a Mode C anchor pair phase-offsets its cycles by BEACON_SCHEDULE_EPOCH_OFFSET_MS
// and their TX_LO slots do not collide (§4.12 item 4).
void     prox_beacon_schedule_init(uint8_t epoch_offset_bit, int8_t tx_hi_dbm);
// Advance the schedule if the current slot has expired. Cheap and idempotent —
// safe to call from the anchor's main loop as often as it likes; it acts only on
// a slot boundary. Returns 1 if a slot change was emitted.
int      prox_beacon_tick(void);
// Current slot id (0..5; only 0 and 3 occur under the fallback) and cycle count.
uint8_t  prox_beacon_slot(void);
uint16_t prox_beacon_cycle_seq(void);

// ---- Calibration-v2 (anchor side) ----
// Phase-labeled training: fold the vector into the fingerprint at full weight,
// skipping the score/RSSI/unambiguous gates entirely (the app guarantees the
// INSIDE label). EDGE (is_inside==0) never trains. Returns 1 if accepted.
int             prox_train_labeled(const ProxScanVector* v, int is_inside);
// Calibration score-distribution collectors. reset() clears both histograms;
// add() records one score into the INSIDE or EDGE histogram; finalize() computes
// the per-anchor near_threshold (decision 3), returns it, reports sample counts
// and a 0..255 confidence (0 = overlap/insufficient → low), and clears the
// collectors.
void            prox_calib_reset(void);
// Direct histogram add — records a score that has ALREADY been computed. Prefer
// prox_calib_collect() during a real calibration: adding live scores mixes
// samples taken before and after the fingerprint matured (see
// PROX_CALIB_BUF_SAMPLES), which is what made a demonstrated 55-point INSIDE
// drift swamp the INSIDE/EDGE separation on hardware.
void            prox_calib_add(int is_inside, uint8_t score);
// Buffer a demonstrated vector for deferred scoring at FINALIZE. Reservoir-
// sampled to PROX_CALIB_BUF_SAMPLES per leg, so a long INSIDE roam is not biased
// toward either end of the walk. When anything has been collected, FINALIZE
// trains once and then re-scores every buffered sample on the final fingerprint,
// putting both legs on one scale.
void            prox_calib_collect(const ProxScanVector* v, int is_inside);
// Total samples OFFERED to a leg (not the retained count) — for progress UI.
int             prox_calib_collected(int is_inside);
uint8_t         prox_calib_finalize(uint16_t* inside_n, uint16_t* edge_n, uint8_t* confidence);

// Why the last prox_calib_finalize() decided what it did. Captured inside
// finalize (which resets the histograms on the way out), so call it afterwards.
// Returns 0 if no calibration has been finalized since boot.
//
// The two "did it work" numbers are dprime_x10 and best_err_pct, and they answer
// a different question from the pass/fail:
//   - `fail` says whether the SHIPPED RULE (edge p90 + margin < inside p10)
//     admitted a cutoff.
//   - `dprime_x10` says whether the two clouds are separable AT ALL, in pooled
//     standard deviations. Near zero means the score itself is not discriminating
//     between the positions and no threshold rule can rescue it — look upstream
//     (shared device count, fingerprint, geometry), not at the rule.
//   - `best_err_pct` is the misclassification rate of the best cutoff that
//     exists. A large d' and a low best_err with fail=OVERLAP means the clouds
//     ARE separable and only the conservative percentile rule refused — usually a
//     few outlier samples in the tails.
typedef struct {
    uint16_t inside_n, edge_n;
    uint8_t  inside_mean, edge_mean;     // score units, from bucket centres
    uint8_t  inside_sd,   edge_sd;
    uint8_t  inside_p10,  edge_p90;      // the two percentiles the rule compares
    uint8_t  inside_min,  inside_max;
    uint8_t  edge_min,    edge_max;
    uint8_t  threshold;                  // what finalize returned
    uint8_t  confidence;
    int16_t  gap;                        // inside_p10 - (edge_p90 + margin); <= 0 is overlap
    uint8_t  dprime_x10;                 // separation in pooled SDs, x10, saturating at 255
    uint8_t  best_thr;                   // error-minimising cutoff over the two histograms
    uint8_t  best_err_pct;               // its total misclassification rate
    uint8_t  edge_above_pct;             // % of EDGE samples at or above inside_p10
    uint8_t  inside_below_pct;           // % of INSIDE samples at or below edge_p90
    uint8_t  fail;                       // PROX_CALIB_FAIL_* bitmask, 0 = accepted
} ProxCalibStats;

#define PROX_CALIB_FAIL_FEW_INSIDE  0x01u
#define PROX_CALIB_FAIL_FEW_EDGE    0x02u
#define PROX_CALIB_FAIL_OVERLAP     0x04u

int             prox_calib_last_stats(ProxCalibStats* out);

// Self-RSSI discriminant (see PROX_SELF_UP_MAX_U8). The near/away levels are
// learned from the two calibration legs and persisted; until they are, the term
// abstains and the score is exactly what it was before.
//   out_near/out_away: learned levels in dBm (0 when uncalibrated)
// Returns 1 when the term is armed.
int             prox_self_levels(int8_t* out_near, int8_t* out_away);
// Score adjustment the last prox_compute_score() applied, in score units.
int             prox_last_self_delta(void);
// Per-anchor decision threshold in score space (0 = uncalibrated → use global
// PROX_CONFIDENCE_THRESHOLD_U8). Persisted alongside the fingerprint NVS blob.
void            prox_set_near_threshold(uint8_t thr);
uint8_t         prox_get_near_threshold(void);
#endif

// ---- Mode A: watch side ----
#ifdef PROXIMITY_ROLE_WATCH
// Called by the platform scan callbacks while assembling a vector. Within one
// window the STRONGEST reading per device wins (do not average here: an anchor
// running the beacon schedule alternates TX_HI/TX_LO 30 dB apart within a single
// window, and the max correctly selects the TX_HI slot).
void   prox_ingest_scan_result(const uint8_t mac[6], uint8_t type, int8_t rssi);
// Close the current scan window: fold this window's per-device maxima into the
// rolling history, age out samples older than PROX_CACHE_TTL_MS, and
// start a fresh window. Call once after each completed scan, before building a
// vector. Idempotent — a window with no ingested results is not folded.
void   prox_scan_window_close(void);
// Drop all cached history (ENFORCEMENT entry, or any known discontinuity).
void   prox_scan_cache_reset(void);
// Build a vector from the cache: per-device RSSI is the median of that device's
// retained per-window maxima, and the PROX_MAX_DEVICES slots are spent on the
// most *persistent* devices first (hit count, then RSSI) rather than simply the
// strongest — a device seen in 4/4 windows is worth far more to the correlation
// than one seen 1/4, and the MTU cap means the choice is forced. Does NOT clear
// the cache; call prox_scan_window_close() to advance the window.
void   prox_build_scan_vector(ProxScanVector* out);
// Diagnostics: how many devices the cache is holding, and how many of those were
// seen in every retained window (the "stable core").
int    prox_scan_cache_count(void);
int    prox_scan_cache_stable_count(void);
// Serialize / parse the wire format (spec §6.3.1). Returns bytes written/read, 0 on error.
size_t prox_serialize_vector(const ProxScanVector* v, uint8_t* out, size_t cap);
#endif

// ---- v2.1 Phase 1: motion channel, integrator, HMM (watch side) ----
#ifdef PROXIMITY_ROLE_WATCH
// Platform -> engine seams (§5.4.5 glue obligations; imu.cpp / main.cpp).
//   xyz: n triples of milli-g (gravity is removed by the burst's own mean).
void    prox_ingest_imu_burst(const int16_t (*xyz)[3], uint16_t n, uint16_t hz);
void    prox_note_motion_interrupt(void);                 // IA1 fired while awake
void    prox_note_sleep_interval(uint32_t slept_ms, int motion_woke);
// Watch-local AWAY evidence: the GATT connect to the anchor failed. Feeds
// LL_CONNFAIL_AWAY_Q8 into the next tick's emission when the last advertisement
// was at or below PROX_FAR_RSSI_THRESHOLD_DBM; abstains otherwise.
void    prox_note_connect_failure(int8_t last_advert_rssi_dbm, int have_advert);

// ---- Observation window & stepped-power PDR (§3.3) ----
// One window per anchor query, wrapped around the full-duty aligned scan:
//   prox_obs_begin(anchor_mac);
//   ... scan callback calls prox_obs_note() for every Impulse advertisement ...
//   prox_obs_close(&hits, &covered);
//
// note() filters by MAC and ignores legacy all-zero Minors itself, so the
// platform can hand it every beacon it parses without pre-checking anything.
// close() reduces the window to hits/covered, folds them into the motion-decayed
// accumulator, and queues the resulting log-LR as watch-local HMM evidence for
// the next tick — the same seam prox_note_connect_failure() uses, and for the
// same reason: PDR needs no wire field, so it does not wait for the P2 trailer.
//
// close() returns 1 when the window produced usable evidence, 0 when it
// abstained (fewer than PDR_MIN_COVERED cycles heard). Both out params may be
// NULL; they are written even when the window abstains, for logging.
void prox_obs_begin(const uint8_t anchor_mac[6]);
void prox_obs_note(const uint8_t mac[6], uint16_t minor, int8_t rssi);
int  prox_obs_close(uint8_t* out_hits, uint8_t* out_covered);

// Accumulator inspection, for shadow logging and the S1a bench. rate_u8 is the
// decayed hit rate scaled 0..255, covered the decayed denominator in whole
// slots, loglr_q8 what close() last queued (0 while abstaining). Returns 0 if
// the channel is currently abstaining.
void prox_pdr_reset(void);
int  prox_pdr_state(uint8_t* out_rate_u8, uint8_t* out_covered, int32_t* out_loglr_q8);

// STILL / FIDGET / LOCOMOTION / UNKNOWN, recomputed on demand from the burst,
// interrupt and sleep-interval evidence. UNKNOWN once the channel goes stale
// (IMU_STALE_MS) and is treated as LOCOMOTION everywhere downstream.
uint8_t prox_motion_state(void);
// Last burst's gravity-removed variance (mg^2, summed over the three axes) and
// whether it showed step cadence. Diagnostics: this is the raw quantity
// IMU_STILL_VAR / IMU_LOCO_VAR are compared against, so it is what Spike S3
// needs to log on real wrists to replace their placeholder values.
uint32_t prox_motion_burst_var(void);
uint8_t  prox_motion_burst_cadence(void);
// Path-length proxy s_IMU expressed as the Q4 N_eff credit accrued over dt_ms
// at the current motion state (§4.1).
uint16_t prox_motion_neff_credit_q4(uint32_t dt_ms);

// ── §13.4-R6 locomotion gate (see the constants above) ──────────────────────
//
// Takes wall-clock seconds from the caller rather than reading a clock, both so
// it is testable and because it must survive light sleep — the watch backs its
// enforcement poll off to minutes and the millisecond clock is not the thing to
// trust across that. Steps backwards (an NTP correction mid-window) are treated
// as a clock reset rather than as elapsed time.

// Call when a getAway window opens. Disarmed, no sightings, liveness clock
// starts now.
void prox_away_gate_reset(uint32_t now_s);

// Feed one motion observation, taken from prox_motion_state(). Call after each
// query, when the burst is fresh.
//
// UNKNOWN is ignored outright. Everywhere else in the engine a stale motion
// channel is treated as LOCOMOTION (fail toward v0.8 behaviour), but here that
// would let an IMU that has simply gone quiet arm the gate on no evidence at
// all — the exact inversion this defence exists to prevent. Arming requires
// POSITIVE locomotion.
void prox_away_gate_note(uint32_t now_s, uint8_t motion_state);

// 1 if an AWAY verdict may currently be believed. Evaluates the liveness
// timeout, so calling it is what revokes a stale arming.
int prox_away_gate_admits(uint32_t now_s);

// Seconds until liveness revokes the arming, or 0 if not armed / already due.
// Lets the caller cap a light sleep so revocation is not slept through.
uint32_t prox_away_gate_ttl_s(uint32_t now_s);

// Diagnostics for the [AWAYGATE] log line. Any out-pointer may be NULL.
void prox_away_gate_state(uint8_t *out_armed, uint8_t *out_hits,
                          uint32_t *out_last_move_s);

// Integrator (§4.2). update() reads the clock and motion state itself.
void     prox_integ_reset(ProxIntegrator* it);
void     prox_integ_update(ProxIntegrator* it, int32_t x_q8);
int      prox_integ_mean_q8(const ProxIntegrator* it, int32_t* out);
int      prox_integ_var_q8(const ProxIntegrator* it, int32_t* out);
uint8_t  prox_integ_neff(const ProxIntegrator* it);

// Two-state motion-conditioned HMM (§6). reset() cold-starts the posterior at
// the criterion-satisfying state (P = 0.65) on ENFORCEMENT entry; tick() folds
// one completed query in and returns the three-way decision; decision() is a
// side-effect-free read of the held decision. A poll that produced no query
// should tick with a NULL result — that still advances the transition model and
// folds any pending watch-local evidence.
void         prox_hmm_reset(uint8_t criterion);
ProxDecision prox_hmm_tick(const ProxScoreResult2* r);
ProxDecision prox_hmm_decision(void);
// Modes B/C: inject an already-summed log-likelihood-ratio (Q8) as the emission
// instead of an anchor score (§6.4).
ProxDecision prox_hmm_tick_loglr_q8(int32_t loglr_q8);
// Diagnostics / shadow logging: current NEAR-vs-AWAY log-odds in Q8, and
// P(NEAR) as 0..255. Both are read-only.
int32_t      prox_hmm_logodds_q8(void);
uint8_t      prox_hmm_p_near_u8(void);
#endif

// ---- Modes B/C: co-location (watch side) ----
void          coloc_init(void);                 // loads defaults / calibration from NVS
ColocConfig*  coloc_config(void);               // direct access for tuning/inspection
void          coloc_finalize_calibration(void); // recompute cached norm constants

// Per-reading ingestion (~1 Hz). have_anchor_diff selects Mode C.
//   r_wp : watch<->phone link RSSI (dBm)
//   r_p  : watch<->parking-anchor RSSI (dBm)   } only used if have_anchor_diff
//   r_d  : watch<->desk-anchor RSSI (dBm)       }
void coloc_ingest_link_rssi(int8_t r_wp, int have_anchor_diff, int8_t r_p, int8_t r_d);
// Per-reading ingestion of one shared external device (watch & phone/anchor views).
void coloc_ingest_shared_device(int8_t watch_rssi, int8_t peer_rssi);
// Call once per reading after ingestion to roll the window / clear per-tick buffers.
void coloc_mark_reading_end(void);

// Run the fusion + hysteresis decision (call every COLOC_DECIDE_INTERVAL_S).
ColocDecision coloc_tick(void);
// Most recent decision without recomputing.
ColocDecision coloc_decision(void);

// Calibration: class 0 = NEAR ("hold your phone"), class 1 = AWAY ("park it, step away").
void coloc_calib_begin(void);
void coloc_calib_add_sample(int klass, int8_t r_wp, int have_anchor_diff,
                            int8_t r_p, int8_t r_d, float s_env, float var_rwp);
void coloc_calib_finalize(void);   // fits Gaussians, writes config, persists to NVS

#ifdef __cplusplus
}
#endif

#endif // PROXIMITY_H
