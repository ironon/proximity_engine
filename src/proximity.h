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
#define PROX_V2_AUTHORITATIVE                0

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
// S3-tuned. Until S3 lands these are deliberately conservative: STILL is hard to
// declare and misclassification lands on LOCOMOTION, which fails toward v0.8
// behavior rather than toward a frozen decision.
#define IMU_STILL_VAR                        400      // (20 mg RMS)^2
#define IMU_LOCO_VAR                         40000    // (200 mg RMS)^2
#define IMU_LOCO_MIN_INTS                    2
#define IMU_STALE_MS                         5000
// Step-cadence detection inside one burst (0.5-3 Hz over IMU_BURST_SAMPLES /
// IMU_BURST_HZ seconds of signal), guarded by a variance floor so sensor noise
// cannot manufacture crossings.
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
#define PROX_MIN_MTU_BYTES                   256
#define PROX_CONFIDENCE_THRESHOLD_U8         170
#define PROX_MISSING_RSSI_DBM                (-100)
#define PROX_MIN_FINGERPRINT_WEIGHT          5.0f
#define PROX_MIN_VARIANCE                    4.0f      // dBm^2 floor (~2 dB sigma)
#define PROX_ALPHA_W0                        2000.0f
#define PROX_LL_CENTER                       (-3.0f)
#define PROX_LL_SCALE                        0.5f
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
int8_t          prox_last_self_rssi(void);
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
void            prox_calib_add(int is_inside, uint8_t score);
uint8_t         prox_calib_finalize(uint16_t* inside_n, uint16_t* edge_n, uint8_t* confidence);
// Per-anchor decision threshold in score space (0 = uncalibrated → use global
// PROX_CONFIDENCE_THRESHOLD_U8). Persisted alongside the fingerprint NVS blob.
void            prox_set_near_threshold(uint8_t thr);
uint8_t         prox_get_near_threshold(void);
#endif

// ---- Mode A: watch side ----
#ifdef PROXIMITY_ROLE_WATCH
// Called by the platform scan callbacks while assembling a vector.
void   prox_ingest_scan_result(const uint8_t mac[6], uint8_t type, int8_t rssi);
// Snapshot the current scan into a vector (keeps strongest PROX_MAX_DEVICES).
void   prox_build_scan_vector(ProxScanVector* out);
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
