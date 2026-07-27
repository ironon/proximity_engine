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
#define ANCHOR_NEAR_RSSI_THRESHOLD_DBM       (-85)     // raw-RSSI fallback only
#define PROX_MAX_PEER_ANCHORS                16

// ── Modes B/C: co-location ───────────────────────────────────────────────────
#define COLOC_EWMA_ALPHA                     0.30f
#define COLOC_WINDOW_SAMPLES                 30
#define COLOC_DECIDE_INTERVAL_S              2
#define COLOC_DEBOUNCE_SAMPLES               3
#define COLOC_TAU_HIGH                       0.80f
#define COLOC_TAU_LOW                        0.45f
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
