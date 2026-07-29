// proximity.cpp
//
// Impulse Proximity Engine — full implementation.
//
//   Mode A : multi-location fingerprinting   (anchor scores, watch submits)
//   Mode B : binary co-location               (watch: phone-in-hand vs parked)
//   Mode C : dorm differential ranging        (watch: two-anchor delta)
//
// Design rationale and math live in proximity_engine_spec.md. This file is the
// single home for all proximity ALGORITHM code; platform glue is reached only
// through the seam functions declared `extern` in proximity.h.
//
// Target: ESP32 family. NOTE: the ESP32-C3 has NO hardware FPU, so log/exp are
// software-emulated. The hot paths cache every transcendental (Gaussian norm
// constants, 1/(2 sigma^2)) so per-query/per-tick work is multiply-add + at most
// one exp. See feature_cache() and the AnchorDev cached fields.

#include "proximity.h"
#include "prox_luts.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static const float LOG_SQRT_2PI = 0.9189385f; // 0.5 * ln(2*pi)

// ============================================================================
// Small numeric helpers (shared by all modes)
// ============================================================================

static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

// 1 / (1 + e^-x). The only exp on the co-location decision path.
static inline float logistic(float x) {
    if (x >= 0.0f) { float z = expf(-x); return 1.0f / (1.0f + z); }
    float z = expf(x); return z / (1.0f + z);
}

static inline int mac_eq(const uint8_t a[6], const uint8_t b[6]) {
    return memcmp(a, b, 6) == 0;
}

// Little-endian (de)serialization — wire formats are LE per spec §6.3.
static inline uint16_t rd_u16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static inline uint32_t rd_u32(const uint8_t* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static inline float rd_f32(const uint8_t* p) { uint32_t u = rd_u32(p); float f; memcpy(&f, &u, 4); return f; }
static inline void wr_u16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void wr_u32(uint8_t* p, uint32_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24); }
static inline void wr_f32(uint8_t* p, float f) { uint32_t u; memcpy(&u, &f, 4); wr_u32(p, u); }

// ============================================================================
// BEACON SCHEDULE — slot tagging (both roles; engine spec §3.1-§3.2)
//
// The anchor cycles through (channel x TX power) slots and stamps each one into
// the iBeacon Minor field. Frequency diversity is the only diversity available
// to a stationary watch (§1.2), and the stepped-power slots turn "can the link
// close at reduced power" into a thresholded, amplitude-noise-immune range
// feature. Both are free on the scan side: the watch reads the slot straight out
// of an advertisement it was already receiving.
// ============================================================================

// Slot layout. Full schedule: 0/1/2 = ch37/38/39 at TX_HI, 3/4/5 = same at TX_LO.
// S1b fallback: only slot ids 0 and 3 occur, each covering all three channels.
static const uint8_t k_slot_ch[6] = { 37, 38, 39, 37, 38, 39 };

uint16_t prox_beacon_minor_encode(uint8_t slot_id, uint16_t cycle_seq) {
    return (uint16_t)(((uint16_t)(slot_id & 0x0F) << 12) | (cycle_seq & 0x0FFF));
}

int prox_beacon_minor_decode(uint16_t minor, uint8_t* out_slot, uint16_t* out_cycle) {
    if (minor == 0x0000) return 0;          // legacy v0.8 anchor: no schedule
    if (out_slot)  *out_slot  = (uint8_t)((minor >> 12) & 0x0F);
    if (out_cycle) *out_cycle = (uint16_t)(minor & 0x0FFF);
    return 1;
}

uint8_t prox_beacon_slot_channel(uint8_t slot_id) {
#if BEACON_CHANNEL_CONTROL
    return (slot_id < 6) ? k_slot_ch[slot_id] : 0;
#else
    (void)slot_id; (void)k_slot_ch;
    return 0;                                // all three; not attributable
#endif
}

uint8_t prox_beacon_slot_channel_map(uint8_t slot_id) {
#if BEACON_CHANNEL_CONTROL
    switch (slot_id % 3) {
        case 0:  return BEACON_CH37_BIT;
        case 1:  return BEACON_CH38_BIT;
        default: return BEACON_CH39_BIT;
    }
#else
    (void)slot_id;
    return BEACON_CH_ALL;
#endif
}

int prox_beacon_slot_is_lo(uint8_t slot_id) { return slot_id >= 3 ? 1 : 0; }

// ============================================================================
// MODE A — ANCHOR SIDE
// ============================================================================
#ifdef PROXIMITY_ROLE_ANCHOR

typedef struct {
    uint8_t  mac[6];
    uint8_t  type;
    uint8_t  in_use;
    // live RF cache
    int8_t   live_rssi;
    uint32_t last_seen_ms;
    // fingerprint (weighted Welford accumulators)
    float    mu;      // running mean RSSI (dBm)
    float    M;       // running weighted sum of squared deviations
    float    W;       // cumulative weight
    // cached for fast scoring (recomputed on update / load)
    float    var;
    float    lognorm; // -0.5*log(2*pi*var)
    float    inv2var; // 1/(2*var)
    uint8_t  fp_active;
} AnchorDev;

static AnchorDev g_reg[ANCHOR_PROX_MAX_FINGERPRINT_DEVICES];
static int       g_reg_count = 0;

static uint8_t   g_self_mac[6];
static uint8_t   g_have_self = 0;
static uint8_t   g_peer_macs[PROX_MAX_PEER_ANCHORS][6];
static int       g_peer_count = 0;
static uint32_t  g_last_persist_ms = 0;

// Calibration-v2 per-anchor threshold (0 = uncalibrated → use global default).
static uint8_t   g_near_threshold = 0;
// Calibration-v2 score-distribution collectors (32-bucket histograms, 0..255).
static uint16_t  g_calib_inside_hist[PROX_CALIB_HIST_BUCKETS];
static uint16_t  g_calib_edge_hist[PROX_CALIB_HIST_BUCKETS];
static uint32_t  g_calib_inside_n = 0;
static uint32_t  g_calib_edge_n   = 0;

static const char* PROX_NVS_KEY = "prox_fp";

static AnchorDev* reg_find(const uint8_t mac[6], uint8_t type) {
    for (int i = 0; i < g_reg_count; ++i)
        if (g_reg[i].in_use && g_reg[i].type == type && mac_eq(g_reg[i].mac, mac))
            return &g_reg[i];
    return NULL;
}

static AnchorDev* reg_add(const uint8_t mac[6], uint8_t type) {
    if (g_reg_count >= ANCHOR_PROX_MAX_FINGERPRINT_DEVICES) return NULL; // cap: ignore new
    AnchorDev* d = &g_reg[g_reg_count++];
    memset(d, 0, sizeof(*d));
    memcpy(d->mac, mac, 6);
    d->type = type;
    d->in_use = 1;
    d->live_rssi = PROX_MISSING_RSSI_DBM;
    d->var = PROX_MIN_VARIANCE;
    return d;
}

static void dev_recache(AnchorDev* d) {
    float v = d->var < PROX_MIN_VARIANCE ? PROX_MIN_VARIANCE : d->var;
    d->var = v;
    d->inv2var = 1.0f / (2.0f * v);
    d->lognorm = -0.5f * logf(v) - LOG_SQRT_2PI;
    d->fp_active = (d->W >= PROX_MIN_FINGERPRINT_WEIGHT) ? 1u : 0u;
}

void prox_set_self_mac(const uint8_t mac[6]) { memcpy(g_self_mac, mac, 6); g_have_self = 1; }

void prox_set_peer_anchor_macs(const uint8_t macs[][6], int count) {
    if (count > PROX_MAX_PEER_ANCHORS) count = PROX_MAX_PEER_ANCHORS;
    for (int i = 0; i < count; ++i) memcpy(g_peer_macs[i], macs[i], 6);
    g_peer_count = count;
}

// Platform BLE/WiFi scan callbacks feed this.
void prox_ingest_scan_result(const uint8_t mac[6], uint8_t type, int8_t rssi) {
    AnchorDev* d = reg_find(mac, type);
    if (!d) d = reg_add(mac, type);
    if (!d) return;
    d->live_rssi = rssi;
    d->last_seen_ms = prox_platform_now_ms();
}

static int dev_is_fresh(const AnchorDev* d, uint32_t now) {
    return d->last_seen_ms != 0 && (now - d->last_seen_ms) <= ANCHOR_PROX_DEVICE_STALE_MS;
}

// Find a device's RSSI inside a watch vector; returns 1 if present.
static int vec_lookup(const ProxScanVector* v, const uint8_t mac[6], uint8_t type, int8_t* out) {
    for (int i = 0; i < v->count; ++i)
        if (v->devices[i].type == type && mac_eq(v->devices[i].mac, mac)) { *out = v->devices[i].rssi; return 1; }
    return 0;
}

// Signal A: Pearson correlation over devices shared by watch vector and fresh cache.
static int signal_correlation(const ProxScanVector* v, float* out_rho) {
    uint32_t now = prox_platform_now_ms();
    float sw = 0, sa = 0; int k = 0;
    // pass 1: means
    for (int i = 0; i < v->count; ++i) {
        AnchorDev* d = reg_find(v->devices[i].mac, v->devices[i].type);
        if (!d || !dev_is_fresh(d, now)) continue;
        sw += v->devices[i].rssi; sa += d->live_rssi; k++;
    }
    if (k < 2) { *out_rho = 0.0f; return 0; }
    float mw = sw / k, ma = sa / k;
    // pass 2: covariance / variances
    float cov = 0, vw = 0, va = 0;
    for (int i = 0; i < v->count; ++i) {
        AnchorDev* d = reg_find(v->devices[i].mac, v->devices[i].type);
        if (!d || !dev_is_fresh(d, now)) continue;
        float dw = v->devices[i].rssi - mw, da = d->live_rssi - ma;
        cov += dw * da; vw += dw * dw; va += da * da;
    }
    if (vw <= 0.0f || va <= 0.0f) { *out_rho = 0.0f; return 0; }
    float rho = cov / (sqrtf(vw) * sqrtf(va));
    *out_rho = clampf(rho, 0.0f, 1.0f);
    return 1;
}

// Signal B: Gaussian Naive Bayes likelihood of the vector under the fingerprint.
// Returns L in [0,1] via logistic on average per-device log-likelihood; out_wtot
// receives the total fingerprint weight (drives the blend factor alpha).
static int signal_fingerprint(const ProxScanVector* v, float* out_L, float* out_wtot) {
    float wtot = 0.0f;
    float loglik = 0.0f;
    int   nb = 0;
    for (int i = 0; i < g_reg_count; ++i) {
        AnchorDev* d = &g_reg[i];
        if (!d->in_use) continue;
        wtot += d->W;
        if (!d->fp_active) continue;
        int8_t x;
        if (!vec_lookup(v, d->mac, d->type, &x)) x = PROX_MISSING_RSSI_DBM;
        float dx = (float)x - d->mu;
        loglik += d->lognorm - dx * dx * d->inv2var;
        nb++;
    }
    *out_wtot = wtot;
    if (nb < PROX_MIN_DEVICE_COUNT) { *out_L = 0.0f; return 0; }
    float avg = loglik / (float)nb;
    *out_L = logistic((avg - PROX_LL_CENTER) * PROX_LL_SCALE);
    return 1;
}

// Parse the watch's wire format (spec §6.3.1): [1 count][per dev: 6 mac,1 type,1 (rssi+128)].
int prox_deserialize_vector(const uint8_t* buf, size_t len, ProxScanVector* out) {
    if (!buf || !out || len < 1) return 0;
    int count = buf[0];
    if (count > PROX_MAX_DEVICES) count = PROX_MAX_DEVICES;
    size_t expected = 1u + (size_t)count * 8u;
    if (len < expected) { out->count = 0; return 0; }
    out->count = (uint8_t)count;
    for (int i = 0; i < count; ++i) {
        const uint8_t* p = buf + 1 + (size_t)i * 8;
        memcpy(out->devices[i].mac, p, 6);
        out->devices[i].type = p[6];
        out->devices[i].rssi = (int8_t)((int)p[7] - 128); // decode rssi+128
    }
    return 1;
}

ProxScoreResult prox_compute_score(const ProxScanVector* watch_vec) {
    ProxScoreResult r = { 0, 0 };

    // Low-device-count fallback: single raw RSSI comparison to self.
    if (watch_vec->count < PROX_MIN_DEVICE_COUNT) {
        r.flags |= PROX_FLAG_LOW_DEVICE_COUNT;
        int8_t self_rssi;
        if (g_have_self && vec_lookup(watch_vec, g_self_mac, PROX_TYPE_BLE, &self_rssi)
            && self_rssi >= ANCHOR_NEAR_RSSI_THRESHOLD_DBM)
            r.score = 200;
        else
            r.score = 50;
        return r;
    }

    float rho; signal_correlation(watch_vec, &rho);
    float L, wtot; int have_fp = signal_fingerprint(watch_vec, &L, &wtot);

    float alpha = have_fp ? expf(-wtot / PROX_ALPHA_W0) : 1.0f;
    float score_f = alpha * rho + (1.0f - alpha) * L;
    score_f = clampf(score_f, 0.0f, 1.0f);

    r.score = (uint8_t)(score_f * 255.0f + 0.5f);
    if (have_fp) r.flags |= PROX_FLAG_FINGERPRINT_ACTIVE;
    return r;
}

// Weighted Welford update of one device with sample x and weight w.
static void dev_welford(AnchorDev* d, float x, float w) {
    float W_new = d->W + w;
    if (W_new <= 0.0f) return;
    float mu_old = d->mu;
    float mu_new = mu_old + (w / W_new) * (x - mu_old);
    d->M += w * (x - mu_old) * (x - mu_new);
    d->mu = mu_new;
    d->W = W_new;
    d->var = d->M / d->W;       // population (West's reliability-weighted) variance
    dev_recache(d);
}

// Diagnostics for the last training decision (see prox_last_train_reason).
static int    g_last_train_reason = -1;
static int8_t g_last_self_rssi    = 0;
int    prox_last_train_reason(void) { return g_last_train_reason; }
int8_t prox_last_self_rssi(void)    { return g_last_self_rssi; }

// Is the sample unambiguous? Requires self present and beating any known peer
// anchor by PROX_COLLECT_AMBIGUITY_MARGIN_DBM. If no peers are known, the self
// presence + reasonable-strength gate is used (see spec §2.5 / §4). Sets the
// training-reason diagnostics on failure.
static int sample_is_unambiguous(const ProxScanVector* v) {
    if (!g_have_self) { g_last_train_reason = 3; return 0; }
    int8_t self_rssi;
    if (!vec_lookup(v, g_self_mac, PROX_TYPE_BLE, &self_rssi)) { g_last_train_reason = 3; return 0; }
    g_last_self_rssi = self_rssi;
    if (self_rssi < ANCHOR_NEAR_RSSI_THRESHOLD_DBM) { g_last_train_reason = 4; return 0; }
    for (int i = 0; i < g_peer_count; ++i) {
        int8_t peer_rssi;
        if (vec_lookup(v, g_peer_macs[i], PROX_TYPE_BLE, &peer_rssi))
            if (peer_rssi >= self_rssi - PROX_COLLECT_AMBIGUITY_MARGIN_DBM)
                { g_last_train_reason = 5; return 0; } // rival anchor comparably close
    }
    return 1;
}

int prox_maybe_update_fingerprint(const ProxScanVector* watch_vec, ProxScoreResult result) {
    g_last_self_rssi = 0;
    if (result.flags & PROX_FLAG_LOW_DEVICE_COUNT) { g_last_train_reason = 1; return 0; }
    float score_f = result.score / 255.0f;
    if (score_f < PROX_COLLECT_SCORE_THRESHOLD) { g_last_train_reason = 2; return 0; }
    if (!sample_is_unambiguous(watch_vec)) return 0;

    // Update every registry device; teach "not visible" for absent ones at half weight.
    for (int i = 0; i < g_reg_count; ++i) {
        AnchorDev* d = &g_reg[i];
        if (!d->in_use) continue;
        int8_t x;
        if (vec_lookup(watch_vec, d->mac, d->type, &x))
            dev_welford(d, (float)x, score_f);
        else
            dev_welford(d, (float)PROX_MISSING_RSSI_DBM, score_f * 0.5f);
    }
    // Register any newly-seen devices and seed them with this sample.
    for (int i = 0; i < watch_vec->count; ++i) {
        const ProxDevice* pd = &watch_vec->devices[i];
        if (reg_find(pd->mac, pd->type)) continue;
        AnchorDev* d = reg_add(pd->mac, pd->type);
        if (d) { d->mu = (float)pd->rssi; dev_welford(d, (float)pd->rssi, score_f); }
    }
    g_last_train_reason = 0; // accepted
    return 1;
}

// ============================================================================
// Beacon schedule task (anchor; §4.12)
//
// Timer-driven slot advance. The anchor is mains/USB powered so this is free,
// and the TX_LO slots actually *reduce* its mean radiated power. It must never
// interfere with connectability: proximity queries and setup have to work in
// every slot (§4.12 item 1), which is the platform seam's responsibility.
// ============================================================================

// Schedule index, which is not the wire slot_id: under the S1b fallback the two
// emitted slots are wire ids 0 and 3 (HI and LO), so a watch can tell a 2-slot
// anchor from a 6-slot one purely from the set of slot ids it observes.
static uint8_t  g_beacon_idx     = 0;
static uint16_t g_beacon_cycle   = 1;      // never 0: see prox_beacon_minor_decode
static uint32_t g_beacon_next_ms = 0;
static int8_t   g_beacon_tx_hi   = 9;
static uint8_t  g_beacon_running = 0;

static uint8_t beacon_wire_slot(uint8_t idx) {
#if BEACON_CHANNEL_CONTROL
    return idx;                              // 0..5 map straight through
#else
    return idx ? 3u : 0u;                    // HI -> 0, LO -> 3
#endif
}

static void beacon_emit(void) {
    uint8_t slot = beacon_wire_slot(g_beacon_idx);
    int8_t  txp  = prox_beacon_slot_is_lo(slot) ? (int8_t)BEACON_TX_LO_DBM : g_beacon_tx_hi;
    prox_platform_set_beacon_slot(prox_beacon_slot_channel_map(slot), txp,
                                  prox_beacon_minor_encode(slot, g_beacon_cycle));
}

void prox_beacon_schedule_init(uint8_t epoch_offset_bit, int8_t tx_hi_dbm) {
    g_beacon_tx_hi   = tx_hi_dbm;
    g_beacon_idx     = 0;
    g_beacon_cycle   = 1;
    g_beacon_running = BEACON_SCHEDULE_ENABLE ? 1u : 0u;
    if (!g_beacon_running) return;
    // Mode C pairs offset their epochs so their TX_LO slots never coincide —
    // otherwise the differential measurement the two anchors exist to provide is
    // taken while both are quiet.
    g_beacon_next_ms = prox_platform_now_ms() +
                       (epoch_offset_bit ? (uint32_t)BEACON_SCHEDULE_EPOCH_OFFSET_MS : 0u);
    beacon_emit();
}

int prox_beacon_tick(void) {
    if (!g_beacon_running) return 0;
    uint32_t now = prox_platform_now_ms();
    if ((int32_t)(now - g_beacon_next_ms) < 0) return 0;

    g_beacon_next_ms += BEACON_SLOT_MS;
    // If the loop was blocked long enough to miss whole slots (a long GATT
    // operation), resync rather than replaying the backlog at full speed.
    if ((int32_t)(now - g_beacon_next_ms) > BEACON_SLOT_MS)
        g_beacon_next_ms = now + BEACON_SLOT_MS;

    if (++g_beacon_idx >= BEACON_SLOT_COUNT) {
        g_beacon_idx = 0;
        if (++g_beacon_cycle > 0x0FFF) g_beacon_cycle = 1;   // skip 0, see decode
    }
    beacon_emit();
    return 1;
}

uint8_t  prox_beacon_slot(void)      { return beacon_wire_slot(g_beacon_idx); }
uint16_t prox_beacon_cycle_seq(void) { return g_beacon_cycle; }

// ============================================================================
// Calibration-v2: phase-labeled training + per-anchor threshold
// ============================================================================

// Fold a vector into the fingerprint under an app-guaranteed label. INSIDE folds
// at full weight with NO score/RSSI/unambiguous gate (decision 2). EDGE never
// trains (its scores are only collected via prox_calib_add). Returns 1 if the
// vector was folded in.
int prox_train_labeled(const ProxScanVector* v, int is_inside) {
    if (!is_inside) return 0;          // EDGE: collect scores only, never train
    if (!v || v->count == 0) return 0;
    // Update every registry device at full weight; teach "not visible" for
    // absent ones at half weight (mirrors the passive path, minus the gates).
    for (int i = 0; i < g_reg_count; ++i) {
        AnchorDev* d = &g_reg[i];
        if (!d->in_use) continue;
        int8_t x;
        if (vec_lookup(v, d->mac, d->type, &x))
            dev_welford(d, (float)x, 1.0f);
        else
            dev_welford(d, (float)PROX_MISSING_RSSI_DBM, 0.5f);
    }
    // Register + seed any newly-seen devices at full weight.
    for (int i = 0; i < v->count; ++i) {
        const ProxDevice* pd = &v->devices[i];
        if (reg_find(pd->mac, pd->type)) continue;
        AnchorDev* d = reg_add(pd->mac, pd->type);
        if (d) { d->mu = (float)pd->rssi; dev_welford(d, (float)pd->rssi, 1.0f); }
    }
    return 1;
}

void prox_calib_reset(void) {
    memset(g_calib_inside_hist, 0, sizeof(g_calib_inside_hist));
    memset(g_calib_edge_hist,   0, sizeof(g_calib_edge_hist));
    g_calib_inside_n = 0;
    g_calib_edge_n   = 0;
}

void prox_calib_add(int is_inside, uint8_t score) {
    int b = score / PROX_CALIB_BUCKET_WIDTH;
    if (b >= PROX_CALIB_HIST_BUCKETS) b = PROX_CALIB_HIST_BUCKETS - 1;
    if (is_inside) {
        g_calib_inside_hist[b]++;
        if (g_calib_inside_n < 0xFFFFu) g_calib_inside_n++;
    } else {
        g_calib_edge_hist[b]++;
        if (g_calib_edge_n < 0xFFFFu) g_calib_edge_n++;
    }
}

// Score at the given percentile (0..100) from a histogram; returns the bucket
// center (0..255). n is the total count that produced the histogram.
static uint8_t histo_percentile(const uint16_t* h, uint32_t n, int pct) {
    if (n == 0) return 0;
    uint32_t target = ((uint64_t)n * (uint32_t)pct + 99u) / 100u; // ceil(n*pct/100)
    if (target == 0) target = 1;
    uint32_t cum = 0;
    for (int b = 0; b < PROX_CALIB_HIST_BUCKETS; ++b) {
        cum += h[b];
        if (cum >= target)
            return (uint8_t)(b * PROX_CALIB_BUCKET_WIDTH + PROX_CALIB_BUCKET_WIDTH / 2);
    }
    return 255;
}

uint8_t prox_calib_finalize(uint16_t* inside_n, uint16_t* edge_n, uint8_t* confidence) {
    uint16_t in_n = (uint16_t)g_calib_inside_n;
    uint16_t ed_n = (uint16_t)g_calib_edge_n;
    uint8_t thr, conf;

    if (g_calib_inside_n < PROX_CALIB_MIN_SAMPLES || g_calib_edge_n < PROX_CALIB_MIN_SAMPLES) {
        // Not enough demonstrated samples to trust a learned cutoff.
        thr  = PROX_CONFIDENCE_THRESHOLD_U8;
        conf = 0;
    } else {
        uint8_t inside_p10 = histo_percentile(g_calib_inside_hist, g_calib_inside_n, 10);
        uint8_t edge_p90   = histo_percentile(g_calib_edge_hist,   g_calib_edge_n,   90);
        int cand = (int)edge_p90 + PROX_CALIB_THRESHOLD_MARGIN_U8; // just above edge top
        if (cand > 255) cand = 255;
        if ((int)inside_p10 > cand) {
            // Clean separation: cutoff sits in the gap, below the inside floor.
            thr = (uint8_t)cand;
            int gap = (int)inside_p10 - cand;      // width of the confidence margin
            int c = gap * 4; if (c > 255) c = 255; // ~64 score-units of gap → full
            conf = (uint8_t)c;
        } else {
            // Overlap (edge scores reach into the inside scores) — bad demo.
            thr  = PROX_CONFIDENCE_THRESHOLD_U8;
            conf = 0;
        }
    }

    if (inside_n)   *inside_n = in_n;
    if (edge_n)     *edge_n = ed_n;
    if (confidence) *confidence = conf;
    prox_calib_reset();
    return thr;
}

void prox_set_near_threshold(uint8_t thr) {
    g_near_threshold = thr;
    g_last_persist_ms = 0; // force a persist on the next due check
}

uint8_t prox_get_near_threshold(void) { return g_near_threshold; }

// ---- persistence ----
// Internal NVS blob keeps full Welford state (mu, M, W) so accumulation resumes
// across reboots. Calibration-v2 versions the blob:
//   v1: [1 version=1][1 near_threshold][2 count][per dev: 6 mac,1 type,4 mu,4 M,4 W]
//   v0 (legacy): [2 count][per dev: ...]   ← read as version 0, near_threshold 0
// Version is detected from byte 0: v1 stamps 0x01 there; the legacy format's
// byte 0 is the low byte of the device count and is only 0x01 for the (invalid,
// sub-min-device) count==1 case, so 0x01 unambiguously marks v1 in practice.
#define PROX_NVS_REC     19
#define PROX_NVS_VERSION 1
#define PROX_NVS_HDR     4   // [ver][thr][count u16]

void prox_persist_if_due(void) {
    uint32_t now = prox_platform_now_ms();
    if (g_last_persist_ms != 0 && (now - g_last_persist_ms) < (uint32_t)PROX_NVS_PERSIST_INTERVAL_S * 1000u) return;
    g_last_persist_ms = now;

    static uint8_t buf[PROX_NVS_HDR + ANCHOR_PROX_MAX_FINGERPRINT_DEVICES * PROX_NVS_REC];
    int n = 0; size_t off = PROX_NVS_HDR;
    for (int i = 0; i < g_reg_count; ++i) {
        AnchorDev* d = &g_reg[i];
        if (!d->in_use) continue;
        memcpy(buf + off, d->mac, 6); off += 6;
        buf[off++] = d->type;
        wr_f32(buf + off, d->mu); off += 4;
        wr_f32(buf + off, d->M);  off += 4;
        wr_f32(buf + off, d->W);  off += 4;
        n++;
    }
    buf[0] = PROX_NVS_VERSION;
    buf[1] = g_near_threshold;
    wr_u16(buf + 2, (uint16_t)n);
    prox_platform_nvs_save(PROX_NVS_KEY, buf, off);
}

static void prox_load_from_nvs(void) {
    static uint8_t buf[PROX_NVS_HDR + ANCHOR_PROX_MAX_FINGERPRINT_DEVICES * PROX_NVS_REC];
    size_t len = 0;
    if (!prox_platform_nvs_load(PROX_NVS_KEY, buf, sizeof(buf), &len) || len < 2) return;
    int n; size_t off;
    if (buf[0] == PROX_NVS_VERSION && len >= PROX_NVS_HDR) {
        g_near_threshold = buf[1];
        n = rd_u16(buf + 2); off = PROX_NVS_HDR;
    } else {
        g_near_threshold = 0;              // legacy blob: uncalibrated
        n = rd_u16(buf); off = 2;
    }
    g_reg_count = 0;
    for (int i = 0; i < n && off + PROX_NVS_REC <= len; ++i) {
        AnchorDev* d = &g_reg[g_reg_count++];
        memset(d, 0, sizeof(*d));
        memcpy(d->mac, buf + off, 6); off += 6;
        d->type = buf[off++];
        d->mu = rd_f32(buf + off); off += 4;
        d->M  = rd_f32(buf + off); off += 4;
        d->W  = rd_f32(buf + off); off += 4;
        d->in_use = 1;
        d->live_rssi = PROX_MISSING_RSSI_DBM;
        d->var = (d->W > 0.0f) ? d->M / d->W : PROX_MIN_VARIANCE;
        dev_recache(d);
    }
}

// App-provided fingerprint blob (spec §6.3.2):
// [2 count][per dev: 6 mac, 1 type, 4 mu, 4 sigma_sq, 4 W]
int prox_load_fingerprint(const uint8_t* blob, size_t len) {
    if (len < 2) return 0;
    int n = rd_u16(blob); size_t off = 2;
    const size_t REC = 19;
    if ((size_t)n * REC + 2 > len) return 0; // truncated
    g_reg_count = 0;
    for (int i = 0; i < n && g_reg_count < ANCHOR_PROX_MAX_FINGERPRINT_DEVICES; ++i) {
        AnchorDev* d = &g_reg[g_reg_count++];
        memset(d, 0, sizeof(*d));
        memcpy(d->mac, blob + off, 6); off += 6;
        d->type = blob[off++];
        d->mu = rd_f32(blob + off); off += 4;
        float sigma_sq = rd_f32(blob + off); off += 4;
        d->W = rd_f32(blob + off); off += 4;
        d->var = sigma_sq;
        d->M = sigma_sq * d->W;     // reconstruct M so Welford can continue
        d->in_use = 1;
        d->live_rssi = PROX_MISSING_RSSI_DBM;
        dev_recache(d);
    }
    g_last_persist_ms = 0; // force a persist on next due check
    return 1;
}

#endif // PROXIMITY_ROLE_ANCHOR

// ============================================================================
// MODE A — WATCH SIDE (scan vector assembly + serialization)
// ============================================================================
#ifdef PROXIMITY_ROLE_WATCH

#define PROX_SCAN_BUF 128
typedef struct { uint8_t mac[6]; uint8_t type; int8_t rssi; uint8_t used; } ScanEntry;
static ScanEntry g_scan[PROX_SCAN_BUF];
static int       g_scan_count = 0;

void prox_ingest_scan_result(const uint8_t mac[6], uint8_t type, int8_t rssi) {
    for (int i = 0; i < g_scan_count; ++i)
        if (g_scan[i].used && g_scan[i].type == type && mac_eq(g_scan[i].mac, mac)) {
            if (rssi > g_scan[i].rssi) g_scan[i].rssi = rssi; // keep strongest
            return;
        }
    if (g_scan_count < PROX_SCAN_BUF) {
        ScanEntry* e = &g_scan[g_scan_count++];
        memcpy(e->mac, mac, 6); e->type = type; e->rssi = rssi; e->used = 1;
    }
}

// Select the strongest PROX_MAX_DEVICES into the output vector, then reset buffer.
void prox_build_scan_vector(ProxScanVector* out) {
    // simple selection sort of top-K by RSSI (K small, buffer small → fine)
    int n = g_scan_count;
    int k = n < PROX_MAX_DEVICES ? n : PROX_MAX_DEVICES;
    for (int i = 0; i < k; ++i) {
        int best = i;
        for (int j = i + 1; j < n; ++j)
            if (g_scan[j].rssi > g_scan[best].rssi) best = j;
        ScanEntry tmp = g_scan[i]; g_scan[i] = g_scan[best]; g_scan[best] = tmp;
    }
    out->count = (uint8_t)k;
    for (int i = 0; i < k; ++i) {
        memcpy(out->devices[i].mac, g_scan[i].mac, 6);
        out->devices[i].type = g_scan[i].type;
        out->devices[i].rssi = g_scan[i].rssi;
    }
    g_scan_count = 0;
}

// Wire format (spec §6.3.1): [1 count][per dev: 6 mac, 1 type, 1 (rssi+128)]
size_t prox_serialize_vector(const ProxScanVector* v, uint8_t* out, size_t cap) {
    size_t need = 1 + (size_t)v->count * 8;
    if (cap < need) return 0;
    out[0] = v->count;
    size_t off = 1;
    for (int i = 0; i < v->count; ++i) {
        memcpy(out + off, v->devices[i].mac, 6); off += 6;
        out[off++] = v->devices[i].type;
        out[off++] = (uint8_t)((int)v->devices[i].rssi + 128);
    }
    return off;
}

#endif // PROXIMITY_ROLE_WATCH

// ============================================================================
// v2.1 PHASE 1 — MOTION CHANNEL, MOTION-GATED INTEGRATOR, HMM (watch side)
//
// Engine spec v2.1 §4 and §6; FIRMWARE_SPEC v0.9 amendment Parts 9, 10, 14.
//
// Why this exists: indoor RSSI is flat not because distance is unmeasurable but
// because a *stationary* receiver sees exactly one small-scale fading draw per
// frequency, forever. Averaging longer does not add information — it only adds
// confidence in whatever fade the wrist happens to be parked in (spec §1.2).
// So the engine (a) counts independent draws honestly instead of samples, and
// (b) refuses to change its mind while the IMU says the wrist could not have
// moved. Everything below is integer math: the C3 has no FPU and this runs on
// every enforcement poll (§7).
// ============================================================================
#ifdef PROXIMITY_ROLE_WATCH

// ---- Q8 fixed-point transcendental substitutes (all LUT-driven) ------------

// log2(x) in Q8 for x >= 1. Exponent from a leading-zero count, mantissa from
// the 64-entry table — no libm, no float.
static int32_t q8_log2_u32(uint32_t x) {
    if (x == 0) return 0;
    int e = 31 - __builtin_clz(x);
    uint32_t frac = (e >= 6) ? ((x >> (e - 6)) & 63u) : ((x << (6 - e)) & 63u);
    return ((int32_t)e << 8) + (int32_t)PROX_LUT_LOG2_MANT[frac];
}

// ln(x) in Q8 for x >= 1.  ln x = log2(x) * ln2, 181 = round(0.693147 * 256).
static int32_t q8_ln_u32(uint32_t x) {
    return (int32_t)(((int64_t)q8_log2_u32(x) * 181) >> 8);
}

// log(e^a + e^b) in Q8: max + log(1 + e^-|a-b|) from the tail table.
static int32_t q8_lse(int32_t a, int32_t b) {
    int32_t hi = a > b ? a : b;
    int32_t lo = a > b ? b : a;
    int32_t k  = (hi - lo) >> 5;              // 1/8-nat steps
    if (k >= 64) return hi;                   // tail below Q8 resolution
    return hi + (int32_t)PROX_LUT_LSE_CORR[k];
}

static inline int32_t clamp_i32(int32_t x, int32_t lo, int32_t hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

// ---- Motion channel (§4.1) -------------------------------------------------
//
// Three inputs, none of which costs a new interrupt source: the ENFORCEMENT-only
// IA1 wake verdict over each light-sleep interval, IA1 firings while awake, and
// one short accelerometer burst sampled *while the radio scans* (the CPU is
// idle there anyway). DORMANT's v0.7 zero-IMU-interrupt guarantee is untouched —
// nothing here runs outside ENFORCEMENT and calibration bursts.

static uint8_t  g_motion_state      = PROX_MOTION_UNKNOWN;
static uint32_t g_motion_evidence_ms = 0;   // clock of the newest motion evidence
static uint32_t g_motion_burst_var   = 0;   // last burst variance, mg^2
static uint8_t  g_motion_cadence     = 0;   // last burst showed step cadence
static uint8_t  g_motion_ints        = 0;   // IA1 firings since the last burst
static uint8_t  g_motion_have_sleep  = 0;   // a sleep interval has been reported
static uint8_t  g_motion_sleep_still = 0;   // ...and it was motionless

static void motion_reset(void) {
    g_motion_state       = PROX_MOTION_UNKNOWN;
    g_motion_evidence_ms = 0;
    g_motion_burst_var   = 0;
    g_motion_cadence     = 0;
    g_motion_ints        = 0;
    g_motion_have_sleep  = 0;
    g_motion_sleep_still = 0;
}

// Classify from whatever evidence is current (§4.1 table). Order matters: the
// LOCOMOTION tests come first so an ambiguous burst can never mask real motion.
static void motion_classify(void) {
    // Cadence is deliberately NOT consulted here — see IMU_CADENCE_* in
    // proximity.h. It proved unreliable at typing amplitudes on real hardware.
    if (g_motion_burst_var > (uint32_t)IMU_LOCO_VAR ||
        g_motion_ints >= IMU_LOCO_MIN_INTS) {
        g_motion_state = PROX_MOTION_LOCOMOTION;
        return;
    }
    // STILL needs *positive* stillness evidence: a quiet burst, no awake IA1
    // firing, and no sleep interval that was broken by motion.
    if (g_motion_burst_var < (uint32_t)IMU_STILL_VAR && g_motion_ints == 0 &&
        (!g_motion_have_sleep || g_motion_sleep_still)) {
        g_motion_state = PROX_MOTION_STILL;
        return;
    }
    g_motion_state = PROX_MOTION_FIDGET;
}

void prox_ingest_imu_burst(const int16_t (*xyz)[3], uint16_t n, uint16_t hz) {
    (void)hz;   // cadence is expressed in samples; hz only scales the reported band
    if (!xyz || n < 4) return;

    // Gravity removal is the burst's own per-axis mean: over ~0.6 s the DC term
    // is the gravity vector in whatever orientation the wrist is holding.
    int32_t sum[3] = {0, 0, 0};
    for (uint16_t i = 0; i < n; ++i)
        for (int a = 0; a < 3; ++a) sum[a] += xyz[i][a];
    int32_t mean[3] = { sum[0] / (int32_t)n, sum[1] / (int32_t)n, sum[2] / (int32_t)n };

    // Variance summed over the three axes (mg^2), plus a scalar motion magnitude
    // per sample for the cadence test (alpha-max-beta-min, no sqrt).
    uint64_t ss = 0;
    int32_t  mag[IMU_BURST_SAMPLES];
    uint16_t nm = n > IMU_BURST_SAMPLES ? IMU_BURST_SAMPLES : n;
    int32_t  magsum = 0;
    for (uint16_t i = 0; i < n; ++i) {
        int32_t d0 = xyz[i][0] - mean[0];
        int32_t d1 = xyz[i][1] - mean[1];
        int32_t d2 = xyz[i][2] - mean[2];
        ss += (uint64_t)(d0 * d0 + d1 * d1 + d2 * d2);
        if (i < nm) {
            int32_t a0 = d0 < 0 ? -d0 : d0, a1 = d1 < 0 ? -d1 : d1, a2 = d2 < 0 ? -d2 : d2;
            int32_t mx = a0 > a1 ? a0 : a1; if (a2 > mx) mx = a2;
            int32_t mn = a0 < a1 ? a0 : a1; if (a2 < mn) mn = a2;
            mag[i] = mx + (mn >> 1);          // ~ magnitude, cheap
            magsum += mag[i];
        }
    }
    uint32_t var = (uint32_t)(ss / n);

    // Step cadence: sign changes of the mean-removed magnitude envelope. At
    // IMU_BURST_SAMPLES / IMU_BURST_HZ seconds of signal a 0.5-3 Hz gait shows a
    // small, bounded number of crossings; noise-driven crossings are excluded by
    // the variance floor.
    int32_t magmean = magsum / (int32_t)nm;
    int crossings = 0, prev = 0;
    for (uint16_t i = 0; i < nm; ++i) {
        int s = (mag[i] > magmean) ? 1 : -1;
        if (prev != 0 && s != prev) crossings++;
        prev = s;
    }
    g_motion_cadence = (var >= (uint32_t)IMU_CADENCE_MIN_VAR &&
                        crossings >= IMU_CADENCE_MIN_CROSSINGS &&
                        crossings <= IMU_CADENCE_MAX_CROSSINGS) ? 1u : 0u;

    g_motion_burst_var   = var;
    g_motion_evidence_ms = prox_platform_now_ms();
    motion_classify();
    g_motion_ints       = 0;   // consumed by this classification
    g_motion_have_sleep = 0;   // the burst supersedes the sleep verdict
}

void prox_note_motion_interrupt(void) {
    if (g_motion_ints < 255) g_motion_ints++;
    g_motion_evidence_ms = prox_platform_now_ms();
    // Promote immediately — the wrist demonstrably moved, and waiting for the
    // next burst to say so would leave the HMM locked exactly when it must open.
    if (g_motion_ints >= IMU_LOCO_MIN_INTS)      g_motion_state = PROX_MOTION_LOCOMOTION;
    else if (g_motion_state == PROX_MOTION_STILL) g_motion_state = PROX_MOTION_FIDGET;
}

void prox_note_sleep_interval(uint32_t slept_ms, int motion_woke) {
    (void)slept_ms;
    g_motion_have_sleep  = 1;
    g_motion_sleep_still = motion_woke ? 0u : 1u;
    g_motion_evidence_ms = prox_platform_now_ms();
    if (motion_woke) {
        if (g_motion_ints < 255) g_motion_ints++;
        if (g_motion_state == PROX_MOTION_STILL) g_motion_state = PROX_MOTION_FIDGET;
    } else {
        // A whole sleep interval with no IA1 is the strongest and cheapest STILL
        // evidence there is; it carries the previous STILL verdict across the
        // sleep instead of letting it go stale (which would read as LOCOMOTION).
        if (g_motion_state == PROX_MOTION_STILL) g_motion_burst_var = 0;
        g_motion_ints = 0;
    }
}

uint32_t prox_motion_burst_var(void)     { return g_motion_burst_var; }
uint8_t  prox_motion_burst_cadence(void) { return g_motion_cadence; }

uint8_t prox_motion_state(void) {
    if (g_motion_evidence_ms == 0) return PROX_MOTION_UNKNOWN;
    if (prox_platform_now_ms() - g_motion_evidence_ms > (uint32_t)IMU_STALE_MS)
        return PROX_MOTION_UNKNOWN;      // stale seam ⇒ fail toward v0.8 behavior
    return g_motion_state;
}

// Independent draws credited over dt_ms at the current motion state, in Q12.
// Q12 rather than the public Q4 because the STILL rate is deliberately far below
// one draw per tick and the remainder has to survive across ticks.
static uint32_t motion_credit_q12(uint32_t dt_ms) {
    uint64_t c;
    switch (prox_motion_state()) {
        case PROX_MOTION_STILL:
            c = ((uint64_t)4096u * dt_ms) / ((uint64_t)HMM_STILL_DRAW_PERIOD_S * 1000u);
            break;
        case PROX_MOTION_FIDGET:
            c = ((uint64_t)NEFF_FIDGET_PER_S * 4096u * dt_ms) / 1000u;
            break;
        default:                                              // LOCOMOTION / UNKNOWN
            c = ((uint64_t)NEFF_LOCO_PER_S * 4096u * dt_ms) / 1000u;
            break;
    }
    uint64_t cap = (uint64_t)NEFF_MAX_Q4 << 8;
    return (uint32_t)(c > cap ? cap : c);
}

uint16_t prox_motion_neff_credit_q4(uint32_t dt_ms) {
    return (uint16_t)(motion_credit_q12(dt_ms) >> 8);
}

// ---- Motion-gated integrator (§4.2) ---------------------------------------

void prox_integ_reset(ProxIntegrator* it) {
    if (!it) return;
    memset(it, 0, sizeof(*it));
    it->var_q8      = (int32_t)INTEG_SINGLE_DRAW_VAR << 8;
    it->last_motion = PROX_MOTION_UNKNOWN;
}

static void integ_prime(ProxIntegrator* it, int32_t x_q8, uint32_t now, uint8_t ms) {
    it->mean_q8     = x_q8;
    it->var_q8      = (int32_t)INTEG_SINGLE_DRAW_VAR << 8;
    it->neff_q4     = NEFF_FLOOR_NOSCHED_Q4;
    it->primed      = 1;
    it->last_motion = ms;
    it->last_ms     = now;
}

void prox_integ_update(ProxIntegrator* it, int32_t x_q8) {
    if (!it) return;
    uint32_t now = prox_platform_now_ms();
    uint8_t  ms  = prox_motion_state();
    if (ms == PROX_MOTION_UNKNOWN) ms = PROX_MOTION_LOCOMOTION;  // fail toward v1

    if (!it->primed) { integ_prime(it, x_q8, now, ms); return; }

    // STILL -> LOCOMOTION: window restart. Position may now be changing, and
    // evidence gathered while parked must not outvote what is arriving fresh.
    if (it->last_motion == PROX_MOTION_STILL && ms == PROX_MOTION_LOCOMOTION) {
        integ_prime(it, x_q8, now, ms);
        return;
    }

    uint32_t dt = now - it->last_ms;
    int32_t  w  = (ms == PROX_MOTION_STILL) ? INTEG_STILL_WEIGHT : INTEG_MOVE_WEIGHT;
    int32_t  d  = x_q8 - it->mean_q8;

    it->mean_q8 += (d * w) >> 8;

    if (ms != PROX_MOTION_STILL) {
        // Residuals between genuinely independent draws are what variance means.
        int32_t inst = (d * d) >> 8;                // residual^2, Q8 of dB^2
        it->var_q8  += ((inst - it->var_q8) * w) >> 8;
    }

    if (ms == PROX_MOTION_STILL) {
        // N_eff frozen: more samples of one fading draw are not more information.
        if (it->neff_q4 < NEFF_FLOOR_NOSCHED_Q4) it->neff_q4 = NEFF_FLOOR_NOSCHED_Q4;
        // ...and the reported variance relaxes back toward a single draw. A still
        // receiver's near-zero residuals are not evidence of a tight estimate —
        // they are evidence that nothing has been re-sampled — so the variance is
        // never allowed to shrink on them, only to relax back outward.
        int32_t target = (int32_t)INTEG_SINGLE_DRAW_VAR << 8;
        if (it->var_q8 < target) {
            int32_t f = (int32_t)(((uint64_t)dt * 256u) / ((uint32_t)INTEG_STILL_RELAX_S * 1000u));
            if (f > 256) f = 256;
            it->var_q8 += ((target - it->var_q8) * f) >> 8;
        }
    } else {
        uint32_t n = (uint32_t)it->neff_q4 + prox_motion_neff_credit_q4(dt);
        it->neff_q4 = (uint16_t)(n > NEFF_MAX_Q4 ? NEFF_MAX_Q4 : n);
    }

    it->last_motion = ms;
    it->last_ms     = now;
}

int prox_integ_mean_q8(const ProxIntegrator* it, int32_t* out) {
    if (!it || !it->primed) return 0;
    if (out) *out = it->mean_q8;
    return 1;
}

int prox_integ_var_q8(const ProxIntegrator* it, int32_t* out) {
    if (!it || !it->primed) return 0;
    if (out) *out = it->var_q8;
    return 1;
}

uint8_t prox_integ_neff(const ProxIntegrator* it) {
    if (!it || !it->primed) return 0;
    uint32_t n = it->neff_q4 >> 4;
    return (uint8_t)(n > 255 ? 255 : n);
}

// ---- Two-state motion-conditioned HMM (§6) --------------------------------
//
// The whole filter is one scalar: Lambda = log( P(NEAR) / P(AWAY) ) in Q8.
// With two states the forward algorithm collapses to
//     Lambda' = lse(Lambda, -c) - lse(Lambda - c, 0),   c = ln((1-p)/p)
// which is exact, needs two table lookups, and saturates at |Lambda| = c — i.e.
// the transition model itself imposes the ceiling on how certain a filter with
// flip probability p is allowed to become.

static int32_t  g_hmm_lam_q8   = 0;
static int32_t  g_hmm_local_q8 = 0;   // pending watch-local evidence for next tick
static uint32_t g_hmm_last_ms  = 0;   // clock at the previous transition step
static uint32_t g_hmm_emit_ms  = 0;   // clock at the previous *emission*
static uint8_t  g_hmm_primed   = 0;
static uint8_t  g_hmm_first    = 1;   // no emission has been folded in yet
static uint32_t g_hmm_draw_resid_q12 = 0;  // fractional draws carried between ticks
static int32_t  g_hmm_still_credited_q8 = 0;  // evidence already granted in this still window

// One transition step for a flip probability expressed in ppm.
static int32_t hmm_transition(int32_t lam_q8, uint32_t pflip_ppm) {
    if (pflip_ppm == 0) return lam_q8;
    if (pflip_ppm >= HMM_PFLIP_MAX_PPM) return 0;      // fully mixed: no memory
    int32_t c = q8_ln_u32(1000000u - pflip_ppm) - q8_ln_u32(pflip_ppm);
    return q8_lse(lam_q8, -c) - q8_lse(lam_q8 - c, 0);
}

// Flip probability for this motion state, linearly scaled to the elapsed time
// (ticks are irregular: 60 s, 180 s, 600 s, or an instant motion re-check).
static uint32_t hmm_pflip_ppm(uint8_t ms, uint32_t dt_ms) {
    uint32_t base;
    switch (ms) {
        case PROX_MOTION_STILL:  base = HMM_PFLIP_STILL_PPM;  break;
        case PROX_MOTION_FIDGET: base = HMM_PFLIP_FIDGET_PPM; break;
        default:                 base = HMM_PFLIP_MOVE_PPM;   break;  // LOCO / UNKNOWN
    }
    uint64_t p = ((uint64_t)base * dt_ms) / ((uint64_t)HMM_TICK_REF_S * 1000u);
    return (uint32_t)(p > HMM_PFLIP_MAX_PPM ? HMM_PFLIP_MAX_PPM : p);
}

static void hmm_advance(uint32_t now) {
    if (!g_hmm_primed) { g_hmm_last_ms = now; g_hmm_primed = 1; return; }
    uint32_t dt = now - g_hmm_last_ms;
    if (dt == 0) return;
    g_hmm_last_ms = now;
    g_hmm_lam_q8  = hmm_transition(g_hmm_lam_q8, hmm_pflip_ppm(prox_motion_state(), dt));
}

static ProxDecision hmm_decide(void) {
    if (g_hmm_lam_q8 >= HMM_TAU_NEAR_Q8) return PROX_HMM_NEAR;
    if (g_hmm_lam_q8 <= HMM_TAU_AWAY_Q8) return PROX_HMM_AWAY;
    return PROX_HMM_AMBIGUOUS;
}

void prox_hmm_reset(uint8_t criterion) {
    // Cold start biased to the criterion-satisfying state at P = 0.65: the
    // engine holds no cross-window state, and an uninformed window must not
    // begin by enforcing (§2, §6.3).
    g_hmm_lam_q8   = (criterion == PROX_CRIT_STAY_NEAR) ? HMM_PRIOR_Q8 : -HMM_PRIOR_Q8;
    g_hmm_local_q8 = 0;
    g_hmm_last_ms  = prox_platform_now_ms();
    g_hmm_emit_ms  = g_hmm_last_ms;
    g_hmm_primed   = 1;
    g_hmm_first    = 1;
    g_hmm_draw_resid_q12 = 0;
    g_hmm_still_credited_q8 = 0;
    motion_reset();                      // motion state UNKNOWN until the first burst
}

// Weight for this tick's emission, Q8 — the crux of the whole v2 design.
//
// Evidence is counted in *independent fading draws*, never in samples. Since the
// last emission the wrist has earned motion_credit_q12() draws (fractions carry
// forward in g_hmm_draw_resid_q12); the gain is sqrt(min(1, draws / NEFF_TRAIN_MIN))
// from the LUT. Consequences:
//
//   - A tick that earned no draw contributes NO evidence, so re-measuring a
//     frozen fade a thousand times is worth exactly as much as measuring it
//     once. That is what makes an RSSI excursion while the wrist is provably
//     still structurally incapable of flipping the decision (§6.2). A transition
//     prior alone could never achieve this: a repeated emission always
//     overwhelms a prior eventually.
//   - Not "never", though: HMM_STILL_DRAW_PERIOD_S concedes one draw per 12 h of
//     unbroken stillness, so sustained contradiction still wins in the end.
//   - Any tick taken while the wrist is moving is worth at least one draw, so
//     rapid motion-triggered re-checks stay fully responsive.
//
// Returns 0 when this tick brought no fresh draw — the caller then treats the
// still window's evidence as a level rather than an increment.
static uint32_t hmm_earn_draws(uint32_t dt_ms) {
    g_hmm_draw_resid_q12 += motion_credit_q12(dt_ms);

    uint32_t draws = g_hmm_draw_resid_q12 >> 12;
    if (draws) {
        g_hmm_draw_resid_q12 -= draws << 12;        // spend what was earned
    } else if (g_hmm_first || prox_motion_state() != PROX_MOTION_STILL) {
        draws = 1;                                  // a look taken in motion is a look
        g_hmm_draw_resid_q12 = 0;
    }
    return draws;
}

void prox_note_connect_failure(int8_t last_advert_rssi_dbm, int have_advert) {
    // Advertisement receivable but the link will not close ⇒ far. Without a
    // recent advertisement there is no evidence either way, so abstain rather
    // than guess (the connect could have failed for a dozen local reasons).
    if (have_advert && last_advert_rssi_dbm <= PROX_FAR_RSSI_THRESHOLD_DBM)
        g_hmm_local_q8 += LL_CONNFAIL_AWAY_Q8;
}

// Fold one emission (already in Q8 log-LR, un-weighted) plus any pending
// watch-local evidence into the posterior, and return the decision.
static ProxDecision hmm_fold(int32_t raw_loglr_q8, uint8_t cap_neff) {
    uint32_t now = prox_platform_now_ms();
    hmm_advance(now);

    // Watch-local evidence (§6.1) is summed with the anchor's verdict BEFORE the
    // draw gate, not after. The spec places it outside the N_eff discount, but
    // hardware showed why that cannot stand: a still watch that repeatedly fails
    // to connect was adding LL_CONNFAIL_AWAY every single poll, marching the
    // posterior to saturation on what is one observation repeated. That is the
    // frozen-fade failure exactly, arriving through a different channel. A
    // re-failed connect with nothing moved is no more independent than a
    // re-measured fade, so it earns evidence on the same terms.
    int32_t total = raw_loglr_q8 + g_hmm_local_q8;
    g_hmm_local_q8 = 0;

    // A zero total is a no-information observation (a failed query with no
    // advertisement to judge it by, or a score sitting exactly on the anchor's
    // cutoff). Don't spend accrued draws on it — the credit belongs to the next
    // tick that actually says something.
    if (total != 0) {
        uint32_t draws = hmm_earn_draws(now - g_hmm_emit_ms);
        g_hmm_emit_ms = now;
        g_hmm_first   = 0;

        if (draws) {
            // Fresh independent draws: full accumulation, and this opens a new
            // still-window budget anchored on what these draws just said.
            if (cap_neff && draws > cap_neff) draws = cap_neff;
            if (draws > 63) draws = 63;
            int32_t add = (total * (int32_t)PROX_LUT_NEFF_GAIN[draws]) >> 8;
            g_hmm_lam_q8 += add;
            // Open a FRESH still-window budget. Carrying this tick's credit
            // forward looked like it would stop double-counting the same
            // observation, but it does something much worse: if the wrist then
            // stops and the next reading disagrees, the delta spans from the old
            // credit to the new target and lands in one step -- a swing several
            // times larger than the one-draw ceiling the still window exists to
            // impose. Starting at zero costs at most one extra draw of
            // double-counting and keeps the ceiling honest.
            g_hmm_still_credited_q8 = 0;
        } else {
            // No fresh draw — the wrist has not moved. The window's evidence is a
            // LEVEL worth at most one observation, not a per-tick increment, so
            // credit only the change since this window's last reading.
            //
            // Both halves matter, and hardware demonstrated both: re-reading the
            // same thing must add nothing (else a still watch that keeps failing
            // to connect marches to saturation), but a *materially different*
            // reading must still land (else whichever observation happens to
            // arrive first in a window silently locks out every later one — a
            // weak connect-failure hint suppressing the anchor's actual score).
            int32_t target = (total * (int32_t)PROX_LUT_NEFF_GAIN[1]) >> 8;
            int32_t delta  = target - g_hmm_still_credited_q8;
            // Only ever move further in the direction the observation points;
            // never claw back evidence that stronger draws already established.
            if ((total > 0 && delta > 0) || (total < 0 && delta < 0)) {
                g_hmm_lam_q8 += delta;
                g_hmm_still_credited_q8 = target;
            }
        }
    }

    g_hmm_lam_q8 = clamp_i32(g_hmm_lam_q8, -HMM_LAMBDA_MAX_Q8, HMM_LAMBDA_MAX_Q8);
    return hmm_decide();
}

ProxDecision prox_hmm_tick(const ProxScoreResult2* r) {
    // A tick with no query still folds in whatever watch-local evidence is
    // pending (a failed connect is itself an observation) and still advances the
    // transition model.
    if (!r) return hmm_fold(0, 0);
    // Bounded linear discriminant about the anchor's cutoff (see HMM_EMIT_* in
    // proximity.h for why this is not logit(score/256)).
    //
    // Three properties, each of which hardware showed was needed:
    //  - Centred on the anchor's OWN cutoff, so calibration-v2's demonstrated
    //    near zone is honoured rather than silently replaced by score 128.
    //  - A dead zone around that cutoff, so noise at the decision point cannot
    //    accumulate into certainty (a watch 2 cm from its anchor scoring
    //    153-167 against a 170 cutoff must read AMBIGUOUS, as v0.8 does).
    //  - Symmetric and proportionate outside it, so walking away generates real
    //    evidence. The previous logit-difference form gave -0.1 nats for a score
    //    of 79 while giving +1.2 for a 237, and the filter could not come back.
    uint8_t thr = r->near_thr ? r->near_thr : (uint8_t)PROX_CONFIDENCE_THRESHOLD_U8;
    int32_t d   = (int32_t)r->score - (int32_t)thr;

    int32_t raw;
    if (d > HMM_EMIT_DEADZONE_U8)       raw = (d - HMM_EMIT_DEADZONE_U8) * HMM_EMIT_SLOPE_Q8;
    else if (d < -HMM_EMIT_DEADZONE_U8) raw = (d + HMM_EMIT_DEADZONE_U8) * HMM_EMIT_SLOPE_Q8;
    else                                raw = 0;       // inside the dead zone: abstain
    raw = clamp_i32(raw, -HMM_EMIT_MAX_Q8, HMM_EMIT_MAX_Q8);
    return hmm_fold(raw, r->neff);
}

// Modes B/C entry point: the coloc factors already produce a summed log-LR, so
// they enter the same filter directly (§6.4 — the old two-threshold hysteresis
// machine is subsumed by the posterior). They ride the same draw-gating, because
// a still watch's coloc factors are exactly as frozen as its Mode A score.
ProxDecision prox_hmm_tick_loglr_q8(int32_t loglr_q8) {
    return hmm_fold(loglr_q8, 0);
}

// Pure read: no transition step, no evidence. Callers that want the elapsed-time
// decay (a poll that produced no query) tick with a NULL result instead. Keeping
// this side-effect-free matters because the poll-tier logic consults it from the
// main loop many times a second.
ProxDecision prox_hmm_decision(void) { return hmm_decide(); }

int32_t prox_hmm_logodds_q8(void) { return g_hmm_lam_q8; }

uint8_t prox_hmm_p_near_u8(void) {
    // Diagnostics only (shadow logging). p = 1 / (1 + e^-|L|) = e^-corr(|L|),
    // with corr from the logsumexp table and a 4-term series for the exponential
    // over its tiny domain [0, ln 2].
    int32_t lam = g_hmm_lam_q8;
    int32_t a   = lam < 0 ? -lam : lam;
    int32_t k   = a >> 5;
    int32_t u   = (k >= 64) ? 0 : (int32_t)PROX_LUT_LSE_CORR[k];    // Q8, 0..177
    int32_t u2  = (u * u) >> 8;
    int32_t u3  = (u2 * u) >> 8;
    int32_t u4  = (u3 * u) >> 8;
    int32_t e   = 256 - u + (u2 / 2) - (u3 / 6) + (u4 / 24);        // e^-u, Q8
    e = clamp_i32(e, 0, 256);
    int32_t p = (lam >= 0) ? e : (256 - e);
    return (uint8_t)clamp_i32(p, 0, 255);
}

#endif // PROXIMITY_ROLE_WATCH

// ============================================================================
// MODES B / C — CO-LOCATION DETECTOR (watch side)
// ============================================================================
#ifdef PROXIMITY_ROLE_WATCH

// ---- config + state ----
// v2.1 §6.4: the flat EWMA rings are replaced by the motion-gated integrator —
// the coloc factors were exactly as vulnerable to a frozen fade as Mode A was.
// Var(R_wp) (the "phone held" signature) now comes from the integrator's own
// variance estimate, which relaxes toward a single draw while the wrist is
// still, so "low variance" can no longer mean "nothing has moved in an hour".
static ColocConfig    g_cfg;
static ProxIntegrator g_rwp;     // watch<->phone RSSI   (Factor 1 + Factor V)
static ProxIntegrator g_delta;   // R_P - R_D            (Factor delta, Mode C)
static ProxIntegrator g_s;       // per-reading std(delta_d) (Factor 2)
static ColocState     g_state = COLOC_AWAY;
static ColocDecision  g_last = { 0.0f, COLOC_AWAY, 0, 0 };

// per-reading scratch
static int   g_have_rwp = 0;   static int8_t g_cur_rwp = 0;
static int   g_have_diff = 0;  static int8_t g_cur_rp = 0, g_cur_rd = 0;
static double g_dsum = 0, g_dsumsq = 0; static uint32_t g_dcount = 0;

static const char* COLOC_NVS_KEY = "coloc_cfg";

static void feature_cache(ColocFeature* f) {
    if (f->near_sigma < 1.0f) f->near_sigma = 1.0f;
    if (f->away_sigma < 1.0f) f->away_sigma = 1.0f;
    f->near_inv2var = 1.0f / (2.0f * f->near_sigma * f->near_sigma);
    f->away_inv2var = 1.0f / (2.0f * f->away_sigma * f->away_sigma);
    f->near_lognorm = -logf(f->near_sigma) - LOG_SQRT_2PI;
    f->away_lognorm = -logf(f->away_sigma) - LOG_SQRT_2PI;
}

// log( N(x;near) / N(x;away) ) using cached constants. 0 if feature disabled.
static float feature_loglr(const ColocFeature* f, float x) {
    if (!f->enabled) return 0.0f;
    float dn = x - f->near_mu, da = x - f->away_mu;
    float ll_near = f->near_lognorm - dn * dn * f->near_inv2var;
    float ll_away = f->away_lognorm - da * da * f->away_inv2var;
    return ll_near - ll_away;
}

static void load_defaults(void) {
    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.prior_near = COLOC_PRIOR_NEAR;
    g_cfg.rwp   = (ColocFeature){ COLOC_DEF_RWP_NEAR_MU, COLOC_DEF_RWP_NEAR_SIGMA,
                                  COLOC_DEF_RWP_AWAY_MU, COLOC_DEF_RWP_AWAY_SIGMA, 0,0,0,0, 1 };
    g_cfg.s     = (ColocFeature){ COLOC_DEF_S_NEAR_MU, COLOC_DEF_S_NEAR_SIGMA,
                                  COLOC_DEF_S_AWAY_MU, COLOC_DEF_S_AWAY_SIGMA, 0,0,0,0, 1 };
    g_cfg.delta = (ColocFeature){ COLOC_DEF_DELTA_NEAR_MU, COLOC_DEF_DELTA_NEAR_SIGMA,
                                  COLOC_DEF_DELTA_AWAY_MU, COLOC_DEF_DELTA_AWAY_SIGMA, 0,0,0,0, 1 };
    g_cfg.var   = (ColocFeature){ COLOC_VAR_NEAR_MU, COLOC_VAR_NEAR_SIGMA,
                                  COLOC_VAR_AWAY_MU, COLOC_VAR_AWAY_SIGMA, 0,0,0,0, 1 };
}

void coloc_finalize_calibration(void) {
    feature_cache(&g_cfg.rwp);
    feature_cache(&g_cfg.s);
    feature_cache(&g_cfg.delta);
    feature_cache(&g_cfg.var);
}

ColocConfig* coloc_config(void) { return &g_cfg; }

void coloc_init(void) {
    prox_integ_reset(&g_rwp); prox_integ_reset(&g_delta); prox_integ_reset(&g_s);
    g_state = COLOC_AWAY;
    // Compliant for a phone-distance commitment is AWAY, so that is the
    // criterion-satisfying cold start (§6.3).
    prox_hmm_reset(PROX_CRIT_PHONE_AWAY);
    size_t len = 0;
    if (prox_platform_nvs_load(COLOC_NVS_KEY, &g_cfg, sizeof(g_cfg), &len) && len == sizeof(g_cfg)) {
        // loaded persisted calibration
    } else {
        load_defaults();
    }
    coloc_finalize_calibration();
}

// ---- per-reading ingestion ----
void coloc_ingest_link_rssi(int8_t r_wp, int have_anchor_diff, int8_t r_p, int8_t r_d) {
    g_have_rwp = 1; g_cur_rwp = r_wp;
    g_have_diff = have_anchor_diff ? 1 : 0;
    g_cur_rp = r_p; g_cur_rd = r_d;
}

void coloc_ingest_shared_device(int8_t watch_rssi, int8_t peer_rssi) {
    if (watch_rssi < COLOC_ENV_RSSI_FLOOR_DBM || peer_rssi < COLOC_ENV_RSSI_FLOOR_DBM) return;
    double d = (double)watch_rssi - (double)peer_rssi;
    g_dsum += d; g_dsumsq += d * d; g_dcount++;
}

void coloc_mark_reading_end(void) {
    if (g_have_rwp)  prox_integ_update(&g_rwp,   (int32_t)g_cur_rwp << 8);
    if (g_have_diff) prox_integ_update(&g_delta, ((int32_t)g_cur_rp - (int32_t)g_cur_rd) << 8);
    if (g_dcount >= (uint32_t)COLOC_ENV_MIN_SHARED_DEVICES) {
        double mean = g_dsum / g_dcount;
        double var  = g_dsumsq / g_dcount - mean * mean;
        if (var < 0) var = 0;
        prox_integ_update(&g_s, (int32_t)(sqrt(var) * 256.0));
    }
    g_have_rwp = 0; g_have_diff = 0;
    g_dsum = g_dsumsq = 0; g_dcount = 0;
}

// ---- fusion + HMM decision layer ----
ColocDecision coloc_tick(void) {
    ColocDecision out = { 0.0f, g_state, 0, 0 };

    // The prior is no longer added per tick: it lives in the HMM's cold start
    // (prox_hmm_reset), and re-adding it every tick would double-count it.
    float log_lr = 0.0f;
    int32_t m = 0;

    float rwp_mean = 0.0f;
    int have_rwp = prox_integ_mean_q8(&g_rwp, &m);
    if (have_rwp) {
        rwp_mean = (float)m / 256.0f;
        log_lr += feature_loglr(&g_cfg.rwp, rwp_mean); out.used_factors |= COLOC_F_RANGE;
    }
    if (prox_integ_mean_q8(&g_delta, &m)) {
        log_lr += feature_loglr(&g_cfg.delta, (float)m / 256.0f); out.used_factors |= COLOC_F_DIFF;
    }
    if (prox_integ_mean_q8(&g_s, &m)) {
        log_lr += feature_loglr(&g_cfg.s, (float)m / 256.0f); out.used_factors |= COLOC_F_ENV;
    }

    float rwp_var = 0.0f;
    int   have_var = prox_integ_var_q8(&g_rwp, &m);
    if (have_var) {
        rwp_var = (float)m / 256.0f;
        log_lr += feature_loglr(&g_cfg.var, rwp_var); out.used_factors |= COLOC_F_VAR;
    }

    // "holding" signature: high mean RSSI + low variance.
    if (have_rwp && have_var) {
        float rwp_mid = 0.5f * (g_cfg.rwp.near_mu + g_cfg.rwp.away_mu);
        float var_mid = 0.5f * (g_cfg.var.near_mu + g_cfg.var.away_mu);
        out.holding = (rwp_mean >= rwp_mid && rwp_var <= var_mid) ? 1u : 0u;
    }

    // The two-threshold hysteresis machine and its debounce counter are deleted:
    // the posterior subsumes both (§6.4). Flip resistance now comes from the
    // motion-conditioned transition model, which is stronger than a debounce
    // count because it is indexed on whether the wrist *could* have moved.
    // AMBIGUOUS holds the current state — and the state starts (and re-cold-starts)
    // at AWAY, so the fail-safe bias toward the compliant side is preserved.
    ProxDecision d = prox_hmm_tick_loglr_q8((int32_t)(log_lr * 256.0f));
    if      (d == PROX_HMM_NEAR) g_state = COLOC_NEAR;
    else if (d == PROX_HMM_AWAY) g_state = COLOC_AWAY;

    out.p_near = (float)prox_hmm_p_near_u8() / 255.0f;
    out.state  = g_state;

    g_last = out;
    return out;
}

ColocDecision coloc_decision(void) { return g_last; }

// ---- calibration ----
// accumulators[class][feature]; class 0 = NEAR, 1 = AWAY; features: 0 rwp,1 delta,2 s,3 var
static ColocAccum g_cal[2][4];

static void accum_reset(ColocAccum* a) { a->mean = 0; a->m2 = 0; a->n = 0; }
static void accum_add(ColocAccum* a, double x) {
    a->n++;
    double d = x - a->mean;
    a->mean += d / a->n;
    a->m2 += d * (x - a->mean);
}
static int accum_to_feature_side(const ColocAccum* a, float* mu, float* sigma) {
    if (a->n < 2) return 0;
    *mu = (float)a->mean;
    float var = (float)(a->m2 / (a->n - 1));
    *sigma = var > 0 ? sqrtf(var) : 1.0f;
    return 1;
}

void coloc_calib_begin(void) {
    for (int c = 0; c < 2; ++c) for (int f = 0; f < 4; ++f) accum_reset(&g_cal[c][f]);
}

void coloc_calib_add_sample(int klass, int8_t r_wp, int have_anchor_diff,
                            int8_t r_p, int8_t r_d, float s_env, float var_rwp) {
    if (klass < 0 || klass > 1) return;
    accum_add(&g_cal[klass][0], (double)r_wp);
    if (have_anchor_diff) accum_add(&g_cal[klass][1], (double)r_p - (double)r_d);
    if (s_env >= 0.0f)    accum_add(&g_cal[klass][2], (double)s_env);
    if (var_rwp >= 0.0f)  accum_add(&g_cal[klass][3], (double)var_rwp);
}

// Update one feature from the near/away accumulators, keeping defaults if a side
// lacked enough samples.
static void calib_apply(ColocFeature* f, int fidx) {
    float mu, sg;
    if (accum_to_feature_side(&g_cal[0][fidx], &mu, &sg)) { f->near_mu = mu; f->near_sigma = sg; }
    if (accum_to_feature_side(&g_cal[1][fidx], &mu, &sg)) { f->away_mu = mu; f->away_sigma = sg; }
    f->enabled = 1;
}

void coloc_calib_finalize(void) {
    calib_apply(&g_cfg.rwp,   0);
    calib_apply(&g_cfg.delta, 1);
    calib_apply(&g_cfg.s,     2);
    calib_apply(&g_cfg.var,   3);
    coloc_finalize_calibration();
    prox_platform_nvs_save(COLOC_NVS_KEY, &g_cfg, sizeof(g_cfg));
}

#endif // PROXIMITY_ROLE_WATCH

// ============================================================================
// Common init
// ============================================================================
void prox_init(void) {
#ifdef PROXIMITY_ROLE_ANCHOR
    g_reg_count = 0;
    g_last_persist_ms = 0;
    g_near_threshold = 0;
    prox_calib_reset();
    prox_load_from_nvs();
#endif
#ifdef PROXIMITY_ROLE_WATCH
    g_scan_count = 0;
    coloc_init();
#endif
}
