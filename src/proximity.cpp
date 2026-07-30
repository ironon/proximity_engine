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
// Separate key on purpose: the fingerprint blob has its own format and
// migrating it to carry two extra bytes would risk every existing anchor.
static const char* PROX_SELF_NVS_KEY = "prox_self";

static AnchorDev* reg_find(const uint8_t mac[6], uint8_t type) {
    for (int i = 0; i < g_reg_count; ++i)
        if (g_reg[i].in_use && g_reg[i].type == type && mac_eq(g_reg[i].mac, mac))
            return &g_reg[i];
    return NULL;
}

static int dev_is_fresh(const AnchorDev* d, uint32_t now);

// BLE Resolvable Private Address: the top two bits of the MSB are 0b01. Both
// roles store addresses big-endian (on-air order), so mac[0] is the MSB.
//
// This matters more than it looks. Essentially every modern phone, watch,
// earbud and fitness device re-randomises its BLE address every ~15 minutes for
// privacy. Such a device is perfectly usable for the LIVE correlation -- within
// one rotation period the watch and the anchor genuinely see the same address --
// but it is worthless as a FINGERPRINT key, because the address it was trained
// under is dead within the quarter hour. WiFi BSSIDs, by contrast, are static,
// and so are public-address BLE devices (TVs, smart-home gear, beacons).
static int mac_is_rotating(const uint8_t mac[6], uint8_t type) {
    if (type != PROX_TYPE_BLE) return 0;          // BSSIDs never rotate
    return (mac[0] & 0xC0) == 0x40;
}

// Pick the least valuable registry slot to recycle, or NULL if every slot is
// genuinely worth keeping.
//
// Without this the registry was a one-way street: it filled to
// ANCHOR_PROX_MAX_FINGERPRINT_DEVICES and reg_add() then refused every new
// device FOREVER. Combined with address rotation the anchor saturated with dead
// addresses within minutes and went permanently blind to its own room -- field
// logs showed it sharing 0-4 devices with a watch sitting beside it that was
// reporting 29. Nothing downstream can recover from that: Signal A had no
// sample, so the score was arithmetic rather than measurement.
static AnchorDev* reg_evict(uint32_t now) {
    int worst = -1;

    // Pass 1: something neither live nor trained. Rotating addresses first --
    // a stale RPA is guaranteed never to be seen again under that address.
    for (int i = 0; i < g_reg_count; ++i) {
        AnchorDev* d = &g_reg[i];
        if (!d->in_use || dev_is_fresh(d, now)) continue;
        if (d->W >= PROX_MIN_FINGERPRINT_WEIGHT) continue;
        if (worst < 0) { worst = i; continue; }
        AnchorDev* b = &g_reg[worst];
        int d_rot = mac_is_rotating(d->mac, d->type), b_rot = mac_is_rotating(b->mac, b->type);
        if (d_rot != b_rot) { if (d_rot) worst = i; continue; }
        if (d->last_seen_ms < b->last_seen_ms) worst = i;
    }
    if (worst >= 0) return &g_reg[worst];

    // Pass 2: everything left is trained. Give up the stalest, lightest one --
    // still preferring rotating addresses, whose training cannot be reused.
    for (int i = 0; i < g_reg_count; ++i) {
        AnchorDev* d = &g_reg[i];
        if (!d->in_use || dev_is_fresh(d, now)) continue;
        if (worst < 0) { worst = i; continue; }
        AnchorDev* b = &g_reg[worst];
        int d_rot = mac_is_rotating(d->mac, d->type), b_rot = mac_is_rotating(b->mac, b->type);
        if (d_rot != b_rot) { if (d_rot) worst = i; continue; }
        if (d->W < b->W) worst = i;
    }
    return worst >= 0 ? &g_reg[worst] : NULL;   // all fresh: legitimately full
}

static AnchorDev* reg_add(const uint8_t mac[6], uint8_t type) {
    AnchorDev* d;
    if (g_reg_count < ANCHOR_PROX_MAX_FINGERPRINT_DEVICES) {
        d = &g_reg[g_reg_count++];
    } else {
        d = reg_evict(prox_platform_now_ms());
        if (!d) return NULL;                    // every slot is live; genuinely full
    }
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

// Per-query diagnostics. These two numbers are what make a bad score legible:
// g_last_shared is the sample size Signal A actually had, and g_last_fp_seen is
// the sample size Signal B actually had. A score without them is unfalsifiable.
static int g_last_shared = 0;
static int g_last_fp_seen = 0;

// Signal A: Pearson correlation over devices shared by watch vector and fresh cache.
//
// The k >= PROX_MIN_SHARED_DEVICES guard is not a nicety, it is the difference
// between a measurement and a coin flip. Pearson's r over k points is DEGENERATE
// at small k: with k == 2 the two points always lie exactly on a line, so r is
// mathematically +/-1 regardless of what was measured, and after the clamp to
// [0,1] the score is exactly 0 or exactly 255. Field logs showed precisely that
// signature -- a still watch beside a still anchor producing 0, 0, 255, 0, 255,
// 0 -- which is not "noise", it is the estimator reporting the arithmetic of
// having too few points. Even at k = 5 the standard error is around 1/sqrt(k-3),
// i.e. most of the output range. Below the floor the correlation abstains and
// the caller falls back to the fingerprint or reports a neutral score.
static int signal_correlation(const ProxScanVector* v, float* out_rho, int* out_k) {
    uint32_t now = prox_platform_now_ms();
    float sw = 0, sa = 0; int k = 0;
    // pass 1: means
    for (int i = 0; i < v->count; ++i) {
        AnchorDev* d = reg_find(v->devices[i].mac, v->devices[i].type);
        if (!d || !dev_is_fresh(d, now)) continue;
        sw += v->devices[i].rssi; sa += d->live_rssi; k++;
    }
    if (out_k) *out_k = k;
    if (k < PROX_MIN_SHARED_DEVICES) { *out_rho = 0.0f; return 0; }
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
        // A fingerprinted device missing from this vector is CENSORED, not
        // observed at PROX_MISSING_RSSI_DBM. Imputing -100 was the single
        // largest noise term in the score: a device with mu=-70, sigma=6 that
        // merely failed to advertise inside the watch's scan window scored
        // dx^2/2sigma^2 = 12.5 nats of penalty while sitting motionless a metre
        // away. Since a device can go missing either by being too weak OR by
        // being asynchronous, "absent" carries no clean evidence in either
        // direction, so it abstains — it contributes to neither loglik nor nb.
        // Far-evidence is not lost: it still arrives through Signal A and
        // through the nb < PROX_MIN_DEVICE_COUNT guard below, which now counts
        // devices actually *present* rather than devices merely enrolled.
        if (!vec_lookup(v, d->mac, d->type, &x)) continue;
        float dx = (float)x - d->mu;
        // Mean standardised residual, NOT the mean log-density. The Gaussian
        // normalisation term (d->lognorm = -0.5*ln(var) - ln(sqrt(2pi))) is a
        // property of the FINGERPRINT'S WIDTH, not of how well this vector
        // matched it, and it dominated the average: a *perfect* match scored
        // L = 0.67 at sigma = 2 dB and only 0.47 at sigma = 10 dB, so L could
        // never approach 1 and the fingerprint could only ever drag the score
        // DOWN from Signal A's.
        //
        // Measured consequence, reproduced on host: one unchanged NEAR vector
        // scored 245 untrained, 211 after 16 training samples, 153 after 80 and
        // 138 after 160, decaying toward L*255 as alpha handed control to
        // Signal B. The score therefore had no stable scale — it drifted
        // downward over an anchor's lifetime, so a threshold calibrated one week
        // was wrong the next, and a calibration's own INSIDE mean (153) landed
        // 70 points below the live scores (223) it was collected from.
        //
        // The standardised residual is scale-free: 0 for a perfect match,
        // -0.5 at 1 sigma, -2 at 2 sigma, -4.5 at 3 sigma, whatever the width.
        loglik += -dx * dx * d->inv2var;
        nb++;
    }
    *out_wtot = wtot;
    g_last_fp_seen = nb;
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

// ---- self-RSSI discriminant -------------------------------------------------
// Learned near/away beacon levels for THIS anchor, from the two calibration legs.
// Zero/invalid until a calibration has run, in which case the term abstains and
// the score is bit-identical to what it was before this feature existed.
static int8_t g_self_near_dbm  = 0;
static int8_t g_self_away_dbm  = 0;
static uint8_t g_self_cal_valid = 0;
static int    g_last_self_delta = 0;

int prox_self_levels(int8_t* out_near, int8_t* out_away) {
    if (out_near) *out_near = g_self_cal_valid ? g_self_near_dbm : 0;
    if (out_away) *out_away = g_self_cal_valid ? g_self_away_dbm : 0;
    return g_self_cal_valid ? 1 : 0;
}
int prox_last_self_delta(void) { return g_last_self_delta; }

// Bounded, asymmetric score adjustment from the anchor's own beacon level.
// Linear in dB between the two demonstrated levels, clamped hard at both ends —
// and clamped TIGHTER on the way down, because a weak reading is what both
// distance and a duvet produce (see PROX_SELF_UP_MAX_U8).
static int self_score_delta(const ProxScanVector* v) {
    if (!g_self_cal_valid || !g_have_self) return 0;
    int8_t self_rssi;
    if (!vec_lookup(v, g_self_mac, PROX_TYPE_BLE, &self_rssi)) return 0;

    const int span = (int)g_self_near_dbm - (int)g_self_away_dbm;   // > 0
    if (span < PROX_SELF_MIN_SPAN_DB) return 0;                     // not separable by level

    const int mid  = ((int)g_self_near_dbm + (int)g_self_away_dbm) / 2;
    const int half = span / 2;

    // Full deflection at the demonstrated level, proportional in between.
    int d = ((int)self_rssi - mid) * (int)PROX_SELF_UP_MAX_U8 / half;
    if (d >  (int)PROX_SELF_UP_MAX_U8)    d =  (int)PROX_SELF_UP_MAX_U8;
    if (d < -(int)PROX_SELF_DOWN_MAX_U8)  d = -(int)PROX_SELF_DOWN_MAX_U8;
    return d;
}

ProxScoreResult prox_compute_score(const ProxScanVector* watch_vec) {
    ProxScoreResult r = { 0, 0 };
    g_last_self_delta = 0;

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

    float rho; int k = 0;
    int have_rho = signal_correlation(watch_vec, &rho, &k);
    float L, wtot; int have_fp = signal_fingerprint(watch_vec, &L, &wtot);
    g_last_shared = k;

    // Neither signal has enough to say anything. Report the criterion-neutral
    // midpoint rather than a number: 0 would read as confident AWAY, and an
    // absent measurement is not evidence of distance (the same asymmetry the
    // engine spec's occlusion section states normatively -- an unobserved
    // environment can only ever fake "far").
    if (!have_rho && !have_fp) {
        r.flags |= PROX_FLAG_LOW_DEVICE_COUNT;
        r.score = PROX_STARVED_SCORE;
        return r;
    }

    // Whichever signal survived carries the weight; alpha only mediates between
    // them when both are actually available.
    float alpha = have_fp ? expf(-wtot / PROX_ALPHA_W0) : 1.0f;
    if (!have_rho) alpha = 0.0f;
    float score_f = alpha * rho + (1.0f - alpha) * L;
    score_f = clampf(score_f, 0.0f, 1.0f);

    int s = (int)(score_f * 255.0f + 0.5f);

    // The anchor's own beacon level, which neither signal above can see.
    g_last_self_delta = self_score_delta(watch_vec);
    s += g_last_self_delta;
    if (s < 0)   s = 0;
    if (s > 255) s = 255;

    r.score = (uint8_t)s;
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
int    prox_last_shared_count(void) { return g_last_shared; }
int    prox_last_fp_seen(void)      { return g_last_fp_seen; }
int8_t prox_last_self_rssi(void)    { return g_last_self_rssi; }

// The anchor's own beacon level as it appears in a GIVEN vector. Distinct from
// prox_last_self_rssi(), which is a leftover of the passive-training path and
// reads 0 on every calibration query because that path never runs there — a
// diagnostic that cost two separate investigations by looking like a
// measurement. Returns 0 when the anchor is absent from the vector, which is
// itself the thing worth logging: it is what silently disarms the self-RSSI
// discriminant.
int8_t prox_vector_self_rssi(const ProxScanVector* v) {
    int8_t s;
    if (!v || !g_have_self) return 0;
    if (!vec_lookup(v, g_self_mac, PROX_TYPE_BLE, &s)) return 0;
    return s;
}

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
        // Rotating (resolvable-private) addresses stay in the registry as live
        // correlation coordinates but never earn fingerprint weight -- the key
        // they would be trained under expires in ~15 minutes.
        if (mac_is_rotating(d->mac, d->type)) continue;
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

// ── Deferred calibration scoring ────────────────────────────────────────────
// See PROX_CALIB_BUF_SAMPLES in proximity.h for the measurement that forced this.
typedef struct {
    uint8_t    count;
    ProxDevice dev[PROX_CALIB_BUF_DEVICES];
} CalibSample;
static CalibSample g_calib_buf[2][PROX_CALIB_BUF_SAMPLES];  // [0] EDGE, [1] INSIDE
static uint8_t     g_calib_buf_n[2]   = {0, 0};             // retained
static uint32_t    g_calib_offered[2] = {0, 0};             // total offered (reservoir denominator)
static uint32_t    g_calib_rng        = 0x2545F491u;

static uint32_t calib_rand(void) {          // xorshift32; only needs to be unbiased
    g_calib_rng ^= g_calib_rng << 13;
    g_calib_rng ^= g_calib_rng >> 17;
    g_calib_rng ^= g_calib_rng << 5;
    return g_calib_rng;
}

void prox_calib_reset(void) {
    memset(g_calib_inside_hist, 0, sizeof(g_calib_inside_hist));
    memset(g_calib_edge_hist,   0, sizeof(g_calib_edge_hist));
    g_calib_inside_n = 0;
    g_calib_edge_n   = 0;
    g_calib_buf_n[0] = g_calib_buf_n[1] = 0;
    g_calib_offered[0] = g_calib_offered[1] = 0;
}

// Buffer one demonstrated vector. Reservoir sampling (Algorithm R) rather than
// keeping the first or last N: during the INSIDE leg the user is roaming the
// whole zone, so a positional bias toward either end of the walk would bias the
// threshold. Every offered sample gets an equal chance of being retained.
void prox_calib_collect(const ProxScanVector* v, int is_inside) {
    if (!v || v->count == 0) return;
    const int leg = is_inside ? 1 : 0;
    uint32_t seen = ++g_calib_offered[leg];

    int slot;
    if (g_calib_buf_n[leg] < PROX_CALIB_BUF_SAMPLES) {
        slot = g_calib_buf_n[leg]++;
    } else {
        uint32_t r = calib_rand() % seen;
        if (r >= PROX_CALIB_BUF_SAMPLES) return;   // not retained
        slot = (int)r;
    }

    // Keep the strongest PROX_CALIB_BUF_DEVICES; the vector arrives already
    // ordered strongest-first from the watch's cache.
    CalibSample* cs = &g_calib_buf[leg][slot];
    int k = v->count < PROX_CALIB_BUF_DEVICES ? v->count : PROX_CALIB_BUF_DEVICES;
    cs->count = (uint8_t)k;
    for (int i = 0; i < k; ++i) cs->dev[i] = v->devices[i];

    // The anchor's own beacon must survive truncation. It is now a SCORED
    // feature (self_score_delta), and it is not especially strong — measured at
    // -80 dBm in a real install, which in a busy room sits well outside the top
    // 24 by RSSI. Truncating it away silently disarms the term, because the two
    // demonstrated levels are learned from exactly these buffered vectors.
    // Displace the weakest kept entry instead.
    if (!g_have_self || k == 0) return;
    for (int i = 0; i < k; ++i) {
        if (cs->dev[i].type == PROX_TYPE_BLE &&
            memcmp(cs->dev[i].mac, g_self_mac, 6) == 0)
            return;                                   // already retained
    }
    for (int i = k; i < v->count; ++i) {
        if (v->devices[i].type == PROX_TYPE_BLE &&
            memcmp(v->devices[i].mac, g_self_mac, 6) == 0) {
            cs->dev[k - 1] = v->devices[i];
            return;
        }
    }
}

int prox_calib_collected(int is_inside) {
    return (int)g_calib_offered[is_inside ? 1 : 0];
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

// Mean of the anchor's own beacon level across one buffered calibration leg.
// Returns 0 if the anchor did not appear in enough of that leg's vectors.
static int calib_leg_self_mean(int leg, int8_t* out) {
    if (!g_have_self) return 0;
    int32_t sum = 0; int n = 0;
    for (int i = 0; i < g_calib_buf_n[leg]; ++i) {
        ProxScanVector v;
        v.count = g_calib_buf[leg][i].count;
        for (int d = 0; d < v.count; ++d) v.devices[d] = g_calib_buf[leg][i].dev[d];
        int8_t s;
        if (vec_lookup(&v, g_self_mac, PROX_TYPE_BLE, &s)) { sum += s; n++; }
    }
    // Demand a majority: a level learned from two stray samples is not a level.
    if (n == 0 || n * 2 < g_calib_buf_n[leg]) return 0;
    *out = (int8_t)(sum / n);
    return 1;
}

// Learn the two demonstrated beacon levels from the calibration legs. Refuses
// rather than guesses: an unlearned term abstains and the score is unchanged.
static void calib_learn_self_levels(void) {
    int8_t near_dbm = 0, away_dbm = 0;
    g_self_cal_valid = 0;
    if (!calib_leg_self_mean(1, &near_dbm)) return;
    if (!calib_leg_self_mean(0, &away_dbm)) return;
    // INSIDE must actually be the stronger of the two by a usable margin,
    // otherwise level carries no information here (or the legs were demonstrated
    // backwards) and the term stays off.
    if ((int)near_dbm - (int)away_dbm < PROX_SELF_MIN_SPAN_DB) return;
    g_self_near_dbm  = near_dbm;
    g_self_away_dbm  = away_dbm;
    g_self_cal_valid = 1;

    uint8_t rec[4];
    rec[0] = 0x53;                       // 'S', format tag
    rec[1] = (uint8_t)near_dbm;
    rec[2] = (uint8_t)away_dbm;
    rec[3] = 1;
    prox_platform_nvs_save(PROX_SELF_NVS_KEY, rec, sizeof(rec));
}

static void prox_load_self_levels(void) {
    uint8_t rec[4]; size_t len = 0;
    g_self_cal_valid = 0;
    if (!prox_platform_nvs_load(PROX_SELF_NVS_KEY, rec, sizeof(rec), &len)) return;
    if (len != sizeof(rec) || rec[0] != 0x53 || rec[3] != 1) return;
    g_self_near_dbm = (int8_t)rec[1];
    g_self_away_dbm = (int8_t)rec[2];
    if ((int)g_self_near_dbm - (int)g_self_away_dbm < PROX_SELF_MIN_SPAN_DB) return;
    g_self_cal_valid = 1;
}

// ---- calibration diagnostics -------------------------------------------------
//
// Everything here reads the two histograms; nothing here influences the decision.
// The point is to distinguish "the rule refused" from "the score never separated
// these two positions", which look identical from the app's side but have
// completely different fixes.

static ProxCalibStats g_calib_stats;
static uint8_t        g_calib_stats_valid = 0;

static inline uint8_t bucket_centre(int b) {
    return (uint8_t)(b * PROX_CALIB_BUCKET_WIDTH + PROX_CALIB_BUCKET_WIDTH / 2);
}

static uint32_t isqrt32(uint32_t x) {
    uint32_t r = 0, bit = 1u << 30;
    while (bit > x) bit >>= 2;
    while (bit) {
        if (x >= r + bit) { x -= r + bit; r = (r >> 1) + bit; }
        else              { r >>= 1; }
        bit >>= 2;
    }
    return r;
}

// Mean and standard deviation over a bucketed histogram, in score units.
static void histo_moments(const uint16_t* h, uint32_t n, uint8_t* mean, uint8_t* sd,
                          uint8_t* lo, uint8_t* hi) {
    *mean = *sd = 0;
    *lo = 0; *hi = 0;
    if (!n) return;
    uint32_t sum = 0;
    int first = -1, last = -1;
    for (int b = 0; b < PROX_CALIB_HIST_BUCKETS; ++b) {
        if (!h[b]) continue;
        if (first < 0) first = b;
        last = b;
        sum += (uint32_t)h[b] * bucket_centre(b);
    }
    const uint32_t m = sum / n;
    uint64_t ss = 0;
    for (int b = 0; b < PROX_CALIB_HIST_BUCKETS; ++b) {
        if (!h[b]) continue;
        const int32_t d = (int32_t)bucket_centre(b) - (int32_t)m;
        ss += (uint64_t)h[b] * (uint64_t)(d * d);
    }
    const uint32_t var = (uint32_t)(ss / n);
    *mean = (uint8_t)(m > 255 ? 255 : m);
    *sd   = (uint8_t)(isqrt32(var) > 255 ? 255 : isqrt32(var));
    *lo   = bucket_centre(first < 0 ? 0 : first);
    *hi   = bucket_centre(last  < 0 ? 0 : last);
}

// Count of samples strictly below / at-or-above a score.
static uint32_t histo_count_below(const uint16_t* h, uint8_t s) {
    uint32_t c = 0;
    for (int b = 0; b < PROX_CALIB_HIST_BUCKETS; ++b)
        if (bucket_centre(b) < s) c += h[b];
    return c;
}
static uint32_t histo_count_atleast(const uint16_t* h, uint8_t s) {
    uint32_t c = 0;
    for (int b = 0; b < PROX_CALIB_HIST_BUCKETS; ++b)
        if (bucket_centre(b) >= s) c += h[b];
    return c;
}

// Error-minimising cutoff over the two demonstrated histograms. Falls back to
// the global default only when there is genuinely nothing to measure.
static uint8_t calib_best_threshold(void) {
    if (!g_calib_inside_n || !g_calib_edge_n) return PROX_CONFIDENCE_THRESHOLD_U8;
    uint32_t best_err = 0xFFFFFFFFu;
    uint8_t  best_t   = PROX_CONFIDENCE_THRESHOLD_U8;
    for (int b = 0; b < PROX_CALIB_HIST_BUCKETS; ++b) {
        const uint8_t t = bucket_centre(b);
        const uint32_t err = histo_count_below(g_calib_inside_hist, t) +
                             histo_count_atleast(g_calib_edge_hist, t);
        if (err < best_err) { best_err = err; best_t = t; }
    }
    return best_t;
}

static void calib_capture_stats(uint8_t thr, uint8_t conf, uint8_t fail) {
    ProxCalibStats* s = &g_calib_stats;
    memset(s, 0, sizeof(*s));
    s->inside_n   = (uint16_t)g_calib_inside_n;
    s->edge_n     = (uint16_t)g_calib_edge_n;
    s->threshold  = thr;
    s->confidence = conf;
    s->fail       = fail;

    histo_moments(g_calib_inside_hist, g_calib_inside_n,
                  &s->inside_mean, &s->inside_sd, &s->inside_min, &s->inside_max);
    histo_moments(g_calib_edge_hist, g_calib_edge_n,
                  &s->edge_mean, &s->edge_sd, &s->edge_min, &s->edge_max);

    if (!g_calib_inside_n || !g_calib_edge_n) { g_calib_stats_valid = 1; return; }

    s->inside_p10 = histo_percentile(g_calib_inside_hist, g_calib_inside_n, 10);
    s->edge_p90   = histo_percentile(g_calib_edge_hist,   g_calib_edge_n,   90);
    s->gap = (int16_t)((int)s->inside_p10 -
                       ((int)s->edge_p90 + PROX_CALIB_THRESHOLD_MARGIN_U8));

    // How badly the clouds interpenetrate, in the rule's own terms.
    s->edge_above_pct   = (uint8_t)((100u * histo_count_atleast(g_calib_edge_hist,
                                                                s->inside_p10)) / g_calib_edge_n);
    s->inside_below_pct = (uint8_t)((100u * histo_count_below(g_calib_inside_hist,
                                        (uint8_t)(s->edge_p90 + 1))) / g_calib_inside_n);

    // Cohen's d over the pooled SD: are these two clouds separable at all?
    const uint32_t pooled_var =
        ((uint32_t)s->inside_sd * s->inside_sd + (uint32_t)s->edge_sd * s->edge_sd) / 2u;
    const uint32_t pooled_sd = isqrt32(pooled_var);
    const int32_t  dmean     = (int32_t)s->inside_mean - (int32_t)s->edge_mean;
    if (pooled_sd == 0) {
        s->dprime_x10 = (uint8_t)(dmean > 0 ? 255 : 0);   // zero spread: perfectly separated
    } else {
        int32_t d10 = (dmean * 10) / (int32_t)pooled_sd;
        if (d10 < 0)   d10 = 0;
        if (d10 > 255) d10 = 255;
        s->dprime_x10 = (uint8_t)d10;
    }

    // The best cutoff that exists, and what it would cost. This is the number
    // that says whether the RULE was the problem.
    const uint32_t total = g_calib_inside_n + g_calib_edge_n;
    uint32_t best_err = 0xFFFFFFFFu;
    uint8_t  best_t   = 0;
    for (int b = 0; b < PROX_CALIB_HIST_BUCKETS; ++b) {
        const uint8_t t = bucket_centre(b);
        const uint32_t err = histo_count_below(g_calib_inside_hist, t) +
                             histo_count_atleast(g_calib_edge_hist, t);
        if (err < best_err) { best_err = err; best_t = t; }
    }
    s->best_thr     = best_t;
    s->best_err_pct = (uint8_t)((100u * best_err) / total);
    g_calib_stats_valid = 1;
}

int prox_calib_last_stats(ProxCalibStats* out) {
    if (!g_calib_stats_valid || !out) return 0;
    *out = g_calib_stats;
    return 1;
}

uint8_t prox_calib_finalize(uint16_t* inside_n, uint16_t* edge_n, uint8_t* confidence) {
    // Deferred path: if vectors were buffered, train once and rebuild BOTH
    // histograms against the resulting fingerprint, so the threshold is measured
    // on the same estimator that will be deployed. Falls back to whatever
    // prox_calib_add() accumulated when nothing was buffered.
    if (g_calib_buf_n[0] || g_calib_buf_n[1]) {
        for (int i = 0; i < g_calib_buf_n[1]; ++i) {
            ProxScanVector v;
            v.count = g_calib_buf[1][i].count;
            for (int d = 0; d < v.count; ++d) v.devices[d] = g_calib_buf[1][i].dev[d];
            prox_train_labeled(&v, 1);
        }
        // Learn the self-RSSI levels BEFORE re-scoring, for the same reason the
        // fingerprint is trained first: the threshold has to be measured on the
        // exact estimator that will be deployed, not on a predecessor of it.
        calib_learn_self_levels();
        memset(g_calib_inside_hist, 0, sizeof(g_calib_inside_hist));
        memset(g_calib_edge_hist,   0, sizeof(g_calib_edge_hist));
        g_calib_inside_n = g_calib_edge_n = 0;
        for (int leg = 0; leg < 2; ++leg) {
            for (int i = 0; i < g_calib_buf_n[leg]; ++i) {
                ProxScanVector v;
                v.count = g_calib_buf[leg][i].count;
                for (int d = 0; d < v.count; ++d) v.devices[d] = g_calib_buf[leg][i].dev[d];
                ProxScoreResult r = prox_compute_score(&v);
                prox_calib_add(leg == 1, r.score);
            }
        }
    }

    uint16_t in_n = (uint16_t)g_calib_inside_n;
    uint16_t ed_n = (uint16_t)g_calib_edge_n;
    uint8_t thr, conf;
    uint8_t fail = 0;

    if (g_calib_inside_n < PROX_CALIB_MIN_SAMPLES || g_calib_edge_n < PROX_CALIB_MIN_SAMPLES) {
        // Not enough demonstrated samples to trust a learned cutoff.
        if (g_calib_inside_n < PROX_CALIB_MIN_SAMPLES) fail |= PROX_CALIB_FAIL_FEW_INSIDE;
        if (g_calib_edge_n   < PROX_CALIB_MIN_SAMPLES) fail |= PROX_CALIB_FAIL_FEW_EDGE;
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
            // Overlap: the conservative percentile rule found no gap. Fall back to
            // the error-minimising cutoff over the two demonstrated histograms
            // rather than to the global constant.
            //
            // The global PROX_CONFIDENCE_THRESHOLD_U8 assumes a score scale this
            // anchor may not have. Measured in the field: one run's scores spanned
            // 156..212, entirely ABOVE the 170 default, so the fallback classified
            // essentially everything as NEAR while the best available cutoff (188)
            // would have been wrong only 19% of the time. Confidence still reports
            // 0 — this is a degraded threshold, not an endorsed one — but a
            // measured degraded threshold beats an unmeasured one.
            fail |= PROX_CALIB_FAIL_OVERLAP;
            thr  = calib_best_threshold();
            conf = 0;
        }
    }

    // Snapshot the diagnostics while the histograms still exist.
    calib_capture_stats(thr, conf, fail);

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
        // Persist only real fingerprint entries. Saving every transient meant
        // the anchor woke up with all 128 slots occupied by addresses that had
        // rotated days ago, and reg_add() could never admit a live device again.
        if (d->W < PROX_MIN_FINGERPRINT_WEIGHT) continue;
        if (mac_is_rotating(d->mac, d->type)) continue;
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
    for (int i = 0; i < n && off + PROX_NVS_REC <= len
                  && g_reg_count < ANCHOR_PROX_MAX_FINGERPRINT_DEVICES; ++i) {
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

// ── Multi-window scan cache ─────────────────────────────────────────────────
// See the PROX_CACHE_TTL_MS commentary in proximity.h for why one scan window
// is not one observation. Structure: each device carries the last
// PROX_CACHE_SAMPLES per-window maxima (newest first) with fold timestamps, plus an
// accumulator for the window currently open.
#define PROX_SCAN_BUF 128
typedef struct {
    uint8_t  mac[6];
    uint8_t  type;
    int8_t   win_max;                        // strongest reading in the open window
    int8_t   rssi[PROX_CACHE_SAMPLES];       // retained per-window maxima, newest first
    uint32_t at_ms[PROX_CACHE_SAMPLES];      // when each was folded
    uint8_t  n;                              // valid entries in rssi/at_ms
    uint8_t  used;
} ScanEntry;
static ScanEntry g_scan[PROX_SCAN_BUF];
static int       g_scan_count = 0;
static int       g_scan_win_open = 0;   // any result ingested into the open window?

void prox_scan_cache_reset(void) {
    g_scan_count = 0;
    g_scan_win_open = 0;
}

void prox_ingest_scan_result(const uint8_t mac[6], uint8_t type, int8_t rssi) {
    g_scan_win_open = 1;
    for (int i = 0; i < g_scan_count; ++i)
        if (g_scan[i].used && g_scan[i].type == type && mac_eq(g_scan[i].mac, mac)) {
            if (rssi > g_scan[i].win_max) g_scan[i].win_max = rssi; // strongest wins
            return;
        }
    if (g_scan_count < PROX_SCAN_BUF) {
        ScanEntry* e = &g_scan[g_scan_count++];
        memcpy(e->mac, mac, 6);
        e->type    = type;
        e->win_max = rssi;
        e->n       = 0;
        e->used    = 1;
    }
}

// Drop samples older than the TTL; returns how many remain.
static int cache_age_out(ScanEntry* e, uint32_t now) {
    int w = 0;
    for (int i = 0; i < e->n; ++i) {
        if ((uint32_t)(now - e->at_ms[i]) > (uint32_t)PROX_CACHE_TTL_MS) continue;
        e->rssi[w]  = e->rssi[i];
        e->at_ms[w] = e->at_ms[i];
        w++;
    }
    e->n = (uint8_t)w;
    return w;
}

void prox_scan_window_close(void) {
    if (!g_scan_win_open) return;   // nothing scanned; don't age history for free
    const uint32_t now = prox_platform_now_ms();

    // While the wrist is moving, successive windows are taken at different
    // positions, so averaging across them smears distinct fades together. The
    // integrator restarts its window on STILL->LOCOMOTION for the same reason
    // (§4.2); do the same here and keep only the freshest sample.
    const int moving = (prox_motion_state() == PROX_MOTION_LOCOMOTION);

    int w = 0;
    for (int i = 0; i < g_scan_count; ++i) {
        ScanEntry* e = &g_scan[i];
        if (!e->used) continue;

        if (moving) e->n = 0;
        else        cache_age_out(e, now);

        if (e->win_max != PROX_RSSI_ABSENT) {
            // Push newest-first, dropping the oldest when the ring is full.
            int keep = e->n < PROX_CACHE_SAMPLES ? e->n : PROX_CACHE_SAMPLES - 1;
            for (int j = keep; j > 0; --j) {
                e->rssi[j]  = e->rssi[j - 1];
                e->at_ms[j] = e->at_ms[j - 1];
            }
            e->rssi[0]  = e->win_max;
            e->at_ms[0] = now;
            if (e->n < PROX_CACHE_SAMPLES) e->n++;
        }

        e->win_max = PROX_RSSI_ABSENT;              // open the next window

        if (e->n == 0) continue;                    // nothing within the TTL: evict
        if (w != i) g_scan[w] = *e;                 // compact in place
        w++;
    }
    g_scan_count = w;
    g_scan_win_open = 0;
}

// Median of a device's retained per-window maxima. For an even count this takes
// the upper of the two central values: a device's misses are downward-biased
// (a weak capture or a missed advertisement, never a spuriously strong one), so
// leaning to the stronger side corrects rather than inflates.
static int8_t cache_rssi(const ScanEntry* e) {
    if (e->n == 0) return PROX_RSSI_ABSENT;
    int8_t v[PROX_CACHE_SAMPLES];
    int n = e->n;
    for (int i = 0; i < n; ++i) v[i] = e->rssi[i];
    for (int a = 1; a < n; ++a) {                     // insertion sort ascending
        int8_t key = v[a]; int b = a - 1;
        while (b >= 0 && v[b] > key) { v[b + 1] = v[b]; b--; }
        v[b + 1] = key;
    }
    return v[n / 2];
}

int prox_scan_cache_count(void) { return g_scan_count; }

int prox_scan_cache_stable_count(void) {
    int n = 0;
    for (int i = 0; i < g_scan_count; ++i)
        if (g_scan[i].used && g_scan[i].n >= PROX_CACHE_SAMPLES) n++;
    return n;
}

void prox_build_scan_vector(ProxScanVector* out) {
    if (g_scan_win_open) prox_scan_window_close();    // safety net for callers that don't

    // Rank by RSSI, with persistence only as a tie-break.
    //
    // Ranking by persistence FIRST was tried and was wrong. The score is a
    // correlation over devices the watch and the anchor BOTH see, so the vector's
    // job is to maximise that intersection -- and signal strength is what makes a
    // device mutually visible, while persistence is a purely watch-local
    // property. Selecting the watch's most-persistent devices optimised the wrong
    // objective and collapsed the shared set to a handful, which then drove
    // Pearson into its degenerate small-k regime (see signal_correlation).
    //
    // Membership stability comes from the cache carrying devices across windows
    // and from ranking on a *median* RSSI instead of one instantaneous draw --
    // not from changing what the vector is selecting for.
    int n = g_scan_count;
    int k = n < PROX_MAX_DEVICES ? n : PROX_MAX_DEVICES;
    int8_t rssi[PROX_SCAN_BUF];
    for (int i = 0; i < n; ++i) rssi[i] = cache_rssi(&g_scan[i]);
    for (int i = 0; i < k; ++i) {
        int best = i;
        for (int j = i + 1; j < n; ++j) {
            if (rssi[j] > rssi[best] ||
                (rssi[j] == rssi[best] && g_scan[j].n > g_scan[best].n))
                best = j;
        }
        ScanEntry tmp = g_scan[i]; g_scan[i] = g_scan[best]; g_scan[best] = tmp;
        int8_t tr = rssi[i]; rssi[i] = rssi[best]; rssi[best] = tr;
    }
    out->count = (uint8_t)k;
    for (int i = 0; i < k; ++i) {
        memcpy(out->devices[i].mac, g_scan[i].mac, 6);
        out->devices[i].type = g_scan[i].type;
        out->devices[i].rssi = rssi[i];
    }
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
// PDR is kept in its own pending slot rather than summed into g_hmm_local_q8,
// because the two channels have different arithmetic. Connect-failure is an
// *event*: each one is a separate observation and they add. PDR is a *level* —
// the current standing estimate of the LO-slot delivery rate — so a second
// window before the next tick must replace the first, not double it.
static int32_t  g_hmm_pdr_q8   = 0;

// ---- Observation window & stepped-power PDR (§3.3) ------------------------
//
// The window ledger is per-cycle, not per-slot, because "covered" is defined by
// having heard the cycle at all (see PDR_OBS_MAX_CYCLES in proximity.h). Eight
// entries is double the ~4 cycles a 1800 ms window spans at BEACON_CYCLE_MS.

typedef struct {
    uint16_t cyc;
    uint8_t  saw_hi;
    uint8_t  saw_lo;
} ObsCycle;

static uint8_t  g_obs_mac[6];
static uint8_t  g_obs_have = 0;
static ObsCycle g_obs_cyc[PDR_OBS_MAX_CYCLES];
static uint8_t  g_obs_n    = 0;

// Motion-decayed Beta counts, Q4.
static uint16_t g_pdr_hits_q4     = 0;
static uint16_t g_pdr_cov_q4      = 0;
static uint8_t  g_pdr_last_motion = PROX_MOTION_UNKNOWN;
static uint8_t  g_pdr_primed      = 0;
static int32_t  g_pdr_loglr_q8    = 0;

void prox_pdr_reset(void) {
    g_pdr_hits_q4 = 0;
    g_pdr_cov_q4  = 0;
    g_pdr_last_motion = PROX_MOTION_UNKNOWN;
    g_pdr_primed   = 0;
    g_pdr_loglr_q8 = 0;
    g_hmm_pdr_q8   = 0;
    g_obs_have = 0;
    g_obs_n    = 0;
}

void prox_obs_begin(const uint8_t anchor_mac[6]) {
    g_obs_n = 0;
    if (anchor_mac) {
        memcpy(g_obs_mac, anchor_mac, 6);
        g_obs_have = 1;
    } else {
        g_obs_have = 0;
    }
}

void prox_obs_note(const uint8_t mac[6], uint16_t minor, int8_t rssi) {
    (void)rssi;                                   // PDR is presence, not amplitude
    if (!g_obs_have || !mac) return;
    if (memcmp(mac, g_obs_mac, 6) != 0) return;   // some other anchor's beacon

    uint8_t  slot = 0;
    uint16_t cyc  = 0;
    // A legacy (BEACON_SCHEDULE_ENABLE = 0) anchor sends Minor 0x0000, which the
    // shipped decoder rejects. Such an anchor simply yields no PDR evidence.
    if (!prox_beacon_minor_decode(minor, &slot, &cyc)) return;

    ObsCycle* e = NULL;
    for (uint8_t i = 0; i < g_obs_n; ++i) {
        if (g_obs_cyc[i].cyc == cyc) { e = &g_obs_cyc[i]; break; }
    }
    if (!e) {
        if (g_obs_n >= PDR_OBS_MAX_CYCLES) return;   // window ran longer than planned
        e = &g_obs_cyc[g_obs_n++];
        e->cyc = cyc; e->saw_hi = 0; e->saw_lo = 0;
    }
    if (prox_beacon_slot_is_lo(slot)) e->saw_lo = 1;
    else                              e->saw_hi = 1;
}

static void pdr_accumulate(uint8_t hits, uint8_t covered) {
    const uint8_t ms = prox_motion_state();

    // STILL -> moving is a window restart (§4.2): the wrist may be somewhere
    // else now, and slot evidence gathered at the old position must not outvote
    // what the new position is about to say.
    if (g_pdr_primed && g_pdr_last_motion == PROX_MOTION_STILL &&
        ms != PROX_MOTION_STILL) {
        g_pdr_hits_q4 = 0;
        g_pdr_cov_q4  = 0;
    }
    g_pdr_last_motion = ms;
    g_pdr_primed      = 1;

    // A motionless wrist re-measures one frozen fade. It earns a fraction of a
    // slot, not a whole one — the same INTEG_STILL_WEIGHT discount every other
    // integrated quantity gets, and for the identical reason.
    const uint32_t w_q8 = (ms == PROX_MOTION_STILL) ? (uint32_t)INTEG_STILL_WEIGHT : 256u;
    g_pdr_hits_q4 = (uint16_t)(g_pdr_hits_q4 + ((((uint32_t)hits    << 4) * w_q8) >> 8));
    g_pdr_cov_q4  = (uint16_t)(g_pdr_cov_q4  + ((((uint32_t)covered << 4) * w_q8) >> 8));

    // Ceiling. Both counts are rescaled together, so capping costs confidence
    // but never shifts the estimated rate.
    const uint16_t cap_q4 = (uint16_t)(PDR_MAX_EFF_SLOTS << 4);
    if (g_pdr_cov_q4 > cap_q4) {
        g_pdr_hits_q4 = (uint16_t)(((uint32_t)g_pdr_hits_q4 * cap_q4) / g_pdr_cov_q4);
        g_pdr_cov_q4  = cap_q4;
    }
}

// Naive-Bayes sum over the decayed slot counts, then clamped asymmetrically.
// See LL_PDR_NEAR_MAX_Q8 / LL_PDR_AWAY_MAX_Q8: the asymmetry is the §13.0
// anti-tamper invariant, not a tuning knob.
static int32_t pdr_loglr_q8(void) {
    if (g_pdr_cov_q4 < (uint16_t)(PDR_MIN_COVERED << 4)) return 0;
    const int32_t miss_q4 = (int32_t)g_pdr_cov_q4 - (int32_t)g_pdr_hits_q4;
    int32_t ll = (((int32_t)g_pdr_hits_q4 * LL_PDR_HIT_Q8) +
                  (miss_q4 * LL_PDR_MISS_Q8)) >> 4;
    if (ll > LL_PDR_NEAR_MAX_Q8) ll = LL_PDR_NEAR_MAX_Q8;
    if (ll < LL_PDR_AWAY_MAX_Q8) ll = LL_PDR_AWAY_MAX_Q8;
    return ll;
}

int prox_obs_close(uint8_t* out_hits, uint8_t* out_covered) {
    uint8_t covered = 0, hits = 0;
    for (uint8_t i = 0; i < g_obs_n; ++i) {
        // Hearing either slot proves the cycle happened AND that the window
        // spanned it. A cycle the window merely clipped is heard in neither slot
        // and contributes to neither count.
        if (!g_obs_cyc[i].saw_hi && !g_obs_cyc[i].saw_lo) continue;
        covered++;
        if (g_obs_cyc[i].saw_lo) hits++;
    }
    g_obs_n = 0;

    if (out_hits)    *out_hits    = hits;
    if (out_covered) *out_covered = covered;

    // Too little of the schedule was heard to say anything. Abstain rather than
    // report 0/1 as a miss — that is how a merely-unlucky scan turns into
    // fabricated AWAY evidence.
    if (covered < PDR_MIN_COVERED) {
        g_pdr_loglr_q8 = 0;
        g_hmm_pdr_q8   = 0;
        return 0;
    }

    pdr_accumulate(hits, covered);
    g_pdr_loglr_q8 = pdr_loglr_q8();
    g_hmm_pdr_q8   = g_pdr_loglr_q8;
    return g_pdr_loglr_q8 != 0;
}

int prox_pdr_state(uint8_t* out_rate_u8, uint8_t* out_covered, int32_t* out_loglr_q8) {
    if (out_rate_u8) {
        *out_rate_u8 = g_pdr_cov_q4
            ? (uint8_t)(((uint32_t)g_pdr_hits_q4 * 255u) / g_pdr_cov_q4)
            : 0u;
    }
    if (out_covered)   *out_covered   = (uint8_t)(g_pdr_cov_q4 >> 4);
    if (out_loglr_q8)  *out_loglr_q8  = g_pdr_loglr_q8;
    return g_pdr_cov_q4 >= (uint16_t)(PDR_MIN_COVERED << 4);
}

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
    prox_pdr_reset();                    // slot counts are per-enforcement-window
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
    int32_t total = raw_loglr_q8 + g_hmm_local_q8 + g_hmm_pdr_q8;
    g_hmm_local_q8 = 0;
    // PDR is consumed, not zeroed-and-forgotten: the level is re-queued by the
    // next prox_obs_close(). A tick with no observation window therefore folds no
    // PDR evidence rather than repeating the last window's.
    g_hmm_pdr_q8   = 0;

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
    prox_load_self_levels();
#endif
#ifdef PROXIMITY_ROLE_WATCH
    g_scan_count = 0;
    coloc_init();
#endif
}
