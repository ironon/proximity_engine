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

// Is the sample unambiguous? Requires self present and beating any known peer
// anchor by PROX_COLLECT_AMBIGUITY_MARGIN_DBM. If no peers are known, the self
// presence + reasonable-strength gate is used (see spec §2.5 / §4).
static int sample_is_unambiguous(const ProxScanVector* v) {
    if (!g_have_self) return 0;
    int8_t self_rssi;
    if (!vec_lookup(v, g_self_mac, PROX_TYPE_BLE, &self_rssi)) return 0;
    if (self_rssi < ANCHOR_NEAR_RSSI_THRESHOLD_DBM) return 0;
    for (int i = 0; i < g_peer_count; ++i) {
        int8_t peer_rssi;
        if (vec_lookup(v, g_peer_macs[i], PROX_TYPE_BLE, &peer_rssi))
            if (peer_rssi >= self_rssi - PROX_COLLECT_AMBIGUITY_MARGIN_DBM)
                return 0; // a rival anchor is comparably close → boundary sample
    }
    return 1;
}

int prox_maybe_update_fingerprint(const ProxScanVector* watch_vec, ProxScoreResult result) {
    if (result.flags & PROX_FLAG_LOW_DEVICE_COUNT) return 0;
    float score_f = result.score / 255.0f;
    if (score_f < PROX_COLLECT_SCORE_THRESHOLD) return 0;
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
    return 1;
}

// ---- persistence ----
// Internal NVS blob keeps full Welford state (mu, M, W) so accumulation resumes
// across reboots: [2 count][per dev: 6 mac, 1 type, 4 mu, 4 M, 4 W].
#define PROX_NVS_REC 19

void prox_persist_if_due(void) {
    uint32_t now = prox_platform_now_ms();
    if (g_last_persist_ms != 0 && (now - g_last_persist_ms) < (uint32_t)PROX_NVS_PERSIST_INTERVAL_S * 1000u) return;
    g_last_persist_ms = now;

    static uint8_t buf[2 + ANCHOR_PROX_MAX_FINGERPRINT_DEVICES * PROX_NVS_REC];
    int n = 0; size_t off = 2;
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
    wr_u16(buf, (uint16_t)n);
    prox_platform_nvs_save(PROX_NVS_KEY, buf, off);
}

static void prox_load_from_nvs(void) {
    static uint8_t buf[2 + ANCHOR_PROX_MAX_FINGERPRINT_DEVICES * PROX_NVS_REC];
    size_t len = 0;
    if (!prox_platform_nvs_load(PROX_NVS_KEY, buf, sizeof(buf), &len) || len < 2) return;
    int n = rd_u16(buf); size_t off = 2;
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
// MODES B / C — CO-LOCATION DETECTOR (watch side)
// ============================================================================
#ifdef PROXIMITY_ROLE_WATCH

// ---- ring buffer of floats (temporal integration, Lever 1) ----
typedef struct { float v[COLOC_WINDOW_SAMPLES]; int idx, cnt; } RingF;
static void ring_reset(RingF* r) { r->idx = 0; r->cnt = 0; }
static void ring_push(RingF* r, float x) {
    r->v[r->idx] = x;
    r->idx = (r->idx + 1) % COLOC_WINDOW_SAMPLES;
    if (r->cnt < COLOC_WINDOW_SAMPLES) r->cnt++;
}
static int ring_mean(const RingF* r, float* out) {
    if (r->cnt == 0) return 0;
    float s = 0; for (int i = 0; i < r->cnt; ++i) s += r->v[i];
    *out = s / r->cnt; return 1;
}
static int ring_var(const RingF* r, float* out) {
    if (r->cnt < 2) return 0;
    float m; ring_mean(r, &m);
    float s = 0; for (int i = 0; i < r->cnt; ++i) { float d = r->v[i] - m; s += d * d; }
    *out = s / (r->cnt - 1); return 1;
}

// ---- config + state ----
static ColocConfig g_cfg;
static RingF       g_rwp;     // watch<->phone RSSI window  (Factor 1 + Factor V)
static RingF       g_delta;   // R_P - R_D window           (Factor delta, Mode C)
static RingF       g_s;       // per-reading std(delta_d)   (Factor 2)
static ColocState  g_state = COLOC_AWAY;
static int         g_debounce = 0;
static ColocDecision g_last = { 0.0f, COLOC_AWAY, 0, 0 };

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
    ring_reset(&g_rwp); ring_reset(&g_delta); ring_reset(&g_s);
    g_state = COLOC_AWAY; g_debounce = 0;
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
    if (g_have_rwp) ring_push(&g_rwp, (float)g_cur_rwp);
    if (g_have_diff) ring_push(&g_delta, (float)g_cur_rp - (float)g_cur_rd);
    if (g_dcount >= (uint32_t)COLOC_ENV_MIN_SHARED_DEVICES) {
        double mean = g_dsum / g_dcount;
        double var  = g_dsumsq / g_dcount - mean * mean;
        if (var < 0) var = 0;
        ring_push(&g_s, (float)sqrt(var));
    }
    g_have_rwp = 0; g_have_diff = 0;
    g_dsum = g_dsumsq = 0; g_dcount = 0;
}

// ---- fusion + hysteresis ----
ColocDecision coloc_tick(void) {
    ColocDecision out = { 0.0f, g_state, 0, 0 };

    float log_odds = logf(g_cfg.prior_near / (1.0f - g_cfg.prior_near));

    float rwp_mean = 0.0f;
    int have_rwp = ring_mean(&g_rwp, &rwp_mean);
    if (have_rwp) { log_odds += feature_loglr(&g_cfg.rwp, rwp_mean); out.used_factors |= COLOC_F_RANGE; }

    float dmean;
    if (ring_mean(&g_delta, &dmean)) { log_odds += feature_loglr(&g_cfg.delta, dmean); out.used_factors |= COLOC_F_DIFF; }

    float smean;
    if (ring_mean(&g_s, &smean)) { log_odds += feature_loglr(&g_cfg.s, smean); out.used_factors |= COLOC_F_ENV; }

    float rwp_var = 0.0f;
    int   have_var = ring_var(&g_rwp, &rwp_var);
    if (have_var) { log_odds += feature_loglr(&g_cfg.var, rwp_var); out.used_factors |= COLOC_F_VAR; }

    out.p_near = logistic(log_odds);

    // "holding" signature: high mean RSSI + low variance.
    if (have_rwp && have_var) {
        float rwp_mid = 0.5f * (g_cfg.rwp.near_mu + g_cfg.rwp.away_mu);
        float var_mid = 0.5f * (g_cfg.var.near_mu + g_cfg.var.away_mu);
        out.holding = (rwp_mean >= rwp_mid && rwp_var <= var_mid) ? 1u : 0u;
    }

    // Two-threshold hysteresis with debounce. Ambiguous band holds current state;
    // fail-safe bias toward AWAY/compliant is realised by TAU_HIGH > TAU_LOW and
    // requiring sustained high evidence to flip to NEAR (spec §4.8).
    ColocState cand = g_state;
    if (out.p_near >= COLOC_TAU_HIGH)      cand = COLOC_NEAR;
    else if (out.p_near <= COLOC_TAU_LOW)  cand = COLOC_AWAY;

    if (cand != g_state) {
        if (++g_debounce >= COLOC_DEBOUNCE_SAMPLES) { g_state = cand; g_debounce = 0; }
    } else {
        g_debounce = 0;
    }
    out.state = g_state;

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
    prox_load_from_nvs();
#endif
#ifdef PROXIMITY_ROLE_WATCH
    g_scan_count = 0;
    coloc_init();
#endif
}
