// prox_capture — see prox_capture.h for what this is and what it is not.

#include "prox_capture.h"

#if PROX_CAPTURE_ENABLED

#include <string.h>

// The clock seam the rest of the engine already uses, so capture timestamps and
// engine timestamps are the same clock by construction.
extern "C" uint32_t prox_platform_now_ms(void);

static ProxCaptureSink s_sink   = 0;
static uint8_t  s_role          = CAP_ROLE_WATCH;
static uint32_t s_seq           = 0;
static uint32_t s_dropped       = 0;
static uint32_t s_wall          = 0;
static int      s_inited        = 0;

// Byte ring of complete, framed records. Records are never split across the
// wrap: a record that will not fit contiguously in the tail is dropped whole,
// which keeps the drain loop trivial and costs at most one record per lap.
static uint8_t  s_buf[PROX_CAPTURE_BUF_BYTES];
static uint16_t s_head = 0;      // next write
static uint16_t s_tail = 0;      // next read
static uint16_t s_used = 0;

void prox_capture_init(uint8_t role, ProxCaptureSink sink) {
    s_role    = role;
    s_sink    = sink;
    s_head = s_tail = s_used = 0;
    s_seq = s_dropped = 0;
    s_wall = 0;
    s_inited  = 1;
}

void prox_capture_set_wall(uint32_t unix_s) { s_wall = unix_s; }

uint32_t prox_capture_cfg_hash(void) {
    // The tunables that change a verdict. Adding one here is the right move
    // whenever a new constant starts affecting scoring — a hash that misses a
    // constant will happily claim two different engines are the same one.
    const int32_t cfg[] = {
        PROX_V2_AUTHORITATIVE, PROX_R2_OFFSET_INVARIANT,
        PROX_MIN_DEVICE_COUNT, PROX_MIN_SHARED_DEVICES, PROX_STARVED_SCORE,
        PROX_CM_MIN_SAMPLES,
        (int32_t)(PROX_ALPHA_W0), (int32_t)(PROX_LL_CENTER * 1000.0f),
        (int32_t)(PROX_LL_SCALE * 1000.0f),
        (int32_t)(PROX_CM_OCCLUSION_DB * 100.0f),
        (int32_t)(PROX_CM_MAD_RATIO * 100.0f),
        PROX_SELF_UP_MAX_U8, PROX_SELF_DOWN_MAX_U8, PROX_SELF_MIN_SPAN_DB,
        PROX_FAR_RSSI_THRESHOLD_DBM,
        IMU_STILL_VAR, IMU_LOCO_VAR, IMU_LOCO_MIN_INTS, IMU_STALE_MS,
        NEFF_LOCO_PER_S, NEFF_FIDGET_PER_S,
        HMM_TAU_NEAR_Q8, HMM_TAU_AWAY_Q8, HMM_PRIOR_Q8, LL_CONNFAIL_AWAY_Q8,
        HMM_EMIT_SLOPE_Q8, HMM_EMIT_MAX_Q8,
        PROX_AWAY_ARM_HITS, PROX_AWAY_ARM_SPACING_S, PROX_AWAY_ARM_WINDOW_S,
        PROX_AWAY_LIVENESS_S,
    };
    uint32_t h = 2166136261u;
    const uint8_t *p = (const uint8_t *)cfg;
    for (size_t i = 0; i < sizeof(cfg); ++i) { h ^= p[i]; h *= 16777619u; }
    return h;
}

void prox_capture_stats(uint32_t *out_seq, uint32_t *out_dropped) {
    if (out_seq)     *out_seq     = s_seq;
    if (out_dropped) *out_dropped = s_dropped;
}

static void ring_write(const uint8_t *p, uint16_t n) {
    for (uint16_t i = 0; i < n; ++i) {
        s_buf[s_head] = p[i];
        s_head = (uint16_t)((s_head + 1) % PROX_CAPTURE_BUF_BYTES);
    }
    s_used = (uint16_t)(s_used + n);
}

void prox_capture_emit2(uint8_t type, const void *head, uint16_t head_len,
                        const void *body, uint16_t body_len) {
    if (!s_inited) return;

    // The sequence number advances even when the record is dropped. That is the
    // whole gap-detection mechanism: the receiver sees the discontinuity and
    // records a hole, rather than silently stitching an incomplete timeline.
    const uint32_t seq = s_seq++;

    const uint16_t payload = (uint16_t)(head_len + body_len);
    const uint16_t total   = (uint16_t)(sizeof(ProxCapHeader) + payload);
    if (total > PROX_CAPTURE_BUF_BYTES - s_used) { s_dropped++; return; }

    ProxCapHeader h;
    h.magic    = PROX_CAPTURE_MAGIC;
    h.ver      = PROX_CAPTURE_VERSION;
    h.type     = type;
    h.role     = s_role;
    h.seq      = seq;
    h.t_ms     = prox_platform_now_ms();
    h.t_wall   = s_wall;
    h.len      = payload;
    h.reserved = 0;

    ring_write((const uint8_t *)&h, sizeof(h));
    if (head_len) ring_write((const uint8_t *)head, head_len);
    if (body_len) ring_write((const uint8_t *)body, body_len);
}

void prox_capture_emit(uint8_t type, const void *payload, uint16_t len) {
    prox_capture_emit2(type, payload, len, 0, 0);
}

void prox_capture_flush(void) {
    if (!s_inited || !s_sink) return;

    // One datagram per record. At these rates (a handful per query) the
    // per-datagram overhead is irrelevant, and a self-contained datagram means a
    // lost one costs exactly one record instead of desynchronising a stream.
    uint8_t scratch[sizeof(ProxCapHeader) + 512];
    while (s_used >= sizeof(ProxCapHeader)) {
        ProxCapHeader h;
        uint16_t t = s_tail;
        for (uint16_t i = 0; i < sizeof(h); ++i) {
            ((uint8_t *)&h)[i] = s_buf[t];
            t = (uint16_t)((t + 1) % PROX_CAPTURE_BUF_BYTES);
        }
        const uint16_t total = (uint16_t)(sizeof(h) + h.len);
        if (h.magic != PROX_CAPTURE_MAGIC || total > sizeof(scratch) ||
            total > s_used) {
            // Corrupt framing should be impossible; if it happens, drop the
            // whole buffer rather than emit garbage into a corpus that later
            // gets treated as ground truth.
            s_head = s_tail = s_used = 0;
            s_dropped++;
            return;
        }

        t = s_tail;
        for (uint16_t i = 0; i < total; ++i) {
            scratch[i] = s_buf[t];
            t = (uint16_t)((t + 1) % PROX_CAPTURE_BUF_BYTES);
        }
        if (!s_sink(scratch, total)) return;   // leave it buffered; retry later
        s_tail = t;
        s_used = (uint16_t)(s_used - total);
    }
}

// ── Typed helpers ───────────────────────────────────────────────────────────

void prox_capture_vector(const ProxScanVector *v) {
    if (!v || v->count == 0) return;
    ProxCapDev body[PROX_MAX_DEVICES];
    const uint8_t n = v->count > PROX_MAX_DEVICES ? PROX_MAX_DEVICES : v->count;
    for (uint8_t i = 0; i < n; ++i) {
        memcpy(body[i].mac, v->devices[i].mac, 6);
        body[i].type = v->devices[i].type;
        body[i].rssi = v->devices[i].rssi;
    }
    const uint16_t count = n;
    prox_capture_emit2(CAP_VECTOR, &count, sizeof(count),
                       body, (uint16_t)(n * sizeof(ProxCapDev)));
}

#ifdef PROXIMITY_ROLE_ANCHOR
void prox_capture_anchor_cache(void) {
    ProxCapDev body[PROX_MAX_DEVICES];
    uint16_t n = 0;
    const int size = prox_reg_size();
    for (int i = 0; i < size && n < PROX_MAX_DEVICES; ++i) {
        uint8_t mac[6], type = 0;
        int8_t  rssi = 0;
        if (!prox_reg_get(i, mac, &type, &rssi, 0, 0, 0, 0, 0)) continue;
        memcpy(body[n].mac, mac, 6);
        body[n].type = type;
        body[n].rssi = rssi;
        n++;
    }
    if (!n) return;
    prox_capture_emit2(CAP_ANCHOR_CACHE, &n, sizeof(n),
                       body, (uint16_t)(n * sizeof(ProxCapDev)));
}

void prox_capture_score_result(const ProxScoreResult *r, uint8_t near_thr,
                               int8_t self_rssi, uint16_t vec_count) {
    if (!r) return;
    ProxCapScore s;
    memset(&s, 0, sizeof(s));
    s.score      = r->score;
    s.flags      = r->flags;
    s.near_thr   = near_thr;
    s.self_rssi  = self_rssi;
    s.self_delta = (int16_t)prox_last_self_delta();
    s.shared     = (uint16_t)prox_last_shared_count();
    s.fp_seen    = (uint16_t)prox_last_fp_seen();
    s.vec_count  = vec_count;
    int n = 0;
    prox_last_common_mode(&s.cm_delta, &s.cm_mad, &s.cm_sigma, &n,
                          &s.L_legacy, &s.L_invariant);
    prox_capture_emit(CAP_SCORE, &s, sizeof(s));
}

void prox_capture_fingerprint(void) {
    // Replay needs the state the engine STARTED from, not just the events. This
    // is the largest record by far, so it is emitted at session start and after
    // training folds — never per query.
    ProxCapFpDev body[64];
    uint16_t n = 0;
    const int size = prox_reg_size();
    for (int i = 0; i < size && n < 64; ++i) {
        uint8_t mac[6], type = 0, active = 0;
        float mu = 0, var = 0, W = 0;
        if (!prox_reg_get(i, mac, &type, 0, 0, &mu, &var, &W, &active)) continue;
        if (!active) continue;
        memcpy(body[n].mac, mac, 6);
        body[n].type     = type;
        body[n].reserved = 0;
        body[n].mu  = mu;
        body[n].var = var;
        body[n].W   = W;
        n++;
    }
    if (!n) return;
    prox_capture_emit2(CAP_FINGERPRINT, &n, sizeof(n),
                       body, (uint16_t)(n * sizeof(ProxCapFpDev)));
}
#endif  // PROXIMITY_ROLE_ANCHOR

#endif  // PROX_CAPTURE_ENABLED
