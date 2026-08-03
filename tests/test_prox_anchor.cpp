// Host-side tests for the ANCHOR role's Mode A scoring path.
//
// This path had no host coverage at all, and two separate field defects lived in
// it at once:
//
//   1. Pearson's r was computed over as few as 2 shared devices, where it is
//      mathematically +/-1 whatever was measured. A still watch beside a still
//      anchor logged scores of 0, 0, 255, 0, 255, 0 — not noise, arithmetic.
//   2. Fingerprinted devices absent from a vector were imputed at -100 dBm, so a
//      device that merely failed to advertise inside the watch's scan window
//      scored ~12.5 nats of penalty while sitting motionless a metre away.
//
// Both are cheap to assert and expensive to rediscover in the field.
//
//   cd proximity_engine/tests && make anchor

#include "proximity.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <map>
#include <vector>

// ---------------------------------------------------------------- platform seam

static uint32_t g_now_ms = 1000;
static std::map<std::string, std::vector<uint8_t> > g_nvs;

extern "C" {
uint32_t prox_platform_now_ms(void) { return g_now_ms; }

int prox_platform_nvs_load(const char* key, void* buf, size_t cap, size_t* out_len) {
    std::map<std::string, std::vector<uint8_t> >::iterator it = g_nvs.find(key);
    if (it == g_nvs.end()) return 0;
    size_t n = it->second.size() < cap ? it->second.size() : cap;
    memcpy(buf, &it->second[0], n);
    if (out_len) *out_len = n;
    return 1;
}

int prox_platform_nvs_save(const char* key, const void* buf, size_t len) {
    const uint8_t* p = (const uint8_t*)buf;
    g_nvs[key] = std::vector<uint8_t>(p, p + len);
    return 1;
}

void prox_platform_set_beacon_slot(uint8_t, int8_t, uint16_t) {}
} // extern "C"

// ---------------------------------------------------------------- harness

static int g_checks = 0;
static int g_fail = 0;
static const char* g_case = "";

#define CHECK(cond, ...) do {                                                  \
    g_checks++;                                                                \
    if (!(cond)) {                                                             \
        g_fail++;                                                              \
        printf("  FAIL [%s] %s:%d: %s\n    ", g_case, __FILE__, __LINE__, #cond); \
        printf(__VA_ARGS__); printf("\n");                                     \
    }                                                                          \
} while (0)

static void begin(const char* name) { g_case = name; printf("- %s\n", name); }

// ---------------------------------------------------------------- aids

static void mk_mac(int i, uint8_t out[6]) {
    out[0] = 0xDE; out[1] = 0xAD; out[2] = 0xBE;
    out[3] = 0x00; out[4] = (uint8_t)(i >> 8); out[5] = (uint8_t)(i & 0xFF);
}

// Populate the anchor's own live cache (what it currently hears).
static void anchor_hears(int idx, int8_t rssi) {
    uint8_t mac[6]; mk_mac(idx, mac);
    prox_ingest_scan_result(mac, PROX_TYPE_BLE, rssi);
}

static void vec_add(ProxScanVector* v, int idx, int8_t rssi) {
    if (v->count >= PROX_MAX_DEVICES) return;
    mk_mac(idx, v->devices[v->count].mac);
    v->devices[v->count].type = PROX_TYPE_BLE;
    v->devices[v->count].rssi = rssi;
    v->count++;
}

// A pseudo-random but deterministic RSSI landscape, so the watch and anchor
// views of the same room correlate without being identical.
static int8_t room_rssi(int idx) { return (int8_t)(-45 - ((idx * 37) % 45)); }

// The watch stands somewhere else in the same room, so every device reaches it
// over a different path: a fixed per-device shadowing offset on top of the
// anchor's view. This is what makes the true correlation high but not 1.0 —
// without it a synthetic vector is an exact affine copy and Pearson saturates
// for reasons that have nothing to do with the code under test.
static int8_t watch_rssi(int idx) {
    int off = ((idx * 61) % 17) - 8;              // deterministic, +/-8 dB
    return (int8_t)(room_rssi(idx) - 6 + off);
}

// ---------------------------------------------------------------- tests

static void test_degenerate_k(void) {
    begin("score: too few shared devices abstains instead of saturating");
    // The exact field failure. Two shared devices always lie on a line, so
    // Pearson is +/-1 and the score lands on a rail. The engine must decline.
    for (int k = 0; k < PROX_MIN_SHARED_DEVICES; ++k) {
        prox_init();
        for (int i = 0; i < k; ++i) anchor_hears(i, room_rssi(i));

        ProxScanVector v; memset(&v, 0, sizeof(v));
        for (int i = 0; i < k; ++i)          vec_add(&v, i, watch_rssi(i));
        for (int i = 100; i < 100 + 20; ++i) vec_add(&v, i, -80);   // unshared padding

        ProxScoreResult r = prox_compute_score(&v);
        CHECK(r.score != 0 && r.score != 255,
              "k=%d must not produce a saturated score, got %d", k, (int)r.score);
        CHECK(r.score == PROX_STARVED_SCORE,
              "k=%d should report the neutral score, got %d", k, (int)r.score);
        CHECK((r.flags & PROX_FLAG_LOW_DEVICE_COUNT) != 0,
              "k=%d should raise LOW_DEVICE_COUNT", k);
    }

    begin("score: k=2 specifically — the case that produced 0/255 on hardware");
    // Walk both sign patterns. Under the old code one gave exactly 255 and the
    // other exactly 0, from the same two devices.
    for (int flip = 0; flip < 2; ++flip) {
        prox_init();
        anchor_hears(1, -50);
        anchor_hears(2, -80);
        ProxScanVector v; memset(&v, 0, sizeof(v));
        vec_add(&v, 1, -55);
        vec_add(&v, 2, flip ? -40 : -85);       // agreeing vs opposing
        for (int i = 100; i < 130; ++i) vec_add(&v, i, -80);
        ProxScoreResult r = prox_compute_score(&v);
        CHECK(r.score == PROX_STARVED_SCORE,
              "k=2 (flip=%d) must abstain, got %d", flip, (int)r.score);
    }

    begin("score: enough shared devices produces a real, unsaturated score");
    prox_init();
    for (int i = 0; i < 20; ++i) anchor_hears(i, room_rssi(i));
    ProxScanVector v; memset(&v, 0, sizeof(v));
    for (int i = 0; i < 20; ++i) vec_add(&v, i, watch_rssi(i));
    ProxScoreResult r = prox_compute_score(&v);
    CHECK(prox_last_shared_count() == 20, "expected 20 shared, got %d", prox_last_shared_count());
    CHECK(r.score > 100 && r.score < 255,
          "a correlated-but-distinct viewpoint should land mid-range, got %d", (int)r.score);
    CHECK((r.flags & PROX_FLAG_LOW_DEVICE_COUNT) == 0, "20 shared devices is not starved");

    begin("score: a stationary pair produces a STABLE score across queries");
    // The headline regression. Same physical situation, different random subset
    // of devices caught each query — the score must not swing across the range.
    prox_init();
    for (int i = 0; i < 40; ++i) anchor_hears(i, room_rssi(i));
    unsigned rng = 99991u;
    int lo = 255, hi = 0;
    for (int q = 0; q < 24; ++q) {
        ProxScanVector w; memset(&w, 0, sizeof(w));
        for (int i = 0; i < 40; ++i) {
            rng = rng * 1103515245u + 12345u;
            if (((rng >> 16) % 100u) < 60u)            // 60% capture per query
                vec_add(&w, i, watch_rssi(i));
        }
        ProxScoreResult rr = prox_compute_score(&w);
        if (rr.score < lo) lo = rr.score;
        if (rr.score > hi) hi = rr.score;
    }
    printf("    stationary score range over 24 queries: %d..%d\n", lo, hi);
    CHECK(hi - lo <= 60, "a frozen channel must not swing the score by %d points", hi - lo);
    CHECK(lo > 0 && hi < 255, "score must never hit a rail here (%d..%d)", lo, hi);
}

static void test_absent_fingerprint_devices(void) {
    begin("fingerprint: an absent device abstains rather than scoring -100 dBm");
    // Train a fingerprint, then score two vectors that agree on every device
    // they share — one complete, one with a third of the devices simply not
    // caught by the scan. Imputing -100 made the second look far away.
    prox_init();
    for (int i = 0; i < 30; ++i) anchor_hears(i, room_rssi(i));

    uint8_t self[6]; mk_mac(900, self);
    prox_set_self_mac(self);

    for (int t = 0; t < 40; ++t) {                     // build the fingerprint
        ProxScanVector v; memset(&v, 0, sizeof(v));
        for (int i = 0; i < 30; ++i) vec_add(&v, i, (int8_t)(room_rssi(i) + (t % 3) - 1));
        vec_add(&v, 900, -50);                         // anchor's own MAC, close
        ProxScoreResult r = prox_compute_score(&v);
        prox_maybe_update_fingerprint(&v, r);
        g_now_ms += 100;
        for (int i = 0; i < 30; ++i) anchor_hears(i, room_rssi(i));
    }

    ProxScanVector full; memset(&full, 0, sizeof(full));
    for (int i = 0; i < 30; ++i) vec_add(&full, i, room_rssi(i));
    ProxScoreResult r_full = prox_compute_score(&full);

    ProxScanVector sparse; memset(&sparse, 0, sizeof(sparse));
    for (int i = 0; i < 30; ++i) if (i % 3) vec_add(&sparse, i, room_rssi(i));
    ProxScoreResult r_sparse = prox_compute_score(&sparse);

    printf("    full=%d (fp seen %d)  sparse=%d\n",
           (int)r_full.score, prox_last_fp_seen(), (int)r_sparse.score);
    int drop = (int)r_full.score - (int)r_sparse.score;
    CHECK(drop < 40,
          "dropping a third of the devices must not move the score %d points", drop);
}

// ---------------------------------------------------------------- registry

// A public (non-rotating) address: top two bits of the MSB are not 0b01.
static void mk_pub(int i, uint8_t out[6]) {
    out[0] = 0x00; out[1] = 0xAB; out[2] = 0xCD;
    out[3] = 0x00; out[4] = (uint8_t)(i >> 8); out[5] = (uint8_t)(i & 0xFF);
}
// A resolvable private address: top two bits of the MSB are 0b01.
static void mk_rpa(int i, uint8_t out[6]) {
    out[0] = 0x40; out[1] = 0x11; out[2] = 0x22;
    out[3] = 0x00; out[4] = (uint8_t)(i >> 8); out[5] = (uint8_t)(i & 0xFF);
}

static void hear(void (*mk)(int, uint8_t[6]), int idx, int8_t rssi) {
    uint8_t mac[6]; mk(idx, mac);
    prox_ingest_scan_result(mac, PROX_TYPE_BLE, rssi);
}

static void test_registry_eviction(void) {
    begin("registry: a saturated registry still admits live devices");
    // The field failure. Without eviction the registry filled to its cap and
    // then refused every new device forever; with address rotation the anchor
    // saturated with dead MACs in minutes and went blind to its own room,
    // sharing 0-4 devices with a watch beside it reporting 29.
    prox_init();
    for (int i = 0; i < ANCHOR_PROX_MAX_FINGERPRINT_DEVICES; ++i)
        hear(mk_rpa, i, -70);                       // fill with rotating addresses

    g_now_ms += ANCHOR_PROX_DEVICE_STALE_MS + 1000; // they all rotate away

    for (int i = 0; i < 30; ++i) hear(mk_pub, i, -60);   // a fresh live population

    ProxScanVector v; memset(&v, 0, sizeof(v));
    for (int i = 0; i < 30; ++i) {
        if (v.count >= PROX_MAX_DEVICES) break;
        mk_pub(i, v.devices[v.count].mac);
        v.devices[v.count].type = PROX_TYPE_BLE;
        v.devices[v.count].rssi = (int8_t)(-60 - (i % 7));
        v.count++;
    }
    prox_compute_score(&v);
    printf("    shared after saturation: %d of %d\n", prox_last_shared_count(), (int)v.count);
    CHECK(prox_last_shared_count() >= 25,
          "a saturated registry must recycle dead slots; shared=%d",
          prox_last_shared_count());

    begin("registry: live devices are never evicted to make room");
    prox_init();
    for (int i = 0; i < 40; ++i) hear(mk_pub, i, -55);          // keep these fresh
    for (int i = 0; i < ANCHOR_PROX_MAX_FINGERPRINT_DEVICES + 60; ++i)
        hear(mk_rpa, 1000 + i, -85);                            // churn
    for (int i = 0; i < 40; ++i) hear(mk_pub, i, -55);          // refresh the keepers

    ProxScanVector w; memset(&w, 0, sizeof(w));
    for (int i = 0; i < 40; ++i) {
        if (w.count >= PROX_MAX_DEVICES) break;
        mk_pub(i, w.devices[w.count].mac);
        w.devices[w.count].type = PROX_TYPE_BLE;
        w.devices[w.count].rssi = (int8_t)(-55 - (i % 5));
        w.count++;
    }
    prox_compute_score(&w);
    printf("    live devices retained through churn: %d of 40\n", prox_last_shared_count());
    CHECK(prox_last_shared_count() >= 35,
          "churn must not evict live devices; shared=%d", prox_last_shared_count());
}

// ---------------------------------------------------------------- calibration

static void fill_vec(ProxScanVector* v, int n, int8_t (*rssi)(int)) {
    memset(v, 0, sizeof(*v));
    for (int i = 0; i < n; ++i) vec_add(v, i, rssi(i));
}
// EDGE viewpoint: same room, but a larger and differently-shaped offset, so the
// correlation with the anchor's view is materially worse than from INSIDE.
static int8_t edge_rssi(int idx) {
    int off = ((idx * 97) % 29) - 14;
    return (int8_t)(room_rssi(idx) - 18 + off);
}

// Feed a run of scores straight into one histogram leg.
static void feed_leg(int is_inside, const uint8_t* scores, int n) {
    for (int i = 0; i < n; ++i) prox_calib_add(is_inside, scores[i]);
}

static ProxCalibStats run_calib(const uint8_t* in, int n_in, const uint8_t* ed, int n_ed) {
    prox_calib_reset();
    feed_leg(1, in, n_in);
    feed_leg(0, ed, n_ed);
    prox_calib_finalize(NULL, NULL, NULL);
    ProxCalibStats s;
    memset(&s, 0, sizeof(s));
    prox_calib_last_stats(&s);
    return s;
}

// The anchor's own beacon, as the watch hears it. Index 900 is reserved for it
// so it never collides with the room's emitters.
#define SELF_IDX 900
static void set_self_mac(void) {
    uint8_t mac[6]; mk_mac(SELF_IDX, mac);
    prox_set_self_mac(mac);
}

// One demonstrated calibration sample: the room as the watch sees it, plus the
// anchor's own beacon at `self_dbm`.
static void calib_sample(int is_inside, int8_t self_dbm) {
    ProxScanVector v; v.count = 0;
    for (int i = 0; i < 24; ++i) vec_add(&v, i, watch_rssi(i));
    vec_add(&v, SELF_IDX, self_dbm);
    prox_calib_collect(&v, is_inside);
}

static void test_self_rssi_term(void) {
    begin("self-RSSI: abstains entirely until calibration teaches it two levels");
    prox_init();
    set_self_mac();
    for (int i = 0; i < 30; ++i) anchor_hears(i, room_rssi(i));
    anchor_hears(SELF_IDX, -40);

    ProxScanVector v; v.count = 0;
    for (int i = 0; i < 24; ++i) vec_add(&v, i, watch_rssi(i));
    vec_add(&v, SELF_IDX, -80);
    const uint8_t bare = prox_compute_score(&v).score;
    CHECK(prox_last_self_delta() == 0,
          "an uncalibrated anchor must not apply the term, delta=%d",
          prox_last_self_delta());
    CHECK(prox_self_levels(NULL, NULL) == 0, "levels must report unarmed");

    begin("self-RSSI: calibration learns the two demonstrated levels");
    prox_calib_reset();
    for (int i = 0; i < 12; ++i) calib_sample(1, (int8_t)(-80 + (i % 3) - 1));
    for (int i = 0; i < 10; ++i) calib_sample(0, (int8_t)(-92 + (i % 3) - 1));
    prox_calib_finalize(NULL, NULL, NULL);

    int8_t near_dbm = 0, away_dbm = 0;
    CHECK(prox_self_levels(&near_dbm, &away_dbm) == 1, "term should be armed after calibration");
    CHECK(near_dbm >= -82 && near_dbm <= -78, "near level ~-80, got %d", near_dbm);
    CHECK(away_dbm >= -94 && away_dbm <= -90, "away level ~-92, got %d", away_dbm);

    begin("self-RSSI: strong beacon raises the score, weak lowers it");
    ProxScanVector vn; vn.count = 0;
    for (int i = 0; i < 24; ++i) vec_add(&vn, i, watch_rssi(i));
    vec_add(&vn, SELF_IDX, -80);
    const uint8_t s_near = prox_compute_score(&vn).score;
    const int d_near = prox_last_self_delta();

    ProxScanVector vf; vf.count = 0;
    for (int i = 0; i < 24; ++i) vec_add(&vf, i, watch_rssi(i));
    vec_add(&vf, SELF_IDX, -92);
    const uint8_t s_far = prox_compute_score(&vf).score;
    const int d_far = prox_last_self_delta();

    CHECK(d_near > 0 && d_far < 0,
          "the term must push both ways (near %d, far %d)", d_near, d_far);
    CHECK(s_near > s_far,
          "identical rooms, 12 dB apart on the anchor's own beacon, must score "
          "differently (%u vs %u)", s_near, s_far);
    printf("    same room, self -80 vs -92: score %u -> %u (delta %+d / %+d), "
           "bare was %u\n", s_near, s_far, d_near, d_far, bare);

    begin("self-RSSI: the AWAY direction is capped at half the NEAR direction");
    // §13.0: attenuation must never buy as much as proximity. Drive the beacon
    // far past the demonstrated away level — the classic occlusion signature.
    ProxScanVector vo; vo.count = 0;
    for (int i = 0; i < 24; ++i) vec_add(&vo, i, watch_rssi(i));
    vec_add(&vo, SELF_IDX, -120);
    prox_compute_score(&vo);
    const int d_occluded = prox_last_self_delta();

    ProxScanVector vc; vc.count = 0;
    for (int i = 0; i < 24; ++i) vec_add(&vc, i, watch_rssi(i));
    vec_add(&vc, SELF_IDX, -20);
    prox_compute_score(&vc);
    const int d_close = prox_last_self_delta();

    CHECK(d_occluded == -(int)PROX_SELF_DOWN_MAX_U8,
          "a smothered beacon must clamp at -%d, got %d",
          PROX_SELF_DOWN_MAX_U8, d_occluded);
    CHECK(d_close == (int)PROX_SELF_UP_MAX_U8,
          "a very strong beacon must clamp at +%d, got %d",
          PROX_SELF_UP_MAX_U8, d_close);
    CHECK(-d_occluded * 2 == d_close,
          "attenuation must buy exactly half what proximity does (%d vs %d)",
          -d_occluded, d_close);

    begin("self-RSSI: an absent beacon abstains rather than reading as far");
    ProxScanVector va; va.count = 0;
    for (int i = 0; i < 24; ++i) vec_add(&va, i, watch_rssi(i));   // no SELF_IDX
    prox_compute_score(&va);
    CHECK(prox_last_self_delta() == 0,
          "not hearing the anchor is not evidence of distance, delta=%d",
          prox_last_self_delta());

    begin("self-RSSI: two positions that do not differ by level leave the term off");
    prox_init();
    set_self_mac();
    for (int i = 0; i < 30; ++i) anchor_hears(i, room_rssi(i));
    prox_calib_reset();
    for (int i = 0; i < 12; ++i) calib_sample(1, -80);
    for (int i = 0; i < 10; ++i) calib_sample(0, -78);   // EDGE stronger: nonsense
    prox_calib_finalize(NULL, NULL, NULL);
    CHECK(prox_self_levels(NULL, NULL) == 0,
          "an inverted or too-narrow span must not arm the term");
}

static void test_calib_fallback_threshold(void) {
    begin("calib fallback: an overlapping demo falls back to the measured best "
          "cutoff, not the global default");
    // The field case: every score sat ABOVE the global default of 170, so the
    // fallback classified everything as NEAR.
    const uint8_t in[] = {172,180,188,196,204,212,196,188,204,196};
    const uint8_t ed[] = {156,164,172,180,188,196,180,172,188,180};
    ProxCalibStats s = run_calib(in, 10, ed, 10);
    CHECK(s.fail & PROX_CALIB_FAIL_OVERLAP, "this demo overlaps, fail=0x%02X", s.fail);
    CHECK(s.threshold == s.best_thr,
          "fallback should be the measured best cutoff %u, got %u",
          s.best_thr, s.threshold);
    CHECK(s.threshold > PROX_CONFIDENCE_THRESHOLD_U8,
          "and for this score scale it must sit above the global %d, got %u",
          PROX_CONFIDENCE_THRESHOLD_U8, s.threshold);
    CHECK(s.confidence == 0, "it is still a degraded threshold, conf=%u", s.confidence);
    printf("    overlap fallback: thr=%u (global default would be %d), err=%u%%\n",
           s.threshold, PROX_CONFIDENCE_THRESHOLD_U8, s.best_err_pct);
}

// A vector taken somewhere else in the same room: same emitters, different
// shadowing realisation, which is what crossing a room actually does.
static int8_t far_rssi(int idx) {
    int off = ((idx * 61) % 19) - 9;
    return (int8_t)(room_rssi(idx) - 6 + off);
}

static void test_score_scale_stability(void) {
    // Field-reported symptom: live INSIDE queries logged 200-239, but the
    // FINALIZE histogram built from those same vectors reported mean 153. The
    // cause was in Signal B's mapping, and it was worse than an offset — the
    // score had NO STABLE SCALE. signal_fingerprint() averaged the Gaussian
    // LOG-DENSITY, whose normalisation term -0.5*ln(var) depends on the
    // fingerprint's width rather than on the match, so a perfect match capped at
    // L = 0.67 (sigma 2 dB) or 0.47 (sigma 10 dB). As alpha handed control from
    // Signal A to Signal B, every score was dragged toward L*255 regardless of
    // where the watch was: one unchanged NEAR vector measured 245 / 211 / 153 /
    // 138 at 0 / 16 / 80 / 160 training samples.
    begin("score scale: a NEAR vector does not decay as the fingerprint matures");

    prox_init();
    for (int i = 0; i < 60; ++i) anchor_hears(i, room_rssi(i));

    ProxScanVector nv; nv.count = 0;
    for (int i = 0; i < 31; ++i) vec_add(&nv, i, watch_rssi(i));
    ProxScanVector fv; fv.count = 0;
    for (int i = 0; i < 31; ++i) vec_add(&fv, i, far_rssi(i));

    const int near0 = prox_compute_score(&nv).score;
    const int far0  = prox_compute_score(&fv).score;

    for (int round = 0; round < 8; ++round)
        for (int s = 0; s < 16; ++s) prox_train_labeled(&nv, 1);

    const int near1 = prox_compute_score(&nv).score;
    const int far1  = prox_compute_score(&fv).score;

    printf("    NEAR %d -> %d | FAR %d -> %d | gap %d -> %d (128 training samples)\n",
           near0, near1, far0, far1, near0 - far0, near1 - far1);

    CHECK(near1 > 180,
          "a NEAR vector must stay NEAR as its own fingerprint matures, fell to %d", near1);
    CHECK(near0 - near1 < 60,
          "the score scale drifted by %d points with no physical change", near0 - near1);
    CHECK((near1 - far1) >= (near0 - far0),
          "training on NEAR must not SHRINK the NEAR/FAR gap (%d -> %d)",
          near0 - far0, near1 - far1);
    CHECK(far1 < near1 - 60,
          "a fingerprint trained on NEAR should push FAR well down (near %d, far %d)",
          near1, far1);
}

static void test_calib_diagnostics(void) {
    // The diagnostics exist to tell apart two failures that look identical from
    // the app: "the percentile rule refused" vs "the score never separated these
    // two spots". Those have opposite fixes, so the distinction has to be right.
    begin("calib diagnostics: a clean demonstration is accepted and reports it");
    {
        const uint8_t in[] = {200,204,196,208,200,204,200,196,204,200};
        const uint8_t ed[] = {100,104,96,108,100,104,100,96,104,100};
        ProxCalibStats s = run_calib(in, 10, ed, 10);
        CHECK(s.fail == 0, "a clean demo must be accepted, fail=0x%02X", s.fail);
        CHECK(s.inside_mean > s.edge_mean + 80,
              "means should be ~100 apart, got %u vs %u", s.inside_mean, s.edge_mean);
        CHECK(s.best_err_pct == 0, "a clean demo has a perfect cutoff, got %u%%",
              s.best_err_pct);
        CHECK(s.dprime_x10 >= 20, "clean separation should give d' >= 2.0, got %u.%u",
              s.dprime_x10 / 10, s.dprime_x10 % 10);
        printf("    clean: d'=%u.%u best_thr=%u err=%u%% gap=%d\n",
               s.dprime_x10 / 10, s.dprime_x10 % 10, s.best_thr, s.best_err_pct, s.gap);
    }

    begin("calib diagnostics: separable-but-refused is distinguishable from not-separable");
    {
        // Two tight clouds, but one stray sample in each tail — exactly what the
        // conservative p10/p90 rule refuses even though a cutoff obviously exists.
        const uint8_t in[] = {200,204,196,208,200,204,200,196,204,100};
        const uint8_t ed[] = {100,104,96,108,100,104,100,96,104,200};
        ProxCalibStats s = run_calib(in, 10, ed, 10);
        CHECK(s.fail & PROX_CALIB_FAIL_OVERLAP,
              "tail outliers should trip the overlap rule, fail=0x%02X", s.fail);
        CHECK(s.best_err_pct <= 10,
              "but a good cutoff still exists; best error %u%%", s.best_err_pct);
        CHECK(s.dprime_x10 >= 15,
              "and the clouds are still well separated, d'=%u.%u",
              s.dprime_x10 / 10, s.dprime_x10 % 10);
        printf("    outliers: d'=%u.%u best_thr=%u err=%u%% gap=%d\n",
               s.dprime_x10 / 10, s.dprime_x10 % 10, s.best_thr, s.best_err_pct, s.gap);
    }

    begin("calib diagnostics: genuinely inseparable clouds report low d' and high error");
    {
        // Both legs drawn from the same place — the failure mode where the score
        // simply does not distinguish the two positions.
        const uint8_t in[] = {150,160,140,155,145,165,135,150,158,142};
        const uint8_t ed[] = {148,158,138,152,142,162,132,148,156,140};
        ProxCalibStats s = run_calib(in, 10, ed, 10);
        CHECK(s.fail & PROX_CALIB_FAIL_OVERLAP,
              "overlapping clouds must fail, fail=0x%02X", s.fail);
        CHECK(s.dprime_x10 < 10,
              "d' must expose the lack of separation, got %u.%u",
              s.dprime_x10 / 10, s.dprime_x10 % 10);
        CHECK(s.best_err_pct >= 20,
              "and no cutoff should work well; best error %u%%", s.best_err_pct);
        printf("    inseparable: d'=%u.%u best_thr=%u err=%u%%\n",
               s.dprime_x10 / 10, s.dprime_x10 % 10, s.best_thr, s.best_err_pct);
    }

    begin("calib diagnostics: starvation is reported per leg, not as overlap");
    {
        const uint8_t in[] = {200,204,196,208,200,204};
        const uint8_t ed[] = {100,104};
        ProxCalibStats s = run_calib(in, 6, ed, 2);
        CHECK(s.fail == PROX_CALIB_FAIL_FEW_EDGE,
              "only the EDGE leg is starved, fail=0x%02X", s.fail);
        CHECK(s.edge_n == 2 && s.inside_n == 6,
              "counts should survive into the stats (%u/%u)", s.inside_n, s.edge_n);
    }
}

static void test_deferred_calibration(void) {
    begin("calibration: buffered scoring is independent of arrival order");
    // The invariant that matters. Under the old live-score path the INSIDE leg
    // trained the fingerprint it was being scored by, so a sample's score
    // depended on how many samples preceded it — a 55-point drift within one leg
    // on hardware. Deferred scoring must make the threshold a function of the
    // demonstrated vectors alone, not of the order they arrived in.
    uint8_t thr[2] = {0, 0}; uint8_t conf[2] = {0, 0};
    for (int order = 0; order < 2; ++order) {
        prox_init();
        for (int i = 0; i < 30; ++i) anchor_hears(i, room_rssi(i));
        prox_calib_reset();

        ProxScanVector in_v, ed_v;
        fill_vec(&in_v, 30, watch_rssi);
        fill_vec(&ed_v, 30, edge_rssi);

        if (order == 0) {                        // all INSIDE, then all EDGE
            for (int i = 0; i < 10; ++i) prox_calib_collect(&in_v, 1);
            for (int i = 0; i < 10; ++i) prox_calib_collect(&ed_v, 0);
        } else {                                 // interleaved
            for (int i = 0; i < 10; ++i) {
                prox_calib_collect(&ed_v, 0);
                prox_calib_collect(&in_v, 1);
            }
        }
        uint16_t a = 0, b = 0;
        thr[order] = prox_calib_finalize(&a, &b, &conf[order]);
    }
    printf("    sequential thr=%u conf=%u | interleaved thr=%u conf=%u\n",
           thr[0], conf[0], thr[1], conf[1]);
    CHECK(thr[0] == thr[1], "threshold must not depend on arrival order (%u vs %u)",
          thr[0], thr[1]);
    CHECK(conf[0] == conf[1], "confidence must not depend on arrival order (%u vs %u)",
          conf[0], conf[1]);

    begin("calibration: a clean demonstration is accepted");
    CHECK(conf[0] > 0, "clearly-separated INSIDE/EDGE vectors should calibrate, conf=%u",
          conf[0]);
    CHECK(thr[0] != PROX_CONFIDENCE_THRESHOLD_U8,
          "an accepted calibration must not return the global default");

    begin("calibration: the reservoir caps retention but counts every sample offered");
    prox_init();
    for (int i = 0; i < 30; ++i) anchor_hears(i, room_rssi(i));
    prox_calib_reset();
    ProxScanVector v; fill_vec(&v, 30, watch_rssi);
    for (int i = 0; i < 100; ++i) prox_calib_collect(&v, 1);
    CHECK(prox_calib_collected(1) == 100, "offered count should be 100, got %d",
          prox_calib_collected(1));
    uint16_t in_n = 0, ed_n = 0; uint8_t c = 0;
    prox_calib_finalize(&in_n, &ed_n, &c);
    CHECK(in_n == PROX_CALIB_BUF_SAMPLES,
          "histogram should hold the retained %d, got %u", PROX_CALIB_BUF_SAMPLES, in_n);

    begin("calibration: buffered vectors take precedence over live-score adds");
    // A caller that does both must not get a mixture of the two scales.
    prox_init();
    for (int i = 0; i < 30; ++i) anchor_hears(i, room_rssi(i));
    prox_calib_reset();
    for (int i = 0; i < 8; ++i) prox_calib_add(1, 250);   // implausible live scores
    for (int i = 0; i < 8; ++i) prox_calib_add(0, 5);
    ProxScanVector iv, ev;
    fill_vec(&iv, 30, watch_rssi);
    fill_vec(&ev, 30, edge_rssi);
    for (int i = 0; i < 8; ++i) { prox_calib_collect(&iv, 1); prox_calib_collect(&ev, 0); }
    uint16_t i2 = 0, e2 = 0; uint8_t c2 = 0;
    prox_calib_finalize(&i2, &e2, &c2);
    CHECK(i2 == 8 && e2 == 8,
          "histograms must be rebuilt from the 8+8 buffered vectors, got %u/%u", i2, e2);

    begin("calibration: reset clears the buffers too");
    prox_calib_reset();
    CHECK(prox_calib_collected(0) == 0 && prox_calib_collected(1) == 0,
          "reset must clear the offered counters");
    uint16_t i3 = 0, e3 = 0; uint8_t c3 = 0;
    prox_calib_finalize(&i3, &e3, &c3);
    CHECK(i3 == 0 && e3 == 0, "nothing collected ⇒ empty histograms");
    CHECK(c3 == 0, "nothing collected ⇒ no confidence");
}

// ----------------------------------------------------------------------------

// §13.4-R1: "truncation is not distance."
//
// The starved-vector branch used to return score 50 — a confident AWAY — from a
// single raw RSSI comparison. Attenuation censors the weakest emitters first, so
// smothering the watch is the cheapest possible way to push the vector under
// PROX_MIN_DEVICE_COUNT, and the branch then bypassed the entire correlation
// architecture in favour of the one statistic the attack directly defeats. The
// attack never had to beat Signal A; it only had to starve it. Field vectors
// already measure 8..31 devices, so the floor sits within one device of normal
// operation, and a human torso is 15-30 dB.
static void test_starved_vector_not_away(void) {
    begin("starved vector: too few devices is an absent measurement, not a far one");

    prox_init();
    uint8_t self[6]; mk_mac(900, self);
    prox_set_self_mac(self);
    for (int i = 0; i < 20; ++i) anchor_hears(i, room_rssi(i));

    // Four devices, none of them the anchor: nothing can be measured.
    ProxScanVector v; v.count = 0;
    for (int i = 0; i < 4; ++i) vec_add(&v, i, watch_rssi(i));
    ProxScoreResult r = prox_compute_score(&v);

    CHECK(r.flags & PROX_FLAG_LOW_DEVICE_COUNT, "the low-count flag must be set");
    CHECK(r.score == PROX_STARVED_SCORE,
          "a starved vector must score the criterion-neutral %d, not a confident "
          "AWAY (got %d)", (int)PROX_STARVED_SCORE, (int)r.score);
    CHECK(r.score > 50,
          "regression: the old score-50 branch is back (score=%d)", (int)r.score);

    // The asymmetry that must survive (§13.0): a strong own-anchor RSSI cannot be
    // manufactured by an attenuating adversary, so near-evidence from a thin
    // vector is still trustworthy even though far-evidence is not.
    ProxScanVector n; n.count = 0;
    for (int i = 0; i < 3; ++i) vec_add(&n, i, watch_rssi(i));
    if (n.count < PROX_MAX_DEVICES) {
        memcpy(n.devices[n.count].mac, self, 6);
        n.devices[n.count].type = PROX_TYPE_BLE;
        n.devices[n.count].rssi = (int8_t)(ANCHOR_NEAR_RSSI_THRESHOLD_DBM + 10);
        n.count++;
    }
    ProxScoreResult rn = prox_compute_score(&n);
    CHECK(rn.score == 200,
          "a thin vector with a strong own-anchor RSSI must still read NEAR "
          "(got %d) — concluding far needs more evidence than concluding near",
          (int)rn.score);
}

// ── §13.4-R2: offset-invariant Signal B ─────────────────────────────────────
//
// The defect these pin: Signal B scored ABSOLUTE dBm, so a duvet — which
// attenuates every emitter by roughly the same amount — was arithmetically
// indistinguishable from having walked out of the room. Worse, alpha hands
// Signal B more authority as the fingerprint matures, so training the anchor
// made the attack EASIER. These tests train hard first, precisely because that
// is the regime where the old code failed worst.

// A fresh observation of the same spot: the trained mean plus per-device noise.
// Scoring the exact mean makes every residual identical and the dispersion
// measures degenerate, which would flatter the common-mode estimator.
static void vec_jitter(ProxScanVector* v, int seed) {
    for (int i = 0; i < v->count; ++i) {
        const int n = ((i * 11 + seed * 5) % 7) - 3;      // deterministic +/-3 dB
        v->devices[i].rssi = (int8_t)(v->devices[i].rssi + n);
    }
}

// Everything the watch hears, attenuated by `db`. What a blanket does.
static void vec_attenuate(ProxScanVector* v, int db) {
    for (int i = 0; i < v->count; ++i) {
        int r = (int)v->devices[i].rssi - db;
        if (r < -120) r = -120;
        v->devices[i].rssi = (int8_t)r;
    }
}

// Train on the NEAR spot with per-sample jitter, so the fingerprint ends up with
// a realistic width. Training on a byte-identical vector collapses sigma toward
// the floor, and a common-mode test run against a zero-width fingerprint proves
// much less than it appears to — every threshold here is relative to sigma.
static void train_near(ProxScanVector* nv, int samples) {
    for (int s = 0; s < samples; ++s) {
        ProxScanVector j = *nv;
        for (int i = 0; i < j.count; ++i) {
            const int n = ((i * 7 + s * 13) % 9) - 4;      // deterministic +/-4 dB
            j.devices[i].rssi = (int8_t)(j.devices[i].rssi + n);
        }
        prox_train_labeled(&j, 1);
    }
}

static void test_r2_uniform_attenuation(void) {
    begin("R2: a uniform attenuation does not move the score");
    prox_init();
    for (int i = 0; i < 60; ++i) anchor_hears(i, room_rssi(i));

    ProxScanVector nv; nv.count = 0;
    for (int i = 0; i < 31; ++i) vec_add(&nv, i, watch_rssi(i));
    train_near(&nv, 128);          // mature fingerprint: alpha now favours B

    ProxScanVector obs = nv;
    vec_jitter(&obs, 3);
    const int clear = prox_compute_score(&obs).score;
    {
        float mad0 = 0, sigma0 = 0;
        prox_last_common_mode(NULL, &mad0, &sigma0, NULL, NULL, NULL);
        printf("    unattenuated baseline: score %d, mad %.1f, sigma %.1f\n",
               clear, mad0, sigma0);
        CHECK(mad0 > 0.0f, "the baseline must have real dispersion to compare against");
    }

    // The attack, at three strengths. A torso is 15-30 dB; a duvet less.
    const int levels[] = {8, 12, 20};
    for (int k = 0; k < 3; ++k) {
        ProxScanVector av = obs;
        vec_attenuate(&av, levels[k]);
        const int covered = prox_compute_score(&av).score;

        float delta = 0, mad = 0, sigma = 0, legacy = 0, invariant = 0;
        int n = 0;
        prox_last_common_mode(&delta, &mad, &sigma, &n, &legacy, &invariant);

        printf("    -%2d dB: score %d -> %d | delta %+.1f mad %.1f sigma %.1f "
               "| L legacy %.2f invariant %.2f\n",
               levels[k], clear, covered, delta, mad, sigma, legacy, invariant);

        CHECK(delta < -(float)levels[k] + 3.0f && delta > -(float)levels[k] - 3.0f,
              "the common-mode estimate should recover the attenuation "
              "(-%d dB), got %+.1f", levels[k], delta);
        CHECK(covered > clear - 25,
              "a %d dB blanket must not move the score (%d -> %d)",
              levels[k], clear, covered);
        CHECK(invariant > legacy + 0.05f,
              "the invariant scoring must beat the legacy one under attenuation "
              "(legacy %.2f, invariant %.2f)", legacy, invariant);
    }
}

static void test_r2_differential_still_reads_far(void) {
    begin("R2: removing the common mode does not blind the score to real moves");
    // The risk of the fix: subtract too much and everything reads NEAR forever.
    // A genuine displacement changes emitters by DIFFERENT amounts, so it must
    // survive the subtraction.
    prox_init();
    for (int i = 0; i < 60; ++i) anchor_hears(i, room_rssi(i));

    ProxScanVector nv; nv.count = 0;
    for (int i = 0; i < 31; ++i) vec_add(&nv, i, watch_rssi(i));
    train_near(&nv, 128);
    const int near_score = prox_compute_score(&nv).score;

    ProxScanVector fv; fv.count = 0;
    for (int i = 0; i < 31; ++i) vec_add(&fv, i, far_rssi(i));
    const int far_score = prox_compute_score(&fv).score;

    float delta = 0, mad = 0, sigma = 0;
    int n = 0;
    prox_last_common_mode(&delta, &mad, &sigma, &n, NULL, NULL);
    printf("    NEAR %d | FAR %d (gap %d) | far vector: delta %+.1f mad %.1f "
           "sigma %.1f\n", near_score, far_score, near_score - far_score,
           delta, mad, sigma);

    CHECK(far_score < near_score - 40,
          "a genuinely different place must still score far below NEAR "
          "(near %d, far %d)", near_score, far_score);
    CHECK(mad > sigma,
          "displacement must leave residual dispersion above the trained width "
          "(mad %.1f, sigma %.1f)", mad, sigma);
}

static void test_r2_median_resists_a_minority(void) {
    begin("R2: a few devices that really changed do not drag the estimate");
    // Why the median and not the mean: your phone leaving the room with you is a
    // handful of devices changing hugely while the rest are unchanged. The
    // common-mode estimate must describe the ROOM, not the outliers.
    prox_init();
    for (int i = 0; i < 60; ++i) anchor_hears(i, room_rssi(i));

    ProxScanVector nv; nv.count = 0;
    for (int i = 0; i < 31; ++i) vec_add(&nv, i, watch_rssi(i));
    train_near(&nv, 128);

    ProxScanVector mv = nv;
    for (int i = 0; i < 5; ++i) mv.devices[i].rssi = (int8_t)-110;   // 5 of 31

    prox_compute_score(&mv);
    float delta = 0;
    prox_last_common_mode(&delta, NULL, NULL, NULL, NULL, NULL);
    printf("    5 of 31 devices collapsed: delta %+.1f\n", delta);
    CHECK(delta > -4.0f && delta < 4.0f,
          "a minority of changed devices must not move the common-mode estimate, "
          "got %+.1f", delta);
}

static void test_r2_common_mode_flag(void) {
    begin("R2: the occlusion signature is flagged, displacement is not");
    prox_init();
    for (int i = 0; i < 60; ++i) anchor_hears(i, room_rssi(i));

    ProxScanVector nv; nv.count = 0;
    for (int i = 0; i < 31; ++i) vec_add(&nv, i, watch_rssi(i));
    train_near(&nv, 128);

    ProxScanVector obs = nv;
    vec_jitter(&obs, 3);
    CHECK((prox_compute_score(&obs).flags & PROX_FLAG_COMMON_MODE) == 0,
          "an unchanged vector must not be flagged");

    ProxScanVector av = obs;
    vec_attenuate(&av, 15);
    CHECK((prox_compute_score(&av).flags & PROX_FLAG_COMMON_MODE) != 0,
          "a 15 dB block shift with the shape intact is the occlusion signature");

    ProxScanVector fv; fv.count = 0;
    for (int i = 0; i < 31; ++i) vec_add(&fv, i, far_rssi(i));
    CHECK((prox_compute_score(&fv).flags & PROX_FLAG_COMMON_MODE) == 0,
          "a genuine displacement must not be flagged as occlusion");
}

static void test_r2_self_rssi_correction(void) {
    begin("R2: the anchor's own beacon is corrected for the common mode");
    // The self term is the strongest near/far discriminator the anchor has, and
    // a duvet attenuates it exactly like everything else. With the common-mode
    // shift known, the part of a weak reading that the whole room also suffered
    // is walked back — so bedding no longer buys far-evidence from this term.
    prox_init();
    set_self_mac();
    for (int i = 0; i < 40; ++i) anchor_hears(i, room_rssi(i));
    anchor_hears(SELF_IDX, -40);

    prox_calib_reset();
    for (int i = 0; i < 12; ++i) calib_sample(1, (int8_t)(-80 + (i % 3) - 1));
    for (int i = 0; i < 10; ++i) calib_sample(0, (int8_t)(-92 + (i % 3) - 1));
    prox_calib_finalize(NULL, NULL, NULL);

    ProxScanVector nv; nv.count = 0;
    for (int i = 0; i < 28; ++i) vec_add(&nv, i, watch_rssi(i));
    vec_add(&nv, SELF_IDX, -80);              // demonstrated NEAR level
    train_near(&nv, 64);
    prox_compute_score(&nv);
    const int d_clear = prox_last_self_delta();

    // Same spot, under a blanket: the whole vector including the anchor's own
    // beacon drops 12 dB, which lands the self reading on the demonstrated AWAY
    // level. Uncorrected this is a confident vote for FAR.
    ProxScanVector cv = nv;
    vec_attenuate(&cv, 12);
    prox_compute_score(&cv);
    const int d_covered = prox_last_self_delta();

    printf("    self delta: clear %+d, under 12 dB blanket %+d\n",
           d_clear, d_covered);
    CHECK(d_covered >= 0,
          "a blanket must not turn the self term into far-evidence, got %+d",
          d_covered);

    begin("R2: the self correction is one-directional");
    // §13.0. A NEGATIVE common mode is walked back, because that is what an
    // attenuating adversary manufactures. A POSITIVE one must not be, because
    // walking that back would make concluding FAR cheaper — and concluding FAR
    // must never get cheaper.
    ProxScanVector uv = nv;
    vec_attenuate(&uv, -10);                  // everything 10 dB STRONGER
    prox_compute_score(&uv);
    float delta_up = 0;
    prox_last_common_mode(&delta_up, NULL, NULL, NULL, NULL, NULL);
    const int d_up = prox_last_self_delta();
    printf("    everything +10 dB: delta %+.1f, self term %+d (clear was %+d)\n",
           delta_up, d_up, d_clear);
    CHECK(delta_up > 6.0f, "the estimate should see the +10 dB, got %+.1f", delta_up);
    CHECK(d_up >= d_clear,
          "a positive common mode must not be subtracted away (%+d vs clear %+d)",
          d_up, d_clear);
}

int main(void) {
    printf("proximity engine — anchor-side Mode A scoring tests\n\n");

    test_degenerate_k();
    test_absent_fingerprint_devices();
    test_registry_eviction();
    test_deferred_calibration();
    test_score_scale_stability();
    test_calib_diagnostics();
    test_self_rssi_term();
    test_calib_fallback_threshold();
    test_starved_vector_not_away();
    test_r2_uniform_attenuation();
    test_r2_differential_still_reads_far();
    test_r2_median_resists_a_minority();
    test_r2_common_mode_flag();
    test_r2_self_rssi_correction();

    printf("\n%d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
