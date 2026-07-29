// Host-side acceptance tests for proximity engine v2.1 Phase 1.
//
// Covers the four acceptance tests named in FIRMWARE_SPEC v0.9 amendment §10-A
// Phase P1 (frozen-fade, walk-approach, teleport-rejection, IMU-stale) plus unit
// coverage of the pieces they depend on: the fixed-point LUT math, the motion
// classifier, and the motion-gated integrator.
//
// Pure host build — no hardware, no Arduino, no test framework. The platform
// seam is stubbed with a controllable clock and an in-memory NVS.
//
//   cd proximity_engine/tests && make

#include "proximity.h"
#include "prox_luts.h"

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
    std::vector<uint8_t> v((const uint8_t*)buf, (const uint8_t*)buf + len);
    g_nvs[key] = v;
    return 1;
}
}

// ---------------------------------------------------------------- tiny harness

static int g_fail = 0;
static int g_checks = 0;
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

// ---------------------------------------------------------------- simulation aids

static void advance_ms(uint32_t ms) { g_now_ms += ms; }

// Synthetic accelerometer bursts (milli-g triples, gravity on Z).
static void feed_burst(int amp_mg, int period_samples) {
    int16_t xyz[IMU_BURST_SAMPLES][3];
    for (int i = 0; i < IMU_BURST_SAMPLES; ++i) {
        double phase = period_samples > 0 ? (2.0 * M_PI * i / period_samples) : 0.0;
        int16_t d = (int16_t)(amp_mg * (period_samples > 0 ? sin(phase) : ((i % 2) ? 1 : -1)));
        xyz[i][0] = d;
        xyz[i][1] = (int16_t)(d / 2);
        xyz[i][2] = (int16_t)(1000 + d / 3);   // 1 g of gravity plus motion
    }
    prox_ingest_imu_burst(xyz, IMU_BURST_SAMPLES, IMU_BURST_HZ);
}

static void feed_still(void)  { feed_burst(3, 0); }      // sensor noise only
static void feed_fidget(void) { feed_burst(60, 0); }     // desk work / typing
static void feed_loco(void)   { feed_burst(400, 12); }   // ~4 Hz sampled gait

// Map an RSSI to the score an anchor would plausibly return, so the tests can be
// written in the physical units the engine is reasoning about. Logistic centred
// on the anchor's near/away crossover.
static uint8_t score_from_rssi(double rssi_dbm, double crossover_dbm = -65.0) {
    double z = (rssi_dbm - crossover_dbm) / 4.0;
    double p = 1.0 / (1.0 + exp(-z));
    int s = (int)(p * 255.0 + 0.5);
    return (uint8_t)(s < 0 ? 0 : (s > 255 ? 255 : s));
}

static ProxScoreResult2 mk(uint8_t score, uint8_t near_thr = 0, uint8_t neff = 0) {
    ProxScoreResult2 r;
    r.score = score; r.flags = 0; r.neff = neff; r.near_thr = near_thr;
    return r;
}

// Deterministic pseudo-fading so traces are reproducible across runs.
static uint32_t g_rng = 12345;
static double frand_gauss(double sigma) {
    // Irwin-Hall approximation: sum of 12 uniforms, mean 6, variance 1.
    double s = 0;
    for (int i = 0; i < 12; ++i) { g_rng = g_rng * 1103515245u + 12345u; s += ((g_rng >> 16) & 0xFFFF) / 65536.0; }
    return (s - 6.0) * sigma;
}

// ---------------------------------------------------------------- LUT / fixed point

static void test_luts(void) {
    begin("fixed-point LUTs match their float definitions");

    for (int i = 0; i < 256; ++i) {
        double want = log((i + 0.5) / (255.5 - i)) * 256.0;
        CHECK(fabs(PROX_LUT_LOGIT8[i] - want) <= 1.0, "logit8[%d]=%d want %.1f", i, PROX_LUT_LOGIT8[i], want);
    }
    CHECK(PROX_LUT_LOGIT8[128] > -6 && PROX_LUT_LOGIT8[128] < 6, "logit8 is centred at score 128");

    CHECK(PROX_LUT_NEFF_GAIN[0] == 0, "no fresh draw ⇒ no evidence gain (got %u)", PROX_LUT_NEFF_GAIN[0]);
    CHECK(PROX_LUT_NEFF_GAIN[NEFF_TRAIN_MIN] == 256, "gain saturates at NEFF_TRAIN_MIN draws");
    for (int i = 1; i < 64; ++i)
        CHECK(PROX_LUT_NEFF_GAIN[i] >= PROX_LUT_NEFF_GAIN[i - 1], "neff gain monotone at %d", i);

    for (int k = 0; k < 64; ++k) {
        double want = 256.0 * log1p(exp(-(k * 32) / 256.0));
        CHECK(fabs(PROX_LUT_LSE_CORR[k] - want) <= 1.0, "lse_corr[%d]=%u want %.1f", k, PROX_LUT_LSE_CORR[k], want);
    }
}

// ---------------------------------------------------------------- motion channel

static void test_motion_classification(void) {
    begin("motion channel classifies STILL / FIDGET / LOCOMOTION");

    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    CHECK(prox_motion_state() == PROX_MOTION_UNKNOWN, "cold start is UNKNOWN, got %u", prox_motion_state());

    feed_still();
    CHECK(prox_motion_state() == PROX_MOTION_STILL, "quiet burst ⇒ STILL, got %u", prox_motion_state());

    feed_fidget();
    CHECK(prox_motion_state() == PROX_MOTION_FIDGET, "desk-work burst ⇒ FIDGET, got %u", prox_motion_state());

    feed_loco();
    CHECK(prox_motion_state() == PROX_MOTION_LOCOMOTION, "gait burst ⇒ LOCOMOTION, got %u", prox_motion_state());

    begin("awake IA1 firings promote out of STILL immediately");
    feed_still();
    prox_note_motion_interrupt();
    CHECK(prox_motion_state() == PROX_MOTION_FIDGET, "one interrupt leaves STILL, got %u", prox_motion_state());
    prox_note_motion_interrupt();
    CHECK(prox_motion_state() == PROX_MOTION_LOCOMOTION, "IMU_LOCO_MIN_INTS interrupts ⇒ LOCOMOTION, got %u",
          prox_motion_state());

    begin("a motionless sleep interval is authoritative STILL evidence");
    prox_hmm_reset(PROX_CRIT_STAY_NEAR);      // clears the interrupts logged above
    feed_still();
    advance_ms(180000);                       // slept through a whole met-tier poll
    prox_note_sleep_interval(180000, /*motion_woke=*/0);
    CHECK(prox_motion_state() == PROX_MOTION_STILL, "motionless sleep keeps STILL across the gap, got %u",
          prox_motion_state());
    prox_note_sleep_interval(60000, /*motion_woke=*/1);
    CHECK(prox_motion_state() != PROX_MOTION_STILL, "IA1-broken sleep cannot read STILL, got %u",
          prox_motion_state());

    begin("a stale IMU seam reads UNKNOWN (fails toward v0.8 behavior)");
    feed_still();
    advance_ms(IMU_STALE_MS + 1);
    CHECK(prox_motion_state() == PROX_MOTION_UNKNOWN, "stale ⇒ UNKNOWN, got %u", prox_motion_state());
}

// ---------------------------------------------------------------- integrator

static void test_integrator(void) {
    begin("integrator: N_eff freezes while STILL, advances while moving");

    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    ProxIntegrator it; prox_integ_reset(&it);

    feed_still();
    for (int i = 0; i < 40; ++i) { advance_ms(2000); feed_still(); prox_integ_update(&it, -70 * 256); }
    uint8_t neff_still = prox_integ_neff(&it);
    CHECK(neff_still <= 1, "80 s of still sampling is still ~1 draw, got %u", neff_still);

    feed_loco();
    for (int i = 0; i < 40; ++i) { advance_ms(2000); feed_loco(); prox_integ_update(&it, -70 * 256); }
    uint8_t neff_moving = prox_integ_neff(&it);
    CHECK(neff_moving > neff_still, "80 s of walking earns draws (%u > %u)", neff_moving, neff_still);
    CHECK(neff_moving >= 8, "80 s at NEFF_LOCO_PER_S should clear NEFF_TRAIN_MIN, got %u", neff_moving);

    begin("integrator: reported variance re-inflates toward a single draw while STILL");
    prox_integ_reset(&it);
    feed_loco();
    // Converge tightly on one value while moving, so the variance estimate collapses.
    for (int i = 0; i < 60; ++i) { advance_ms(1000); feed_loco(); prox_integ_update(&it, -70 * 256); }
    int32_t var_tight = 0; prox_integ_var_q8(&it, &var_tight);
    CHECK(var_tight < (INTEG_SINGLE_DRAW_VAR << 8) / 2, "converged variance is small, got %.1f dB^2",
          var_tight / 256.0);

    feed_still();
    for (int i = 0; i < 60; ++i) { advance_ms(2000); feed_still(); prox_integ_update(&it, -70 * 256); }
    int32_t var_relaxed = 0; prox_integ_var_q8(&it, &var_relaxed);
    CHECK(var_relaxed > var_tight, "variance re-inflates while still (%.1f -> %.1f dB^2)",
          var_tight / 256.0, var_relaxed / 256.0);
    CHECK(var_relaxed >= (INTEG_SINGLE_DRAW_VAR << 8) * 3 / 4,
          "...toward the single-draw variance, got %.1f dB^2", var_relaxed / 256.0);

    begin("integrator: STILL -> LOCOMOTION restarts the window");
    prox_integ_reset(&it);
    feed_still();
    for (int i = 0; i < 20; ++i) { advance_ms(2000); feed_still(); prox_integ_update(&it, -70 * 256); }
    advance_ms(2000); feed_loco(); prox_integ_update(&it, -50 * 256);
    int32_t mean = 0; prox_integ_mean_q8(&it, &mean);
    CHECK(mean == (-50 * 256), "stale still-window evidence is discarded, mean=%.1f dBm", mean / 256.0);
}

// ---------------------------------------------------------------- acceptance: P1

// Frozen-fade: the user walks to their desk beside a stayNear anchor, then sits
// motionless for a whole night in a spot that happens to be a deep null for that
// anchor. The v1 engine averaged the constant fade into a confident AWAY and
// alarmed — the exact failure this engine exists to fix.
//
// Note what is NOT claimed: the *first* observation of a window still moves the
// posterior, fade or not. A single fresh look is genuine evidence and there is
// nothing yet to contradict it. What must not happen is a still wrist turning
// one frozen look into hours of accumulating confidence.
static void test_frozen_fade(void) {
    begin("ACCEPTANCE frozen-fade: an 8 h still stream of deep-fade RSSI never flips");

    prox_hmm_reset(PROX_CRIT_STAY_NEAR);

    // Walk to the desk: real motion, real evidence, decision settles NEAR.
    feed_loco();
    for (int i = 0; i < 10; ++i) {
        advance_ms(3000); feed_loco();
        ProxScoreResult2 r = mk(score_from_rssi(-50.0));
        prox_hmm_tick(&r);
    }
    ProxDecision start = prox_hmm_decision();
    CHECK(start == PROX_HMM_NEAR, "setup failed to establish NEAR (lam=%d)", (int)prox_hmm_logodds_q8());

    // Sit down. The chair is in a null: every poll from here reads clearly AWAY.
    uint8_t deep_fade = score_from_rssi(-88.0);
    int flips = 0;
    for (int i = 0; i < 480; ++i) {                  // 480 polls x 60 s = 8 h
        advance_ms(60000);
        prox_note_sleep_interval(60000, /*motion_woke=*/0);
        feed_still();
        ProxScoreResult2 r = mk(deep_fade);
        if (prox_hmm_tick(&r) != start) flips++;
    }
    CHECK(flips == 0, "%d of 480 still ticks left NEAR (lam=%d)", flips, (int)prox_hmm_logodds_q8());
    CHECK(prox_hmm_decision() == start, "final decision changed: %d", (int)prox_hmm_decision());
}

// Walk-approach: the user actually walks toward the anchor. Crossing must be
// detected promptly, and the still segments on either side must be silent.
static void test_walk_approach(void) {
    begin("ACCEPTANCE walk-approach: flips within 3 ticks of crossing, silent while still");

    prox_hmm_reset(PROX_CRIT_GET_AWAY);              // cold start biased AWAY
    feed_still();

    // Segment 1: standing across the room, still. Weak signal, agrees with AWAY.
    int flips_still = 0;
    for (int i = 0; i < 30; ++i) {
        advance_ms(60000);
        prox_note_sleep_interval(60000, 0);
        feed_still();
        ProxScoreResult2 r = mk(score_from_rssi(-85.0 + frand_gauss(5.0)));
        if (prox_hmm_tick(&r) != PROX_HMM_AWAY) flips_still++;
    }
    CHECK(flips_still == 0, "%d spurious departures from AWAY while standing still", flips_still);

    // Segment 2: walk in. Path loss climbs from -85 to -50 dBm with 5 dB fading.
    // "Crossing" is where the underlying position first scores above the anchor's
    // decision cutoff — that is the moment a correct detector should react to.
    int ticks_after_cross = -1, cross_tick = -1;
    for (int i = 0; i < 20; ++i) {
        advance_ms(3000);                            // motion-triggered re-checks
        feed_loco();
        double truth = -85.0 + 35.0 * (i / 12.0);
        if (truth > -50.0) truth = -50.0;
        if (cross_tick < 0 && score_from_rssi(truth) >= PROX_CONFIDENCE_THRESHOLD_U8) cross_tick = i;
        ProxScoreResult2 r = mk(score_from_rssi(truth + frand_gauss(5.0)));
        ProxDecision d = prox_hmm_tick(&r);
        if (d == PROX_HMM_NEAR && ticks_after_cross < 0 && cross_tick >= 0)
            ticks_after_cross = i - cross_tick;
    }
    CHECK(ticks_after_cross >= 0, "never reached NEAR after crossing (lam=%d)", (int)prox_hmm_logodds_q8());
    CHECK(ticks_after_cross >= 0 && ticks_after_cross <= 3,
          "took %d ticks after crossing to flip", ticks_after_cross);

    // Segment 3: standing at the anchor, still. Must hold NEAR.
    int flips_back = 0;
    for (int i = 0; i < 60; ++i) {
        advance_ms(60000);
        prox_note_sleep_interval(60000, 0);
        feed_still();
        ProxScoreResult2 r = mk(score_from_rssi(-52.0 + frand_gauss(5.0)));
        if (prox_hmm_tick(&r) != PROX_HMM_NEAR) flips_back++;
    }
    CHECK(flips_back == 0, "%d spurious departures from NEAR while standing at the anchor", flips_back);
}

// Teleport rejection: a 20 dB step with the IMU reporting STILL throughout is
// physically impossible as a position change; it is multipath, interference or a
// body-shadow shift. The decision must survive it for at least 1/PFLIP_STILL
// tick-seconds (1e4 s).
static void test_teleport_rejection(void) {
    begin("ACCEPTANCE teleport-rejection: 20 dB step while STILL holds for >= 1/PFLIP_STILL tick-seconds");

    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    feed_loco();

    // Establish a genuinely confident NEAR while moving around the anchor.
    for (int i = 0; i < 10; ++i) {
        advance_ms(30000); feed_loco();
        ProxScoreResult2 r = mk(score_from_rssi(-50.0));
        prox_hmm_tick(&r);
    }
    CHECK(prox_hmm_decision() == PROX_HMM_NEAR, "setup failed to reach NEAR (lam=%d)", (int)prox_hmm_logodds_q8());

    // Now teleport the RSSI 20 dB down and hold the wrist provably still.
    uint32_t t0 = g_now_ms;
    uint32_t held_ms = 0;
    for (int i = 0; i < 400; ++i) {
        advance_ms(60000);
        prox_note_sleep_interval(60000, 0);
        feed_still();
        ProxScoreResult2 r = mk(score_from_rssi(-70.0));
        if (prox_hmm_tick(&r) == PROX_HMM_NEAR) held_ms = g_now_ms - t0;
        else break;
    }
    CHECK(held_ms >= 10000u * 1000u, "held NEAR only %u s, need >= 10000 s", held_ms / 1000);

    begin("...but sustained contradiction eventually wins (the trickle is not zero)");
    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    feed_loco();
    for (int i = 0; i < 10; ++i) { advance_ms(30000); feed_loco(); ProxScoreResult2 r = mk(score_from_rssi(-50.0)); prox_hmm_tick(&r); }
    int flipped = 0;
    for (int i = 0; i < 5000 && !flipped; ++i) {
        advance_ms(60000);
        prox_note_sleep_interval(60000, 0);
        feed_still();
        ProxScoreResult2 r = mk(score_from_rssi(-95.0));    // saturated contradiction
        if (prox_hmm_tick(&r) == PROX_HMM_AWAY) flipped = 1;
    }
    CHECK(flipped, "a saturated contradiction held forever — the still trickle is dead");
}

// IMU-stale: with no motion evidence the engine must behave like a PFLIP_MOVE-only
// HMM, i.e. v0.8-with-smoothing, never like a frozen decision.
static void test_imu_stale(void) {
    begin("ACCEPTANCE IMU-stale: no motion evidence ⇒ PFLIP_MOVE behavior");

    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    // Never feed a burst: the seam is dead.
    CHECK(prox_motion_state() == PROX_MOTION_UNKNOWN, "no evidence ⇒ UNKNOWN");

    int flipped_at = -1;
    for (int i = 0; i < 10; ++i) {
        advance_ms(60000);
        ProxScoreResult2 r = mk(score_from_rssi(-90.0));
        if (prox_hmm_tick(&r) == PROX_HMM_AWAY && flipped_at < 0) flipped_at = i;
    }
    CHECK(flipped_at >= 0 && flipped_at <= 1,
          "a dead IMU must not freeze the decision — flipped at tick %d", flipped_at);

    begin("...and a stale burst is treated exactly like no burst");
    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    feed_still();
    advance_ms(IMU_STALE_MS + 1000);
    CHECK(prox_motion_state() == PROX_MOTION_UNKNOWN, "stale burst ⇒ UNKNOWN");
    int flipped2 = -1;
    for (int i = 0; i < 10; ++i) {
        advance_ms(60000);
        ProxScoreResult2 r = mk(score_from_rssi(-90.0));
        if (prox_hmm_tick(&r) == PROX_HMM_AWAY && flipped2 < 0) flipped2 = i;
    }
    CHECK(flipped2 >= 0 && flipped2 <= 1, "stale seam froze the decision — flipped at tick %d", flipped2);
}

// ---------------------------------------------------------------- decision layer

static void test_decision_layer(void) {
    begin("decision layer: cold start sits on the criterion-satisfying side");

    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    CHECK(prox_hmm_logodds_q8() > 0, "stayNear cold start biases NEAR, lam=%d", (int)prox_hmm_logodds_q8());
    CHECK(prox_hmm_decision() == PROX_HMM_AMBIGUOUS, "...but not yet confident (P=0.65 < TAU_NEAR)");

    prox_hmm_reset(PROX_CRIT_GET_AWAY);
    CHECK(prox_hmm_logodds_q8() < 0, "getAway cold start biases AWAY, lam=%d", (int)prox_hmm_logodds_q8());
    prox_hmm_reset(PROX_CRIT_PHONE_AWAY);
    CHECK(prox_hmm_logodds_q8() < 0, "phoneAway cold start biases AWAY (compliant), lam=%d",
          (int)prox_hmm_logodds_q8());

    begin("decision layer: P(NEAR) tracks the log-odds monotonically");
    CHECK(prox_hmm_p_near_u8() < 128, "negative log-odds ⇒ P(NEAR) < 0.5, got %u", prox_hmm_p_near_u8());
    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    uint8_t p65 = prox_hmm_p_near_u8();
    CHECK(p65 > 155 && p65 < 178, "P=0.65 should read ~166/255, got %u", p65);

    begin("decision layer: per-anchor calibrated cutoff recentres the emission");
    // An anchor whose demonstrated near zone ends at score 210 must read a score
    // of 180 as evidence AGAINST near, even though 180 > the global default 170.
    prox_hmm_reset(PROX_CRIT_GET_AWAY);
    feed_loco();
    advance_ms(60000);
    ProxScoreResult2 r = mk(180, /*near_thr=*/210);
    prox_hmm_tick(&r);
    CHECK(prox_hmm_logodds_q8() < -HMM_PRIOR_Q8,
          "score below the anchor's demonstrated cutoff must push AWAY, lam=%d", (int)prox_hmm_logodds_q8());

    prox_hmm_reset(PROX_CRIT_GET_AWAY);
    feed_loco();
    advance_ms(60000);
    ProxScoreResult2 r2 = mk(180, /*near_thr=*/120);
    prox_hmm_tick(&r2);
    CHECK(prox_hmm_logodds_q8() > -HMM_PRIOR_Q8,
          "the same score above a lower cutoff must push NEAR, lam=%d", (int)prox_hmm_logodds_q8());
}

// Replay one identical failed-connect tick, optionally reporting it, and return
// the resulting log-odds. Comparing two replays isolates the rule's contribution
// from the transition decay that every tick carries.
static int32_t replay_connect_failure(int report, int8_t advert_rssi, int have_advert) {
    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    feed_loco();
    advance_ms(60000);
    if (report) prox_note_connect_failure(advert_rssi, have_advert);
    prox_hmm_tick(NULL);
    return prox_hmm_logodds_q8();
}

static void test_connect_failure(void) {
    begin("connect-failure evidence: weak advert ⇒ AWAY log-LR, otherwise abstain");

    int32_t base = replay_connect_failure(0, 0, 0);

    int32_t weak = replay_connect_failure(1, -90, 1);
    CHECK(weak - base == LL_CONNFAIL_AWAY_Q8,
          "a failed connect beside a -90 dBm advert must add exactly %d, added %d",
          LL_CONNFAIL_AWAY_Q8, (int)(weak - base));

    int32_t strong = replay_connect_failure(1, -40, 1);
    CHECK(strong == base, "a strong advert is not evidence of being far (%d vs %d)", (int)strong, (int)base);

    int32_t none = replay_connect_failure(1, 0, 0);
    CHECK(none == base, "with no advertisement the rule must abstain (%d vs %d)", (int)none, (int)base);

    // Regression, found on hardware: a still watch that cannot reach its anchor
    // re-fails the connect on every poll. Before the fix each failure added a
    // full LL_CONNFAIL_AWAY and the posterior marched to saturation on what is
    // one observation repeated — frozen-fade accumulation via the watch-local
    // channel. Watch-local evidence rides the same draw gate as everything else.
    begin("connect-failure evidence does not accumulate while STILL");
    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    feed_still();
    advance_ms(60000);
    prox_note_connect_failure(-90, 1);
    prox_hmm_tick(NULL);
    int32_t after_first = prox_hmm_logodds_q8();

    for (int i = 0; i < 200; ++i) {              // 200 polls x 60 s of failing while still
        advance_ms(60000);
        prox_note_sleep_interval(60000, 0);
        feed_still();
        prox_note_connect_failure(-90, 1);
        prox_hmm_tick(NULL);
    }
    int32_t after_many = prox_hmm_logodds_q8();
    CHECK(after_many > after_first + LL_CONNFAIL_AWAY_Q8,
          "200 repeated failures added more than one extra observation's worth "
          "(%d -> %d, one observation is %d)", (int)after_first, (int)after_many, LL_CONNFAIL_AWAY_Q8);
    CHECK(after_many > -HMM_LAMBDA_MAX_Q8 / 2,
          "repeated identical failures drove the posterior to saturation (lam=%d)", (int)after_many);
}

// ---------------------------------------------------------------- coloc (Modes B/C)

static void test_coloc(void) {
    begin("coloc: hysteresis machine replaced by the HMM, still fail-safe to AWAY");

    coloc_init();
    ColocDecision d = coloc_decision();
    CHECK(d.state == COLOC_AWAY, "coloc cold-starts compliant (AWAY)");

    // Phone in hand: strong, steady R_wp, while walking.
    feed_loco();
    for (int i = 0; i < 20; ++i) {
        advance_ms(2000); feed_loco();
        coloc_ingest_link_rssi(-50, 0, 0, 0);
        for (int k = 0; k < 6; ++k) coloc_ingest_shared_device((int8_t)(-60 + k), (int8_t)(-61 + k));
        coloc_mark_reading_end();
        d = coloc_tick();
    }
    CHECK(d.state == COLOC_NEAR, "sustained near evidence while moving reaches NEAR (p=%.2f)", d.p_near);
    CHECK((d.used_factors & COLOC_F_RANGE) != 0, "range factor contributed");
    CHECK((d.used_factors & COLOC_F_VAR) != 0, "variance factor contributed");

    // Phone parked across the room; the user walks away.
    for (int i = 0; i < 30; ++i) {
        advance_ms(2000); feed_loco();
        coloc_ingest_link_rssi(-80, 0, 0, 0);
        for (int k = 0; k < 6; ++k) coloc_ingest_shared_device((int8_t)(-60 + k), (int8_t)(-75 + k));
        coloc_mark_reading_end();
        d = coloc_tick();
    }
    CHECK(d.state == COLOC_AWAY, "sustained away evidence returns to AWAY (p=%.2f)", d.p_near);

    begin("coloc: a still watch cannot be flipped by an R_wp excursion");
    // Re-establish AWAY, then park the wrist and inject a strong near-looking
    // excursion — the classic multipath artifact.
    ColocState before = d.state;
    for (int i = 0; i < 200; ++i) {
        advance_ms(2000); feed_still();
        coloc_ingest_link_rssi(-45, 0, 0, 0);
        for (int k = 0; k < 6; ++k) coloc_ingest_shared_device((int8_t)(-60 + k), (int8_t)(-61 + k));
        coloc_mark_reading_end();
        d = coloc_tick();
    }
    CHECK(d.state == before, "a still watch flipped on an RSSI excursion (p=%.2f)", d.p_near);
}


// ---------------------------------------------------------------- hardware regressions

// Feed a burst with a target gravity-removed variance, so tests can be written
// in the units Spike S3 actually measured on a wrist.
static void feed_var(uint32_t target_var) {
    // feed_burst's square wave gives var ~= amp^2 * (1 + 1/4 + 1/9); solve for amp.
    double amp = sqrt((double)target_var / 1.361);
    feed_burst((int)(amp + 0.5), 0);
}

static void test_s3_measured_classes(void) {
    begin("S3: measured wrist variances land in the right motion classes");

    // Numbers straight from the 2026-07-28 bench run on a real wrist.
    const uint32_t desk[]   = { 9, 22, 34 };
    const uint32_t still[]  = { 95, 120, 150, 291, 338 };
    const uint32_t typing[] = { 756, 1730, 2253, 3616, 4679, 7404, 9278, 9788 };
    const uint32_t walk[]   = { 19645, 20242, 27357, 51111, 105445, 415024 };

    for (size_t i = 0; i < sizeof(desk)/sizeof(desk[0]); ++i) {
        prox_hmm_reset(PROX_CRIT_STAY_NEAR); feed_var(desk[i]);
        CHECK(prox_motion_state() == PROX_MOTION_STILL, "desk var %u => STILL, got %u",
              desk[i], prox_motion_state());
    }
    for (size_t i = 0; i < sizeof(still)/sizeof(still[0]); ++i) {
        prox_hmm_reset(PROX_CRIT_STAY_NEAR); feed_var(still[i]);
        CHECK(prox_motion_state() == PROX_MOTION_STILL, "worn-still var %u => STILL, got %u",
              still[i], prox_motion_state());
    }
    // The regression the user hit: ~50% of typing read as LOCOMOTION, because
    // the cadence flag flipped on noise at these amplitudes.
    for (size_t i = 0; i < sizeof(typing)/sizeof(typing[0]); ++i) {
        prox_hmm_reset(PROX_CRIT_STAY_NEAR); feed_var(typing[i]);
        CHECK(prox_motion_state() == PROX_MOTION_FIDGET, "typing var %u => FIDGET, got %u",
              typing[i], prox_motion_state());
    }
    for (size_t i = 0; i < sizeof(walk)/sizeof(walk[0]); ++i) {
        prox_hmm_reset(PROX_CRIT_STAY_NEAR); feed_var(walk[i]);
        CHECK(prox_motion_state() == PROX_MOTION_LOCOMOTION, "walking var %u => LOCOMOTION, got %u",
              walk[i], prox_motion_state());
    }
}

static void test_ambiguous_band_abstains(void) {
    begin("REGRESSION: scores inside the anchor's band never accumulate certainty");

    // The A2 trace: watch 2 cm from its anchor, scores 153-167 against a cutoff
    // of 170 -- i.e. inside the anchor's own hysteresis band, which v0.8 calls
    // AMBIGUOUS. v2 was converting each 3-17 point shortfall into evidence and
    // marching to lam=-800: a confident AWAY, which for stayNear inverts the
    // fail-safe from compliant to alarming.
    const uint8_t observed[] = { 159, 160, 162, 163, 164, 167, 153, 156 };

    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    int32_t start = prox_hmm_logodds_q8();
    for (int rep = 0; rep < 20; ++rep) {
        for (size_t i = 0; i < sizeof(observed)/sizeof(observed[0]); ++i) {
            advance_ms(5000);
            feed_loco();                      // full draw credit: the worst case
            ProxScoreResult2 r = mk(observed[i], /*near_thr=*/170);
            prox_hmm_tick(&r);
        }
    }
    // The posterior may DRIFT TOWARD 0 here — that is the transition model
    // correctly forgetting an old belief while the wrist moves and nothing
    // informative arrives. What must not happen is it acquiring certainty in
    // either direction off the back of in-band scores.
    int32_t lam = prox_hmm_logodds_q8();
    CHECK(lam <= start && lam > HMM_TAU_AWAY_Q8,
          "160 in-band scores must not build certainty: %d -> %d", (int)start, (int)lam);
    CHECK(prox_hmm_decision() == PROX_HMM_AMBIGUOUS,
          "in-band scores must read AMBIGUOUS, got %d", (int)prox_hmm_decision());

    begin("...but scores outside the band still decide, in both directions");
    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    for (int i = 0; i < 6; ++i) {
        advance_ms(5000); feed_loco();
        ProxScoreResult2 r = mk(240, 170);
        prox_hmm_tick(&r);
    }
    CHECK(prox_hmm_decision() == PROX_HMM_NEAR, "a clearly-near score must reach NEAR (lam=%d)",
          (int)prox_hmm_logodds_q8());

    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    for (int i = 0; i < 6; ++i) {
        advance_ms(5000); feed_loco();
        ProxScoreResult2 r = mk(10, 170);
        prox_hmm_tick(&r);
    }
    CHECK(prox_hmm_decision() == PROX_HMM_AWAY, "a clearly-far score must reach AWAY (lam=%d)",
          (int)prox_hmm_logodds_q8());

    begin("...and a score below the band still counts, matching v0.8");
    // Scores of 133/145 appeared in the same trace, measured from across the
    // room. Those sit below the band and must remain informative -- abstaining
    // everywhere would be just as wrong as never abstaining. v0.8 calls these
    // AWAY too, so the two layers agree.
    // Note this takes more ticks than v0.8's instant threshold: a score just
    // outside the band is genuinely weak evidence (the score->logit map is flat
    // near the middle), so the filter accumulates rather than jumping. That is
    // the intended behavior, not a shortfall.
    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    for (int i = 0; i < 12; ++i) {
        advance_ms(5000); feed_loco();
        ProxScoreResult2 r = mk(133, 170);
        prox_hmm_tick(&r);
    }
    CHECK(prox_hmm_decision() == PROX_HMM_AWAY,
          "a below-band score must still drive AWAY (lam=%d)", (int)prox_hmm_logodds_q8());

    begin("...and the band narrows to a calibrated anchor's demonstrated cutoff");
    // A calibrated anchor gets the narrow hysteresis band, so a score 30 below a
    // demonstrated cutoff of 210 IS informative -- where the same score sits
    // in-band under the wide global rule.
    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    for (int i = 0; i < 6; ++i) {
        advance_ms(5000); feed_loco();
        ProxScoreResult2 r = mk(180, /*near_thr=*/210);
        prox_hmm_tick(&r);
    }
    CHECK(prox_hmm_logodds_q8() < 0, "below a demonstrated cutoff => AWAY evidence (lam=%d)",
          (int)prox_hmm_logodds_q8());
}

// ---------------------------------------------------------------- P2: beacon schedule

static void test_beacon_minor(void) {
    begin("beacon Minor: round-trips slot id and cycle sequence");
    for (int slot = 0; slot < 6; ++slot) {
        for (uint16_t cyc = 1; cyc < 0x1000; cyc += 37) {
            uint16_t m = prox_beacon_minor_encode((uint8_t)slot, cyc);
            uint8_t gs = 0xFF; uint16_t gc = 0xFFFF;
            CHECK(prox_beacon_minor_decode(m, &gs, &gc) == 1, "slot %d cyc %u should decode", slot, cyc);
            CHECK(gs == slot && gc == cyc, "round-trip slot %d cyc %u -> %u/%u", slot, cyc, gs, gc);
        }
    }

    begin("beacon Minor: an all-zero Minor is 'no schedule', not slot 0 cycle 0");
    // A v0.8 anchor emits Minor 0x0000 forever. If cycle_seq could be 0 the
    // watch could not tell that apart from a real slot-0 tag, and would
    // attribute phantom slots to an anchor that has no schedule at all.
    uint8_t s2 = 0xFF; uint16_t c2 = 0xFFFF;
    CHECK(prox_beacon_minor_decode(0x0000, &s2, &c2) == 0, "legacy Minor must not decode");
    CHECK(prox_beacon_minor_encode(0, 1) != 0x0000, "the live schedule never emits 0x0000");

    begin("beacon Minor: slot id occupies the high nibble of the first on-air byte");
    // iBeacon sends Major/Minor big-endian, so the anchor writes msd[22]=minor>>8.
    // The slot id must land in the high nibble there -- the byte order the
    // amendment (Part 3) singles out as error-prone.
    uint16_t m = prox_beacon_minor_encode(5, 0x0AB);
    CHECK((uint8_t)((m >> 8) >> 4) == 5, "slot id in high nibble of first byte, got 0x%02X",
          (uint8_t)(m >> 8));
    CHECK((m & 0x0FFF) == 0x0AB, "cycle survives in the low 12 bits, got 0x%03X", m & 0x0FFF);

    begin("beacon slots: TX_LO slots are the ones PDR is measured over");
    CHECK(!prox_beacon_slot_is_lo(0) && !prox_beacon_slot_is_lo(2), "slots 0-2 are TX_HI");
    CHECK(prox_beacon_slot_is_lo(3) && prox_beacon_slot_is_lo(5), "slots 3-5 are TX_LO");

    begin("beacon slots: channel attribution follows the S1b build switch");
#if BEACON_CHANNEL_CONTROL
    CHECK(prox_beacon_slot_channel(0) == 37 && prox_beacon_slot_channel(2) == 39,
          "full schedule attributes channels");
    CHECK(prox_beacon_slot_channel_map(1) == BEACON_CH38_BIT, "slot 1 maps to ch38 only");
    CHECK(BEACON_SLOT_COUNT == 6, "full schedule has 6 slots");
#else
    // Under the fallback the watch must be told the channel is unknown rather
    // than handed a plausible-looking wrong answer.
    CHECK(prox_beacon_slot_channel(0) == 0 && prox_beacon_slot_channel(4) == 0,
          "fallback reports channel unknown");
    CHECK(prox_beacon_slot_channel_map(0) == BEACON_CH_ALL, "fallback uses all channels");
    CHECK(BEACON_SLOT_COUNT == 2, "fallback has 2 slots");
#endif
}

// ----------------------------------------------------------------------------

int main(void) {
    printf("proximity engine v2.1 — Phase 1 acceptance tests\n\n");

    prox_init();

    test_luts();
    test_motion_classification();
    test_integrator();
    test_decision_layer();
    test_connect_failure();
    test_frozen_fade();
    test_walk_approach();
    test_teleport_rejection();
    test_imu_stale();
    test_coloc();
    test_s3_measured_classes();
    test_ambiguous_band_abstains();
    test_beacon_minor();

    printf("\n%d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
