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
        // Count from where the engine can first SEE the crossing, not from the
        // bare threshold: HMM_EMIT_DEADZONE_U8 deliberately discards scores
        // within 20 counts of the cutoff, because hardware showed noise there
        // accumulating into false certainty. A detector cannot be asked to react
        // to a signal it is designed to ignore.
        if (cross_tick < 0 &&
            score_from_rssi(truth) >= PROX_CONFIDENCE_THRESHOLD_U8 + HMM_EMIT_DEADZONE_U8)
            cross_tick = i;
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


static void test_field_traces(void) {
    // Both sequences are the anchor scores actually observed on 2026-07-28,
    // replayed at the 5 s bench cadence with the motion states that were logged.

    begin("FIELD: walking away from the anchor must leave NEAR");
    // Log 1. The filter had climbed to NEAR on 198/180/199, then the user walked
    // away. Scores 157 and 103 were being discarded as "in band" and 79 was
    // worth -0.1 nats, so v2 stayed convinced of NEAR for the entire walk while
    // v0.8 had already said AWAY.
    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    const uint8_t near_leg[] = { 237, 233, 235, 198, 199 };
    for (size_t i = 0; i < sizeof(near_leg)/sizeof(near_leg[0]); ++i) {
        advance_ms(5000); feed_loco();
        ProxScoreResult2 r = mk(near_leg[i]); prox_hmm_tick(&r);
    }
    CHECK(prox_hmm_decision() == PROX_HMM_NEAR, "setup: should be NEAR (lam=%d)",
          (int)prox_hmm_logodds_q8());

    const uint8_t away_leg[] = { 79, 157, 103, 103, 79 };
    int left_near = -1;
    for (size_t i = 0; i < sizeof(away_leg)/sizeof(away_leg[0]); ++i) {
        advance_ms(5000); feed_loco();
        ProxScoreResult2 r = mk(away_leg[i]); prox_hmm_tick(&r);
        if (left_near < 0 && prox_hmm_decision() != PROX_HMM_NEAR) left_near = (int)i;
    }
    CHECK(left_near >= 0 && left_near <= 2, "left NEAR after %d ticks of walking away (lam=%d)",
          left_near, (int)prox_hmm_logodds_q8());
    CHECK(prox_hmm_decision() == PROX_HMM_AWAY, "should end AWAY, got %d (lam=%d)",
          (int)prox_hmm_decision(), (int)prox_hmm_logodds_q8());

    begin("FIELD: walking toward the anchor must reach NEAR promptly");
    // Log 2, from a cold start across the room: one genuinely-far score of 0,
    // then the approach. Recovery took ~6 ticks because logit() let that single
    // score claim -6.2 nats.
    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    const uint8_t approach[] = { 0, 147, 203, 219, 237, 233, 205, 235 };
    int reached = -1;
    for (size_t i = 0; i < sizeof(approach)/sizeof(approach[0]); ++i) {
        advance_ms(5000); feed_loco();
        ProxScoreResult2 r = mk(approach[i]); prox_hmm_tick(&r);
        if (reached < 0 && prox_hmm_decision() == PROX_HMM_NEAR) reached = (int)i;
    }
    CHECK(reached >= 0, "never reached NEAR on approach (lam=%d)", (int)prox_hmm_logodds_q8());
    // The approach genuinely starts at score 147, so count from there.
    CHECK(reached >= 0 && reached <= 5, "reached NEAR at tick %d of the approach", reached);

    begin("FIELD: one extreme score cannot claim more than the cap");
    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    int32_t before = prox_hmm_logodds_q8();
    advance_ms(5000); feed_loco();
    ProxScoreResult2 z = mk(0); prox_hmm_tick(&z);
    // Allow a little slack for the transition step, which also moves lam.
    CHECK(before - prox_hmm_logodds_q8() <= HMM_EMIT_MAX_Q8 + 64,
          "a single score of 0 moved the posterior by %d, cap is %d",
          (int)(before - prox_hmm_logodds_q8()), HMM_EMIT_MAX_Q8);

    begin("FIELD: emission is symmetric about the cutoff");
    // Equal deviations either side must carry equal weight; the logit form gave
    // +1.2 nats for 237 and -0.1 for 79.
    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    advance_ms(5000); feed_loco();
    int32_t base = prox_hmm_logodds_q8();
    ProxScoreResult2 up = mk(170 + 60); prox_hmm_tick(&up);
    int32_t gain_up = prox_hmm_logodds_q8() - base;

    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    advance_ms(5000); feed_loco();
    base = prox_hmm_logodds_q8();
    ProxScoreResult2 dn = mk(170 - 60); prox_hmm_tick(&dn);
    int32_t gain_dn = base - prox_hmm_logodds_q8();
    // Within the transition step's own contribution, which is sign-dependent.
    int32_t skew = gain_up > gain_dn ? gain_up - gain_dn : gain_dn - gain_up;
    CHECK(skew <= 16, "asymmetric: +60 gave %d, -60 gave %d", (int)gain_up, (int)gain_dn);
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


// ---------------------------------------------------------------- scan cache

// Build a distinct MAC from an index so tests can talk about "device i".
static void mk_mac(int i, uint8_t out[6]) {
    out[0] = 0xAA; out[1] = 0xBB; out[2] = 0xCC;
    out[3] = 0x00; out[4] = (uint8_t)(i >> 8); out[5] = (uint8_t)(i & 0xFF);
}

static int vec_find(const ProxScanVector* v, int idx, int8_t* out_rssi) {
    uint8_t mac[6]; mk_mac(idx, mac);
    for (int i = 0; i < v->count; ++i)
        if (memcmp(v->devices[i].mac, mac, 6) == 0) {
            if (out_rssi) *out_rssi = v->devices[i].rssi;
            return 1;
        }
    return 0;
}

static void ingest(int idx, int8_t rssi) {
    uint8_t mac[6]; mk_mac(idx, mac);
    prox_ingest_scan_result(mac, PROX_TYPE_BLE, rssi);
}

// The firmware samples one IMU burst per observation window (§5.4.5), so the
// tests do too -- the cache consults motion state when it folds a window.
// One observation window: an IMU burst (the cache consults motion state when it
// folds) plus enough wall-clock advance that windows are distinguishable — the
// cache ages samples by time now, not by window count.
static void close_window_still(void) {
    feed_still();
    prox_scan_window_close();
    advance_ms(2000);
}

static void test_scan_cache(void) {
    // The whole point of the cache: one scan window is a random subset of the
    // BLE population, so a vector built from one window is a random draw. Field
    // data (still watch, still anchor) showed device counts of 21-31 and scores
    // of 26-159 with a physically frozen channel.

    begin("scan cache: within one window the strongest reading wins");
    // An anchor running the beacon schedule alternates TX_HI (+9) and TX_LO
    // (-21) inside a single window -- 30 dB apart. Averaging those two would
    // invent a reading that was never received; the max picks the TX_HI slot.
    prox_scan_cache_reset();
    ingest(1, -97);   // TX_LO advert
    ingest(1, -67);   // TX_HI advert, same window
    ingest(1, -95);
    close_window_still();
    ProxScanVector v; prox_build_scan_vector(&v);
    int8_t r = 0;
    CHECK(vec_find(&v, 1, &r) && r == -67, "expected -67 from the TX_HI slot, got %d", (int)r);

    begin("scan cache: median across windows rejects a single bad window");
    prox_scan_cache_reset();
    const int8_t seq[5] = { -70, -70, -95, -70, -70 };   // one deep-fade window
    for (int w = 0; w < 5; ++w) { ingest(2, seq[w]); close_window_still(); }
    prox_build_scan_vector(&v);
    CHECK(vec_find(&v, 2, &r) && r == -70,
          "median must ignore the -95 outlier (a mean would give -76), got %d", (int)r);

    begin("scan cache: a device missed in some windows stays in the vector");
    // This is the headline fix. A 2 s advertiser is caught by a 700 ms window
    // ~35% of the time; under the old one-shot buffer it vanished from the
    // vector on ~2 of every 3 queries, changing the correlation's device set
    // underneath it. Seen once in four windows is enough to be carried.
    prox_scan_cache_reset();
    ingest(3, -80);                       // seen in window 0 only
    for (int w = 0; w < 4; ++w) {
        ingest(99, -60);                  // a persistent device keeps windows non-empty
        close_window_still();
    }
    prox_build_scan_vector(&v);
    CHECK(vec_find(&v, 3, &r), "a device seen in 1 of the last 4 windows must survive");

    begin("scan cache: a device is evicted once its samples pass the TTL");
    prox_scan_cache_reset();
    ingest(4, -80);
    close_window_still();
    // Age past PROX_CACHE_TTL_MS while another device keeps windows non-empty.
    for (uint32_t t = 0; t < (uint32_t)PROX_CACHE_TTL_MS + 4000; t += 2000) {
        ingest(99, -60);
        close_window_still();
    }
    prox_build_scan_vector(&v);
    CHECK(!vec_find(&v, 4, NULL), "device unheard for > %d ms should be gone",
          PROX_CACHE_TTL_MS);

    begin("scan cache: coverage is set by TTL, not by query cadence");
    // The regression this replaced: holding one GATT link open made queries ~5x
    // faster, and a window-COUNT cache then spanned 5x less wall time, so vectors
    // collapsed from ~30 devices to 8-13. Fast and slow cadences must now retain
    // the same device.
    for (int fast = 0; fast < 2; ++fast) {
        prox_scan_cache_reset();
        const uint32_t step = fast ? 500 : 8000;   // 0.5 s vs 8 s between queries
        ingest(5, -75);
        feed_still(); prox_scan_window_close(); advance_ms(step);
        for (uint32_t t = step; t < 20000; t += step) {
            ingest(98, -60);
            feed_still(); prox_scan_window_close(); advance_ms(step);
        }
        prox_build_scan_vector(&v);
        CHECK(vec_find(&v, 5, NULL),
              "%s cadence: a device heard 20 s ago is inside the %d ms TTL",
              fast ? "fast" : "slow", PROX_CACHE_TTL_MS);
    }

    begin("scan cache: membership is stable across queries with flaky advertisers");
    // 40 devices, each independently caught with ~40% probability per window --
    // the measured regime. Measure the device-set overlap between consecutive
    // queries BOTH ways: once through the cache, and once using single-window
    // membership (the old one-shot buffer's behaviour) over the same draws.
    // Overlap is what the correlation actually depends on: every device that
    // enters or leaves changes the statistic underneath it.
    unsigned rng = 12345u;
    int one_shot_a[40], one_shot_b[40];
    ProxScanVector a, b;
    prox_scan_cache_reset();
    for (int w = 0; w < 8; ++w) {                     // prime the history
        for (int d = 0; d < 40; ++d) {
            rng = rng * 1103515245u + 12345u;
            if (((rng >> 16) % 100u) < 40u) ingest(d, (int8_t)(-60 - d));
        }
        close_window_still();
    }
    for (int d = 0; d < 40; ++d) {                    // window A
        rng = rng * 1103515245u + 12345u;
        one_shot_a[d] = (((rng >> 16) % 100u) < 40u);
        if (one_shot_a[d]) ingest(d, (int8_t)(-60 - d));
    }
    close_window_still();
    prox_build_scan_vector(&a);
    for (int d = 0; d < 40; ++d) {                    // window B
        rng = rng * 1103515245u + 12345u;
        one_shot_b[d] = (((rng >> 16) % 100u) < 40u);
        if (one_shot_b[d]) ingest(d, (int8_t)(-60 - d));
    }
    close_window_still();
    prox_build_scan_vector(&b);

    int same = 0, total = 0, os_same = 0, os_total = 0;
    for (int d = 0; d < 40; ++d) {
        int in_a = vec_find(&a, d, NULL), in_b = vec_find(&b, d, NULL);
        if (in_a || in_b) { total++; if (in_a && in_b) same++; }
        if (one_shot_a[d] || one_shot_b[d]) { os_total++; if (one_shot_a[d] && one_shot_b[d]) os_same++; }
    }
    int pct    = total    ? (same * 100 / total)       : 0;
    int os_pct = os_total ? (os_same * 100 / os_total) : 0;
    printf("    membership overlap: cache %d%% vs one-shot %d%%\n", pct, os_pct);
    CHECK(pct >= 80, "cache overlap should be >=80%%, got %d%% (%d/%d)", pct, same, total);
    CHECK(pct >= os_pct * 2,
          "cache should at least double one-shot overlap: %d%% vs %d%%", pct, os_pct);

    begin("scan cache: scarce vector slots go to the STRONGEST devices");
    // Selection must optimise mutual visibility, not watch-local persistence.
    // The score is a correlation over devices the watch and anchor BOTH see, and
    // signal strength is what makes a device mutually visible. Ranking by hit
    // count instead was tried on hardware: it collapsed the shared set and drove
    // Pearson into its degenerate small-k regime (scores of exactly 0 or 255).
    prox_scan_cache_reset();
    for (int w = 0; w < 4; ++w) {
        for (int d = 0; d < PROX_MAX_DEVICES; ++d) ingest(d, -90);  // weak, always present
        close_window_still();
    }
    for (int d = 0; d < 20; ++d) ingest(500 + d, -40);              // loud, seen once
    close_window_still();
    prox_build_scan_vector(&v);
    int loud = 0;
    for (int d = 0; d < 20; ++d) if (vec_find(&v, 500 + d, NULL)) loud++;
    CHECK(loud == 20, "all 20 strong devices must make the vector, got %d", loud);

    begin("scan cache: persistence breaks ties at equal strength");
    // The MTU caps the vector at ~31 devices. A device present in every window
    // is a usable correlation coordinate; one glimpsed once is a coordinate
    // that will be absent next query.
    prox_scan_cache_reset();
    for (int w = 0; w < 4; ++w) {
        for (int d = 0; d < PROX_MAX_DEVICES; ++d) ingest(d, -90);   // weak but always there
        close_window_still();
    }
    for (int d = 0; d < 20; ++d) ingest(500 + d, -40);               // loud, seen once
    close_window_still();
    prox_scan_cache_reset();
    for (int w = 0; w < 4; ++w) {
        for (int d = 0; d < PROX_MAX_DEVICES; ++d) ingest(d, -70);   // equal RSSI, always seen
        close_window_still();
    }
    for (int d = 0; d < 20; ++d) ingest(700 + d, -70);               // equal RSSI, seen once
    close_window_still();
    prox_build_scan_vector(&v);
    int stable = 0;
    for (int d = 0; d < PROX_MAX_DEVICES; ++d) if (vec_find(&v, d, NULL)) stable++;
    CHECK(v.count == PROX_MAX_DEVICES, "vector should be full, got %d", (int)v.count);
    CHECK(stable == PROX_MAX_DEVICES,
          "at equal RSSI the persistent devices win the slots, got %d of %d",
          stable, PROX_MAX_DEVICES);

    begin("scan cache: LOCOMOTION collapses history instead of smearing positions");
    // Averaging across windows is only valid while the windows describe the
    // same place. Moving windows must not blend, exactly as the integrator
    // restarts on STILL->LOCOMOTION.
    prox_scan_cache_reset();
    for (int w = 0; w < 3; ++w) { ingest(7, -50); close_window_still(); }
    ingest(7, -90);
    feed_loco();
    prox_scan_window_close();
    prox_build_scan_vector(&v);
    CHECK(vec_find(&v, 7, &r) && r == -90,
          "while moving, the freshest window alone should be reported, got %d", (int)r);

    begin("scan cache: reset drops everything");
    prox_scan_cache_reset();
    CHECK(prox_scan_cache_count() == 0, "cache should be empty after reset");
    prox_build_scan_vector(&v);
    CHECK(v.count == 0, "vector from an empty cache should be empty");
}

// ----------------------------------------------------------------------------

// ---------------------------------------------------------------- PDR (§3.3)

static const uint8_t PDR_MAC[6]   = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x33};
static const uint8_t OTHER_MAC[6] = {0xAA, 0xBB, 0xCC, 0x11, 0x22, 0x99};

// Feed one observation window. hi[i]/lo[i] say whether that cycle's HI and LO
// slot was heard; cyc0 must be >= 1 because Minor 0x0000 is the reserved legacy
// value. Returns prox_obs_close()'s verdict.
static int feed_window(uint16_t cyc0, int n, const int* hi, const int* lo,
                       uint8_t* hits, uint8_t* covered) {
    prox_obs_begin(PDR_MAC);
    for (int i = 0; i < n; ++i) {
        uint16_t c = (uint16_t)(cyc0 + i);
        if (hi[i]) prox_obs_note(PDR_MAC, prox_beacon_minor_encode(0, c), -70);
        if (lo[i]) prox_obs_note(PDR_MAC, prox_beacon_minor_encode(3, c), -95);
    }
    return prox_obs_close(hits, covered);
}

static int32_t window_loglr(uint16_t cyc0, int n, const int* hi, const int* lo) {
    uint8_t h = 0, c = 0;
    feed_window(cyc0, n, hi, lo, &h, &c);
    int32_t ll = 0;
    prox_pdr_state(NULL, NULL, &ll);
    return ll;
}

static void test_pdr_window(void) {
    begin("PDR window: 'covered' means the cycle was heard, in either slot");

    const int all[4]  = {1, 1, 1, 1};
    const int none[4] = {0, 0, 0, 0};
    uint8_t h, c;

    prox_pdr_reset();
    feed_window(10, 4, all, all, &h, &c);
    CHECK(h == 4 && c == 4, "4 cycles heard in both slots = 4 hits of 4 covered, got %u/%u", h, c);

    prox_pdr_reset();
    feed_window(10, 4, all, none, &h, &c);
    CHECK(h == 0 && c == 4, "HI heard, LO silent = 0 hits of 4 covered, got %u/%u", h, c);

    // The LO slot alone still proves the cycle happened.
    prox_pdr_reset();
    feed_window(10, 4, none, all, &h, &c);
    CHECK(h == 4 && c == 4, "LO alone is both a hit and its own denominator, got %u/%u", h, c);

    // A cycle heard in neither slot is a clipped window edge, not a miss. This is
    // the distinction that stops an unlucky scan becoming AWAY evidence.
    const int half[4] = {1, 1, 0, 0};
    prox_pdr_reset();
    feed_window(10, 4, half, none, &h, &c);
    CHECK(c == 2, "cycles heard in neither slot must not be counted, covered=%u", c);

    begin("PDR window: foreign MACs and legacy Minors contribute nothing");
    prox_pdr_reset();
    prox_obs_begin(PDR_MAC);
    for (int i = 0; i < 4; ++i) {
        prox_obs_note(OTHER_MAC, prox_beacon_minor_encode(0, (uint16_t)(10 + i)), -70);
        prox_obs_note(OTHER_MAC, prox_beacon_minor_encode(3, (uint16_t)(10 + i)), -95);
        prox_obs_note(PDR_MAC, 0x0000, -70);          // legacy, unscheduled anchor
    }
    CHECK(prox_obs_close(&h, &c) == 0 && c == 0,
          "another anchor's beacons and legacy Minors must not register, got %u/%u", h, c);

    begin("PDR abstains below PDR_MIN_COVERED rather than reporting a miss");
    const int one[1] = {1};
    const int zero[1] = {0};
    prox_pdr_reset();
    CHECK(feed_window(10, 1, one, zero, &h, &c) == 0,
          "a single covered cycle is not enough to claim anything (covered=%u)", c);
    int32_t ll = 0;
    prox_pdr_state(NULL, NULL, &ll);
    CHECK(ll == 0, "an abstaining window must queue no evidence, queued %d", (int)ll);
}

static void test_pdr_schedule_disabled(void) {
    // The safety property that lets PDR ship dark. With BEACON_SCHEDULE_ENABLE=0
    // the anchor still stamps a Minor — prox_beacon_schedule_init() returns before
    // beacon_emit(), so start_ble_advertising() sends encode(slot 0, cycle 1) = 1,
    // NOT the all-zero legacy value the spec's §3.2 note assumes. It therefore
    // decodes successfully, and every advertisement in the window lands in the
    // same cycle. One cycle is below PDR_MIN_COVERED, so the channel abstains and
    // contributes exactly nothing to the posterior.
    begin("PDR: a schedule-disabled anchor yields no evidence (ships dark)");

    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    feed_loco();
    advance_ms(1000);

    const uint16_t legacy_minor = prox_beacon_minor_encode(0, 1);
    CHECK(legacy_minor == 1, "a schedule-disabled anchor should stamp Minor 1, got %u",
          (unsigned)legacy_minor);

    prox_obs_begin(PDR_MAC);
    for (int i = 0; i < 40; ++i) prox_obs_note(PDR_MAC, legacy_minor, -70);
    uint8_t h = 0, c = 0;
    CHECK(prox_obs_close(&h, &c) == 0,
          "40 beacons all in one cycle must abstain, not score (%u/%u)", h, c);
    CHECK(c == 1, "all beacons share one cycle, so covered must be 1, got %u", c);

    int32_t ll = 0;
    prox_pdr_state(NULL, NULL, &ll);
    CHECK(ll == 0, "a dark schedule must queue zero evidence, queued %d", (int)ll);

    // Paired comparison, because a bare tick still advances the transition model
    // and decays the posterior on its own: run the identical tick sequence with
    // and without dark windows and require the two to be indistinguishable.
    int32_t with_windows, without_windows;
    for (int pass = 0; pass < 2; ++pass) {
        prox_hmm_reset(PROX_CRIT_STAY_NEAR);
        for (int i = 0; i < 20; ++i) {
            advance_ms(60000);
            feed_loco();
            if (pass == 0) {
                prox_obs_begin(PDR_MAC);
                for (int j = 0; j < 40; ++j) prox_obs_note(PDR_MAC, legacy_minor, -70);
                prox_obs_close(&h, &c);
            }
            prox_hmm_tick(NULL);
        }
        (pass == 0 ? with_windows : without_windows) = prox_hmm_logodds_q8();
    }
    CHECK(with_windows == without_windows,
          "20 dark windows must leave the posterior exactly where no windows would "
          "(%d vs %d)", (int)with_windows, (int)without_windows);
}

static void test_pdr_loglr(void) {
    begin("PDR log-LR: monotone in hit rate, capped asymmetrically");

    const int all[4] = {1, 1, 1, 1};
    int lo[4];

    // Moving wrist: full-weight slots, so 4 cycles is enough to hit both caps.
    int32_t prev = 0;
    for (int hits = 4; hits >= 0; --hits) {
        prox_hmm_reset(PROX_CRIT_STAY_NEAR);
        feed_loco();
        advance_ms(1000);
        for (int i = 0; i < 4; ++i) lo[i] = (i < hits) ? 1 : 0;
        int32_t ll = window_loglr(10, 4, all, lo);
        if (hits < 4) {
            CHECK(ll <= prev, "log-LR must not rise as hits fall (%d hits -> %d, was %d)",
                  hits, (int)ll, (int)prev);
        }
        prev = ll;
    }

    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    feed_loco();
    advance_ms(1000);
    int32_t full = window_loglr(10, 4, all, all);
    CHECK(full == LL_PDR_NEAR_MAX_Q8,
          "four delivered LO slots should saturate the NEAR cap %d, got %d",
          LL_PDR_NEAR_MAX_Q8, (int)full);

    const int none[4] = {0, 0, 0, 0};
    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    feed_loco();
    advance_ms(1000);
    int32_t empty = window_loglr(10, 4, all, none);
    CHECK(empty == LL_PDR_AWAY_MAX_Q8,
          "four lost LO slots should saturate the AWAY cap %d, got %d",
          LL_PDR_AWAY_MAX_Q8, (int)empty);

    // The asymmetry is the §13.0 invariant, so assert it as such rather than
    // trusting two constants to stay in the right relation to each other.
    CHECK(-empty < full,
          "attenuation must not buy as much AWAY as delivery buys NEAR (%d vs %d)",
          (int)-empty, (int)full);

    begin("PDR: the accumulator cap preserves the estimated rate");
    prox_pdr_reset();
    const int alt[4] = {1, 0, 1, 0};
    for (int w = 0; w < 20; ++w) {                 // far more than PDR_MAX_EFF_SLOTS
        feed_loco();
        advance_ms(1000);
        uint8_t hh, cc;
        feed_window((uint16_t)(10 + 4 * w), 4, all, alt, &hh, &cc);
    }
    uint8_t rate = 0, cov = 0;
    prox_pdr_state(&rate, &cov, NULL);
    CHECK(cov <= PDR_MAX_EFF_SLOTS,
          "covered count must stay bounded, got %u (cap %d)", cov, PDR_MAX_EFF_SLOTS);
    CHECK(rate > 100 && rate < 155,
          "a steady 50%% delivery rate must survive capping, read %u/255", rate);
}

static void test_pdr_tamper(void) {
    // The product constraint this whole feature is checked against: a user who
    // smothers the watch in bedding must not be able to manufacture AWAY. PDR is
    // the feature most exposed to that, because lost LO slots are exactly what
    // both occlusion and distance produce.
    begin("PDR: sustained total LO loss cannot drive the posterior to AWAY alone");

    const int all[4]  = {1, 1, 1, 1};
    const int none[4] = {0, 0, 0, 0};

    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    feed_still();
    advance_ms(60000);
    int32_t start = prox_hmm_logodds_q8();

    for (int i = 0; i < 200; ++i) {               // 200 polls, wrist never moves
        advance_ms(60000);
        prox_note_sleep_interval(60000, 0);
        feed_still();
        uint8_t h, c;
        feed_window((uint16_t)(10 + 4 * i), 4, all, none, &h, &c);
        prox_hmm_tick(NULL);
    }
    int32_t after = prox_hmm_logodds_q8();

    CHECK(after > -HMM_LAMBDA_MAX_Q8 / 2,
          "200 identical occluded windows saturated the posterior (lam=%d)", (int)after);
    CHECK(after < start,
          "total LO loss should still count for something (%d -> %d)", (int)start, (int)after);

    // And the same evidence arriving while the wrist is genuinely moving is
    // allowed to be much stronger, because those are independent draws.
    prox_hmm_reset(PROX_CRIT_STAY_NEAR);
    feed_loco();
    advance_ms(60000);
    for (int i = 0; i < 10; ++i) {
        advance_ms(60000);
        feed_loco();
        uint8_t h, c;
        feed_window((uint16_t)(10 + 4 * i), 4, all, none, &h, &c);
        prox_hmm_tick(NULL);
    }
    int32_t moving = prox_hmm_logodds_q8();
    CHECK(moving < after,
          "10 moving windows must outweigh 200 frozen ones (moving=%d frozen=%d)",
          (int)moving, (int)after);
    printf("    frozen 200 windows: lam=%d | moving 10 windows: lam=%d\n",
           (int)after, (int)moving);

    begin("PDR: a still wrist that starts moving discards its old slot counts");
    prox_pdr_reset();
    feed_still();
    advance_ms(1000);
    uint8_t h, c;
    for (int i = 0; i < 6; ++i) {                 // build up a NEAR-ish history
        feed_still();
        advance_ms(1000);
        feed_window((uint16_t)(10 + 4 * i), 4, all, all, &h, &c);
    }
    uint8_t cov_before = 0;
    prox_pdr_state(NULL, &cov_before, NULL);

    feed_loco();                                  // wrist starts walking
    advance_ms(1000);
    feed_window(200, 4, all, all, &h, &c);
    uint8_t cov_after = 0;
    prox_pdr_state(NULL, &cov_after, NULL);

    // The two weights differ by 10x, so raw magnitudes are not comparable: six
    // STILL windows bank 2.25 slots, and one moving window is worth 4 outright.
    // The invariant is that nothing of the still history survives the restart,
    // so the count must be exactly the new window's 4 — it would read 6 if the
    // old 2.25 had been carried across.
    CHECK(cov_after == 4,
          "STILL -> moving must restart the window (had %u slots, expected exactly "
          "the fresh 4, got %u)", cov_before, cov_after);
}

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
    test_field_traces();
    test_beacon_minor();
    test_scan_cache();
    test_pdr_window();
    test_pdr_schedule_disabled();
    test_pdr_loglr();
    test_pdr_tamper();

    printf("\n%d checks, %d failures\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
