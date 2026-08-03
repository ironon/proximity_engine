// ============================================================================
//  Corpus replay — run the CURRENT engine over PAST captures
// ============================================================================
//
// The point of capturing the engine's inputs rather than its conclusions is
// this program. Every session ever recorded can be re-scored by whatever the
// engine happens to be today, which turns "did that change help?" from an
// argument into a number.
//
// Two things are checked per frame, and they are different questions:
//
//   AGREEMENT — does today's engine reproduce what the device said at the time?
//     A disagreement is not automatically a failure. If you deliberately
//     changed the scoring, EVERY frame should disagree, and the interesting
//     number is by how much and in which direction. Use --expect-drift to say
//     you meant it.
//
//   TRUTH — does today's engine get the LABELLED answer right?
//     This is the one that matters. It only works on labelled frames, which is
//     why capture_ingest.py refuses to call an unlabelled session scoreable.
//
// Input is the derived .replay format (capture_ingest.py replay-export), not
// JSONL: parsing JSON here would be more code and more risk than the corpus is
// worth, and the .replay file is regenerable from the archive at any time.
//
//   make corpus                     # every session in ../corpus
//   ./run_replay session.replay     # one session, verbose

#include "proximity.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
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

// ---------------------------------------------------------------- helpers

struct Dev  { uint8_t mac[6]; uint8_t type; int8_t rssi; };
struct FpEntry { uint8_t mac[6]; uint8_t type; float mu, var, W; };

static int parse_mac(const char* s, uint8_t out[6]) {
    unsigned v[6];
    if (sscanf(s, "%x:%x:%x:%x:%x:%x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6)
        return 0;
    for (int i = 0; i < 6; ++i) out[i] = (uint8_t)v[i];
    return 1;
}

static void wr_u16(uint8_t* p, uint16_t v) { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)(v >> 8); }
static void wr_f32(uint8_t* p, float v)    { memcpy(p, &v, 4); }

// Rebuild the anchor's fingerprint through the engine's own loader, so replay
// enters the registry by the same door the firmware does — no back channel that
// could drift from the real load path.
static void load_fingerprint(const std::vector<FpEntry>& fp) {
    if (fp.empty()) return;
    std::vector<uint8_t> blob(2 + fp.size() * 19);
    wr_u16(&blob[0], (uint16_t)fp.size());
    size_t off = 2;
    for (size_t i = 0; i < fp.size(); ++i) {
        memcpy(&blob[off], fp[i].mac, 6); off += 6;
        blob[off++] = fp[i].type;
        wr_f32(&blob[off], fp[i].mu);  off += 4;
        wr_f32(&blob[off], fp[i].var); off += 4;
        wr_f32(&blob[off], fp[i].W);   off += 4;
    }
    prox_load_fingerprint(&blob[0], blob.size());
}

struct Frame {
    std::vector<FpEntry> fp;
    std::vector<Dev>     cache;
    std::vector<Dev>     vec;
    std::vector<std::string> labels;
    int    exp_score, exp_flags;
    float  exp_cm, exp_mad, exp_sigma, exp_Lb, exp_Li;
    bool   have_expect;
    Frame() : exp_score(0), exp_flags(0), exp_cm(0), exp_mad(0), exp_sigma(0),
              exp_Lb(0), exp_Li(0), have_expect(false) {}
};

// A label is "near" ground truth if it says the watch was where the anchor is.
// Deliberately conservative: anything it does not recognise scores nothing
// rather than guessing, because a mislabelled frame is worse than a missing one.
static int label_truth(const std::vector<std::string>& labels) {
    for (size_t i = 0; i < labels.size(); ++i) {
        const std::string& s = labels[i];
        if (s.find("[near]") != std::string::npos) return 1;
        if (s.find("[away]") != std::string::npos) return 0;
    }
    return -1;   // unlabelled for scoring purposes
}

struct Stats {
    int frames, scored, agree, disagree, truth_frames, truth_ok;
    int max_abs_drift, sum_abs_drift;
    int occl_flagged;
    Stats() : frames(0), scored(0), agree(0), disagree(0), truth_frames(0),
              truth_ok(0), max_abs_drift(0), sum_abs_drift(0), occl_flagged(0) {}
};

static void run_frame(Frame& f, Stats& st, int verbose, int near_thr_hint) {
    if (f.vec.empty() || !f.have_expect) return;
    st.frames++;

    prox_init();
    load_fingerprint(f.fp);
    for (size_t i = 0; i < f.cache.size(); ++i)
        prox_ingest_scan_result(f.cache[i].mac, f.cache[i].type, f.cache[i].rssi);

    ProxScanVector v;
    v.count = (uint8_t)(f.vec.size() > PROX_MAX_DEVICES ? PROX_MAX_DEVICES
                                                        : f.vec.size());
    for (int i = 0; i < v.count; ++i) {
        memcpy(v.devices[i].mac, f.vec[i].mac, 6);
        v.devices[i].type = f.vec[i].type;
        v.devices[i].rssi = f.vec[i].rssi;
    }

    ProxScoreResult r = prox_compute_score(&v);
    st.scored++;

    const int drift = (int)r.score - f.exp_score;
    const int adrift = drift < 0 ? -drift : drift;
    st.sum_abs_drift += adrift;
    if (adrift > st.max_abs_drift) st.max_abs_drift = adrift;
    if (adrift == 0) st.agree++; else st.disagree++;
    if (r.flags & PROX_FLAG_COMMON_MODE) st.occl_flagged++;

    float cm = 0, mad = 0, sigma = 0, Lb = 0, Li = 0;
    int n = 0;
    prox_last_common_mode(&cm, &mad, &sigma, &n, &Lb, &Li);

    const int truth = label_truth(f.labels);
    if (truth >= 0) {
        st.truth_frames++;
        // The verdict the enforcement path would reach, using the calibrated
        // cutoff the anchor reported at capture time when there was one.
        const int thr = near_thr_hint ? near_thr_hint : PROX_CONFIDENCE_THRESHOLD_U8;
        const int said_near = (r.score >= thr) ? 1 : 0;
        if (said_near == truth) st.truth_ok++;
        else if (verbose)
            printf("    MISS  label=%s score=%d thr=%d  cm=%+.1f mad=%.1f sig=%.1f\n",
                   truth ? "near" : "away", (int)r.score, thr, cm, mad, sigma);
    }

    if (verbose > 1) {
        printf("    score %3d (was %3d, %+d)  cm=%+.1f mad=%.1f sig=%.1f "
               "Lb=%.2f Li=%.2f  n=%d\n",
               (int)r.score, f.exp_score, drift, cm, mad, sigma, Lb, Li, n);
    }
}

int main(int argc, char** argv) {
    int verbose = 1;
    int expect_drift = 0;
    std::vector<std::string> files;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "-q") verbose = 0;
        else if (a == "-v") verbose = 2;
        else if (a == "--expect-drift") expect_drift = 1;
        else files.push_back(a);
    }
    if (files.empty()) {
        fprintf(stderr,
                "usage: run_replay [-q|-v] [--expect-drift] <session.replay>...\n");
        return 2;
    }

    Stats total;
    int sessions = 0, bad_sessions = 0;

    for (size_t fi = 0; fi < files.size(); ++fi) {
        FILE* fh = fopen(files[fi].c_str(), "r");
        if (!fh) { fprintf(stderr, "cannot open %s\n", files[fi].c_str()); return 2; }

        Stats st;
        Frame cur;
        std::string build;
        char line[512];
        int pending = 0;          // remaining continuation lines
        int pending_kind = 0;     // 1 FP, 2 CACHE, 3 VEC

        while (fgets(line, sizeof(line), fh)) {
            if (line[0] == '#' || line[0] == '\n') continue;

            if (pending > 0) {
                char mac[64];
                if (pending_kind == 1) {
                    FpEntry e; float mu, var, W; int type;
                    if (sscanf(line, " %63s %d %f %f %f", mac, &type, &mu, &var, &W) == 5
                        && parse_mac(mac, e.mac)) {
                        e.type = (uint8_t)type; e.mu = mu; e.var = var; e.W = W;
                        cur.fp.push_back(e);
                    }
                } else {
                    Dev d; int type, rssi;
                    if (sscanf(line, " %63s %d %d", mac, &type, &rssi) == 3
                        && parse_mac(mac, d.mac)) {
                        d.type = (uint8_t)type; d.rssi = (int8_t)rssi;
                        if (pending_kind == 2) cur.cache.push_back(d);
                        else                   cur.vec.push_back(d);
                    }
                }
                pending--;
                continue;
            }

            int n = 0;
            char text[256];
            if (sscanf(line, "SESSION %255[^\n]", text) == 1) {
                build = text;
            } else if (strncmp(line, "FRAME", 5) == 0) {
                run_frame(cur, st, verbose, 0);
                cur = Frame();
            } else if (sscanf(line, "LABEL %255[^\n]", text) == 1) {
                cur.labels.push_back(text);
            } else if (sscanf(line, "FP %d", &n) == 1) {
                cur.fp.clear(); pending = n; pending_kind = 1;
            } else if (sscanf(line, "CACHE %d", &n) == 1) {
                cur.cache.clear(); pending = n; pending_kind = 2;
            } else if (sscanf(line, "VEC %d", &n) == 1) {
                cur.vec.clear(); pending = n; pending_kind = 3;
            } else if (strncmp(line, "EXPECT", 6) == 0) {
                if (sscanf(line, "EXPECT %d %d %f %f %f %f %f",
                           &cur.exp_score, &cur.exp_flags, &cur.exp_cm,
                           &cur.exp_mad, &cur.exp_sigma, &cur.exp_Lb,
                           &cur.exp_Li) == 7)
                    cur.have_expect = true;
            }
        }
        run_frame(cur, st, verbose, 0);
        fclose(fh);

        sessions++;
        if (verbose) {
            printf("%s\n", files[fi].c_str());
            if (!build.empty()) printf("  captured by: %s\n", build.c_str());
            printf("  frames %d  agree %d  disagree %d", st.frames, st.agree,
                   st.disagree);
            if (st.scored)
                printf("  (mean |drift| %.1f, max %d)",
                       (double)st.sum_abs_drift / st.scored, st.max_abs_drift);
            printf("\n");
            if (st.truth_frames)
                printf("  labelled %d  correct %d  (%.0f%%)\n", st.truth_frames,
                       st.truth_ok,
                       100.0 * st.truth_ok / (double)st.truth_frames);
            else
                printf("  labelled 0  — nothing to score against\n");
            if (st.occl_flagged)
                printf("  common-mode flagged on %d frame(s)\n", st.occl_flagged);
        }

        // A session fails if the engine no longer reproduces itself and nobody
        // said they meant to change it. Drift is the loud signal that a tuning
        // change has landed — it should never be a surprise.
        if (st.disagree && !expect_drift) bad_sessions++;

        total.frames += st.frames; total.scored += st.scored;
        total.agree += st.agree;   total.disagree += st.disagree;
        total.truth_frames += st.truth_frames; total.truth_ok += st.truth_ok;
        total.sum_abs_drift += st.sum_abs_drift;
        total.occl_flagged += st.occl_flagged;
        if (st.max_abs_drift > total.max_abs_drift)
            total.max_abs_drift = st.max_abs_drift;
    }

    printf("\n%d session(s), %d frames: %d agree, %d disagree",
           sessions, total.frames, total.agree, total.disagree);
    if (total.scored)
        printf(" (mean |drift| %.1f, max %d)",
               (double)total.sum_abs_drift / total.scored, total.max_abs_drift);
    printf("\n");
    if (total.truth_frames)
        printf("ground truth: %d/%d correct (%.0f%%)\n", total.truth_ok,
               total.truth_frames,
               100.0 * total.truth_ok / (double)total.truth_frames);

    if (bad_sessions && !expect_drift) {
        printf("\n%d session(s) disagree with their capture. If you changed the\n"
               "scoring on purpose, re-run with --expect-drift and read the\n"
               "drift numbers; otherwise this is a regression.\n", bad_sessions);
        return 1;
    }
    return 0;
}
