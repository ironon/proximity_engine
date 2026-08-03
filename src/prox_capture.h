// ============================================================================
//  prox_capture — structured capture of the proximity engine's INPUTS
// ============================================================================
//
// This is not logging. The serial output is logging: it is for a human reading
// a terminal, it is prose, and it stays exactly as it is. Nothing here replaces
// a Serial.printf.
//
// This is a DATA CHANNEL. Its consumer is a replay harness on a laptop, and its
// job is to make a run reproducible months later against an engine that has
// since changed.
//
// ── The principle: capture inputs, not conclusions ──────────────────────────
//
// A corpus of scores dies the moment the scoring changes — you cannot
// re-evaluate last month's night against this month's engine. A corpus of
// INPUTS replays against every future version forever. So the records below are
// arranged around the engine's entry points, not around what happened:
//
//     prox_ingest_scan_result   -> CAP_ANCHOR_CACHE
//     prox_deserialize_vector   -> CAP_VECTOR
//     prox_obs_note             -> CAP_BEACON
//     prox_note_motion_*        -> CAP_MOTION / CAP_SLEEP
//     prox_load_fingerprint     -> CAP_FINGERPRINT
//     (the clock)               -> every record's t_ms / t_wall
//
// Replay is re-issuing those calls in order. That is complete BY CONSTRUCTION:
// if every non-const entry point in proximity.h has a record type, any run can
// be reproduced exactly. When you add an entry point, add a record.
//
// Engine OUTPUTS (CAP_SCORE, CAP_HMM, CAP_AWAYGATE) are captured too, but they
// are assertions, not inputs. Replay recomputes them; a mismatch is either a
// regression or a deliberate change you are about to review. That is what makes
// the corpus a test suite rather than an archive.
//
// ── Gaps are explicit ───────────────────────────────────────────────────────
//
// `seq` increments on every ATTEMPTED emit, including ones dropped for want of
// buffer. The receiver therefore detects loss for free, from the sequence
// discontinuity alone. A corpus that silently drops records is worse than one
// with holes in it, because you will trust it.
//
// ── Capture time is decoupled from transmit time ────────────────────────────
//
// Records are timestamped and buffered on emit; they leave the device only when
// prox_capture_flush() is called. Call that when the radio is otherwise idle —
// after a scan window closes, never during one. On a C3 the BLE and WiFi share
// one radio behind a coex arbiter, so transmitting mid-scan drops
// advertisements, shrinks the scan vector, and corrupts the very measurement
// being captured. Logging that perturbs the experiment is worse than none.
//
// ── Transport is the least important decision here ──────────────────────────
//
// Everything above is independent of how bytes reach the laptop. The sink is
// one function pointer. Today the firmwares hand it a UDP broadcast; swapping
// that for ESP-NOW, a serial framer or an SD card touches nothing else.

#ifndef PROX_CAPTURE_H
#define PROX_CAPTURE_H

#include <stdint.h>
#include <stddef.h>

#include "proximity.h"

// Master switch. Off costs nothing: every entry point compiles to nothing.
#ifndef PROX_CAPTURE_ENABLED
#define PROX_CAPTURE_ENABLED 1
#endif

// Buffer for records awaiting a flush. Sized for a burst between flushes, not
// for a session — a vector record is ~250 bytes and flushes happen every query,
// so this holds roughly thirty records of slack for a stalled transmit.
#ifndef PROX_CAPTURE_BUF_BYTES
#define PROX_CAPTURE_BUF_BYTES 8192
#endif

#define PROX_CAPTURE_MAGIC   0xC5u
#define PROX_CAPTURE_VERSION 1u

// Short git SHA of the firmware that produced a session. Injected from
// platformio.ini; "dev" means someone built with uncommitted changes, which is
// itself worth knowing when a capture looks strange.
#ifndef PROX_FW_SHA
#define PROX_FW_SHA "dev"
#endif

// Roles, so one collector can hold several devices in one timeline.
#define CAP_ROLE_WATCH   0u
#define CAP_ROLE_ANCHOR  1u

// Record types. APPEND ONLY — these numbers appear in every corpus file ever
// written, and renumbering silently reinterprets historical captures.
#define CAP_SESSION      0x01u  // provenance; first record after boot
#define CAP_VECTOR       0x02u  // the watch's scan vector (engine input)
#define CAP_ANCHOR_CACHE 0x03u  // what the anchor itself hears (engine input)
#define CAP_SCORE        0x04u  // Mode A result + all scoring diagnostics
#define CAP_MOTION       0x05u  // one IMU burst (engine input)
#define CAP_HMM          0x06u  // one decision-layer tick (output/assertion)
#define CAP_ENFORCE      0x07u  // window and condition transitions
#define CAP_LABEL        0x08u  // ground truth, typed by a human
#define CAP_FINGERPRINT  0x09u  // registry snapshot: replay needs initial state
#define CAP_BEACON       0x0Au  // prox_obs_note (engine input, PDR)
#define CAP_SLEEP        0x0Bu  // a light-sleep interval (engine input)
#define CAP_AWAYGATE     0x0Cu  // §13.4-R6 gate state (output/assertion)

// Wire header, little-endian, packed. 20 bytes, then `len` bytes of payload.
typedef struct __attribute__((packed)) {
    uint8_t  magic;      // PROX_CAPTURE_MAGIC — resync point for a byte stream
    uint8_t  ver;        // PROX_CAPTURE_VERSION
    uint8_t  type;       // CAP_*
    uint8_t  role;       // CAP_ROLE_*
    uint32_t seq;        // per-device, increments on every ATTEMPTED emit
    uint32_t t_ms;       // device millis() at capture
    uint32_t t_wall;     // unix seconds, 0 if the clock is unset
    uint16_t len;        // payload bytes following this header
    uint16_t reserved;
} ProxCapHeader;

// ── Payloads ────────────────────────────────────────────────────────────────
// Every payload is packed and little-endian. Floats are IEEE-754 binary32,
// which is what both the C3 and the laptop use natively.

typedef struct __attribute__((packed)) {
    uint8_t  device_uuid[16];
    uint32_t boot_count;
    uint32_t engine_cfg_hash;  // hash of the compile-time engine constants
    char     fw_sha[16];       // git describe / short SHA, NUL-padded
    uint8_t  v2_authoritative;
    uint8_t  r2_offset_invariant;
    uint8_t  reserved[2];
} ProxCapSession;

// Followed by `count` ProxCapDev entries.
typedef struct __attribute__((packed)) {
    uint8_t mac[6];
    uint8_t type;
    int8_t  rssi;
} ProxCapDev;

typedef struct __attribute__((packed)) {
    uint8_t score;
    uint8_t flags;
    uint8_t near_thr;
    int8_t  self_rssi;
    int16_t self_delta;
    uint16_t shared;        // Signal A's k
    uint16_t fp_seen;       // Signal B's nb
    uint16_t vec_count;
    float   cm_delta;       // §13.4-R2 common-mode estimate
    float   cm_mad;
    float   cm_sigma;
    float   L_legacy;       // Signal B scored the old way
    float   L_invariant;    // ...and the new way
} ProxCapScore;

typedef struct __attribute__((packed)) {
    uint32_t burst_var;
    uint8_t  cadence;
    uint8_t  state;         // PROX_MOTION_*
    uint8_t  ints;
    uint8_t  reserved;
} ProxCapMotion;

typedef struct __attribute__((packed)) {
    int32_t lambda_q8;
    int32_t emit_q8;
    uint8_t decision;       // ProxProximity
    uint8_t motion_state;
    uint8_t neff;
    uint8_t score;
} ProxCapHmm;

typedef struct __attribute__((packed)) {
    uint8_t  event;         // ProxCapEnforceEvent
    uint8_t  criteria;
    uint8_t  condition_met;
    uint8_t  reserved;
    uint8_t  event_uuid[16];
} ProxCapEnforce;

#define CAP_ENF_WINDOW_OPEN   0u
#define CAP_ENF_WINDOW_CLOSE  1u
#define CAP_ENF_MET           2u   // condition became met (outputs stop)
#define CAP_ENF_UNMET         3u   // condition became unmet (outputs start)

typedef struct __attribute__((packed)) {
    uint8_t  armed;
    uint8_t  hits;
    uint8_t  admits;
    uint8_t  motion_state;
    uint32_t still_for_s;
} ProxCapAwayGate;

typedef struct __attribute__((packed)) {
    uint32_t slept_ms;
    uint8_t  motion_woke;
    uint8_t  reserved[3];
} ProxCapSleep;

typedef struct __attribute__((packed)) {
    uint8_t  mac[6];
    uint16_t minor;
    int8_t   rssi;
    uint8_t  reserved[3];
} ProxCapBeacon;

// Followed by `count` ProxCapFpDev entries. Large; emitted at session start and
// after training folds, not per query.
typedef struct __attribute__((packed)) {
    uint8_t mac[6];
    uint8_t type;
    uint8_t reserved;
    float   mu;
    float   var;
    float   W;
} ProxCapFpDev;

// ── API ─────────────────────────────────────────────────────────────────────

#if PROX_CAPTURE_ENABLED

// Drains up to `len` bytes to wherever the corpus is collected. Returns
// non-zero on success; a failure leaves the record buffered for the next flush.
typedef int (*ProxCaptureSink)(const uint8_t *buf, uint16_t len);

void prox_capture_init(uint8_t role, ProxCaptureSink sink);

// FNV-1a over the compile-time constants that actually change what the engine
// decides. Computed rather than hand-maintained: a version integer someone has
// to remember to bump is a version integer that silently goes stale, and then a
// corpus quietly mixes two engines under one label. Two sessions with the same
// hash were scored by the same configuration; different hashes mean do not
// pool them without looking.
uint32_t prox_capture_cfg_hash(void);

// Wall clock for subsequent records. 0 until the device has one; records
// emitted before then carry t_wall = 0 and are aligned by t_ms alone.
void prox_capture_set_wall(uint32_t unix_s);

// Buffer one record. Never transmits, never blocks.
void prox_capture_emit(uint8_t type, const void *payload, uint16_t len);

// Buffer a record whose payload is a fixed head followed by an array.
void prox_capture_emit2(uint8_t type, const void *head, uint16_t head_len,
                        const void *body, uint16_t body_len);

// Push buffered records through the sink. Call ONLY where the radio is free.
void prox_capture_flush(void);

// Records attempted and records actually written, for a health line.
void prox_capture_stats(uint32_t *out_seq, uint32_t *out_dropped);

// ── Typed helpers for the engine's entry points ─────────────────────────────
void prox_capture_vector(const ProxScanVector *v);

// Anchor-role only: these read the device registry and the Mode A scoring
// diagnostics, neither of which exists in a watch build (proximity.h gates them
// on PROXIMITY_ROLE_ANCHOR). The watch captures what only IT knows — motion,
// sleep, HMM ticks, enforcement transitions — and the two halves are stitched
// into one timeline on the laptop.
#ifdef PROXIMITY_ROLE_ANCHOR
void prox_capture_anchor_cache(void);
void prox_capture_score_result(const ProxScoreResult *r, uint8_t near_thr,
                               int8_t self_rssi, uint16_t vec_count);
void prox_capture_fingerprint(void);
#endif

#else   // capture compiled out

typedef int (*ProxCaptureSink)(const uint8_t *buf, uint16_t len);
#define prox_capture_init(role, sink)                 ((void)0)
#define prox_capture_set_wall(s)                      ((void)0)
#define prox_capture_emit(t, p, l)                    ((void)0)
#define prox_capture_emit2(t, h, hl, b, bl)           ((void)0)
#define prox_capture_flush()                          ((void)0)
#define prox_capture_stats(a, b)                      ((void)0)
#define prox_capture_vector(v)                        ((void)0)
#define prox_capture_anchor_cache()                   ((void)0)
#define prox_capture_score_result(r, n, s, c)         ((void)0)
#define prox_capture_fingerprint()                    ((void)0)

#endif  // PROX_CAPTURE_ENABLED
#endif  // PROX_CAPTURE_H
