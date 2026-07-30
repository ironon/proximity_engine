// ============================================================
//  Spike S1a / S2 bench — shared TX/RX configuration
//
//  Both roles must agree on the level ladder, the slot timing and the
//  cycle->level mapping, because the receiver derives the *denominator* of
//  every hit ratio arithmetically from the cycle_seq range it observed rather
//  than from anything the transmitter reports. See README.md.
// ============================================================
#pragma once
#include <stdint.h>

// ── Level ladder ─────────────────────────────────────────────
// Every 3 dB step that esp_power_level_t can express on the C3. NimBLE maps
// dBm -> index as (dbm/3 + ESP_PWR_LVL_N0) with ESP_PWR_LVL_N0 == 8, so:
//   +9 -> 11 (P9), 0 -> 8 (N0), -21 -> 1 (N21), -24 -> 0 (N24).
// -27 would compute index -1 and fail, so -24 dBm is the hard floor.
// P12..P21 exist in the enum; whether the C3's PHY honours them is one of the
// things this bench measures rather than assumes.
#define BENCH_N_LEVELS 12
static const int8_t BENCH_LEVEL_DBM[BENCH_N_LEVELS] = {
    -24, -21, -18, -15, -12, -9, -6, -3, 0, 3, 6, 9
};

// ── Slot timing (mirrors the real schedule) ──────────────────
#define BENCH_SLOT_MS           250    // == BEACON_SLOT_MS
#define BENCH_ADV_ITVL_MS       50     // == BEACON_ADV_INTERVAL_MS, ~5 PDU/slot
#define BENCH_CYCLES_PER_LEVEL  40     // 40 cycles x 500 ms = 20 s per level

// ── Minor slot_id encoding for the sweep ─────────────────────
// Chosen to stay consistent with prox_beacon_slot_is_lo() (slot_id >= 3 is a
// reduced-power slot): 0 is the full-power reference, 3+idx is the swept slot
// at BENCH_LEVEL_DBM[idx]. Max id is 3+11 = 14, inside the Minor's 4-bit field.
#define BENCH_SLOT_HI           0
#define BENCH_SLOT_LO_BASE      3

#define BENCH_TX_HI_DBM         9      // reference slot, == PROX_QUERY_TX_POWER_DBM

// Receiver duty. Mirrors the watch's PROX_QUERY_SCAN_DUTY_MS, which lives in
// WatchFIrmware/src/main.cpp rather than the engine, so it is restated here.
#define BENCH_SCAN_DUTY_MS      100    // window == interval == ~100% duty

// Spike S2 GATT peer: the transmitter must stay connectable while its advertiser
// is reconfigured every slot. One service, one readable characteristic whose
// value the receiver verifies, so a "success" means the whole exchange survived
// and not merely that the link came up.
#define BENCH_SVC_UUID          "0000f00d-0000-1000-8000-00805f9b34fb"
#define BENCH_CHR_UUID          "0000beef-0000-1000-8000-00805f9b34fb"
#define BENCH_S2_VALUE          "s2ok"

// Level in effect for a given cycle. Both roles compute this identically; the
// receiver also cross-checks it against the slot_id it actually decoded.
static inline uint8_t bench_level_for_cycle(uint16_t cycle_seq) {
    return (uint8_t)((cycle_seq / BENCH_CYCLES_PER_LEVEL) % BENCH_N_LEVELS);
}
