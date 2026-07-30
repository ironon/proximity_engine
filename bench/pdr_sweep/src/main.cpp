// ============================================================
//  Spike S1a / S2 bench firmware — TX and RX roles
//
//  Purpose (proximity_engine/tests/spike_s1a_txlo.md):
//    1. Establish that NimBLEDevice::setPower() actually reaches the PHY, and
//       what dBm each commanded level really produces. Every downstream PDR
//       number is void if this is not true, so it is measured first.
//    2. Measure the per-slot hit ratio the engine's PDR feature is defined over
//       (engine spec v2.1 §3.3), with the LO slot's TX power swept across the
//       whole ladder.
//    3. Answer Spike S2: does a 250 ms stop/reconfigure/start cycle on the
//       advertiser keep the device connectable?
//
//  Deliberately standalone rather than bolted onto the product firmwares: no
//  provisioning, no pairing, no enforcement state machine, no light sleep. It
//  does link the real engine, so prox_beacon_minor_encode/decode and
//  prox_beacon_tick are the shipped implementations, not reimplementations.
//
//  Everything is driven over serial so a whole experiment runs from a script;
//  send `?` for the command list.
// ============================================================
#include <Arduino.h>
#include <NimBLEDevice.h>

#include "proximity.h"
#include "bench_cfg.h"

// ── Engine platform seams ────────────────────────────────────
// The bench keeps no persistent state: a spike that silently resumed a previous
// run's registry would be measuring the wrong thing.
extern "C" uint32_t prox_platform_now_ms(void) { return millis(); }
extern "C" int prox_platform_nvs_load(const char*, void*, size_t, size_t*) { return 0; }
extern "C" int prox_platform_nvs_save(const char*, const void*, size_t) { return 1; }

// ── Serial line reader (shared) ──────────────────────────────
static char  g_line[64];
static size_t g_line_len = 0;
static void handle_line(char* s);

static void serial_service() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            g_line[g_line_len] = '\0';
            if (g_line_len) handle_line(g_line);
            g_line_len = 0;
            continue;
        }
        if (g_line_len < sizeof(g_line) - 1) g_line[g_line_len++] = c;
    }
}

// ============================================================
//  TX role — swept-power slot transmitter
// ============================================================
#if BENCH_ROLE_TX

static const uint8_t g_uuid[16] = {
    0x49, 0x4D, 0x50, 0x55, 0x4C, 0x53, 0x45, 0x00,
    0x53, 0x31, 0x41, 0x42, 0x45, 0x4E, 0x43, 0x48   // "IMPULSE\0S1ABENCH"
};

static bool     g_sweep      = false;
static bool     g_restart    = true;   // stop()/start() around each slot change
static int8_t   g_fixed_dbm  = 127;    // 127 = follow the sweep ladder
static uint16_t g_cycle      = 1;      // never 0: see prox_beacon_minor_decode
static bool     g_slot_lo    = false;
static uint32_t g_next_ms    = 0;
static uint8_t  g_last_level = 0xFF;

// Byte-for-byte the anchor's payload (AnchorFirmware/src/main.cpp
// set_beacon_payload) so the receiver's parse is the product's parse.
static void set_beacon_payload(NimBLEAdvertising* pAdv, uint16_t minor) {
    NimBLEAdvertisementData advData;
    uint8_t msd[25];
    msd[0] = 0xFF; msd[1] = 0xFF;   // Impulse (custom, not Apple)
    msd[2] = 0x02; msd[3] = 0x15;   // type / length markers
    memcpy(msd + 4, g_uuid, 16);
    msd[20] = 0x4A; msd[21] = 0x0F; // Major = 0x4A0F
    msd[22] = (uint8_t)(minor >> 8);
    msd[23] = (uint8_t)(minor & 0xFF);
    msd[24] = 0xC5;
    advData.setManufacturerData(std::string((char*)msd, 25));
    pAdv->setAdvertisementData(advData);
}

// One slot boundary. Mirrors prox_platform_set_beacon_slot, except the TX power
// is applied to the *advertiser only*. The product code passes NimBLE's default
// NimBLETxPowerType::All, which also reprograms SCAN and DEFAULT (connection)
// power — see README.md "Finding 2".
static void apply_slot(int8_t dbm, uint8_t slot_id) {
    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    if (g_restart) pAdv->stop();
    bool ok = NimBLEDevice::setPower(dbm, NimBLETxPowerType::Advertise);
    set_beacon_payload(pAdv, prox_beacon_minor_encode(slot_id, g_cycle));
    if (g_restart) pAdv->start();
    if (!ok) Serial.printf("TX setPower(%d) REJECTED\n", (int)dbm);
}

// Pure-API probe: which dBm values does this stack accept, and what does it
// read back? Answers the "setPower silently rounds or ignores" trap without
// needing a receiver at all.
static void cmd_probe() {
    Serial.println("PROBE req_dbm,accepted,readback_dbm");
    for (int dbm = -30; dbm <= 21; dbm += 1) {
        bool ok = NimBLEDevice::setPower((int8_t)dbm, NimBLETxPowerType::Advertise);
        int  rb = NimBLEDevice::getPower(NimBLETxPowerType::Advertise);
        Serial.printf("PROBE %d,%d,%d\n", dbm, ok ? 1 : 0, rb);
        delay(5);
    }
    NimBLEDevice::setPower(BENCH_TX_HI_DBM, NimBLETxPowerType::Advertise);
    Serial.println("PROBE done");
}

// Spike S2: the transmitter must remain a real GATT peer while its advertiser is
// reconfigured every 250 ms — the anchor has to stay connectable in every slot
// (§3.1). Counted here, verified from the receiver side.
static uint32_t g_connects = 0;
static uint32_t g_disconnects = 0;

static void handle_line(char* s) {
    if (!strcmp(s, "?") || !strcmp(s, "help")) {
        Serial.println("TX cmds: probe | sweep on|off | fixed <dbm>|auto | restart on|off | stat");
        return;
    }
    if (!strcmp(s, "probe")) { cmd_probe(); return; }
    if (!strcmp(s, "sweep on")) {
        g_sweep = true; g_cycle = 1; g_slot_lo = false;
        g_next_ms = millis(); g_last_level = 0xFF;
        Serial.printf("TX sweep START cycles_per_level=%d levels=%d cycle_ms=%d\n",
                      BENCH_CYCLES_PER_LEVEL, BENCH_N_LEVELS, 2 * BENCH_SLOT_MS);
        return;
    }
    if (!strcmp(s, "sweep off")) {
        g_sweep = false;
        Serial.println("TX sweep STOP");
        return;
    }
    if (!strcmp(s, "restart on"))  { g_restart = true;  Serial.println("TX restart=on");  return; }
    if (!strcmp(s, "restart off")) { g_restart = false; Serial.println("TX restart=off"); return; }
    if (!strcmp(s, "fixed auto"))  { g_fixed_dbm = 127; Serial.println("TX fixed=auto");  return; }
    if (!strncmp(s, "fixed ", 6)) {
        g_fixed_dbm = (int8_t)atoi(s + 6);
        Serial.printf("TX fixed=%d\n", (int)g_fixed_dbm);
        // `fixed` only pins the level *inside* the slot loop, and that loop only
        // runs while the sweep is on. Setting it with the sweep off emits no LO
        // slots at all, which reads at the receiver as a perfect 0% PDR rather
        // than as "nothing was transmitted" — say so loudly instead.
        if (!g_sweep)
            Serial.println("TX WARNING: sweep is OFF — no LO slots are being sent. "
                           "Send 'sweep on' or this level does nothing.");
        return;
    }
    if (!strcmp(s, "stat")) {
        Serial.printf("TX sweep=%d cycle=%u level=%u restart=%d adv_pwr=%d "
                      "conn=%u disc=%u heap=%u\n",
                      g_sweep ? 1 : 0, (unsigned)g_cycle,
                      (unsigned)bench_level_for_cycle(g_cycle), g_restart ? 1 : 0,
                      NimBLEDevice::getPower(NimBLETxPowerType::Advertise),
                      (unsigned)g_connects, (unsigned)g_disconnects,
                      (unsigned)ESP.getFreeHeap());
        return;
    }
    Serial.printf("TX ? unknown: %s\n", s);
}

class TxServerCB : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer*, NimBLEConnInfo&) override { g_connects++; }
    void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
        g_disconnects++;
        // The schedule owns the advertiser; re-arm so the next connect can land.
        NimBLEDevice::getAdvertising()->start();
    }
};
static TxServerCB g_srv_cb;

void setup() {
    Serial.begin(115200);
    delay(400);
    Serial.println("\n=== S1a bench: TX role ===");

    NimBLEDevice::init("S1a-TX");
    NimBLEDevice::setPower(BENCH_TX_HI_DBM, NimBLETxPowerType::Advertise);

    NimBLEServer* srv = NimBLEDevice::createServer();
    srv->setCallbacks(&g_srv_cb);
    NimBLEService* svc = srv->createService(BENCH_SVC_UUID);
    NimBLECharacteristic* chr =
        svc->createCharacteristic(BENCH_CHR_UUID, NIMBLE_PROPERTY::READ);
    chr->setValue(BENCH_S2_VALUE);
    svc->start();

    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->setName("S1a-TX");
    const uint16_t itvl = (uint16_t)(BENCH_ADV_ITVL_MS * 8 / 5);  // 0.625 ms units
    pAdv->setMinInterval(itvl);
    pAdv->setMaxInterval(itvl);
    set_beacon_payload(pAdv, prox_beacon_minor_encode(BENCH_SLOT_HI, g_cycle));
    pAdv->start();

    Serial.printf("TX ready. adv_itvl=%dms readback_pwr=%d dBm\n",
                  BENCH_ADV_ITVL_MS, NimBLEDevice::getPower(NimBLETxPowerType::Advertise));
    Serial.println("TX send 'probe' then 'sweep on'");
}

void loop() {
    serial_service();
    if (!g_sweep) { delay(5); return; }

    uint32_t now = millis();
    if ((int32_t)(now - g_next_ms) < 0) { delay(2); return; }
    g_next_ms += BENCH_SLOT_MS;
    if ((int32_t)(now - g_next_ms) > BENCH_SLOT_MS) g_next_ms = now + BENCH_SLOT_MS;

    const uint8_t level = bench_level_for_cycle(g_cycle);
    if (level != g_last_level && !g_slot_lo) {
        g_last_level = level;
        Serial.printf("TX level=%u dbm=%d cycle=%u\n",
                      (unsigned)level, (int)BENCH_LEVEL_DBM[level], (unsigned)g_cycle);
    }

    if (!g_slot_lo) {
        apply_slot(BENCH_TX_HI_DBM, BENCH_SLOT_HI);
        g_slot_lo = true;
    } else {
        const int8_t dbm = (g_fixed_dbm == 127) ? BENCH_LEVEL_DBM[level] : g_fixed_dbm;
        apply_slot(dbm, (uint8_t)(BENCH_SLOT_LO_BASE + level));
        g_slot_lo = false;
        if (++g_cycle > 0x0FFF) g_cycle = 1;   // skip 0, see prox_beacon_minor_decode
    }
}

#endif // BENCH_ROLE_TX

// ============================================================
//  RX role — per-slot hit counter
// ============================================================
#if BENCH_ROLE_RX

struct LevelStat {
    uint16_t cyc_hi, cyc_lo;        // distinct cycles in which the slot was heard
    uint16_t pdu_hi, pdu_lo;        // total PDUs (margin, not just presence)
    int32_t  rssi_sum_hi, rssi_sum_lo;
    int8_t   rssi_min_lo, rssi_max_lo;
    int8_t   rssi_min_hi, rssi_max_hi;
    uint16_t last_cyc_hi, last_cyc_lo;
    uint16_t slotid_mismatch;
};

static LevelStat g_st[BENCH_N_LEVELS];
static uint16_t  g_cyc_min = 0, g_cyc_max = 0;
static bool      g_have_cyc = false;
static bool      g_wrapped = false;
static uint32_t  g_other_impulse = 0;   // Impulse adverts with an all-zero Minor
static NimBLEScan* g_scan = nullptr;
static bool      g_active = false;

// Learned from the beacons themselves, so S2 never needs a hardcoded address.
static NimBLEAddress g_tx_addr;
static bool          g_tx_addr_known = false;
// Staleness watchdog for the S2 pass criterion "scan tasks stay fed": the gap
// between consecutive received beacons, which must not blow up while connects
// are hammering the advertiser.
static uint32_t g_last_beacon_ms = 0;
static uint32_t g_max_gap_ms     = 0;

static void stats_reset() {
    memset(g_st, 0, sizeof(g_st));
    for (int i = 0; i < BENCH_N_LEVELS; ++i) {
        g_st[i].rssi_min_lo = 127; g_st[i].rssi_max_lo = -127;
        g_st[i].rssi_min_hi = 127; g_st[i].rssi_max_hi = -127;
        g_st[i].last_cyc_hi = 0;   g_st[i].last_cyc_lo = 0;
    }
    g_cyc_min = g_cyc_max = 0;
    g_have_cyc = false;
    g_wrapped  = false;
    g_other_impulse = 0;
}

class RxCB : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* dev) override {
        if (!dev->haveManufacturerData()) return;
        std::string mfr = dev->getManufacturerData();
        if (mfr.size() < 25) return;
        if ((uint8_t)mfr[0] != 0xFF || (uint8_t)mfr[1] != 0xFF) return;
        if ((uint8_t)mfr[2] != 0x02 || (uint8_t)mfr[3] != 0x15) return;
        if ((uint8_t)mfr[20] != 0x4A || (uint8_t)mfr[21] != 0x0F) return;

        const uint16_t minor = (uint16_t)(((uint8_t)mfr[22] << 8) | (uint8_t)mfr[23]);
        uint8_t  slot_id = 0;
        uint16_t cycle   = 0;
        // The shipped decoder, not a local reimplementation: returns 0 for a
        // legacy all-zero Minor, which is how a non-scheduled anchor looks.
        if (!prox_beacon_minor_decode(minor, &slot_id, &cycle)) {
            g_other_impulse++;
            return;
        }

        const int8_t rssi = (int8_t)dev->getRSSI();

        if (!g_tx_addr_known) {
            g_tx_addr       = dev->getAddress();
            g_tx_addr_known = true;
        }
        const uint32_t now = millis();
        if (g_last_beacon_ms) {
            const uint32_t gap = now - g_last_beacon_ms;
            if (gap > g_max_gap_ms) g_max_gap_ms = gap;
        }
        g_last_beacon_ms = now;

        if (!g_have_cyc) {
            g_cyc_min = g_cyc_max = cycle;
            g_have_cyc = true;
        } else if (cycle < g_cyc_min) {
            g_wrapped = true;          // cycle_seq rolled over; denominators invalid
        } else if (cycle > g_cyc_max) {
            g_cyc_max = cycle;
        }

        const uint8_t level = bench_level_for_cycle(cycle);
        LevelStat* st = &g_st[level];

        if (slot_id == BENCH_SLOT_HI) {
            st->pdu_hi++;
            st->rssi_sum_hi += rssi;
            if (rssi < st->rssi_min_hi) st->rssi_min_hi = rssi;
            if (rssi > st->rssi_max_hi) st->rssi_max_hi = rssi;
            if (cycle != st->last_cyc_hi) { st->cyc_hi++; st->last_cyc_hi = cycle; }
        } else if (slot_id >= BENCH_SLOT_LO_BASE) {
            // Cross-check the arithmetic cycle->level map against the level the
            // transmitter actually stamped. A mismatch means the two roles are
            // out of sync and the whole table is suspect.
            if ((uint8_t)(slot_id - BENCH_SLOT_LO_BASE) != level) st->slotid_mismatch++;
            st->pdu_lo++;
            st->rssi_sum_lo += rssi;
            if (rssi < st->rssi_min_lo) st->rssi_min_lo = rssi;
            if (rssi > st->rssi_max_lo) st->rssi_max_lo = rssi;
            if (cycle != st->last_cyc_lo) { st->cyc_lo++; st->last_cyc_lo = cycle; }
        }
    }
};
static RxCB g_cb;

// Exact denominator: how many cycles in the observed [min,max] range belonged to
// each level. Derived arithmetically, so a level whose LO slot was never heard
// at all still has a valid denominator from the HI slots that bracket it.
static uint16_t cycles_for_level(uint8_t level) {
    if (!g_have_cyc) return 0;
    uint16_t n = 0;
    for (uint32_t c = g_cyc_min; c <= g_cyc_max; ++c)
        if (bench_level_for_cycle((uint16_t)c) == level) n++;
    return n;
}

static void cmd_report() {
    Serial.printf("RPT cyc_range=%u..%u wrapped=%d scan=%s legacy_minor=%u\n",
                  (unsigned)g_cyc_min, (unsigned)g_cyc_max, g_wrapped ? 1 : 0,
                  g_active ? "active" : "passive", (unsigned)g_other_impulse);
    Serial.println("RPT lvl,dbm,cycles,hi_hit,lo_hit,hi_pct,lo_pct,"
                   "hi_pdu,lo_pdu,hi_rssi,lo_rssi,lo_min,lo_max,mism");
    for (int i = 0; i < BENCH_N_LEVELS; ++i) {
        const LevelStat* st = &g_st[i];
        const uint16_t n = cycles_for_level((uint8_t)i);
        if (n == 0) continue;
        const int hi_pct = (int)((100L * st->cyc_hi) / n);
        const int lo_pct = (int)((100L * st->cyc_lo) / n);
        const int hi_r = st->pdu_hi ? (int)(st->rssi_sum_hi / (int32_t)st->pdu_hi) : 0;
        const int lo_r = st->pdu_lo ? (int)(st->rssi_sum_lo / (int32_t)st->pdu_lo) : 0;
        Serial.printf("RPT %d,%d,%u,%u,%u,%d,%d,%u,%u,%d,%d,%d,%d,%u\n",
                      i, (int)BENCH_LEVEL_DBM[i], (unsigned)n,
                      (unsigned)st->cyc_hi, (unsigned)st->cyc_lo, hi_pct, lo_pct,
                      (unsigned)st->pdu_hi, (unsigned)st->pdu_lo, hi_r, lo_r,
                      st->pdu_lo ? st->rssi_min_lo : 0,
                      st->pdu_lo ? st->rssi_max_lo : 0,
                      (unsigned)st->slotid_mismatch);
    }
    Serial.println("RPT end");
}

static void scan_restart() {
    if (g_scan->isScanning()) g_scan->stop();
    g_scan->setActiveScan(g_active);
    g_scan->setInterval(BENCH_SCAN_DUTY_MS);
    g_scan->setWindow(BENCH_SCAN_DUTY_MS);   // 100% duty
    g_scan->setMaxResults(0);                      // callback-only, no dedup buffer
    g_scan->setDuplicateFilter(false);             // every PDU, not first-per-device
    g_scan->start(0, false);                       // 0 = until stopped
}

// Spike S2: hammer the transmitter with connects while its advertiser is being
// reconfigured every 250 ms. Pass criterion (amendment §10-A): N consecutive
// connects succeed and the beacon stream does not go stale.
static void cmd_connect(int n, uint32_t settle_ms) {
    if (!g_tx_addr_known) {
        Serial.println("S2 no TX address learned yet — wait for beacons");
        return;
    }
    uint32_t ok = 0, fail = 0, read_fail = 0;
    uint32_t t_sum = 0, t_max = 0;
    g_max_gap_ms = 0;
    g_last_beacon_ms = 0;

    for (int i = 0; i < n; ++i) {
        // One radio: scanning must yield before a connect can be attempted. This
        // is the same Option A hand-off the watch performs.
        if (g_scan->isScanning()) g_scan->stop();

        NimBLEClient* cl = NimBLEDevice::createClient();
        const uint32_t t0 = millis();
        bool connected = cl->connect(g_tx_addr);
        const uint32_t dt = millis() - t0;

        if (connected) {
            NimBLERemoteService* svc = cl->getService(BENCH_SVC_UUID);
            NimBLERemoteCharacteristic* chr =
                svc ? svc->getCharacteristic(BENCH_CHR_UUID) : nullptr;
            const bool got = chr && strcmp(chr->readValue().c_str(), BENCH_S2_VALUE) == 0;
            if (got) {
                ok++;
                t_sum += dt;
                if (dt > t_max) t_max = dt;
            } else {
                read_fail++;      // connected but the GATT exchange did not survive
            }
            cl->disconnect();
        } else {
            fail++;
        }
        NimBLEDevice::deleteClient(cl);

        scan_restart();
        // Let slot boundaries pass so successive attempts land in different
        // phases of the schedule rather than all hitting the same one. Also the
        // knob that separates a real connectability problem from this harness
        // simply not giving the radio long enough to settle between attempts.
        delay(settle_ms);
        if (((i + 1) % 20) == 0)
            Serial.printf("S2 progress %d/%d ok=%u fail=%u readfail=%u\n",
                          i + 1, n, (unsigned)ok, (unsigned)fail, (unsigned)read_fail);
    }

    Serial.printf("S2 RESULT attempts=%d settle=%ums ok=%u connect_fail=%u read_fail=%u "
                  "t_mean=%ums t_max=%ums beacon_max_gap=%ums heap=%u\n",
                  n, (unsigned)settle_ms, (unsigned)ok, (unsigned)fail, (unsigned)read_fail,
                  (unsigned)(ok ? t_sum / ok : 0), (unsigned)t_max,
                  (unsigned)g_max_gap_ms, (unsigned)ESP.getFreeHeap());
}

static void handle_line(char* s) {
    if (!strcmp(s, "?") || !strcmp(s, "help")) {
        Serial.println("RX cmds: reset | report | scan active|passive | stat | connect <n>");
        return;
    }
    if (!strncmp(s, "connect ", 8)) {
        const char* sp = strchr(s + 8, ' ');
        cmd_connect(atoi(s + 8), sp ? (uint32_t)atoi(sp + 1) : 300u);
        return;
    }
    if (!strcmp(s, "reset"))  { stats_reset(); Serial.println("RX reset"); return; }
    if (!strcmp(s, "report")) { cmd_report(); return; }
    if (!strcmp(s, "scan active"))  { g_active = true;  scan_restart(); Serial.println("RX scan=active");  return; }
    if (!strcmp(s, "scan passive")) { g_active = false; scan_restart(); Serial.println("RX scan=passive"); return; }
    if (!strcmp(s, "stat")) {
        Serial.printf("RX scanning=%d heap=%u cyc=%u..%u\n",
                      g_scan->isScanning() ? 1 : 0, (unsigned)ESP.getFreeHeap(),
                      (unsigned)g_cyc_min, (unsigned)g_cyc_max);
        return;
    }
    Serial.printf("RX ? unknown: %s\n", s);
}

void setup() {
    Serial.begin(115200);
    delay(400);
    Serial.println("\n=== S1a bench: RX role ===");

    stats_reset();
    NimBLEDevice::init("S1a-RX");
    g_scan = NimBLEDevice::getScan();
    g_scan->setScanCallbacks(&g_cb, false);
    scan_restart();

    Serial.println("RX ready, scanning passive at 100% duty. 'reset' then 'report'.");
}

void loop() {
    serial_service();
    // A duration-0 scan should run until stopped, but re-arm defensively: a
    // silent scan would read as 0% PDR, i.e. exactly the result being measured.
    if (!g_scan->isScanning()) scan_restart();
    delay(20);
}

#endif // BENCH_ROLE_RX
