#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "config.h"
#include "filters.h"

// ╔══════════════════════════════════════════════════════════════════╗
// ║           BLE Distance Meter v3.0  —  main.cpp                 ║
// ║   Dual-Kalman pipeline + variance confidence + fast scan        ║
// ╚══════════════════════════════════════════════════════════════════╝

// ── Filter pipeline ───────────────────────────────────────────────
static MedianFilter  medFilter;
static EWMA          ewmaFilter;
static KalmanFilter  rssiKalman(KALMAN_Q,      KALMAN_R);      // on RSSI
static KalmanFilter  distKalman(DIST_KALMAN_Q, DIST_KALMAN_R); // on distance
static RunningStats  distStats;   // tracks distance variance for confidence

// ── State ─────────────────────────────────────────────────────────
static BLEScan*      scan        = nullptr;
static unsigned long lastSeenMs  = 0;
static float         lastDistM   = 0.0f;
static bool          tagPresent  = false;
static uint32_t      packetCount = 0;
static uint32_t      missedCount = 0;

// ── Emit one reading line ─────────────────────────────────────────
void emitReading(int rawRssi, float medRssi, float ewmaRssi,
                 float kRssi, float rawDist, float smoothDist, uint8_t conf)
{
    float delta = fabsf(smoothDist - lastDistM);
    if (delta < STABLE_THRESH_M && conf < 95)
        conf = (uint8_t)min(95, (int)conf + STABLE_BONUS);
    lastDistM = smoothDist;

#if OUTPUT_JSON
    Serial.printf(
        "{\"rssi_raw\":%d,\"rssi_med\":%.0f,\"rssi_ewma\":%.1f,"
        "\"rssi_kalman\":%.2f,\"distance_m\":%.3f,\"confidence\":%d,"
        "\"packets\":%lu,\"missed\":%lu}\n",
        rawRssi, medRssi, ewmaRssi, kRssi,
        smoothDist, conf, packetCount, missedCount
    );
#else
    Serial.printf(
        "raw:%d  med:%.0f  ewma:%.1f  kalman:%.1f  ->  %.2f m  [conf:%d%%]  #%lu\n",
        rawRssi, medRssi, ewmaRssi, kRssi,
        smoothDist, conf, packetCount
    );
#endif
}

// ── Core processing pipeline ──────────────────────────────────────
void processRssi(int rawRssi) {
    packetCount++;

    // Stage 1: Median
    medFilter.push(rawRssi);
    if (!medFilter.ready()) {
#if !OUTPUT_JSON
        Serial.printf("[warming up %d/%d]\n", medFilter.filled(), MEDIAN_WINDOW);
#endif
        return;
    }
    float medRssi = medFilter.value();

    // Stage 2: EWMA
    float ewmaRssi = ewmaFilter.update(medRssi);

    // Stage 3: Kalman on RSSI
    float kRssi = rssiKalman.update(ewmaRssi);

    // Stage 4: Convert to distance
    float rawDist = Distance::metres(kRssi);

    // Stage 5: Second Kalman on distance
    float smoothDist = distKalman.update(rawDist);
    smoothDist = constrain(smoothDist, MIN_DISTANCE_M, MAX_DISTANCE_M);

    // Stage 6: Running variance for confidence
    distStats.push(smoothDist);
    float stddev = distStats.stddev();
    uint8_t conf = Distance::confidence(kRssi, stddev);

    emitReading(rawRssi, medRssi, ewmaRssi, kRssi, rawDist, smoothDist, conf);
}

// ── BLE callback ─────────────────────────────────────────────────
class ScanCallback : public BLEAdvertisedDeviceCallbacks {
public:
    void onResult(BLEAdvertisedDevice dev) override {
#if USE_NAME_MATCH
        if (!dev.haveName()) return;
        if (String(dev.getName().c_str()) != TARGET_NAME) return;
#else
        if (dev.getAddress().toString() != TARGET_ADDR) return;
#endif
        int rssi = dev.getRSSI();
        if (rssi < RSSI_FLOOR) { missedCount++; return; }
        lastSeenMs = millis();
        tagPresent = true;
        processRssi(rssi);
    }
};

static ScanCallback scanCB;

// ── Watchdog ─────────────────────────────────────────────────────
void checkTagTimeout() {
    if (!tagPresent) return;
    if ((millis() - lastSeenMs) > SEEN_TIMEOUT_MS) {
        tagPresent = false;
        // Reset filters so stale state doesn't poison next detection
        ewmaFilter.reset();
        rssiKalman.reset();
        distKalman.reset();
#if OUTPUT_JSON
        Serial.printf("{\"event\":\"tag_lost\",\"packets\":%lu,\"missed\":%lu}\n",
                      packetCount, missedCount);
#else
        Serial.printf("[TAG LOST — no signal %.1f s]  packets:%lu  missed:%lu\n",
                      SEEN_TIMEOUT_MS / 1000.0f, packetCount, missedCount);
#endif
    }
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(500);

#if !OUTPUT_JSON
    Serial.println("\n╔══════════════════════════════════════╗");
    Serial.println(  "║   BLE Distance Meter  v3.0           ║");
    Serial.println(  "╚══════════════════════════════════════╝");
    Serial.printf("Target  : %s\n",  USE_NAME_MATCH ? TARGET_NAME : TARGET_ADDR);
    Serial.printf("TX_POWER: %d dBm  |  N: %.2f\n", TX_POWER, PATH_LOSS_N);
    Serial.printf("Scan    : %d ms window / %d ms interval\n",
                  SCAN_WINDOW_MS, SCAN_INTERVAL_MS);
    Serial.println("──────────────────────────────────────\n");
#endif

    BLEDevice::init("ESP32-BLEDist");
    scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(&scanCB, true);
    scan->setActiveScan(false);
    scan->setInterval(SCAN_INTERVAL_MS);
    scan->setWindow(SCAN_WINDOW_MS);
    scan->start(0, false);   // continuous scan
}

// ── Loop ──────────────────────────────────────────────────────────
void loop() {
    checkTagTimeout();
    delay(20);   // tight loop for fast watchdog response
}