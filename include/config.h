#pragma once

// ╔══════════════════════════════════════════════════════════════════╗
// ║                  BLE DISTANCE METER  v3.0                       ║
// ║            config.h — all tunable parameters here               ║
// ╚══════════════════════════════════════════════════════════════════╝

// ── Target device ────────────────────────────────────────────────────
#define TARGET_ADDR     "aa:bb:cc:dd:ee:ff"   // MAC address (lower-case)
#define TARGET_NAME     "MyTag"                // BLE advertised name
#define USE_NAME_MATCH  0   // 0 = match by MAC   1 = match by name

// ── Path-loss calibration ─────────────────────────────────────────────
// HOW TO CALIBRATE TX_POWER:
//   1. Place tag exactly 1.0 m from ESP32, clear line of sight
//   2. Watch serial output for 60 s
//   3. Ignore the top 10% and bottom 10% of kalman: values
//   4. Average the middle 80% — paste that number below
//
// PATH_LOSS_N guide:
//   Free space / outdoor        : 2.0 – 2.2
//   Indoor open area            : 2.3 – 2.6
//   Indoor with walls/furniture : 2.7 – 3.0
//   Dense obstacles / concrete  : 3.0 – 3.5
#define TX_POWER        -48     // RSSI (dBm) at exactly 1 m — MEASURE THIS
#define PATH_LOSS_N     2.4f    // environment path-loss exponent

// ── Distance limits ───────────────────────────────────────────────────
#define MAX_DISTANCE_M  15.0f
#define MIN_DISTANCE_M  0.05f

// ── BLE scan — tightest possible window for fastest updates ──────────
#define SCAN_INTERVAL_MS  50    // scan slot every 50 ms
#define SCAN_WINDOW_MS    49    // listen for 49 ms of every 50 ms slot
#define RSSI_FLOOR        -90   // discard very weak hits

// ── Filter stack ──────────────────────────────────────────────────────
// Stage 1 – Median (spike/multipath removal)
#define MEDIAN_WINDOW   7       // odd; 5=fast 9=smooth; 7 is sweet spot

// Stage 2 – EWMA (trend smoothing before Kalman)
#define EWMA_ALPHA      0.25f   // 0.1=very smooth  0.4=responsive

// Stage 3 – Kalman (optimal RSSI estimator)
#define KALMAN_Q        0.001f  // process noise (↓ = smoother/slower)
#define KALMAN_R        0.08f   // sensor noise  (↑ = smoother/slower)

// Stage 4 – Distance Kalman (second Kalman on distance output)
// Runs AFTER RSSI→distance conversion for extra smoothing
#define DIST_KALMAN_Q   0.005f
#define DIST_KALMAN_R   0.20f

// Stage 5 – Stability gate
#define STABLE_THRESH_M 0.08f   // movement < this = "stable"
#define STABLE_BONUS    12      // extra confidence % when stable

// ── Output format ─────────────────────────────────────────────────────
// 0 = human-readable   1 = JSON (used by Flask web app)
#define OUTPUT_JSON     0

// ── Watchdog ──────────────────────────────────────────────────────────
#define SEEN_TIMEOUT_MS 3000    // ms without packet = tag lost

// ── Serial ────────────────────────────────────────────────────────────
#define SERIAL_BAUD     115200