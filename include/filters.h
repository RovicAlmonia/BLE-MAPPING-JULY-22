#pragma once
#include <Arduino.h>
#include <algorithm>
#include "config.h"

// ╔══════════════════════════════════════════════════════════════════╗
// ║  filters.h  v3.0 — Median · EWMA · Kalman × 2 · Distance       ║
// ╚══════════════════════════════════════════════════════════════════╝

// ── 1. Median Filter ─────────────────────────────────────────────
class MedianFilter {
    int     _buf[MEDIAN_WINDOW];
    int     _tmp[MEDIAN_WINDOW];
    uint8_t _idx   = 0;
    uint8_t _count = 0;
public:
    void push(int v) {
        _buf[_idx] = v;
        _idx = (_idx + 1) % MEDIAN_WINDOW;
        if (_count < MEDIAN_WINDOW) _count++;
    }
    bool    ready()  const { return _count == MEDIAN_WINDOW; }
    uint8_t filled() const { return _count; }
    float   value() {
        memcpy(_tmp, _buf, _count * sizeof(int));
        std::sort(_tmp, _tmp + _count);
        return (float)_tmp[_count / 2];
    }
};

// ── 2. EWMA ──────────────────────────────────────────────────────
class EWMA {
    float _v    = 0.0f;
    bool  _init = false;
public:
    float update(float x) {
        if (!_init) { _v = x; _init = true; }
        else         { _v = EWMA_ALPHA * x + (1.0f - EWMA_ALPHA) * _v; }
        return _v;
    }
    float value() const { return _v; }
    void  reset()       { _init = false; }
};

// ── 3. Kalman Filter (generic, reusable) ─────────────────────────
class KalmanFilter {
    float _x    = 0.0f;
    float _p    = 1.0f;
    bool  _init = false;
    float _q, _r;
public:
    KalmanFilter(float q, float r) : _q(q), _r(r) {}
    float update(float z) {
        if (!_init) { _x = z; _init = true; return _x; }
        _p += _q;
        float k = _p / (_p + _r);
        _x += k * (z - _x);
        _p *= (1.0f - k);
        return _x;
    }
    float value() const { return _x; }
    void  reset()       { _init = false; _p = 1.0f; }
};

// ── 4. Running statistics (for variance-based confidence) ─────────
class RunningStats {
    static const int N = 16;
    float _buf[N];
    int   _idx   = 0;
    int   _count = 0;
public:
    void push(float v) {
        _buf[_idx] = v;
        _idx = (_idx + 1) % N;
        if (_count < N) _count++;
    }
    float variance() const {
        if (_count < 2) return 999.0f;
        float sum = 0, sq = 0;
        for (int i = 0; i < _count; i++) { sum += _buf[i]; sq += _buf[i]*_buf[i]; }
        float mean = sum / _count;
        return (sq / _count) - (mean * mean);
    }
    float stddev() const { return sqrtf(variance()); }
};

// ── 5. RSSI → Distance + Confidence ─────────────────────────────
namespace Distance {
    // Log-distance path loss model
    inline float metres(float rssi) {
        float d = powf(10.0f, ((float)TX_POWER - rssi) / (10.0f * PATH_LOSS_N));
        return constrain(d, MIN_DISTANCE_M, MAX_DISTANCE_M);
    }

    // Confidence from RSSI strength: [-90,-50] dBm → [10,95]%
    inline uint8_t confidence(float rssi, float distStddev = -1.0f) {
        // Base: signal strength
        float c = ((rssi - (-90.0f)) / 40.0f) * 80.0f + 10.0f;
        c = constrain(c, 5.0f, 95.0f);
        // Penalty: high variance = lower confidence
        if (distStddev >= 0.0f) {
            float penalty = constrain(distStddev * 20.0f, 0.0f, 30.0f);
            c -= penalty;
        }
        return (uint8_t)constrain(c, 5.0f, 98.0f);
    }
}