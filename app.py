"""
BLE Distance Map  v2.0  —  app.py
──────────────────────────────────
Reads BLE distance from ESP32 over Serial.
Supports both human-readable and JSON output from firmware.
Serves a Leaflet map using browser geolocation as anchor point.

Install:  pip install flask pyserial
Run:      python app.py
Open:     http://localhost:5000
"""

import threading
import re
import json
import time
import serial
import serial.tools.list_ports
from collections import deque
from flask import Flask, jsonify, render_template

# ── Config ────────────────────────────────────────────────────────
SERIAL_PORT  = None       # None = auto-detect; or force e.g. "COM4"
BAUD_RATE    = 115200
HISTORY_LEN  = 60         # number of readings to keep for sparkline
STALE_AFTER  = 5.0        # seconds before marking reading as stale
# ─────────────────────────────────────────────────────────────────

app = Flask(__name__)

_lock    = threading.Lock()
_history = deque(maxlen=HISTORY_LEN)
_state   = {
    "distance_m":   None,
    "rssi_raw":     None,
    "rssi_kalman":  None,
    "confidence":   None,
    "packets":      0,
    "last_update":  None,
    "raw_line":     "",
    "port":         "",
    "tag_present":  False,
}

# ── Regex for human-readable format ──────────────────────────────
# raw:-49  med:-48  ewma:-48.0  kalman:-48.1  ->  0.25 m  [conf:85%]  #42
_re_human = re.compile(
    r"raw:(?P<raw>-?\d+).*?"
    r"kalman:(?P<kalman>-?[\d.]+).*?"
    r"(?P<dist>[\d.]+)\s*m.*?"
    r"conf:(?P<conf>\d+)"
    r"(?:.*?#(?P<pkts>\d+))?"
)

# ── Auto-detect ESP32 port ────────────────────────────────────────
def find_esp32_port():
    keywords = ["cp210", "ch340", "ch341", "ftdi", "usb serial", "usb-serial", "uart"]
    for p in serial.tools.list_ports.comports():
        desc = ((p.description or "") + (p.manufacturer or "")).lower()
        if any(k in desc for k in keywords):
            return p.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None

# ── Parse one serial line ─────────────────────────────────────────
def parse_line(line: str) -> bool:
    line = line.strip()
    if not line:
        return False

    # ── JSON mode ─────────────────────────────────────────────────
    if line.startswith("{"):
        try:
            d = json.loads(line)
            if "event" in d and d["event"] == "tag_lost":
                with _lock:
                    _state["tag_present"] = False
                return True
            dist = d.get("distance_m")
            if dist is None:
                return False
            with _lock:
                _state.update({
                    "distance_m":  float(dist),
                    "rssi_raw":    d.get("rssi_raw"),
                    "rssi_kalman": d.get("rssi_kalman"),
                    "confidence":  d.get("confidence"),
                    "packets":     d.get("packets", _state["packets"]),
                    "last_update": time.time(),
                    "raw_line":    line,
                    "tag_present": True,
                })
                _history.append({"t": time.time(), "d": float(dist)})
            return True
        except (json.JSONDecodeError, ValueError):
            pass

    # ── Human-readable mode ───────────────────────────────────────
    if "tag lost" in line.lower():
        with _lock:
            _state["tag_present"] = False
        return True

    m = _re_human.search(line)
    if m:
        dist = float(m.group("dist"))
        with _lock:
            _state.update({
                "distance_m":  dist,
                "rssi_raw":    int(m.group("raw")),
                "rssi_kalman": float(m.group("kalman")),
                "confidence":  int(m.group("conf")),
                "packets":     int(m.group("pkts")) if m.group("pkts") else _state["packets"] + 1,
                "last_update": time.time(),
                "raw_line":    line,
                "tag_present": True,
            })
            _history.append({"t": time.time(), "d": dist})
        return True

    return False

# ── Serial reader thread ──────────────────────────────────────────
def serial_reader():
    port = SERIAL_PORT or find_esp32_port()
    if not port:
        print("[Serial] No port found — plug in ESP32 and restart")
        return

    with _lock:
        _state["port"] = port

    while True:
        try:
            with serial.Serial(port, BAUD_RATE, timeout=2) as ser:
                print(f"[Serial] Connected on {port} @ {BAUD_RATE}")
                while True:
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="replace").strip()
                    if line:
                        if not parse_line(line):
                            print(f"[Serial] {line}")
        except serial.SerialException as e:
            print(f"[Serial] Error: {e} — retrying in 3 s")
            time.sleep(3)
            port = SERIAL_PORT or find_esp32_port() or port

threading.Thread(target=serial_reader, daemon=True).start()

# ── Routes ────────────────────────────────────────────────────────
@app.get("/")
def index():
    return render_template("map.html")

@app.get("/data")
def data():
    with _lock:
        s = dict(_state)
        hist = list(_history)
    age = (time.time() - s["last_update"]) if s["last_update"] else None
    s["age_s"] = round(age, 1) if age is not None else None
    s["stale"]  = age is None or age > STALE_AFTER
    # Compact history for sparkline: just distances
    s["history"] = [h["d"] for h in hist]
    return jsonify(s)

@app.get("/health")
def health():
    return jsonify({"status": "ok", "time": time.time()})

if __name__ == "__main__":
    print("[Flask] Open http://localhost:5000")
    app.run(host="0.0.0.0", port=5000, debug=False)