/*
 * VL53L8CH — ESP32-C3 SuperMini — Full data output
 * ─────────────────────────────────────────────────
 * Outputs one CSV line per ranging frame.
 *
 * Line format (one line per frame, CSV):
 *   FRAME,<seq>,<side>,<d0>,<d1>,...,<dN>
 * Where:
 *   - side is 4 or 8
 *   - distances are mm (integer), -1 means "no target"
 *   - N = side*side - 1 (so total fields = 3 + side*side)
 *
 * Wiring:
 *   VL53L8CH VIN    → 3V3
 *   VL53L8CH GND    → GND
 *   VL53L8CH SDA    → GPIO8
 *   VL53L8CH SCL    → GPIO9
 *   VL53L8CH LPN    → GPIO4   (active-low shutdown)
 *   VL53L8CH PS     → GND     (MANDATORY — selects I2C mode)
 *   VL53L8CH INT    → not connected (polling mode)
 *
 * Commands (over Serial):
 *   p = pause stream
 *   r = resume stream
 */

#include <Wire.h>
#include <vl53l8ch.h>

// ── Pins ─────────────────────────────────────────────────────────────────────
#define PIN_SDA     8
#define PIN_SCL     9
#define PIN_LPN     4    // LPN = Low Power eNable = XSHUT (active-low)
// No PWREN pin on your breakout — skip it

// ── Sensor ───────────────────────────────────────────────────────────────────
VL53L8CH sensor(&Wire, PIN_LPN);

uint8_t   status;
uint8_t   res        = VL53LMZ_RESOLUTION_8X8;  // start with 8x8
uint32_t  seq        = 0;
bool      stream_enabled = true;

// Buffered CSV writer to minimize Serial overhead.
struct CsvWriter {
    static constexpr size_t CAP = 256;
    char buf[CAP];
    size_t n = 0;

    inline void flush() {
        if (n) {
            Serial.write((const uint8_t*)buf, n);
            n = 0;
        }
    }
    inline void put(char c) {
        if (n >= CAP) flush();
        buf[n++] = c;
    }
    inline void writeStr(const char* s) {
        while (*s) put(*s++);
    }
    inline void writeUInt(uint32_t v) {
        char tmp[10];
        int i = 0;
        do {
            tmp[i++] = char('0' + (v % 10));
            v /= 10;
        } while (v);
        while (i--) put(tmp[i]);
    }
    inline void writeInt(int32_t v) {
        if (v < 0) {
            put('-');
            v = -v;
        }
        writeUInt((uint32_t)v);
    }
    inline void comma() { put(','); }
    inline void nl() { put('\n'); flush(); }
};

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(460800);
    delay(1000);
    Serial.println("# VL53L8CH stream on ESP32-C3");

    // I2C — 400kHz, remapped to GPIO8/9
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    // begin() drives LPN HIGH and loads sensor firmware (~100ms)
    Serial.println("# begin()...");
    status = sensor.begin();
    if (status != VL53LMZ_STATUS_OK) {
        Serial.printf("ERROR:begin() failed status=%d\n", status);
        Serial.println("# Check: SDA/SCL wiring, 3V3 power, PS pin tied to GND");
        while (1) delay(1000);
    }
    Serial.println("# begin() OK");

    // init() configures the sensor internals
    Serial.println("# init()...");
    status = sensor.init();
    if (status != VL53LMZ_STATUS_OK) {
        Serial.printf("ERROR:init() failed status=%d\n", status);
        while (1) delay(1000);
    }
    Serial.println("# init() OK");

    // Set 8x8 resolution
    status = sensor.set_resolution(res);
    if (status != VL53LMZ_STATUS_OK) {
        Serial.printf("ERROR:set_resolution() failed %d\n", status);
        while (1) delay(1000);
    }

    // 15Hz — conservative for debug
    status = sensor.set_ranging_frequency_hz(15);
    if (status != VL53LMZ_STATUS_OK) {
        Serial.printf("# WARN: set_ranging_frequency_hz() failed %d\n", status);
    }

    // Start ranging
    status = sensor.start_ranging();
    if (status != VL53LMZ_STATUS_OK) {
        Serial.printf("ERROR:start_ranging() failed %d\n", status);
        while (1) delay(1000);
    }

    uint8_t side = (res == VL53LMZ_RESOLUTION_4X4) ? 4 : 8;
    Serial.printf("# Ranging started — %dx%d @ 15Hz\n", side, side);
    Serial.println("# Columns: seq, side, d[0..side*side-1] mm (-1=no target)");
    Serial.println("# Commands: p=pause stream, r=resume stream");
    Serial.println("READY");
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    if (Serial.available() > 0) {
        int c = Serial.read();
        if (c == 'p') {
            stream_enabled = false;
            Serial.println("# Stream paused");
        } else if (c == 'r') {
            stream_enabled = true;
            Serial.println("# Stream resumed");
        }
    }
    if (!stream_enabled) {
        delay(20);
        return;
    }

    VL53LMZ_ResultsData results;
    uint8_t data_ready = 0;

    // Poll until data ready (blocking — fine for debug)
    uint32_t t0 = millis();
    do {
        status = sensor.check_data_ready(&data_ready);
        if (millis() - t0 > 2000) {
            Serial.println("ERROR:Timeout waiting for data");
            return;
        }
    } while (!data_ready);

    if (status != VL53LMZ_STATUS_OK) {
        Serial.printf("ERROR:check_data_ready %d\n", status);
        return;
    }

    status = sensor.get_ranging_data(&results);
    if (status != VL53LMZ_STATUS_OK) {
        Serial.printf("ERROR:get_ranging_data %d\n", status);
        return;
    }

    uint8_t side = (res == VL53LMZ_RESOLUTION_4X4) ? 4 : 8;
    uint8_t zones = side * side;

    // Emit CSV line
    CsvWriter w;
    w.writeStr("FRAME,");
    w.writeUInt(seq++);
    w.comma();
    w.writeUInt(side);
    for (int row = 0; row < side; row++) {
        for (int col = 0; col < side; col++) {
            // Zone ordering: sensor reports top-right to bottom-left.
            // Flip horizontally so the output grid matches the physical scene.
            int zone = row * side + (side - 1 - col);
            int dist = -1;
            if (zone < zones && results.nb_target_detected[zone] > 0) {
                int idx = VL53LMZ_NB_TARGET_PER_ZONE * zone;
                dist = (int)results.distance_mm[idx];
            }
            w.comma();
            w.writeInt(dist);
        }
        yield();
    }
    w.nl();
}
