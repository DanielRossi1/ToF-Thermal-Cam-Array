/*
 * MLX90640 — ESP32-C3 SuperMini — Full data output
 * ──────────────────────────────────────────────────
 * Outputs every value the Adafruit MLX90640 library exposes:
 *   - 768 pixel temperatures (°C)
 *   - Ambient temperature Ta (°C)
 *
 * The Adafruit library internally also reads Vdd and applies
 * BadPixelsCorrection — all of this is already baked into getFrame().
 * There is no additional raw data accessible through this library.
 *
 * Line format (one line per frame, CSV):
 *   FRAME,<seq>,<ta>,<t0>,<t1>,...,<t767>
 *   Total: 771 comma-separated fields
 *
 * Wiring:
 *   MLX90640 VIN → 3V3
 *   MLX90640 GND → GND
 *   MLX90640 SDA → GPIO8
 *   MLX90640 SCL → GPIO9
 */

#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <esp_system.h>
#include <math.h>

#define PIN_SDA  8
#define PIN_SCL  9

// ── Boot/perf tuning ─────────────────────────────────────────────────────────
// Keep the default behavior robust, but avoid slow/verbose I2C scans on every
// boot (USB CDC printing + scanning can dominate boot time).
#ifndef WAIT_FOR_SERIAL_MS
#define WAIT_FOR_SERIAL_MS 500
#endif

#ifndef SENSOR_STABILIZE_MS
#define SENSOR_STABILIZE_MS 1200
#endif

#ifndef BEGIN_RETRY_DELAY_MS
#define BEGIN_RETRY_DELAY_MS 150
#endif

// Set to 1 only when actively debugging the I2C bus.
#ifndef ENABLE_VERBOSE_I2C_SCAN
#define ENABLE_VERBOSE_I2C_SCAN 0
#endif

Adafruit_MLX90640 mlx;
float frame[32 * 24];
uint32_t seq = 0;
bool stream_enabled = true;
uint32_t last_frame_ms = 0;
uint8_t reinit_failures = 0;
uint32_t next_reinit_ms = 0;

static void configureMlx() {
    mlx.setMode(MLX90640_CHESS);
    // Conservative defaults for stability on real wiring.
    // 18-bit @ 8Hz is aggressive and tends to amplify marginal I2C issues.
    mlx.setResolution(MLX90640_ADC_16BIT);
    mlx.setRefreshRate(MLX90640_4_HZ);
}

static bool reinitMlx() {
    Serial.println("# Reinit MLX90640...");
    i2cRecoverBus();
    printI2CBusLevels();

    // Fully tear down and re-init I2C.
    Wire.end();
    delay(20);
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(100000);
    Wire.setTimeout(200);

    bool ok = false;
    for (int attempt = 1; attempt <= 4; attempt++) {
        if (mlx.begin(0x33, &Wire)) {
            ok = true;
            break;
        }
        delay(25);
        if (mlx.begin(0x32, &Wire)) {
            ok = true;
            break;
        }
        Serial.printf("# Reinit begin() attempt %d failed\n", attempt);
        i2cRecoverBus();
        delay(30);
    }

    if (!ok) {
        Serial.println("# Reinit failed");
        return false;
    }

    // Keep I2C conservative after reinit as well.
    Wire.setClock(100000);
    configureMlx();
    Serial.println("# Reinit OK");
    Serial.println("READY");
    return true;
}

// Buffered CSV writer to minimize Serial overhead (small RAM footprint).
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

static inline int16_t toCentiC(float c) {
    // Fixed-point encoding: °C * 100 (centi-degrees).
    return (int16_t)lrintf(c * 100.0f);
}

static void printI2CBusLevels() {
    // Check whether lines are HIGH at idle (they must be pulled up).
    pinMode(PIN_SDA, INPUT_PULLUP);
    pinMode(PIN_SCL, INPUT_PULLUP);
    delay(5);
    int sda = digitalRead(PIN_SDA);
    int scl = digitalRead(PIN_SCL);
    Serial.printf("# I2C idle levels: SDA=%d SCL=%d (expect 1/1)\n", sda, scl);
    if (sda == 0 || scl == 0) {
        Serial.println("# I2C ERROR: bus line stuck LOW. Check wiring/shorts/pullups/power.");
    }
}

static bool i2cRecoverBus() {
    // Standard I2C bus recovery: if SDA is held low by a slave, pulse SCL up
    // to 9 cycles to advance it, then generate a STOP.
    pinMode(PIN_SDA, INPUT_PULLUP);
    pinMode(PIN_SCL, INPUT_PULLUP);
    delay(5);

    int sda0 = digitalRead(PIN_SDA);
    int scl0 = digitalRead(PIN_SCL);
    if (sda0 == 1 && scl0 == 1) {
        return true;
    }

    Serial.printf("# I2C recover: start SDA=%d SCL=%d\n", sda0, scl0);

#if defined(OUTPUT_OPEN_DRAIN)
    pinMode(PIN_SCL, OUTPUT_OPEN_DRAIN);
    pinMode(PIN_SDA, OUTPUT_OPEN_DRAIN);
#else
    pinMode(PIN_SCL, OUTPUT);
    pinMode(PIN_SDA, OUTPUT);
#endif
    // Release both lines
    digitalWrite(PIN_SCL, HIGH);
    digitalWrite(PIN_SDA, HIGH);
    delayMicroseconds(10);

    for (int i = 0; i < 18; i++) {
        // If SCL is held low externally, pulses won't help.
        pinMode(PIN_SCL, INPUT_PULLUP);
        delayMicroseconds(5);
        if (digitalRead(PIN_SCL) == 0) {
            Serial.println("# I2C recover: SCL held LOW (cannot pulse)");
            break;
        }
        pinMode(PIN_SCL,
#if defined(OUTPUT_OPEN_DRAIN)
                OUTPUT_OPEN_DRAIN
#else
                OUTPUT
#endif
        );

        digitalWrite(PIN_SCL, LOW);
        delayMicroseconds(6);
        digitalWrite(PIN_SCL, HIGH);
        delayMicroseconds(6);

        pinMode(PIN_SDA, INPUT_PULLUP);
        delayMicroseconds(2);
        if (digitalRead(PIN_SDA) == 1) {
            // SDA released; we can attempt STOP
            break;
        }
    }

    // Generate a STOP condition: SDA rising while SCL high
#if defined(OUTPUT_OPEN_DRAIN)
    pinMode(PIN_SCL, OUTPUT_OPEN_DRAIN);
    pinMode(PIN_SDA, OUTPUT_OPEN_DRAIN);
#else
    pinMode(PIN_SCL, OUTPUT);
    pinMode(PIN_SDA, OUTPUT);
#endif
    digitalWrite(PIN_SDA, LOW);
    delayMicroseconds(6);
    digitalWrite(PIN_SCL, HIGH);
    delayMicroseconds(6);
    digitalWrite(PIN_SDA, HIGH);
    delayMicroseconds(6);

    pinMode(PIN_SDA, INPUT_PULLUP);
    pinMode(PIN_SCL, INPUT_PULLUP);
    delay(5);
    int sda1 = digitalRead(PIN_SDA);
    int scl1 = digitalRead(PIN_SCL);
    Serial.printf("# I2C recover: end SDA=%d SCL=%d\n", sda1, scl1);
    return (sda1 == 1 && scl1 == 1);
}

static void i2cScan() {
    Serial.println("# I2C scan...");
    uint8_t found = 0;
    uint16_t err_count[5] = {0, 0, 0, 0, 0};
    bool saw_33 = false;
    bool saw_32 = false;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            found++;
            if (addr == 0x33) saw_33 = true;
            if (addr == 0x32) saw_32 = true;
        } else if (err != 2) {
            // err=2 is NACK on address (normal for most addresses)
            if (err < 5) err_count[err]++;
        }
        // No per-address delay: keep scan short.
    }
    if (!found) {
        Serial.println("#   none found");
    } else if (found > 20) {
        Serial.printf("# WARNING: suspicious scan (%u addresses). This usually means SDA is stuck LOW.\n", found);
    }

    if (saw_33 || saw_32) {
        Serial.printf("# MLX90640 address detected: %s%s\n",
                      saw_33 ? "0x33 " : "",
                      saw_32 ? "0x32" : "");
    } else {
        Serial.println("# MLX90640 address NOT detected (expected 0x33 or 0x32)");
    }

    uint16_t non_nack = err_count[1] + err_count[3] + err_count[4];
    if (non_nack) {
        Serial.printf("# I2C errors during scan (non-NACK): err1=%u err3=%u err4=%u\n",
                      err_count[1], err_count[3], err_count[4]);
    }
}

static bool i2cProbe(uint8_t addr) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    return err == 0;
}

static const char* resetReasonStr(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON: return "POWERON";
        case ESP_RST_EXT: return "EXT";
        case ESP_RST_SW: return "SW";
        case ESP_RST_PANIC: return "PANIC";
        case ESP_RST_INT_WDT: return "INT_WDT";
        case ESP_RST_TASK_WDT: return "TASK_WDT";
        case ESP_RST_WDT: return "WDT";
#if defined(ESP_RST_USB)
        case ESP_RST_USB: return "USB";
#endif
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT: return "BROWNOUT";
        case ESP_RST_SDIO: return "SDIO";
        default: return "UNKNOWN";
    }
}

void setup() {
    Serial.begin(921600);
    // Some ESP32 variants use native USB CDC. Waiting briefly helps ensure
    // the host has opened the port before we emit startup lines.
    uint32_t t0 = millis();
    while (!Serial && (millis() - t0) < WAIT_FOR_SERIAL_MS) {
        delay(10);
    }
    delay(SENSOR_STABILIZE_MS);   // MLX90640 power-on stabilization

    esp_reset_reason_t rr = esp_reset_reason();
    Serial.printf("# Reset reason: %s (%d)\n", resetReasonStr(rr), (int)rr);

    Serial.printf("# I2C pins: SDA=%d SCL=%d\n", PIN_SDA, PIN_SCL);
    printI2CBusLevels();

    // If a previous run left the bus stuck, try recovery before starting I2C.
    i2cRecoverBus();

    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(100000);
    Wire.setTimeout(200);

    // Fast probe of expected MLX90640 addresses (avoid slow scans at boot).
    bool ack33 = i2cProbe(0x33);
    bool ack32 = i2cProbe(0x32);
    Serial.printf("# I2C probe: 0x33=%s 0x32=%s\n",
                  ack33 ? "ACK" : "NACK",
                  ack32 ? "ACK" : "NACK");

#if ENABLE_VERBOSE_I2C_SCAN
    i2cScan();
#endif

    // Retry begin() up to 5 times
    bool found = false;
    for (int i = 1; i <= 5; i++) {
        // Try both possible MLX90640 addresses (0x33 default, 0x32 alt)
        if (mlx.begin(0x33, &Wire)) {
            found = true;
            Serial.printf("# MLX90640 found (attempt %d)\n", i);
            break;
        }
        delay(25);
        if (mlx.begin(0x32, &Wire)) {
            found = true;
            Serial.printf("# MLX90640 found @0x32 (attempt %d)\n", i);
            break;
        }
        Serial.printf("# begin() attempt %d failed\n", i);

        // Attempt bus recovery + I2C re-init between tries.
        i2cRecoverBus();
        Wire.end();
        delay(10);
        Wire.begin(PIN_SDA, PIN_SCL);
        Wire.setClock(100000);
        Wire.setTimeout(200);
        delay(BEGIN_RETRY_DELAY_MS);
    }
    if (!found) {
        Serial.println("ERROR:MLX90640 not found on I2C");
        Serial.println("# Check wiring + pullups, and consider trying different GPIOs.");
        while (1) delay(1000);
    }

    // Keep I2C conservative for robustness (faster clocks amplify marginal wiring)
    Wire.setClock(100000);

    Serial.printf("# Serial: %04X%04X%04X\n",
                  mlx.serialNumber[0],
                  mlx.serialNumber[1],
                  mlx.serialNumber[2]);

    configureMlx();

    Serial.println("# Mode:Chess Resolution:16bit Rate:4Hz");
    Serial.println("# Columns: seq, Ta_cC, T_cC[0..767]  (cC = centi-°C)");
    Serial.println("# Commands: p=pause stream, r=resume stream");
    Serial.println("READY");
}

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

    // If the stream stalls (e.g. I2C hang), attempt recovery.
    uint32_t now_ms = millis();
    if (last_frame_ms && (now_ms - last_frame_ms) > 1500) {
        Serial.println("# WARN: stream stall detected (>1.5s without frames)");
        reinitMlx();
        last_frame_ms = now_ms;
    }

    // getFrame() internally:
    //   1. reads raw frame data (2 sub-pages in chess mode)
    //   2. extracts Vdd and Ta from raw data
    //   3. calls CalculateTo() with emissivity=0.95, tr=Ta-8
    //   4. calls BadPixelsCorrection()
    //   5. fills framebuf with 768 calibrated temperatures in °C
    // getFrame() may return a non-zero code when a chess sub-page isn't ready.
    // Don't "return" (that looks like a stuck stream); instead retry briefly.
    uint32_t t_start = millis();
    int rc = 0;
    while ((rc = mlx.getFrame(frame)) != 0) {
        if ((millis() - t_start) > 600) {
            Serial.println("# WARN: getFrame() timeout — attempting recovery");
            uint32_t now = millis();
            if (now >= next_reinit_ms) {
                bool ok = reinitMlx();
                if (ok) {
                    reinit_failures = 0;
                    next_reinit_ms = 0;
                } else {
                    reinit_failures = (reinit_failures < 10) ? (uint8_t)(reinit_failures + 1) : reinit_failures;
                    uint32_t backoff = 250UL << (reinit_failures >= 6 ? 6 : reinit_failures); // caps around 16s
                    if (backoff > 15000UL) backoff = 15000UL;
                    next_reinit_ms = now + backoff;
                }
            } else {
                Serial.println("# WARN: recovery backoff active (skipping reinit)");
            }
            return;
        }
        delay(1);
        yield();
    }

    // getTa(false) reuses the Ta computed during the last getFrame() call
    float ta = mlx.getTa(false);

    // Emit CSV line (fixed-point integers, fast)
    CsvWriter w;
    w.writeStr("FRAMEC,");
    w.writeUInt(seq++);
    w.comma();
    w.writeInt((int32_t)toCentiC(ta));
    for (int i = 0; i < 768; i++) {
        w.comma();
        w.writeInt((int32_t)toCentiC(frame[i]));
        if ((i & 0x7F) == 0x7F) yield();
    }
    w.nl();

    last_frame_ms = millis();
}
