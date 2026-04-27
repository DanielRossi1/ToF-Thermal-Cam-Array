/*
 * ESP32-S3-N16R8 — Synchronized Sensor Hub  v2.0
 * ═══════════════════════════════════════════════
 *
 * HARDWARE:
 *   VL53L8CH  ToF 8×8     I2C 0x52   GPIO8=SDA GPIO9=SCL
 *                          LPn=GPIO5  INT=GPIO4
 *   MLX90640  Thermal      I2C 0x33   same bus
 *   USB Camera UVC         Left  USB-C (OTG port)
 *   PC data stream         Right USB-C (CH343p UART port)  ← connect here
 *
 * ARDUINO IDE TOOLS — REQUIRED SETTINGS:
 *   Board:              ESP32S3 Dev Module
 *   USB CDC On Boot:    Disabled          ← CRITICAL for OTG camera
 *   USB Mode:           USB-OTG (TinyUSB) ← CRITICAL for OTG camera
 *   PSRAM:              OPI PSRAM
 *   Partition Scheme:   Huge APP (3MB No OTA / 1MB SPIFFS)
 *   CPU Freq:           240MHz
 *
 * LIBRARIES (Arduino Library Manager):
 *   - STM32duino VL53L8CH      (search "VL53L8CH" → by STMicroelectronics)
 *   - Adafruit MLX90640        (already installed)
 *   - ESP32 USB STREAM         (search "USB_STREAM" → by Espressif)
 *
 * ARCHITECTURE:
 *   Core 1 (loop):  ToF ISR → read ToF + MLX → copy cam → enqueue packet
 *   Core 0:         tx_task → dequeue → write binary packet over UART
 *
 * PACKET FORMAT (binary, little-endian):
 *   [4B]  magic    0xDEADBEEF
 *   [4B]  seq      uint32
 *   [8B]  ts_us    uint64  (µs since boot)
 *   [128B] tof_dist  uint16[64] — zone distances mm
 *   [128B] tof_sigma uint16[64] — zone sigma mm
 *   [64B]  tof_status uint8[64] — target status (5=valid)
 *   [4B]  mlx_w    uint16 (always 32)
 *   [4B]  mlx_h    uint16 (always 24)
 *   [3072B] mlx_frame float32[768] — °C
 *   [4B]  cam_w    uint32
 *   [4B]  cam_h    uint32
 *   [4B]  cam_len  uint32
 *   [cam_len B] cam_jpeg
 *   [4B]  crc32    CRC32 of all above
 */

// ─────────────────────────────────────────────────────────────────────────────
// Modular sensor hub (ESP32-S3)
// - Synchronized acquisition on ToF INT
// - Binary stream framed via SLIP (robust resync) + CRC32
// - Bidirectional commands to tune ToF/MLX/Camera settings
//
// IMPORTANT: do not print raw text to Serial once streaming starts.
// All host-visible messages are sent as framed Event/Resp messages.
// ─────────────────────────────────────────────────────────────────────────────

#include <Arduino.h>

#include "esp_wifi.h"
#include "esp_bt.h"
#include "nvs_flash.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "hub_config.h"
#include "hub_control.h"
#include "hub_frame.h"
#include "hub_transport.h"
#include "hub_runtime.h"

#include "camera_sync.h"
#include "i2c_bus.h"
#include "mlx90640_driver.h"
#include "tof_vl53l8ch.h"

#if HUB_USE_UVC_CAMERA
#include "uvc_camera.h"
#endif

// ─────────────────────────────────────────────────────────────────────────────
// Globals
// ─────────────────────────────────────────────────────────────────────────────

static hub::I2CBus g_i2c;
static hub::TofVl53l8ch g_tof(g_i2c, kPinTofLpn);
static hub::Mlx90640Driver g_mlx(g_i2c);
static hub::CameraSync g_cam_sync;

#if HUB_USE_UVC_CAMERA
static hub::UvcCamera* g_cam = nullptr;
static bool g_cam_started = false;
#endif

static HardwareSerial& g_link = Serial;  // CH343 UART when CDC-on-boot is disabled
static hub::Transport g_tx(g_link);

volatile bool g_stream_enabled = true;
static volatile bool g_tof_ready = false;
static volatile uint64_t g_tof_irq_ts_us = 0;

volatile hub::StreamMode hub::g_mode = hub::StreamMode::All;
static uint64_t g_next_sample_us = 0;
static constexpr uint64_t kTofWaitTimeoutUs = 200000;  // 200 ms fallback

static bool g_cam_autostart_pending = false;  // Deferred to loop() after scheduler ready

static hub::FrameBuffer* g_frame[2] = {nullptr, nullptr};
static uint8_t g_fill_idx = 0;
static uint32_t g_frame_seq = 0;

static QueueHandle_t g_tx_queue = nullptr;

static uint8_t g_slip_rx_buf[512];
static hub::ControlContext g_ctrl;
static hub::SlipDecoder g_slip(g_slip_rx_buf, sizeof(g_slip_rx_buf), &hub::handleSlipFrame, &g_ctrl);

struct BootI2cScan {
  uint8_t found = 0;
  bool has_mlx_33 = false;
  bool has_vl53_29 = false;  // 7-bit form (0x52 >> 1)
};

static BootI2cScan scan_i2c_boot() {
  BootI2cScan s;
  auto& w = g_i2c.wire();
  for (uint8_t addr = 1; addr < 127; addr++) {
    w.beginTransmission(addr);
    uint8_t err = w.endTransmission();
    if (err == 0) {
      s.found++;
      if (addr == 0x33) s.has_mlx_33 = true;
      if (addr == 0x29) s.has_vl53_29 = true;
    }
  }
  return s;
}

// ─────────────────────────────────────────────────────────────────────────────
// ISR — ToF data-ready
// ─────────────────────────────────────────────────────────────────────────────
static void IRAM_ATTR tof_int_isr() {
  g_tof_irq_ts_us = (uint64_t)esp_timer_get_time();
  g_tof_ready = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// TX task — Core 0
// ─────────────────────────────────────────────────────────────────────────────
static void tx_task(void* /*arg*/) {
  hub::FrameBuffer* fb = nullptr;
  for (;;) {
    if (xQueueReceive(g_tx_queue, &fb, portMAX_DELAY) != pdTRUE) continue;
    if (!fb) continue;

    const uint32_t payload_len = (uint32_t)(sizeof(hub::FrameFixedV1) + fb->fixed.cam.len);
    (void)g_tx.send(hub::MsgType::Frame,
                    fb->fixed.frame_seq,
                    fb->fixed.hub_ts_us,
                    &fb->fixed,
                    payload_len);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// RX task — Core 0 (command channel)
// ─────────────────────────────────────────────────────────────────────────────
static void rx_task(void* /*arg*/) {
  uint8_t buf[128];
  for (;;) {
    int avail = g_link.available();
    if (avail <= 0) {
      vTaskDelay(1);
      continue;
    }
    int n = avail;
    if (n > (int)sizeof(buf)) n = (int)sizeof(buf);
    n = g_link.readBytes((char*)buf, n);
    if (n > 0) {
      g_slip.feed(buf, (size_t)n);
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// setup()  — runs on Core 1
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  // ── 0. Blink pattern to confirm code is running ─────────────────────────
  // If you see LED blink = code uploaded & running
  // If no blink = compilation or upload failed
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }

  // ── 1. Kill radio to free IRQ bandwidth ──────────────────────────────────
  nvs_flash_init();
  esp_wifi_stop();
  esp_bt_controller_disable();

  // ── 2. Serial (UART via CH343p — right USB-C port) ───────────────────────
  // Best-effort: bigger UART TX buffer reduces stalls on large frames.
  // (Not available on all cores; ignored if unsupported.)
  (void)g_link.setTxBufferSize(64 * 1024);
  g_link.begin(kSerialBaud);

  // ── 3. I2C ───────────────────────────────────────────────────────────────
  g_i2c.begin();
  const BootI2cScan i2c_scan = scan_i2c_boot();

  // ── 4. Camera sync pins (GPIO 19/20) ─────────────────────────────────────
  g_cam_sync.begin();

  // ── 5. PSRAM double-buffer for frames ───────────────────────────────────
  for (int i = 0; i < 2; i++) {
    g_frame[i] = (hub::FrameBuffer*)ps_malloc(sizeof(hub::FrameBuffer));
    if (!g_frame[i]) {
      // Can't use framed stream if we can't allocate; blink/loop.
      while (1) {
        delay(200);
      }
    }
    memset(g_frame[i], 0, sizeof(hub::FrameBuffer));
  }

  #if HUB_USE_UVC_CAMERA
  // Construct UvcCamera only after PSRAM and USB stack are ready.
  // Global construction before setup() is unsafe on ESP32-S3 with OPI PSRAM.
  g_cam = new (ps_malloc(sizeof(hub::UvcCamera))) hub::UvcCamera();
  if (!g_cam) { while(1) delay(200); }  // PSRAM exhausted
#endif

  // ── 5. Sensors ───────────────────────────────────────────────────────────
  pinMode(kPinTofInt, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kPinTofInt), tof_int_isr, FALLING);

  const bool tof_ok = g_tof.begin();
  const bool mlx_ok = g_mlx.begin();

  const bool cam_ok = false;  // Default when UVC is disabled or not started yet
#if HUB_USE_UVC_CAMERA
  // Camera autostart deferred to loop() after FreeRTOS scheduler fully boots.
  // Starting USB_STREAM tasks in setup() before scheduler ready → GuruMeditation panic.
#if HUB_UVC_AUTOSTART
  g_cam_autostart_pending = true;
#else
  g_cam_autostart_pending = false;
#endif
  g_cam_started = false;
#endif

  // ── 6. TX task on Core 0 ─────────────────────────────────────────────────
  g_tx_queue = xQueueCreate(2, sizeof(hub::FrameBuffer*));
  if (!g_tx_queue) {
    while (1) delay(200);
  }

  // Command context
  g_ctrl.tx = &g_tx;
  g_ctrl.i2c = &g_i2c;
  g_ctrl.tof = &g_tof;
  g_ctrl.mlx = &g_mlx;
  g_ctrl.cam_sync = &g_cam_sync;
#if HUB_USE_UVC_CAMERA
  g_ctrl.cam = g_cam;
  g_ctrl.cam_started = &g_cam_started;
#endif
  g_ctrl.stream_enabled = &g_stream_enabled;

  // Expose mode to the command handler via a global (simplest for Arduino build).
  // (hub_control.cpp references it via extern declarations.)

  xTaskCreatePinnedToCore(tx_task, "tx_task", 8192, nullptr, 10, nullptr, 0);
  xTaskCreatePinnedToCore(rx_task, "rx_task", 4096, nullptr, 6, nullptr, 0);

  // Send a framed boot event so the host can sync without depending on raw prints.
  char boot[256];
  snprintf(boot, sizeof(boot),
           "BOOT OK\n"
           "tof=%u mlx=%u cam=%u\n"
           "i2c_found=%u mlx_0x33=%u vl53_0x29=%u\n"
           "link=slip+crc v=%u\n",
           (unsigned)tof_ok, (unsigned)mlx_ok, (unsigned)cam_ok,
           (unsigned)i2c_scan.found,
           (unsigned)i2c_scan.has_mlx_33,
           (unsigned)i2c_scan.has_vl53_29,
           (unsigned)hub::kVersion);
  g_tx.send(hub::MsgType::Event, 0, (uint64_t)esp_timer_get_time(), boot, (uint32_t)strlen(boot));
}

// ─────────────────────────────────────────────────────────────────────────────
// loop()  — runs on Core 1
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  g_cam_sync.service();

  // ── Deferred camera autostart (after FreeRTOS scheduler ready) ───────────────
#if HUB_USE_UVC_CAMERA
  static uint32_t loop_count = 0;
  static bool cam_autostart_done = false;
  loop_count++;
  
  // Start camera after scheduler has run ~50 loop iterations (~100ms at normal cadence)
  if (g_cam_autostart_pending && !cam_autostart_done && loop_count > 50) {
    cam_autostart_done = true;
    g_cam_started = g_cam->deferredBegin();
  }
#endif

  if (!g_stream_enabled) {
    vTaskDelay(2);
    return;
  }

  const hub::StreamMode mode = hub::g_mode;
  const uint64_t now_us = (uint64_t)esp_timer_get_time();

  if (mode == hub::StreamMode::All || mode == hub::StreamMode::TofOnly) {
    // Master sync: ToF INT drives cadence.
    if (!g_tof_ready && (now_us - g_tof_irq_ts_us) < kTofWaitTimeoutUs) {
      vTaskDelay(1);
      return;
    }
    g_tof_ready = false;
  } else {
    // Debug modes without ToF: periodic sampling.
    if (g_next_sample_us == 0) g_next_sample_us = now_us;
    if (now_us < g_next_sample_us) {
      vTaskDelay(1);
      return;
    }
    g_next_sample_us = now_us + 50000;  // 20 Hz scheduler tick (sensor libs may run slower)
  }

  hub::FrameBuffer* fb = g_frame[g_fill_idx];
  if (!fb) return;

  fb->fixed.frame_seq = g_frame_seq++;
  fb->fixed.hub_ts_us = (uint64_t)esp_timer_get_time();
  fb->fixed.flags = 0;
  fb->fixed.reserved = 0;

  if (mode == hub::StreamMode::All || mode == hub::StreamMode::TofOnly) {
    if (g_tof.read(fb->fixed.tof)) {
      fb->fixed.tof.ts_us = g_tof_irq_ts_us;
      fb->fixed.flags |= hub::kFlagTofValid;
    }
  }

  if (mode == hub::StreamMode::All || mode == hub::StreamMode::MlxOnly) {
    if (g_mlx.readFrame(fb->fixed.mlx)) {
      fb->fixed.flags |= hub::kFlagMlxValid;
    }
  }

  g_cam_sync.fill(fb->fixed.cam_sync);
#if HUB_USE_CAM_SYNC
  fb->fixed.flags |= hub::kFlagCamSyncValid;
#endif

#if HUB_USE_UVC_CAMERA
  uint32_t cam_len   = 0;
  uint64_t cam_ts_us = 0;
  const bool cam_ok =
      (g_cam && g_cam_started && g_cam->isStarted() &&
       (mode == hub::StreamMode::All || mode == hub::StreamMode::CamOnly))
          ? g_cam->snapshot(fb->cam_bytes, kCamJpegMax, cam_len, cam_ts_us)
          : false;
  fb->fixed.cam.ts_us        = cam_ts_us;
  fb->fixed.cam.cfg.w        = g_cam ? g_cam->settings().w : 0;
  fb->fixed.cam.cfg.h        = g_cam ? g_cam->settings().h : 0;
  fb->fixed.cam.cfg.format_fourcc = hub::kFourCC_MJPG;
  fb->fixed.cam.len          = cam_len;
  if (cam_ok) fb->fixed.flags |= hub::kFlagCamValid;
#endif

  // Enqueue for TX (never block Core 1; drop if behind).
  hub::FrameBuffer* ptr = fb;
  if (xQueueSend(g_tx_queue, &ptr, 0) == pdTRUE) {
    g_fill_idx ^= 1;
  }
}