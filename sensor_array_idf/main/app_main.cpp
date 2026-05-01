#include <new>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"

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

static const char* TAG = "Main";

// ── Globals ───────────────────────────────────────────────────────────────────

static hub::I2CBus       g_i2c;
static hub::TofVl53l8ch  g_tof(g_i2c, kPinTofLpn);
static hub::Mlx90640Driver g_mlx(g_i2c);
static hub::CameraSync   g_cam_sync;

#if HUB_USE_UVC_CAMERA
static hub::UvcCamera*   g_cam = nullptr;
static bool              g_cam_started = false;
#endif

static uart_port_t       g_uart = UART_NUM_0;
static hub::Transport    g_tx(g_uart);
static bool              g_link_init_done = false;

volatile bool            g_stream_enabled = true;
static volatile bool     g_tof_ready      = false;
static volatile uint64_t g_tof_irq_ts_us  = 0;

volatile hub::StreamMode hub::g_mode = hub::StreamMode::All;
static uint64_t          g_next_sample_us = 0;
static constexpr uint64_t kTofWaitTimeoutUs = 200000;

static bool              g_cam_autostart_pending = false;

static hub::FrameBuffer* g_frame[2]  = {nullptr, nullptr};
static uint8_t           g_fill_idx  = 0;
static uint32_t          g_frame_seq = 0;

static QueueHandle_t     g_tx_queue  = nullptr;

static uint8_t           g_slip_rx_buf[512];
static hub::ControlContext g_ctrl;
static hub::SlipDecoder  g_slip(g_slip_rx_buf, sizeof(g_slip_rx_buf),
                                 &hub::handleSlipFrame, &g_ctrl);

// ── I2C boot scan (pure ESP-IDF) ─────────────────────────────────────────────

struct BootI2cScan {
  uint8_t found       = 0;
  bool    has_mlx_33  = false;
  bool    has_vl53_29 = false;
};

static BootI2cScan scan_i2c_boot() {
  BootI2cScan s;
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (g_i2c.probe(addr)) {
      s.found++;
      if (addr == 0x33) s.has_mlx_33  = true;
      if (addr == 0x29) s.has_vl53_29 = true;
    }
  }
  return s;
}

// ── ISR ───────────────────────────────────────────────────────────────────────

static void IRAM_ATTR tof_int_isr(void* /*arg*/) {
  g_tof_irq_ts_us = (uint64_t)esp_timer_get_time();
  g_tof_ready     = true;
}

// ── TX task (Core 0) ─────────────────────────────────────────────────────────

static void tx_task(void* /*arg*/) {
  hub::FrameBuffer* fb = nullptr;
  for (;;) {
    if (xQueueReceive(g_tx_queue, &fb, portMAX_DELAY) != pdTRUE) continue;
    if (!fb) continue;
    const uint32_t plen =
        (uint32_t)(sizeof(hub::FrameFixedV1) + fb->fixed.cam.len);
    g_tx.send(hub::MsgType::Frame, fb->fixed.frame_seq,
              fb->fixed.hub_ts_us, &fb->fixed, plen);
  }
}

// ── RX task (Core 0) ─────────────────────────────────────────────────────────

static void rx_task(void* /*arg*/) {
  uint8_t buf[128];
  for (;;) {
    size_t avail = 0;
    uart_get_buffered_data_len(g_uart, &avail);
    if (avail == 0) { vTaskDelay(1); continue; }
    size_t n = (avail < sizeof(buf)) ? avail : sizeof(buf);
    n = uart_read_bytes(g_uart, buf, n, pdMS_TO_TICKS(10));
    if (n > 0) g_slip.feed(buf, n);
  }
}

// ── Loop task (Core 1) ───────────────────────────────────────────────────────

static void loop_task(void* /*arg*/) {
  for (;;) {
    g_cam_sync.service();

#if HUB_USE_UVC_CAMERA
    static uint32_t loop_count       = 0;
    static bool     cam_autostart_done = false;
    loop_count++;
    if (g_cam_autostart_pending && !cam_autostart_done && loop_count > 50) {
      cam_autostart_done = true;
      g_cam_started      = g_cam->deferredBegin();
      ESP_LOGI("CAM", "deferredBegin -> %d", (int)g_cam_started);
    }
#endif

    if (!g_stream_enabled) { vTaskDelay(2); continue; }

    const hub::StreamMode mode   = hub::g_mode;
    const uint64_t        now_us = (uint64_t)esp_timer_get_time();

    if (mode == hub::StreamMode::All || mode == hub::StreamMode::TofOnly) {
      if (!g_tof_ready && (now_us - g_tof_irq_ts_us) < kTofWaitTimeoutUs) {
        vTaskDelay(1); continue;
      }
      g_tof_ready = false;
    } else {
      if (g_next_sample_us == 0) g_next_sample_us = now_us;
      if (now_us < g_next_sample_us) { vTaskDelay(1); continue; }
      g_next_sample_us = now_us + 50000;
    }

    hub::FrameBuffer* fb = g_frame[g_fill_idx];
    if (!fb) continue;

    fb->fixed.frame_seq  = g_frame_seq++;
    fb->fixed.hub_ts_us  = (uint64_t)esp_timer_get_time();
    fb->fixed.flags      = 0;
    fb->fixed.reserved   = 0;

    if (mode == hub::StreamMode::All || mode == hub::StreamMode::TofOnly) {
      if (g_tof.read(fb->fixed.tof)) {
        fb->fixed.tof.ts_us = g_tof_irq_ts_us;
        fb->fixed.flags |= hub::kFlagTofValid;
      }
    }

    if (mode == hub::StreamMode::All || mode == hub::StreamMode::MlxOnly) {
      if (g_mlx.readFrame(fb->fixed.mlx))
        fb->fixed.flags |= hub::kFlagMlxValid;
    }

    g_cam_sync.fill(fb->fixed.cam_sync);
#if HUB_USE_CAM_SYNC
    fb->fixed.flags |= hub::kFlagCamSyncValid;
#endif

#if HUB_USE_UVC_CAMERA
    uint32_t cam_len   = 0;
    uint64_t cam_ts_us = 0;

    static bool cam_diag_printed = false;
    if (!cam_diag_printed && g_cam_started) {
        cam_diag_printed = true;
        ESP_LOGI("CAM", "diag: cam=%d started=%d ready=%d mode=%d",
                 (int)(g_cam != nullptr),
                 (int)g_cam_started,
                 (int)(g_cam && g_cam->isReady()),
                 (int)mode);
    }

    const bool cam_ok =
        (g_cam && g_cam_started && g_cam->isReady() &&
         (mode == hub::StreamMode::All || mode == hub::StreamMode::CamOnly))
            ? g_cam->snapshot(fb->cam_bytes, kCamJpegMax, cam_len, cam_ts_us)
            : false;
    fb->fixed.cam.ts_us          = cam_ts_us;
    fb->fixed.cam.cfg.w          = g_cam ? g_cam->settings().w : 0;
    fb->fixed.cam.cfg.h          = g_cam ? g_cam->settings().h : 0;
    fb->fixed.cam.cfg.format_fourcc = hub::kFourCC_MJPG;
    fb->fixed.cam.len            = cam_len;
    if (cam_ok) fb->fixed.flags |= hub::kFlagCamValid;
#endif

    hub::FrameBuffer* ptr = fb;
    if (xQueueSend(g_tx_queue, &ptr, 0) == pdTRUE)
      g_fill_idx ^= 1;
  }
}

// ── app_main ─────────────────────────────────────────────────────────────────

extern "C" void app_main(void) {
  // NVS init (required by some ESP-IDF subsystems)
  nvs_flash_init();

  // UART (CH343p USB-UART bridge)
  if (!g_link_init_done) {
    uart_driver_install(g_uart, 64 * 1024, 64 * 1024, 0, NULL, 0);
    uart_config_t ucfg = {};
    ucfg.baud_rate  = kSerialBaud;
    ucfg.data_bits  = UART_DATA_8_BITS;
    ucfg.parity     = UART_PARITY_DISABLE;
    ucfg.stop_bits  = UART_STOP_BITS_1;
    ucfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    uart_param_config(g_uart, &ucfg);
    uart_set_pin(g_uart, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    g_link_init_done = true;
  }

  // I2C
  g_i2c.begin();
  const BootI2cScan i2c_scan = scan_i2c_boot();

  // Camera sync
  g_cam_sync.begin();

  // PSRAM double-buffer
  for (int i = 0; i < 2; i++) {
    g_frame[i] = (hub::FrameBuffer*)heap_caps_malloc(
        sizeof(hub::FrameBuffer), MALLOC_CAP_SPIRAM);
    if (!g_frame[i]) {
      ESP_LOGE(TAG, "PSRAM alloc failed");
      while (1) vTaskDelay(pdMS_TO_TICKS(500));
    }
    memset(g_frame[i], 0, sizeof(hub::FrameBuffer));
  }

#if HUB_USE_UVC_CAMERA
  g_cam = new (heap_caps_malloc(sizeof(hub::UvcCamera), MALLOC_CAP_SPIRAM))
              hub::UvcCamera();
  if (!g_cam) { ESP_LOGE(TAG, "UvcCamera alloc failed"); while (1) vTaskDelay(500); }
#endif

  // ToF INT GPIO
  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = (1ULL << kPinTofInt);
  io_conf.mode         = GPIO_MODE_INPUT;
  io_conf.pull_up_en   = GPIO_PULLUP_ENABLE;
  io_conf.intr_type    = GPIO_INTR_NEGEDGE;
  gpio_config(&io_conf);
  gpio_install_isr_service(0);
  gpio_isr_handler_add((gpio_num_t)kPinTofInt, tof_int_isr, nullptr);

  const bool tof_ok = g_tof.begin();
  const bool mlx_ok = g_mlx.begin();

#if HUB_USE_UVC_CAMERA
#if HUB_UVC_AUTOSTART
  g_cam_autostart_pending = true;
#else
  g_cam_autostart_pending = false;
#endif
  g_cam_started = false;
#endif

  // TX queue
  g_tx_queue = xQueueCreate(2, sizeof(hub::FrameBuffer*));
  if (!g_tx_queue) { while (1) vTaskDelay(200); }

  // Control context
  g_ctrl.tx           = &g_tx;
  g_ctrl.i2c          = &g_i2c;
  g_ctrl.tof          = &g_tof;
  g_ctrl.mlx          = &g_mlx;
  g_ctrl.cam_sync     = &g_cam_sync;
#if HUB_USE_UVC_CAMERA
  g_ctrl.cam          = g_cam;
  g_ctrl.cam_started  = &g_cam_started;
#endif
  g_ctrl.stream_enabled = &g_stream_enabled;

  xTaskCreatePinnedToCore(tx_task,   "tx",   8192, nullptr, 10, nullptr, 0);
  xTaskCreatePinnedToCore(rx_task,   "rx",   4096, nullptr,  6, nullptr, 0);

  // Boot event
  char boot[256];
  snprintf(boot, sizeof(boot),
           "BOOT OK\ntof=%u mlx=%u\ni2c_found=%u mlx_0x33=%u vl53_0x29=%u\n"
           "link=slip+crc32 v=%u\n",
           (unsigned)tof_ok, (unsigned)mlx_ok,
           (unsigned)i2c_scan.found,
           (unsigned)i2c_scan.has_mlx_33,
           (unsigned)i2c_scan.has_vl53_29,
           (unsigned)hub::kVersion);
  g_tx.send(hub::MsgType::Event, 0, (uint64_t)esp_timer_get_time(),
            boot, (uint32_t)strlen(boot));

  xTaskCreatePinnedToCore(loop_task, "loop", 8192, nullptr, 5, nullptr, 1);

  vTaskDelete(nullptr);
}
