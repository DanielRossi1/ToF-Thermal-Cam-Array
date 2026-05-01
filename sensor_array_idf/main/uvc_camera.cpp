#include "uvc_camera.h"
#include <inttypes.h>   // PRIu32, PRIu64, etc.

#if HUB_USE_UVC_CAMERA

#include <string.h>
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "usb_stream.h"

static const char* TAG = "UVC";

namespace hub {

// ─── Descriptor constants (read directly from lsusb output) ──────────────────
//
// Format index 1 = MJPEG,  frame_interval = 83333 (100ns units = ~12fps... wait)
//
// IMPORTANT: the camera's dwFrameInterval is 83333 in 100-nanosecond units.
// 83333 × 100 ns = 8.333 ms → 120 fps.  That is correct per the descriptor.
// The YUYV format uses 333333 × 100ns = 33.3ms = 30fps.
//
// Interface 1, alternate settings and their max packet sizes (from lsusb):
//   Alt 1 →  128 B   Alt 2 →  512 B   Alt 3 → 1024 B
//   Alt 4 → 1536 B   Alt 5 → 2048 B   Alt 6 → 2688 B   Alt 7 → 3072 B
//
// Rule: ep_mps MUST equal the wMaxPacketSize of the chosen alternate setting.
// The USB host controller and the UVC driver use ep_mps to reserve the bus
// bandwidth. A mismatch → the isochronous pipe gets the wrong bandwidth slice
// and frames are never delivered.
//
// For 320×240 MJPEG @ 120 fps the peak bitrate is ~147 Mbit/s compressed,
// realistically far lower. Alt 4 (1536 B/pkt at 8000 pkts/s = ~98 Mbit/s
// headroom) works well. Use Alt 5 (2048 B/pkt) if you see frame drops.
// ─────────────────────────────────────────────────────────────────────────────

// Chosen configuration — easy to adjust:
static constexpr uint8_t  kUvcInterface    = 1;
static constexpr uint8_t  kUvcAlt         = 4;      // wMaxPacketSize=1536
static constexpr uint16_t kUvcEpMps       = 1536u;   // MUST match alt above
static constexpr uint8_t  kUvcEpAddr      = 0x81;   // EP 1 IN (from descriptor)
static constexpr uint32_t kFrameW         = 320u;
static constexpr uint32_t kFrameH         = 240u;
// BUG FIX 1: Frame interval must match the FORMAT, not be borrowed from YUYV.
// MJPEG descriptor says dwFrameInterval=83333 (100ns units).
// YUYV descriptor says 333333. We are using MJPEG → use 83333.
static constexpr uint32_t kFrameInterval  = 83333u;  // was wrong: 333333

UvcCamera::UvcCamera() {}

UvcCamera::~UvcCamera() {
  stop();
  heap_caps_free(cam_buf_);
  heap_caps_free(xfer_a_);
  heap_caps_free(xfer_b_);
  heap_caps_free(frame_buf_);
  if (mutex_) vSemaphoreDelete(mutex_);
}

bool UvcCamera::deferredBegin() { return begin(); }

bool UvcCamera::begin() {
  if (started_) return true;

  if (!mutex_) {
    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) return false;
  }

  // Allocate buffers on first call only.
  if (!cam_buf_)
    cam_buf_   = (uint8_t*)heap_caps_malloc(kCamJpegMax,
                                             MALLOC_CAP_SPIRAM);
  if (!xfer_a_)
    xfer_a_    = (uint8_t*)heap_caps_malloc(kXferBufSize,
                                             MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  if (!xfer_b_)
    xfer_b_    = (uint8_t*)heap_caps_malloc(kXferBufSize,
                                             MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  if (!frame_buf_)
    frame_buf_ = (uint8_t*)heap_caps_malloc(kCamJpegMax,
                                             MALLOC_CAP_SPIRAM);

  if (!cam_buf_ || !xfer_a_ || !xfer_b_ || !frame_buf_) {
    ESP_LOGE(TAG, "buffer alloc failed");
    return false;
  }

  uvc_config_t cfg = {};

  cfg.frame_width        = kFrameW;
  cfg.frame_height       = kFrameH;

  // BUG FIX 2 (same as above): correct interval for MJPEG.
  cfg.frame_interval     = kFrameInterval;

  cfg.xfer_buffer_size   = kXferBufSize;
  cfg.xfer_buffer_a      = xfer_a_;
  cfg.xfer_buffer_b      = xfer_b_;
  cfg.frame_buffer_size  = kCamJpegMax;
  cfg.frame_buffer       = frame_buf_;
  cfg.frame_cb           = UvcCamera::onFrame;
  cfg.frame_cb_arg       = this;
  cfg.format             = UVC_FORMAT_MJPEG;
  cfg.xfer_type          = UVC_XFER_ISOC;

  cfg.interface          = kUvcInterface;
  // BUG FIX 3: interface_alt was assigned TWICE in the original code:
  //   cfg.interface_alt = 2;   ← first assignment (512 B/pkt)
  //   ...
  //   cfg.interface_alt = 4;   ← second assignment silently overwrites
  // Only the last value survives, but ep_mps was still 512, not 1536.
  // Now set once, consistently:
  cfg.interface_alt      = kUvcAlt;   // 4 → wMaxPacketSize = 1536

  cfg.ep_addr            = kUvcEpAddr;
  // BUG FIX 4: ep_mps must equal wMaxPacketSize of the chosen alt setting.
  // Original had ep_mps=512 while interface_alt ended up as 4 (1536 B/pkt).
  // The driver uses ep_mps for isochronous bandwidth reservation.
  // With a mismatch the pipe is underallocated and no data flows.
  cfg.ep_mps             = kUvcEpMps;   // 1536 to match alt 4

  // format_index=1 selects the MJPEG format descriptor (correct, unchanged).
  cfg.format_index       = 1;

  // frame_index is 1-based and selects the resolution within the format.
  // From the descriptor: MJPEG frame index 3 = 320×240.
  // Some usb_stream versions use frame_width/frame_height for matching instead,
  // so set both to be safe.
  cfg.frame_index        = 3;

  esp_err_t ret = uvc_streaming_config(&cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "uvc_streaming_config failed: 0x%x", ret);
    return false;
  }

  ret = usb_streaming_start();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "usb_streaming_start failed: 0x%x", ret);
    vTaskDelay(pdMS_TO_TICKS(2000));
    return false;
  }

  started_ = true;
  ESP_LOGI(TAG, "UVC started %" PRIu32 "x%" PRIu32
              " MJPEG alt=%" PRIu32 " ep_mps=%" PRIu32
              " interval=%" PRIu32,
         (uint32_t)kFrameW,
         (uint32_t)kFrameH,
         (uint32_t)kUvcAlt,
         (uint32_t)kUvcEpMps,
         (uint32_t)kFrameInterval);
  return true;
}

void UvcCamera::stop() {
  if (!started_) return;
  usb_streaming_stop();
  started_   = false;
  cam_ready_ = false;
}

bool UvcCamera::applySettings(const UvcSettings& s) {
  settings_ = s;
  if (!started_) return true;
  stop();
  vTaskDelay(pdMS_TO_TICKS(100));
  return begin();
}

void UvcCamera::onFrame(uvc_frame_t* frame, void* user) {
  auto* self = static_cast<UvcCamera*>(user);
  if (!frame || !frame->data || frame->data_bytes == 0) return;

  if (xSemaphoreTake(self->mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
    const uint32_t len = (frame->data_bytes <= kCamJpegMax)
                           ? (uint32_t)frame->data_bytes
                           : kCamJpegMax;
    memcpy(self->cam_buf_, frame->data, len);
    self->cam_len_   = len;
    self->cam_ts_us_ = (uint64_t)esp_timer_get_time();
    self->cam_ready_ = true;
    xSemaphoreGive(self->mutex_);
  }
}

bool UvcCamera::snapshot(uint8_t* dst, uint32_t dst_cap,
                          uint32_t& out_len, uint64_t& out_ts_us) {
  out_len = out_ts_us = 0;
  if (!started_ || !cam_ready_ || !mutex_ || !dst || dst_cap == 0) return false;

  if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(2)) != pdTRUE) return false;
  const uint32_t n  = cam_len_;
  const uint64_t ts = cam_ts_us_;
  if (n > 0 && n <= dst_cap) {
    memcpy(dst, cam_buf_, n);
    out_len   = n;
    out_ts_us = ts;
  }
  xSemaphoreGive(mutex_);
  return out_len > 0;
}

}  // namespace hub
#endif // HUB_USE_UVC_CAMERA
