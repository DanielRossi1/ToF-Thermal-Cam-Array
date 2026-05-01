#include "uvc_camera.h"

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

UvcCamera::UvcCamera() {}

UvcCamera::~UvcCamera() {
  stop();
  heap_caps_free(cam_buf_);
  heap_caps_free(xfer_a_);
  heap_caps_free(xfer_b_);
  heap_caps_free(frame_buf_);
  if (mutex_) vSemaphoreDelete(mutex_);
}

bool UvcCamera::deferredBegin() {
  return begin();
}

bool UvcCamera::begin() {
  if (started_) return true;

  if (!mutex_) {
    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) return false;
  }

  if (!cam_buf_)
    cam_buf_   = (uint8_t*)heap_caps_malloc(kCamJpegMax, MALLOC_CAP_SPIRAM);
  if (!xfer_a_)
    xfer_a_    = (uint8_t*)heap_caps_malloc(kXferBufSize,
                                             MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  if (!xfer_b_)
    xfer_b_    = (uint8_t*)heap_caps_malloc(kXferBufSize,
                                             MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  if (!frame_buf_)
    frame_buf_ = (uint8_t*)heap_caps_malloc(kCamJpegMax, MALLOC_CAP_SPIRAM);

  if (!cam_buf_ || !xfer_a_ || !xfer_b_ || !frame_buf_) {
    ESP_LOGE(TAG, "buffer alloc failed");
    return false;
  }

  // Frame interval: usb_stream expects 100 ns units.
  const uint32_t interval_100ns = settings_.interval_us * 10u;
  uvc_config_t cfg = {};
  cfg.frame_width      = 320;
  cfg.frame_height     = 240;
  cfg.frame_interval   = 333333;
  cfg.xfer_buffer_size = kXferBufSize;
  cfg.xfer_buffer_a    = xfer_a_;
  cfg.xfer_buffer_b    = xfer_b_;
  cfg.frame_buffer_size = kCamJpegMax;
  cfg.frame_buffer     = frame_buf_;
  cfg.frame_cb         = UvcCamera::onFrame;
  cfg.frame_cb_arg     = this;
  cfg.format           = UVC_FORMAT_MJPEG;
  cfg.xfer_type        = UVC_XFER_ISOC;
  cfg.interface        = 1;
  cfg.interface_alt    = 2;
  cfg.ep_addr          = 0x81;
  cfg.ep_mps           = 512;
  cfg.format_index     = 1;
  cfg.interface_alt    = 4;

  esp_err_t ret = uvc_streaming_config(&cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "uvc_streaming_config failed: %d", ret);
    return false;
  }

  ret = usb_streaming_start();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "usb_streaming_start failed: %d", ret);
    vTaskDelay(pdMS_TO_TICKS(2000));
    return false;
  }

  started_ = true;
  ESP_LOGI(TAG, "started %lux%lu @ %lu fps",
           (unsigned long)settings_.w,
           (unsigned long)settings_.h,
           (unsigned long)(1000000u / settings_.interval_us));
  return true;
}

void UvcCamera::stop() {
  if (!started_) return;
  usb_streaming_stop();
  started_    = false;
  cam_ready_  = false;
}

bool UvcCamera::applySettings(const UvcSettings& s) {
  settings_ = s;
  if (!started_) return true;

  // Live reconfiguration requires a stop/start cycle.
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
    self->cam_len_    = len;
    self->cam_ts_us_  = (uint64_t)esp_timer_get_time();
    self->cam_ready_  = true;
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

#endif  // HUB_USE_UVC_CAMERA
