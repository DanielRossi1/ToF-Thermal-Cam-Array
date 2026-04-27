#include "uvc_camera.h"

#if HUB_USE_UVC_CAMERA

#include <string.h>

#include "esp_timer.h"
#include "esp_heap_caps.h"

namespace hub {

// ESP32_USB_STREAM reference examples use 55 KiB buffers for stable UVC handling.
static constexpr uint32_t kUsbTransSize = 55u * 1024u;

static uint32_t intervalUsTo100ns(uint32_t us) {
  // USB_STREAM expects 100ns units.
  // 1 us = 10 * 100ns.
  return us * 10u;
}

UvcCamera::UvcCamera() {}

bool UvcCamera::deferredBegin() {
  // Call after FreeRTOS scheduler is running (from loop, not setup).
  // Avoids GuruMeditation panic from task creation in setup().
  return begin();
}

bool UvcCamera::begin() {
  if (started_) return true;

  // ── Mutex ─────────────────────────────────────────────────────────────────
  if (!mutex_) {
    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) return false;
  }

  // ── Buffers ───────────────────────────────────────────────────────────────
  // USB HCD requires DMA-capable INTERNAL RAM for transfer buffers.
  // Frame buffers can live in PSRAM.
  if (!cam_buf_) cam_buf_ = (uint8_t*)ps_malloc(kCamJpegMax);
  if (!usb_ta_)  usb_ta_  = (uint8_t*)heap_caps_malloc(kUsbTransSize, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  if (!usb_tb_)  usb_tb_  = (uint8_t*)heap_caps_malloc(kUsbTransSize, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  if (!usb_fb_)  usb_fb_  = (uint8_t*)ps_malloc(kCamJpegMax);

  if (!cam_buf_ || !usb_ta_ || !usb_tb_ || !usb_fb_) {
    free(cam_buf_); cam_buf_ = nullptr;
    heap_caps_free(usb_ta_);  usb_ta_  = nullptr;
    heap_caps_free(usb_tb_);  usb_tb_  = nullptr;
    free(usb_fb_);  usb_fb_  = nullptr;
    return false;
  }

  // ── Configure UVC ─────────────────────────────────────────────────────────
  const uint32_t fi = settings_.interval_us * 10;

  usb_.uvcConfiguration(
    (uint16_t)settings_.w,
    (uint16_t)settings_.h,
    fi,
    kUsbTransSize,
    usb_ta_,
    usb_tb_,
    kCamJpegMax,
    usb_fb_
  );

  usb_.uvcCamRegisterCb(UvcCamera::onFrame, this);

  usb_.start();
  // Note: NO connectWait() here — that's a blocking call.
  // USB_STREAM will enumerate asynchronously; frames arrive when ready.

  started_ = true;
  return true;
}

void UvcCamera::stop() {
  if (!started_) return;
  usb_.stop();
  started_ = false;
}

bool UvcCamera::applySettings(const UvcSettings& s) {
  settings_ = s;

  if (!started_) {
    return true;
  }

  // USB_STREAM does not currently support live reconfiguration cleanly; restart.
  // Best-effort.
  usb_.stop();
  delay(50);
  usb_.uvcConfiguration(settings_.w, settings_.h, intervalUsTo100ns(settings_.interval_us),
                        kUsbTransSize, usb_ta_, usb_tb_,
                        kCamJpegMax, usb_fb_);
  usb_.start();
  delay(200);
  return true;
}

void UvcCamera::onFrame(uvc_frame_t* frame, void* user) {
  auto* self = static_cast<UvcCamera*>(user);
  if (!frame || !frame->data || frame->data_bytes == 0) return;

  if (xSemaphoreTake(self->mutex_, 0) == pdTRUE) {
    const uint32_t len = (frame->data_bytes <= kCamJpegMax)
                           ? (uint32_t)frame->data_bytes
                           : kCamJpegMax;
    memcpy(self->cam_buf_, frame->data, len);
    self->cam_len_   = len;
    self->cam_ts_us_ = (uint64_t)esp_timer_get_time();
    xSemaphoreGive(self->mutex_);
  }
}

bool UvcCamera::snapshot(uint8_t* dst, uint32_t dst_cap, uint32_t& out_len, uint64_t& out_ts_us) {
  out_len   = 0;
  out_ts_us = 0;
  
  if (!started_ || !mutex_ || !dst || dst_cap == 0) return false;

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
