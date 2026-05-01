#pragma once
#include <stdint.h>
#include "hub_config.h"

#if HUB_USE_UVC_CAMERA

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "usb_stream.h"
#include "hub_frame.h"

namespace hub {

struct UvcSettings {
  uint32_t w           = 320;
  uint32_t h           = 240;
  uint32_t interval_us = 66666;
};

class UvcCamera {
 public:
  UvcCamera();
  ~UvcCamera();

  bool begin();
  void stop();

  bool isStarted() const { return started_; }
  bool isReady()   const { return cam_ready_; }
  bool deferredBegin();

  const UvcSettings& settings() const { return settings_; }
  bool applySettings(const UvcSettings& s);

  // Copy latest JPEG frame into dst.  Returns true if a frame was available.
  bool snapshot(uint8_t* dst, uint32_t dst_cap,
                uint32_t& out_len, uint64_t& out_ts_us);

 private:
  static void onFrame(uvc_frame_t* frame, void* user);

  UvcSettings      settings_;
  SemaphoreHandle_t mutex_      = nullptr;

  // Frame buffer (PSRAM) — latest decoded JPEG lives here.
  uint8_t* cam_buf_  = nullptr;
  uint32_t cam_len_  = 0;
  uint64_t cam_ts_us_= 0;

  // USB transfer buffers (DMA-capable internal RAM).
  uint8_t* xfer_a_ = nullptr;
  uint8_t* xfer_b_ = nullptr;
  // USB frame assembly buffer (PSRAM OK).
  uint8_t* frame_buf_ = nullptr;

  bool started_               = false;
  volatile bool cam_ready_    = false;

  static constexpr uint32_t kXferBufSize = 55u * 1024u;
};

}  // namespace hub

#endif  // HUB_USE_UVC_CAMERA
