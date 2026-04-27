#pragma once

#include <stdint.h>

#include "hub_config.h"

#if HUB_USE_UVC_CAMERA

#include <Arduino.h>
#include "USB_STREAM.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "hub_frame.h"

namespace hub {

struct UvcSettings {
  uint32_t w = 320;
  uint32_t h = 240;
  uint32_t interval_us = 66667;   // frame period in microseconds (~15 FPS)
};

class UvcCamera {
 public:
  UvcCamera();

  bool begin();
  void stop();
  bool isStarted() const { return started_; }
  bool deferredBegin();  // Start after FreeRTOS scheduler fully initialized

  const UvcSettings& settings() const { return settings_; }
  bool applySettings(const UvcSettings& s);

  // Copies the latest JPEG (if any) into dst. Returns true if a frame was copied.
  bool snapshot(uint8_t* dst, uint32_t dst_cap, uint32_t& out_len, uint64_t& out_ts_us);

 private:
  static void onFrame(uvc_frame_t* frame, void* user);

  USB_STREAM usb_;
  UvcSettings settings_;

  SemaphoreHandle_t mutex_ = nullptr;
  uint8_t* cam_buf_ = nullptr;
  uint32_t cam_len_ = 0;
  uint64_t cam_ts_us_ = 0;

  uint8_t* usb_ta_ = nullptr;
  uint8_t* usb_tb_ = nullptr;
  uint8_t* usb_fb_ = nullptr;
  bool started_ = false;
};

}  // namespace hub

#endif  // HUB_USE_UVC_CAMERA
