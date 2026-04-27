#include "camera_sync.h"

#include <Arduino.h>

#include "esp_timer.h"

#include "hub_config.h"

namespace hub {

#if HUB_USE_CAM_SYNC

volatile uint64_t CameraSync::last_edge_ts_us_ = 0;

CameraSync::CameraSync() {}

void CameraSync::begin() {
  pinMode(kPinCamSyncOut, OUTPUT);
  digitalWrite(kPinCamSyncOut, LOW);

  pinMode(kPinCamSyncIn, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kPinCamSyncIn), &CameraSync::isrEdge, CHANGE);
}

void CameraSync::applySettings(const CamSyncSettings& s) {
  settings_ = s;
  if (!settings_.enabled) {
    settings_.trigger_period_us = 0;
  }
  next_trigger_ts_us_ = 0;
}

void CameraSync::service() {
  if (!settings_.enabled) return;
  if (settings_.trigger_period_us == 0) return;

  const uint64_t now = (uint64_t)esp_timer_get_time();
  if (next_trigger_ts_us_ == 0) {
    next_trigger_ts_us_ = now;
  }
  if (now < next_trigger_ts_us_) return;

  // Emit one trigger pulse.
  last_trigger_ts_us_ = now;
  digitalWrite(kPinCamSyncOut, HIGH);
  delayMicroseconds(settings_.trigger_pulse_us);
  digitalWrite(kPinCamSyncOut, LOW);

  next_trigger_ts_us_ += (uint64_t)settings_.trigger_period_us;
  // Catch up if we fell behind.
  if (next_trigger_ts_us_ + settings_.trigger_period_us < now) {
    next_trigger_ts_us_ = now + settings_.trigger_period_us;
  }
}

void CameraSync::fill(CamSyncV1& out) const {
  out.last_trigger_ts_us = last_trigger_ts_us_;
  out.last_edge_ts_us = last_edge_ts_us_;
  out.trigger_period_us = settings_.trigger_period_us;
  out.trigger_pulse_us = settings_.trigger_pulse_us;
}

void IRAM_ATTR CameraSync::isrEdge() {
  last_edge_ts_us_ = (uint64_t)esp_timer_get_time();
}

#endif  // HUB_USE_CAM_SYNC

}  // namespace hub
