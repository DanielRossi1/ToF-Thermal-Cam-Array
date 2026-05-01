#include "camera_sync.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "hub_config.h"

static const char* TAG = "CamSync";

namespace hub {

#if HUB_USE_CAM_SYNC

volatile uint64_t CameraSync::last_edge_ts_us_ = 0;

CameraSync::CameraSync() {}

void CameraSync::begin() {
  gpio_set_direction((gpio_num_t)kPinCamSyncOut, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)kPinCamSyncOut, 0);

  gpio_set_direction((gpio_num_t)kPinCamSyncIn, GPIO_MODE_INPUT);
  gpio_set_pull_mode((gpio_num_t)kPinCamSyncIn, GPIO_PULLUP_ONLY);
  gpio_install_isr_service(0);
  gpio_isr_handler_add((gpio_num_t)kPinCamSyncIn, &CameraSync::isrEdge, nullptr);
  gpio_set_intr_type((gpio_num_t)kPinCamSyncIn, GPIO_INTR_POSEDGE);
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
  gpio_set_level((gpio_num_t)kPinCamSyncOut, 1);
  vTaskDelay(pdMS_TO_TICKS(settings_.trigger_pulse_us / 1000));
  gpio_set_level((gpio_num_t)kPinCamSyncOut, 0);

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
