#include "tof_vl53l8ch.h"

#include <string.h>

#include <Arduino.h>

#include "esp_timer.h"

namespace hub {

TofVl53l8ch::TofVl53l8ch(I2CBus& bus, int lpn_pin)
    : bus_(bus), sensor_(&bus.wire(), lpn_pin, -1) {}

bool TofVl53l8ch::begin() {
  {
    I2CLock lk(bus_, pdMS_TO_TICKS(250));
    if (!lk.ok()) return false;

    // Configure LPn pin sequence (no Wire.begin side effects in this library version).
    (void)sensor_.begin();

    uint8_t status = sensor_.init();
    if (status != VL53LMZ_STATUS_OK) {
      Serial.printf("[ToF] init failed: %u\n", (unsigned)status);
      return false;
    }
  }

  return applySettings(settings_);
}

bool TofVl53l8ch::startRangingLocked() {
  if (ranging_started_) return true;
  uint8_t status = sensor_.start_ranging();
  if (status != VL53LMZ_STATUS_OK) {
    Serial.printf("[ToF] start_ranging failed: %u\n", (unsigned)status);
    return false;
  }
  ranging_started_ = true;
  return true;
}

bool TofVl53l8ch::applySettings(const TofSettings& s) {
  TofSettings next = s;
  if (next.side != 4 && next.side != 8) next.side = 8;
  if (next.ranging_hz == 0) next.ranging_hz = 15;
  if (next.integration_time_ms == 0) next.integration_time_ms = 50;

  I2CLock lk(bus_, pdMS_TO_TICKS(250));
  if (!lk.ok()) return false;

  // Stop ranging before reconfig (best-effort)
  if (ranging_started_) {
    (void)sensor_.stop_ranging();
    ranging_started_ = false;
  }

  uint8_t res = (next.side == 4) ? VL53LMZ_RESOLUTION_4X4 : VL53LMZ_RESOLUTION_8X8;
  uint8_t status = sensor_.set_resolution(res);
  if (status != VL53LMZ_STATUS_OK) {
    Serial.printf("[ToF] set_resolution failed: %u\n", (unsigned)status);
    return false;
  }

  status = sensor_.set_ranging_frequency_hz((uint8_t)next.ranging_hz);
  if (status != VL53LMZ_STATUS_OK) {
    Serial.printf("[ToF] set_ranging_frequency_hz failed: %u\n", (unsigned)status);
    // keep going
  }

  status = sensor_.set_ranging_mode(next.continuous ? VL53LMZ_RANGING_MODE_CONTINUOUS
                                                    : VL53LMZ_RANGING_MODE_AUTONOMOUS);
  if (status != VL53LMZ_STATUS_OK) {
    Serial.printf("[ToF] set_ranging_mode failed: %u\n", (unsigned)status);
  }

  status = sensor_.set_integration_time_ms((uint16_t)next.integration_time_ms);
  if (status != VL53LMZ_STATUS_OK) {
    Serial.printf("[ToF] set_integration_time_ms failed: %u\n", (unsigned)status);
  }

  if (!startRangingLocked()) return false;

  settings_ = next;
  return true;
}

bool TofVl53l8ch::read(TofDataV1& out) {
  I2CLock lk(bus_, pdMS_TO_TICKS(80));
  if (!lk.ok()) return false;

  VL53LMZ_ResultsData results;
  uint8_t status = sensor_.get_ranging_data(&results);
  if (status != VL53LMZ_STATUS_OK) return false;

  out.ts_us = (uint64_t)esp_timer_get_time();

  const uint8_t tpz = (VL53LMZ_NB_TARGET_PER_ZONE <= kTofMaxTargetsPerZone)
                          ? (uint8_t)VL53LMZ_NB_TARGET_PER_ZONE
                          : (uint8_t)kTofMaxTargetsPerZone;

  out.cfg.side = settings_.side;
  out.cfg.targets_per_zone = tpz;
  out.cfg.ranging_hz = settings_.ranging_hz;
  out.cfg.integration_time_ms = settings_.integration_time_ms;
  out.cfg.reserved = 0;

  // Initialize with invalids
  memset(out.nb_targets, 0, sizeof(out.nb_targets));
  for (size_t i = 0; i < kTofZones * kTofMaxTargetsPerZone; i++) {
    out.distance_mm[i] = 0xFFFF;
    out.sigma_mm[i] = 0xFFFF;
    out.status[i] = 0;
  }

  for (uint8_t zone = 0; zone < kTofZones; zone++) {
    uint8_t n = results.nb_target_detected[zone];
    out.nb_targets[zone] = n;

    for (uint8_t t = 0; t < tpz; t++) {
      const size_t idx = (size_t)zone * tpz + t;
      const size_t out_idx = (size_t)zone * kTofMaxTargetsPerZone + t;

      if (t >= n) continue;

      // The VL53L8CH library flattens per-target arrays zone-major.
      out.distance_mm[out_idx] = results.distance_mm[idx];
      out.sigma_mm[out_idx] = results.range_sigma_mm[idx];
      out.status[out_idx] = results.target_status[idx];
    }
  }

  return true;
}

}  // namespace hub
