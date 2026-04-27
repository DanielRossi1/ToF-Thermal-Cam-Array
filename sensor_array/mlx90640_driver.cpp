#include "mlx90640_driver.h"

#include <math.h>
#include <string.h>

#include <Arduino.h>

#include "esp_timer.h"

namespace hub {

Mlx90640Driver::Mlx90640Driver(I2CBus& bus) : bus_(bus) {}

int16_t Mlx90640Driver::toCentiC(float c) {
  // Clamp to int16 range for safety.
  float v = c * 100.0f;
  if (v > 32767.0f) v = 32767.0f;
  if (v < -32768.0f) v = -32768.0f;
  return (int16_t)lrintf(v);
}

bool Mlx90640Driver::begin() {
  {
    I2CLock lk(bus_, pdMS_TO_TICKS(250));
    if (!lk.ok()) return false;

    // Try both common addresses (0x33 default, 0x32 alt).
    bool ok = false;
    for (int attempt = 0; attempt < 4 && !ok; attempt++) {
      ok = mlx_.begin(0x33, &bus_.wire());
      if (!ok) ok = mlx_.begin(0x32, &bus_.wire());
      if (!ok) delay(40);
    }
    if (!ok) {
      Serial.println("[MLX] Not found at 0x33/0x32");
      return false;
    }
  }

  return applySettings(settings_);
}

bool Mlx90640Driver::applySettings(const MlxSettings& s) {
  I2CLock lk(bus_, pdMS_TO_TICKS(250));
  if (!lk.ok()) return false;

  mlx_.setMode((mlx90640_mode_t)s.mode);
  mlx_.setResolution((mlx90640_resolution_t)s.resolution);
  mlx_.setRefreshRate((mlx90640_refreshrate_t)s.refresh);

  settings_ = s;
  return true;
}

bool Mlx90640Driver::readFrame(MlxDataV1& out) {
  // getFrame() can block until the needed subpage is ready.
  I2CLock lk(bus_, pdMS_TO_TICKS(150));
  if (!lk.ok()) return false;

  float frame[kMlxPixels];
  int rc = mlx_.getFrame(frame);
  if (rc != 0) return false;

  const float ta = mlx_.getTa(false);

  out.ts_us = (uint64_t)esp_timer_get_time();
  out.cfg.w = kMlxW;
  out.cfg.h = kMlxH;
  out.cfg.mode = settings_.mode;
  out.cfg.resolution = settings_.resolution;
  out.cfg.refresh = settings_.refresh;
  out.cfg.reserved = 0;

  out.ta_cC = toCentiC(ta);
  out.vdd_mV = 0;  // Adafruit library does not expose Vdd reliably on all versions.

  for (uint16_t i = 0; i < kMlxPixels; i++) {
    out.frame_cC[i] = toCentiC(frame[i]);
  }
  return true;
}

}  // namespace hub
