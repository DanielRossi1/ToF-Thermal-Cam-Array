#pragma once

#include <stdint.h>

#include <vl53l8ch.h>

#include "hub_frame.h"
#include "i2c_bus.h"

namespace hub {

struct TofSettings {
  uint8_t side = 8;              // 4 or 8
  uint16_t ranging_hz = 15;
  uint16_t integration_time_ms = 50;
  bool continuous = true;
};

class TofVl53l8ch {
 public:
  // If your breakout ties LPn HIGH internally, pass -1.
  explicit TofVl53l8ch(I2CBus& bus, int lpn_pin = -1);

  bool begin();

  const TofSettings& settings() const { return settings_; }
  bool applySettings(const TofSettings& s);

  // Returns true if new data was read.
  bool read(TofDataV1& out);

 private:
  I2CBus& bus_;
  VL53L8CH sensor_;
  TofSettings settings_;
  bool ranging_started_ = false;

  bool startRangingLocked();
};

}  // namespace hub
