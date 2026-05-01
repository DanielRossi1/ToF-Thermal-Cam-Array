#pragma once

#include <stdint.h>

#include "hub_frame.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include "driver/gpio.h"

// ST VL53L8CX ULD
#include "vl53l8cx_api.h"

namespace hub {

struct TofSettings {
  uint8_t  side               = 8;   // 4 or 8
  uint16_t ranging_hz         = 15;
  uint16_t integration_time_ms = 50;
  bool     continuous         = true;
};

class TofVl53l8ch {
 public:
  explicit TofVl53l8ch(I2CBus& bus, int lpn_pin = -1);

  bool begin();

  const TofSettings& settings() const { return settings_; }
  bool applySettings(const TofSettings& s);

  // Read one result frame.  Returns true on success.
  bool read(TofDataV1& out);

 private:
  I2CBus&              bus_;
  int                  lpn_pin_;
  VL53L8CX_Configuration dev_;
  TofSettings          settings_;
  bool                 ranging_started_ = false;

  // ESP-IDF I2C callbacks installed into dev_.platform
  static uint8_t idf_write(void* handle, uint16_t reg,
                            uint8_t* data, uint32_t size);
  static uint8_t idf_read (void* handle, uint16_t reg,
                            uint8_t* data, uint32_t size);
  static uint8_t idf_wait (void* handle, uint32_t ms);

  bool startRangingLocked();
};

}  // namespace hub
