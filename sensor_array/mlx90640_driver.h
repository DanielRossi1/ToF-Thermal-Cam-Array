#pragma once

#include <stdint.h>

#include <Adafruit_MLX90640.h>

#include "hub_frame.h"
#include "i2c_bus.h"

namespace hub {

struct MlxSettings {
  uint8_t mode = MLX90640_CHESS;
  uint8_t resolution = MLX90640_ADC_18BIT;
  uint8_t refresh = MLX90640_16_HZ;
};

class Mlx90640Driver {
 public:
  explicit Mlx90640Driver(I2CBus& bus);

  bool begin();

  const MlxSettings& settings() const { return settings_; }
  bool applySettings(const MlxSettings& s);

  // Blocking read of one full temperature frame (°C) and Ta.
  bool readFrame(MlxDataV1& out);

 private:
  I2CBus& bus_;
  Adafruit_MLX90640 mlx_;
  MlxSettings settings_;

  static int16_t toCentiC(float c);
};

}  // namespace hub
