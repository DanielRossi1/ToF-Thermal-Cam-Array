#pragma once

#include <stdint.h>

#include "hub_frame.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include "MLX90640_API.h"

// Refresh rate enum values from MLX90640_API.h
#define MLX90640_0_5_HZ  0
#define MLX90640_1_HZ    1
#define MLX90640_2_HZ    2
#define MLX90640_4_HZ    3
#define MLX90640_8_HZ    4
#define MLX90640_16_HZ   5
#define MLX90640_32_HZ   6
#define MLX90640_64_HZ   7

// ADC resolution codes
#define MLX90640_ADC_16BIT 0
#define MLX90640_ADC_17BIT 1
#define MLX90640_ADC_18BIT 2
#define MLX90640_ADC_19BIT 3

// Measurement mode codes
#define MLX90640_CHESS       1
#define MLX90640_INTERLEAVED 0

namespace hub {

struct MlxSettings {
  uint8_t mode       = MLX90640_CHESS;
  uint8_t resolution = MLX90640_ADC_18BIT;
  uint8_t refresh    = MLX90640_16_HZ;
};

class Mlx90640Driver {
 public:
  explicit Mlx90640Driver(I2CBus& bus);

  bool begin();

  const MlxSettings& settings() const { return settings_; }
  bool applySettings(const MlxSettings& s);

  bool readFrame(MlxDataV1& out);

 private:
  I2CBus&       bus_;
  paramsMLX90640 params_{};
  uint8_t        addr_ = 0x33;
  MlxSettings    settings_;

  static int16_t toCentiC(float c);
};

}  // namespace hub
