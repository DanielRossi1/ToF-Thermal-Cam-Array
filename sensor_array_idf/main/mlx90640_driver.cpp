#include "mlx90640_driver.h"

#include <initializer_list>
#include <math.h>
#include <string.h>

#include "esp_timer.h"
#include "esp_log.h"
#include "MLX90640_I2C_Driver.h"
#include "mlx90640_idf_hal.h"

static const char* TAG = "MLX";

namespace hub {

int16_t Mlx90640Driver::toCentiC(float c) {
  float v = c * 100.0f;
  if (v > 32767.0f)  v = 32767.0f;
  if (v < -32768.0f) v = -32768.0f;
  return (int16_t)lrintf(v);
}

Mlx90640Driver::Mlx90640Driver(I2CBus& bus) : bus_(bus) {}

bool Mlx90640Driver::begin() {
  // Point the Melexis HAL at our I2C port.
  MLX90640_IDF_SetPort(bus_.port());

  uint16_t eeData[MLX90640_EEPROM_DUMP_NUM];
  bool found = false;

  {
    I2CLock lk(bus_, pdMS_TO_TICKS(500));
    if (!lk.ok()) return false;

    // Try default address then alternate.
    for (int attempt = 0; attempt < 4 && !found; attempt++) {
      for (uint8_t a : {(uint8_t)0x33, (uint8_t)0x32}) {
        if (MLX90640_DumpEE(a, eeData) == MLX90640_NO_ERROR) {
          addr_ = a;
          found = true;
          break;
        }
      }
      if (!found) vTaskDelay(pdMS_TO_TICKS(40));
    }
  }

  if (!found) {
    ESP_LOGE(TAG, "MLX90640 not found at 0x33 / 0x32");
    return false;
  }

  {
    I2CLock lk(bus_, pdMS_TO_TICKS(250));
    if (!lk.ok()) return false;
    int rc = MLX90640_ExtractParameters(eeData, &params_);
    if (rc != MLX90640_NO_ERROR) {
      ESP_LOGE(TAG, "ExtractParameters failed: %d", rc);
      return false;
    }
  }

  return applySettings(settings_);
}

bool Mlx90640Driver::applySettings(const MlxSettings& s) {
  I2CLock lk(bus_, pdMS_TO_TICKS(250));
  if (!lk.ok()) return false;

  if (s.mode == MLX90640_CHESS)
    MLX90640_SetChessMode(addr_);
  else
    MLX90640_SetInterleavedMode(addr_);

  MLX90640_SetResolution(addr_, s.resolution);
  MLX90640_SetRefreshRate(addr_, s.refresh);

  settings_ = s;
  return true;
}

bool Mlx90640Driver::readFrame(MlxDataV1& out) {
  I2CLock lk(bus_, pdMS_TO_TICKS(250));
  if (!lk.ok()) return false;

  // Read two sub-pages and accumulate into the float buffer.
  float frame[kMlxPixels];
  uint16_t frameData[834];

  for (uint8_t page = 0; page < 2; page++) {
    int rc = MLX90640_GetFrameData(addr_, frameData);
    if (rc < 0) return false;

    const float ta = MLX90640_GetTa(frameData, &params_);
    const float tr = ta - 8.0f;  // typical open-air offset
    MLX90640_CalculateTo(frameData, &params_, 0.95f, tr, frame);
  }

  const float ta = MLX90640_GetTa(frameData, &params_);

  out.ts_us          = (uint64_t)esp_timer_get_time();
  out.cfg.w          = kMlxW;
  out.cfg.h          = kMlxH;
  out.cfg.mode       = settings_.mode;
  out.cfg.resolution = settings_.resolution;
  out.cfg.refresh    = settings_.refresh;
  out.cfg.reserved   = 0;
  out.ta_cC          = toCentiC(ta);
  out.vdd_mV         = 0;

  for (uint16_t i = 0; i < kMlxPixels; i++) {
    out.frame_cC[i] = toCentiC(frame[i]);
  }
  return true;
}

}  // namespace hub
