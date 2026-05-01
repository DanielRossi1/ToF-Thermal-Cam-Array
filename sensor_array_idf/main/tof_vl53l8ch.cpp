#include "tof_vl53l8ch.h"

#include <string.h>

#include "esp_timer.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "ToF";

// VL53L8CX I2C address (7-bit)
static constexpr uint8_t kVL53Addr7 = 0x29;
// Max bytes written per I2C transaction (hardware/DMA safe)
static constexpr size_t kI2cMaxChunk = 512;

namespace hub {

// ── ESP-IDF platform callbacks ────────────────────────────────────────────────

struct TofHandle {
  i2c_port_t port;
};

uint8_t TofVl53l8ch::idf_write(void* handle, uint16_t reg,
                                uint8_t* data, uint32_t size) {
  auto* h = static_cast<TofHandle*>(handle);
  uint32_t offset = 0;

  while (offset < size) {
    const uint32_t chunk = (size - offset < kI2cMaxChunk)
                               ? (size - offset)
                               : kI2cMaxChunk;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (kVL53Addr7 << 1) | I2C_MASTER_WRITE, true);
    uint16_t addr = reg + (uint16_t)offset;
    i2c_master_write_byte(cmd, (addr >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, addr & 0xFF, true);
    i2c_master_write(cmd, data + offset, chunk, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(h->port, cmd, pdMS_TO_TICKS(500));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) return 1;
    offset += chunk;
  }
  return 0;
}

uint8_t TofVl53l8ch::idf_read(void* handle, uint16_t reg,
                               uint8_t* data, uint32_t size) {
  auto* h = static_cast<TofHandle*>(handle);
  uint32_t offset = 0;

  while (offset < size) {
    const uint32_t chunk = (size - offset < kI2cMaxChunk)
                               ? (size - offset)
                               : kI2cMaxChunk;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (kVL53Addr7 << 1) | I2C_MASTER_WRITE, true);
    uint16_t addr = reg + (uint16_t)offset;
    i2c_master_write_byte(cmd, (addr >> 8) & 0xFF, true);
    i2c_master_write_byte(cmd, addr & 0xFF, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (kVL53Addr7 << 1) | I2C_MASTER_READ, true);
    if (chunk > 1) {
      i2c_master_read(cmd, data + offset, chunk - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + offset + chunk - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(h->port, cmd, pdMS_TO_TICKS(500));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) return 1;
    offset += chunk;
  }
  return 0;
}

uint8_t TofVl53l8ch::idf_wait(void* /*handle*/, uint32_t ms) {
  vTaskDelay(pdMS_TO_TICKS(ms ? ms : 1));
  return 0;
}

// ── TofVl53l8ch ──────────────────────────────────────────────────────────────

TofVl53l8ch::TofVl53l8ch(I2CBus& bus, int lpn_pin)
    : bus_(bus), lpn_pin_(lpn_pin) {
  memset(&dev_, 0, sizeof(dev_));
}

bool TofVl53l8ch::begin() {
  // Optional: toggle LPn to force a hardware reset.
  if (lpn_pin_ >= 0) {
    gpio_set_direction((gpio_num_t)lpn_pin_, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)lpn_pin_, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level((gpio_num_t)lpn_pin_, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // Allocate a static handle (lives as long as the object).
  static TofHandle s_handle;
  s_handle.port = bus_.port();

  // Wire up platform function pointers.
  dev_.platform.address = kVL53Addr7;
  dev_.platform.Write   = idf_write;
  dev_.platform.Read    = idf_read;
  dev_.platform.Wait    = idf_wait;
  dev_.platform.handle  = &s_handle;

  {
    I2CLock lk(bus_, pdMS_TO_TICKS(5000));
    if (!lk.ok()) { ESP_LOGE(TAG, "I2C lock timeout"); return false; }

    uint8_t status = vl53l8cx_init(&dev_);
    if (status != VL53L8CX_STATUS_OK) {
      ESP_LOGE(TAG, "vl53l8cx_init failed: %u", (unsigned)status);
      return false;
    }
  }

  return applySettings(settings_);
}

bool TofVl53l8ch::startRangingLocked() {
  if (ranging_started_) return true;
  uint8_t status = vl53l8cx_start_ranging(&dev_);
  if (status != VL53L8CX_STATUS_OK) {
    ESP_LOGE(TAG, "start_ranging failed: %u", (unsigned)status);
    return false;
  }
  ranging_started_ = true;
  return true;
}

bool TofVl53l8ch::applySettings(const TofSettings& s) {
  TofSettings next = s;
  if (next.side != 4 && next.side != 8) next.side = 8;
  if (next.ranging_hz == 0)             next.ranging_hz = 15;
  if (next.integration_time_ms == 0)   next.integration_time_ms = 50;

  I2CLock lk(bus_, pdMS_TO_TICKS(500));
  if (!lk.ok()) return false;

  if (ranging_started_) {
    vl53l8cx_stop_ranging(&dev_);
    ranging_started_ = false;
  }

  uint8_t res = (next.side == 4) ? VL53L8CX_RESOLUTION_4X4
                                  : VL53L8CX_RESOLUTION_8X8;
  uint8_t st = vl53l8cx_set_resolution(&dev_, res);
  if (st != VL53L8CX_STATUS_OK)
    ESP_LOGW(TAG, "set_resolution: %u", (unsigned)st);

  st = vl53l8cx_set_ranging_frequency_hz(&dev_, (uint8_t)next.ranging_hz);
  if (st != VL53L8CX_STATUS_OK)
    ESP_LOGW(TAG, "set_ranging_frequency_hz: %u", (unsigned)st);

  uint8_t mode = next.continuous ? VL53L8CX_RANGING_MODE_CONTINUOUS
                                 : VL53L8CX_RANGING_MODE_AUTONOMOUS;
  st = vl53l8cx_set_ranging_mode(&dev_, mode);
  if (st != VL53L8CX_STATUS_OK)
    ESP_LOGW(TAG, "set_ranging_mode: %u", (unsigned)st);

  st = vl53l8cx_set_integration_time_ms(&dev_, next.integration_time_ms);
  if (st != VL53L8CX_STATUS_OK)
    ESP_LOGW(TAG, "set_integration_time_ms: %u", (unsigned)st);

  if (!startRangingLocked()) return false;

  settings_ = next;
  return true;
}

bool TofVl53l8ch::read(TofDataV1& out) {
  I2CLock lk(bus_, pdMS_TO_TICKS(80));
  if (!lk.ok()) return false;

  uint8_t ready = 0;
  uint8_t st = vl53l8cx_check_data_ready(&dev_, &ready);
  if (st != VL53L8CX_STATUS_OK || !ready) return false;

  VL53L8CX_ResultsData res;
  st = vl53l8cx_get_ranging_data(&dev_, &res);
  if (st != VL53L8CX_STATUS_OK) return false;

  out.ts_us = (uint64_t)esp_timer_get_time();
  out.cfg.side               = settings_.side;
  out.cfg.targets_per_zone   = (uint8_t)VL53L8CX_NB_TARGET_PER_ZONE;
  out.cfg.ranging_hz         = settings_.ranging_hz;
  out.cfg.integration_time_ms = settings_.integration_time_ms;
  out.cfg.reserved           = 0;
  out.silicon_temp_degc      = res.silicon_temp_degc;
  out._pad[0] = out._pad[1] = out._pad[2] = 0;

  const uint8_t tpz = (uint8_t)VL53L8CX_NB_TARGET_PER_ZONE;
  const uint8_t zones = (uint8_t)kTofZones;  // 64

  for (uint8_t z = 0; z < zones; z++) {
    out.nb_target_detected[z] = res.nb_target_detected[z];
    out.nb_spads_enabled[z]   = (uint8_t)res.nb_spads_enabled[z];
    out.ambient_per_spad[z]   = res.ambient_per_spad[z];

    for (uint8_t t = 0; t < tpz && t < (uint8_t)kTofMaxTargetsPerZone; t++) {
      const size_t src = (size_t)z * tpz + t;
      const size_t dst = (size_t)z * kTofMaxTargetsPerZone + t;
      out.distance_mm[dst]    = res.distance_mm[src];
      out.range_sigma_mm[dst] = res.range_sigma_mm[src];
      out.target_status[dst]  = res.target_status[src];
      out.reflectance[dst]    = res.reflectance[src];
      out.signal_per_spad[dst]= res.signal_per_spad[src];
    }
    // Zero-fill unused target slots
    for (uint8_t t = tpz; t < (uint8_t)kTofMaxTargetsPerZone; t++) {
      const size_t dst = (size_t)z * kTofMaxTargetsPerZone + t;
      out.distance_mm[dst]     = 0;
      out.range_sigma_mm[dst]  = 0;
      out.target_status[dst]   = 0;
      out.reflectance[dst]     = 0;
      out.signal_per_spad[dst] = 0;
    }
  }
  return true;
}

}  // namespace hub
