#include "i2c_bus.h"

#include <string.h>

static const char* TAG = "I2CBus";

namespace hub {

I2CBus::I2CBus() : mutex_(nullptr) {}

void I2CBus::begin() {
  if (!mutex_) {
    mutex_ = xSemaphoreCreateMutex();
  }
  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = kPinSda;
  conf.scl_io_num = kPinScl;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = kI2cClockHz;
  i2c_param_config(I2C_NUM_0, &conf);
  // In master mode, rx/tx buffer sizes are unused (pass 0).
  i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

bool I2CBus::lock(TickType_t ticks) {
  if (!mutex_) return false;
  return xSemaphoreTake(mutex_, ticks) == pdTRUE;
}

void I2CBus::unlock() {
  if (mutex_) xSemaphoreGive(mutex_);
}

bool I2CBus::probe(uint8_t dev_addr7) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_addr7 << 1) | I2C_MASTER_WRITE, true);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(10));
  i2c_cmd_link_delete(cmd);
  return ret == ESP_OK;
}

esp_err_t I2CBus::write_reg16(uint8_t dev_addr7, uint16_t reg,
                               const uint8_t* data, size_t len) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_addr7 << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, true);
  i2c_master_write_byte(cmd, reg & 0xFF, true);
  if (data && len > 0) {
    i2c_master_write(cmd, data, len, true);
  }
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(200));
  i2c_cmd_link_delete(cmd);
  return ret;
}

esp_err_t I2CBus::read_reg16(uint8_t dev_addr7, uint16_t reg,
                              uint8_t* data, size_t len) {
  if (!data || len == 0) return ESP_ERR_INVALID_ARG;

  // Write register address
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_addr7 << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, (reg >> 8) & 0xFF, true);
  i2c_master_write_byte(cmd, reg & 0xFF, true);
  // Repeated start + read
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (dev_addr7 << 1) | I2C_MASTER_READ, true);
  if (len > 1) {
    i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
  }
  i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(500));
  i2c_cmd_link_delete(cmd);
  return ret;
}

esp_err_t I2CBus::write_reg16_word(uint8_t dev_addr7, uint16_t reg,
                                    uint16_t word) {
  uint8_t buf[2] = {(uint8_t)(word >> 8), (uint8_t)(word & 0xFF)};
  return write_reg16(dev_addr7, reg, buf, 2);
}

esp_err_t I2CBus::read_reg16_words(uint8_t dev_addr7, uint16_t reg,
                                    uint16_t* data, size_t word_count) {
  if (!data || word_count == 0) return ESP_ERR_INVALID_ARG;
  const size_t byte_count = word_count * 2;
  // Read raw bytes (big-endian from sensor)
  esp_err_t ret = read_reg16(dev_addr7, reg,
                              reinterpret_cast<uint8_t*>(data), byte_count);
  if (ret != ESP_OK) return ret;
  // Swap bytes: sensor sends big-endian, convert to host (little-endian)
  for (size_t i = 0; i < word_count; i++) {
    uint8_t* p = reinterpret_cast<uint8_t*>(&data[i]);
    uint8_t tmp = p[0];
    p[0] = p[1];
    p[1] = tmp;
  }
  return ESP_OK;
}

}  // namespace hub
