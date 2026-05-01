#pragma once

#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#include "hub_config.h"

namespace hub {

class I2CBus {
 public:
  I2CBus();
  void begin();
  i2c_port_t port() const { return I2C_NUM_0; }

  // Mutex for shared-bus arbitration.
  bool lock(TickType_t ticks);
  void unlock();

  // --- Raw I2C operations (caller must hold lock if needed) ---

  // Write to a 16-bit register address (VL53L8CX style).
  esp_err_t write_reg16(uint8_t dev_addr7, uint16_t reg,
                        const uint8_t* data, size_t len);

  // Read from a 16-bit register address.
  esp_err_t read_reg16(uint8_t dev_addr7, uint16_t reg,
                       uint8_t* data, size_t len);

  // Write to an 8-bit register address (MLX90640 uses 16-bit addr but same fn works).
  // reg is sent big-endian as two bytes.
  esp_err_t write_reg16_word(uint8_t dev_addr7, uint16_t reg, uint16_t word);

  // Read n 16-bit words from 16-bit register address, returned in host order.
  esp_err_t read_reg16_words(uint8_t dev_addr7, uint16_t reg,
                              uint16_t* data, size_t word_count);

  // Probe: returns true if device ACKs.
  bool probe(uint8_t dev_addr7);

 private:
  SemaphoreHandle_t mutex_;
};

// RAII lock wrapper.
class I2CLock {
 public:
  I2CLock(I2CBus& bus, TickType_t ticks)
      : bus_(bus), locked_(bus_.lock(ticks)) {}
  ~I2CLock() {
    if (locked_) bus_.unlock();
  }
  bool ok() const { return locked_; }

 private:
  I2CBus& bus_;
  bool locked_;
};

}  // namespace hub
