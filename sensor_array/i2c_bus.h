#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "hub_config.h"

namespace hub {

class I2CBus {
 public:
  I2CBus();

  void begin();
  TwoWire& wire();

  // Locking for shared I2C use across tasks.
  bool lock(TickType_t ticks);
  void unlock();

 private:
  SemaphoreHandle_t mutex_;
};

class I2CLock {
 public:
  I2CLock(I2CBus& bus, TickType_t ticks) : bus_(bus), locked_(bus_.lock(ticks)) {}
  ~I2CLock() {
    if (locked_) bus_.unlock();
  }
  bool ok() const { return locked_; }

 private:
  I2CBus& bus_;
  bool locked_;
};

}  // namespace hub
