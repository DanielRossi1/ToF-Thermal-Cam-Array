#include "i2c_bus.h"

namespace hub {

I2CBus::I2CBus() : mutex_(nullptr) {}

void I2CBus::begin() {
  if (!mutex_) {
    mutex_ = xSemaphoreCreateMutex();
  }
  Wire.begin(kPinSda, kPinScl);
  Wire.setClock(kI2cClockHz);
  Wire.setBufferSize(kI2cBufSize);
}

TwoWire& I2CBus::wire() { return Wire; }

bool I2CBus::lock(TickType_t ticks) {
  if (!mutex_) return false;
  return xSemaphoreTake(mutex_, ticks) == pdTRUE;
}

void I2CBus::unlock() {
  if (!mutex_) return;
  xSemaphoreGive(mutex_);
}

}  // namespace hub
