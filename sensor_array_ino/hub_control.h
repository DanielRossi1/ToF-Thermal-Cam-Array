#pragma once

#include <stddef.h>
#include <stdint.h>

#include "hub_protocol.h"
#include "hub_transport.h"

#include "camera_sync.h"
#include "i2c_bus.h"
#include "mlx90640_driver.h"
#include "tof_vl53l8ch.h"
#include "uvc_camera.h"

namespace hub {

struct ControlContext {
  Transport* tx = nullptr;
  I2CBus* i2c = nullptr;
  TofVl53l8ch* tof = nullptr;
  Mlx90640Driver* mlx = nullptr;
  CameraSync* cam_sync = nullptr;

#if HUB_USE_UVC_CAMERA
  UvcCamera* cam = nullptr;
  bool* cam_started = nullptr;
#endif

  volatile bool* stream_enabled = nullptr;
};

// Parses a full decoded SLIP frame.
// If it is a valid Cmd message, applies it and emits a Resp message.
void handleSlipFrame(const uint8_t* data, size_t len, void* user);

}  // namespace hub
