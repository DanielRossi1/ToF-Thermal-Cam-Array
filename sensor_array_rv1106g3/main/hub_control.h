#pragma once

#include <stdint.h>
#include <stddef.h>
#include "hub_protocol.h"
#include "hub_runtime.h"
#include "hub_transport.h"
#include "i2c_bus.h"
#include "tof_vl53l8ch.h"
#include "mlx90640_driver.h"
#include "camera_sync.h"
#if USE_UVC_CAMERA
#include "v4l2_camera.h"
#endif

typedef struct {
    Transport       *tx;
    I2CBus          *i2c;
    TofVl53l8ch     *tof;
    Mlx90640Driver  *mlx;
    CameraSync      *cam_sync;
#if USE_UVC_CAMERA
    V4L2Camera      *cam;
    int             *cam_started;
#endif
    volatile int    *stream_enabled;
    volatile StreamMode  *mode;
} ControlContext;

void hub_control_init(void);
void hub_control_set_context(ControlContext *ctx);
void hub_handle_slip_frame(const uint8_t *data, size_t len, void *user);