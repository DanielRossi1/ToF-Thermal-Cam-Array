#pragma once
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

// Call once before using any MLX90640 driver functions.
void MLX90640_IDF_SetPort(i2c_port_t port);

#ifdef __cplusplus
}
#endif
