#pragma once

#include <stdint.h>

typedef enum {
    STREAM_MODE_ALL      = 0,
    STREAM_MODE_TOF_ONLY = 1,
    STREAM_MODE_MLX_ONLY = 2,
    STREAM_MODE_CAM_ONLY = 3,
    STREAM_MODE_NONE     = 4,
} StreamMode;