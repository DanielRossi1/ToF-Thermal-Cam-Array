#pragma once

#include <stdint.h>
#include "hub_config.h"

// Frame payload v1 (little-endian, packed).
// Sent as MSG_FRAME payload.

enum FrameFlags {
    FLAG_TOF_VALID      = 1u << 0,
    FLAG_MLX_VALID      = 1u << 1,
    FLAG_CAM_VALID      = 1u << 2,
    FLAG_CAM_SYNC_VALID = 1u << 3,
};

#pragma pack(push, 1)

typedef struct {
    uint8_t  side;
    uint8_t  targets_per_zone;
    uint16_t ranging_hz;
    uint16_t integration_time_ms;
    uint16_t reserved;
} TofConfigV1;

typedef struct {
    uint64_t    ts_us;
    TofConfigV1 cfg;
    uint8_t     silicon_temp_degc;
    uint8_t     _pad[3];
    uint8_t     nb_target_detected[TOF_ZONES];
    uint8_t     nb_spads_enabled[TOF_ZONES];
    int16_t     distance_mm[TOF_ZONES * TOF_MAX_TARGETS_PER_ZONE];
    uint16_t    range_sigma_mm[TOF_ZONES * TOF_MAX_TARGETS_PER_ZONE];
    uint8_t     target_status[TOF_ZONES * TOF_MAX_TARGETS_PER_ZONE];
    uint8_t     reflectance[TOF_ZONES * TOF_MAX_TARGETS_PER_ZONE];
    uint32_t    signal_per_spad[TOF_ZONES * TOF_MAX_TARGETS_PER_ZONE];
    uint32_t    ambient_per_spad[TOF_ZONES];
} TofDataV1;

typedef struct {
    uint16_t w;
    uint16_t h;
    uint8_t  mode;
    uint8_t  resolution;
    uint8_t  refresh;
    uint8_t  reserved;
} MlxConfigV1;

typedef struct {
    uint64_t    ts_us;
    MlxConfigV1 cfg;
    int16_t     ta_cC;
    int16_t     vdd_mV;
    int16_t     frame_cC[MLX_PIXELS];
} MlxDataV1;

typedef struct {
    uint32_t w;
    uint32_t h;
    uint32_t format_fourcc;
} CamConfigV1;

typedef struct {
    uint64_t    ts_us;
    CamConfigV1 cfg;
    uint32_t    len;
} CamDataV1;

typedef struct {
    uint64_t last_trigger_ts_us;
    uint64_t last_edge_ts_us;
    uint32_t trigger_period_us;
    uint32_t trigger_pulse_us;
} CamSyncV1;

typedef struct {
    uint32_t frame_seq;
    uint64_t hub_ts_us;
    uint32_t flags;
    uint32_t reserved;
    TofDataV1  tof;
    MlxDataV1  mlx;
    CamSyncV1  cam_sync;
    CamDataV1  cam;
} FrameFixedV1;

#pragma pack(pop)

#define FOURCC_MJPG  0x47504A4D  // 'MJPG' LE

typedef struct {
    FrameFixedV1 fixed;
    uint8_t      cam_bytes[CAM_JPEG_MAX];
} FrameBuffer;