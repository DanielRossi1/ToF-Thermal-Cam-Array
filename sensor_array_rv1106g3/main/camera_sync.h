#pragma once

#include <stdint.h>
#include "hub_frame.h"
#include "hub_config.h"

typedef struct {
    int enabled;
    uint32_t trigger_period_us;
    uint32_t trigger_pulse_us;
} CamSyncSettings;

typedef struct {
    CamSyncSettings settings;
#if USE_CAM_SYNC
    uint64_t next_trigger_ts_us;
    uint64_t last_trigger_ts_us;
    uint64_t last_edge_ts_us;
#endif
} CameraSync;

void cam_sync_init(CameraSync *cs);
void cam_sync_service(CameraSync *cs);
void cam_sync_fill(CameraSync *cs, CamSyncV1 *out);
void cam_sync_get_settings(CameraSync *cs, CamSyncSettings *s);
void cam_sync_apply_settings(CameraSync *cs, const CamSyncSettings *s);