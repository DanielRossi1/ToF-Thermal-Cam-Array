#include "camera_sync.h"
#include <string.h>
#include <unistd.h>
#include <time.h>

void cam_sync_init(CameraSync *cs) {
    memset(cs, 0, sizeof(*cs));
}

void cam_sync_service(CameraSync *cs) {
#if USE_CAM_SYNC
    if (!cs->settings.enabled || cs->settings.trigger_period_us == 0) return;

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;

    if (cs->next_trigger_ts_us == 0)
        cs->next_trigger_ts_us = now_us;
    if (now_us < cs->next_trigger_ts_us)
        return;

    cs->last_trigger_ts_us = now_us;
    usleep(cs->settings.trigger_pulse_us);
    cs->next_trigger_ts_us += cs->settings.trigger_period_us;
    if (cs->next_trigger_ts_us + cs->settings.trigger_period_us < now_us)
        cs->next_trigger_ts_us = now_us + cs->settings.trigger_period_us;
#else
    (void)cs;
#endif
}

void cam_sync_fill(CameraSync *cs, CamSyncV1 *out) {
#if USE_CAM_SYNC
    out->last_trigger_ts_us = cs->last_trigger_ts_us;
    out->last_edge_ts_us    = cs->last_edge_ts_us;
    out->trigger_period_us   = cs->settings.trigger_period_us;
    out->trigger_pulse_us    = cs->settings.trigger_pulse_us;
#else
    (void)cs;
    memset(out, 0, sizeof(*out));
#endif
}

void cam_sync_get_settings(CameraSync *cs, CamSyncSettings *s) {
    *s = cs->settings;
}

void cam_sync_apply_settings(CameraSync *cs, const CamSyncSettings *s) {
    cs->settings = *s;
#if USE_CAM_SYNC
    cs->next_trigger_ts_us = 0;
#else
    (void)cs; (void)s;
#endif
}