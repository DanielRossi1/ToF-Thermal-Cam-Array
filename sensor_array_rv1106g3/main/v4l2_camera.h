#pragma once

#include <stdint.h>
#include <pthread.h>
#include "hub_config.h"
#include "fast_aec.h"

#define V4L2_NB_BUFFERS  4

typedef struct {
    uint32_t w;
    uint32_t h;
    uint32_t interval_us;
    int32_t  exposure_abs;   // V4L2_CID_EXPOSURE_ABSOLUTE in 100 µs units
                             // used only when aperture-priority fails
                             // 166 = ~16 ms, 333 = ~33 ms, 0 = let driver pick
} UvcSettings;

typedef struct {
    int             fd;
    int             started;
    int             running;
    pthread_t       thread;
    pthread_mutex_t mutex;
    pthread_cond_t  cond;

    void           *mmap_bufs[V4L2_NB_BUFFERS];
    uint32_t        mmap_lens[V4L2_NB_BUFFERS];
    uint32_t        nbuffers;

    uint8_t        *frame_buf;
    uint32_t        frame_len;
    uint64_t        frame_ts_us;
    int             frame_ready;
    uint32_t        frame_gen;

    uint32_t        w, h;
    uint32_t        pixfmt;        // negotiated V4L2_PIX_FMT_*
    UvcSettings     settings;
    int32_t         aec_target;     // target brightness 0-255
    int32_t         aec_enabled;
    
    FastAecContext  aec_ctx;
    float           roi_x;
    float           roi_y;
    float           roi_w;
} V4L2Camera;

int  v4l2_camera_init(V4L2Camera *cam);
int  v4l2_camera_start(V4L2Camera *cam, uint32_t w, uint32_t h);
void v4l2_camera_stop(V4L2Camera *cam);
void v4l2_camera_deinit(V4L2Camera *cam);

int  v4l2_camera_is_ready(V4L2Camera *cam);
int  v4l2_camera_snapshot(V4L2Camera *cam, uint8_t *dst, uint32_t dst_cap,
                          uint32_t *out_len, uint64_t *out_ts_us);
int  v4l2_camera_snapshot_wait(V4L2Camera *cam, uint8_t *dst, uint32_t dst_cap,
                               uint32_t *out_len, uint64_t *out_ts_us,
                               uint64_t min_ts_us, uint32_t timeout_ms);
void v4l2_camera_get_settings(V4L2Camera *cam, UvcSettings *s);
int  v4l2_camera_apply_settings(V4L2Camera *cam, const UvcSettings *s);