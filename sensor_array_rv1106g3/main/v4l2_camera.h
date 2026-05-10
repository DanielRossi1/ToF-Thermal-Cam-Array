#pragma once

#include <stdint.h>
#include <pthread.h>
#include "hub_config.h"

#define V4L2_NB_BUFFERS  4

typedef struct {
    uint32_t w;
    uint32_t h;
    uint32_t interval_us;
} UvcSettings;

typedef struct {
    int             fd;
    int             started;
    int             running;
    pthread_t       thread;
    pthread_mutex_t mutex;

    void           *mmap_bufs[V4L2_NB_BUFFERS];
    uint32_t        mmap_lens[V4L2_NB_BUFFERS];
    uint32_t        nbuffers;

    uint8_t        *frame_buf;
    uint32_t        frame_len;
    uint64_t        frame_ts_us;
    int             frame_ready;

    uint32_t        w, h;
    UvcSettings     settings;
} V4L2Camera;

int  v4l2_camera_init(V4L2Camera *cam);
int  v4l2_camera_start(V4L2Camera *cam, uint32_t w, uint32_t h);
void v4l2_camera_stop(V4L2Camera *cam);
void v4l2_camera_deinit(V4L2Camera *cam);

int  v4l2_camera_is_ready(V4L2Camera *cam);
int  v4l2_camera_snapshot(V4L2Camera *cam, uint8_t *dst, uint32_t dst_cap,
                          uint32_t *out_len, uint64_t *out_ts_us);
void v4l2_camera_get_settings(V4L2Camera *cam, UvcSettings *s);
int  v4l2_camera_apply_settings(V4L2Camera *cam, const UvcSettings *s);