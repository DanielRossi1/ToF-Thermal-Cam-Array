#include "v4l2_camera.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

static void *capture_thread(void *arg) {
    V4L2Camera *cam = (V4L2Camera *)arg;

    while (cam->running) {
        if (cam->fd < 0) { usleep(10000); continue; }

        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(cam->fd, VIDIOC_DQBUF, &buf) < 0) {
            usleep(5000);
            continue;
        }

        if (buf.index < cam->nbuffers && cam->mmap_bufs[buf.index]) {
            pthread_mutex_lock(&cam->mutex);

            uint32_t n = buf.bytesused;
            if (n > CAM_JPEG_MAX) n = CAM_JPEG_MAX;

            memcpy(cam->frame_buf, cam->mmap_bufs[buf.index], n);
            cam->frame_len = n;

            // Use monotonic clock for consistent timestamps aligned with hub's now_us()
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            cam->frame_ts_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
            cam->frame_ready = 1;
            pthread_mutex_unlock(&cam->mutex);
        }

        ioctl(cam->fd, VIDIOC_QBUF, &buf);
    }
    return NULL;
}

int v4l2_camera_init(V4L2Camera *cam) {
    memset(cam, 0, sizeof(*cam));
    cam->fd = -1;
    pthread_mutex_init(&cam->mutex, NULL);
    cam->settings.w = 320;
    cam->settings.h = 240;
    cam->settings.interval_us = 83333;

    cam->frame_buf = (uint8_t *)malloc(CAM_JPEG_MAX);
    if (!cam->frame_buf) return -1;
    return 0;
}

int v4l2_camera_start(V4L2Camera *cam, uint32_t w, uint32_t h) {
    if (cam->started) return 0;

    int fd = open("/dev/video0", O_RDWR);
    if (fd < 0) { perror("v4l2 open"); return -1; }
    cam->fd = fd;

    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) { perror("VIDIOC_QUERYCAP"); goto fail; }
        fprintf(stderr, "[V4L2] device: driver=%s card=%s bus=%s version=%u\n",
                cap.driver, cap.card, cap.bus_info, cap.version);

        // Enumerate available formats for diagnostics
        struct v4l2_fmtdesc fmtdesc;
        memset(&fmtdesc, 0, sizeof(fmtdesc));
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fprintf(stderr, "[V4L2] supported formats:\n");
        for (fmtdesc.index = 0; ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0; fmtdesc.index++) {
            char fourcc[5] = {0,0,0,0,0};
            memcpy(fourcc, &fmtdesc.pixelformat, 4);
            fprintf(stderr, "  - %u: %s (%s)\n", fmtdesc.index, fourcc, fmtdesc.description);
        }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = w;
    fmt.fmt.pix.height      = h;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    // Prefer progressive / global-shutter style capture when available
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("VIDIOC_S_FMT MJPEG");
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) { perror("VIDIOC_S_FMT YUYV"); goto fail; }
    }
    cam->w = fmt.fmt.pix.width;
    cam->h = fmt.fmt.pix.height;

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count  = V4L2_NB_BUFFERS;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) { perror("VIDIOC_REQBUFS"); goto fail; }
    cam->nbuffers = req.count;

            // Log negotiated format
            char negotiated[5] = {0,0,0,0,0};
            memcpy(negotiated, &fmt.fmt.pix.pixelformat, 4);
            fprintf(stderr, "[V4L2] negotiated: %ux%u fourcc=%s field=%d\n",
                    fmt.fmt.pix.width, fmt.fmt.pix.height, negotiated, fmt.fmt.pix.field);
    for (uint32_t i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) goto fail;

        cam->mmap_bufs[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                 MAP_SHARED, fd, buf.m.offset);
        if (cam->mmap_bufs[i] == MAP_FAILED) goto fail;
        cam->mmap_lens[i] = buf.length;
    }

    for (uint32_t i = 0; i < req.count; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) goto fail;
    }

    for (uint32_t bi = 0; bi < cam->nbuffers; bi++) {
        fprintf(stderr, "[V4L2] buffer %u length=%u\n", bi, (unsigned)cam->mmap_lens[bi]);
    }
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) { perror("VIDIOC_STREAMON"); goto fail; }

    cam->running = 1;
    pthread_create(&cam->thread, NULL, capture_thread, cam);

    cam->started = 1;
    return 0;

fail:
    close(fd);
    cam->fd = -1;
    return -1;
}

void v4l2_camera_stop(V4L2Camera *cam) {
    if (!cam->started) return;

    cam->running = 0;
    pthread_join(cam->thread, NULL);

    if (cam->fd >= 0) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(cam->fd, VIDIOC_STREAMOFF, &type);

        for (uint32_t i = 0; i < cam->nbuffers; i++) {
            if (cam->mmap_bufs[i] && cam->mmap_bufs[i] != MAP_FAILED)
                munmap(cam->mmap_bufs[i], cam->mmap_lens[i]);
            cam->mmap_bufs[i] = NULL;
        }
        cam->nbuffers = 0;

        close(cam->fd);
        cam->fd = -1;
    }
    cam->started = 0;
    cam->frame_ready = 0;
}

void v4l2_camera_deinit(V4L2Camera *cam) {
    v4l2_camera_stop(cam);
    free(cam->frame_buf);
    cam->frame_buf = NULL;
    pthread_mutex_destroy(&cam->mutex);
}

int v4l2_camera_is_ready(V4L2Camera *cam) {
    return cam->started && cam->frame_ready;
}

int v4l2_camera_snapshot(V4L2Camera *cam, uint8_t *dst, uint32_t dst_cap,
                         uint32_t *out_len, uint64_t *out_ts_us) {
    *out_len = 0;
    *out_ts_us = 0;
    if (!cam->started || !dst || !dst_cap) return 0;
                    static uint32_t frame_dbg_count = 0;
                    if ((frame_dbg_count++ & 0x1F) == 0) {
                        fprintf(stderr, "[V4L2] frame dbg=%u bytes=%u ts_us=%llu\n",
                                frame_dbg_count, (unsigned)cam->frame_len, (unsigned long long)cam->frame_ts_us);
                    }

    pthread_mutex_lock(&cam->mutex);
    if (!cam->frame_ready || cam->frame_len == 0) {
        pthread_mutex_unlock(&cam->mutex);
        return 0;
    }

    uint32_t n = cam->frame_len;
    if (n > dst_cap) n = dst_cap;
    memcpy(dst, cam->frame_buf, n);
    *out_len    = n;
    *out_ts_us  = cam->frame_ts_us;
    cam->frame_ready = 0;  // consumed
    pthread_mutex_unlock(&cam->mutex);
    return 1;
}

void v4l2_camera_get_settings(V4L2Camera *cam, UvcSettings *s) {
    *s = cam->settings;
}

int v4l2_camera_apply_settings(V4L2Camera *cam, const UvcSettings *s) {
    cam->settings = *s;
    if (!cam->started) return 0;
    v4l2_camera_stop(cam);
    usleep(100000);
    return v4l2_camera_start(cam, s->w, s->h);
}