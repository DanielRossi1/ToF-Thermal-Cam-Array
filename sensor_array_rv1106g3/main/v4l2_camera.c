#include "v4l2_camera.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

static int v4l2_ctrl_supported(int fd, uint32_t id) {
    struct v4l2_queryctrl q;
    memset(&q, 0, sizeof(q));
    q.id = id;
    if (ioctl(fd, VIDIOC_QUERYCTRL, &q) < 0) return 0;
    if (q.flags & V4L2_CTRL_FLAG_DISABLED) return 0;
    return 1;
}

static int v4l2_try_set_ctrl(int fd, uint32_t id, int32_t value, const char *name) {
    struct v4l2_control c;
    memset(&c, 0, sizeof(c));
    c.id = id;
    c.value = value;
    if (ioctl(fd, VIDIOC_S_CTRL, &c) < 0) {
        if (name) perror(name);
        return -1;
    }
    return 0;
}

static struct timespec mono_deadline_ms(uint32_t timeout_ms) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    ts.tv_nsec += (long)timeout_ms * 1000000L;
    while (ts.tv_nsec >= 1000000000L) {
        ts.tv_nsec -= 1000000000L;
        ts.tv_sec += 1;
    }
    return ts;
}

static void *capture_thread(void *arg) {
    V4L2Camera *cam = (V4L2Camera *)arg;

    while (cam->running) {
        if (cam->fd < 0) { usleep(1000); continue; }

        struct pollfd pfd = { .fd = cam->fd, .events = POLLIN };
        int pr = poll(&pfd, 1, 100);
        if (pr <= 0) continue;
        if (!(pfd.revents & POLLIN)) continue;

        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(cam->fd, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) continue;
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
            cam->frame_gen++;

            // Only run AEC on MJPEG, and only every 4th frame (30Hz calculation for a 120Hz stream)
            if (cam->aec_enabled && cam->pixfmt == V4L2_PIX_FMT_MJPEG && (cam->frame_gen % 4 == 0)) {
                
                // Grab thread-safe copies of ROI
                float rx = cam->roi_x;
                float ry = cam->roi_y;
                float rw = cam->roi_w;

                int32_t new_exp = fast_aec_process_frame(&cam->aec_ctx, cam->frame_buf, cam->frame_len, rx, ry, rw);

                if (new_exp != cam->settings.exposure_abs) {
                    fprintf(stderr, "[AEC] target=40, setting exposure to: %d\n", new_exp);
                    cam->settings.exposure_abs = new_exp;
                    struct v4l2_control ctrl;
                    memset(&ctrl, 0, sizeof(ctrl));
                    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
                    ctrl.value = new_exp;
                    (void)ioctl(cam->fd, VIDIOC_S_CTRL, &ctrl);
                }
            }
            
            pthread_cond_broadcast(&cam->cond);
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
    {
        pthread_condattr_t attr;
        pthread_condattr_init(&attr);
        pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
        pthread_cond_init(&cam->cond, &attr);
        pthread_condattr_destroy(&attr);
    }
    cam->settings.w            = UVC_DEFAULT_W;
    cam->settings.h            = UVC_DEFAULT_H;
    cam->settings.interval_us  = UVC_DEFAULT_INTERVAL_US;
    cam->settings.exposure_abs = 70;
    cam->aec_target  = 110;             // target mean brightness (0-255); tune down for bright scenes
    cam->aec_enabled = 1;
    cam->pixfmt = 0;

    fast_aec_init(&cam->aec_ctx, 40, 50, 1500, 70);
    cam->roi_w = 0.0f;

    cam->frame_buf = (uint8_t *)malloc(CAM_JPEG_MAX);
    if (!cam->frame_buf) return -1;
    return 0;
}

static void dump_camera_controls(int fd) {
    fprintf(stderr, "[V4L2] --- Camera control enumeration ---\n");
    struct v4l2_queryctrl qctrl;
    memset(&qctrl, 0, sizeof(qctrl));
    qctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
    while (ioctl(fd, VIDIOC_QUERYCTRL, &qctrl) == 0) {
        if (!(qctrl.flags & V4L2_CTRL_FLAG_DISABLED)) {
            struct v4l2_control ctrl = { .id = qctrl.id };
            ioctl(fd, VIDIOC_G_CTRL, &ctrl);
            fprintf(stderr, "  id=0x%08X %-32s min=%d max=%d step=%d default=%d current=%d\n",
                    qctrl.id, qctrl.name,
                    qctrl.minimum, qctrl.maximum, qctrl.step,
                    qctrl.default_value, ctrl.value);
        }
        qctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    }
    fprintf(stderr, "[V4L2] --- End of controls ---\n");
}

int v4l2_camera_start(V4L2Camera *cam, uint32_t w, uint32_t h) {
    if (cam->started) return 0;

    int fd = open(UVC_DEVICE_PATH, O_RDWR | O_NONBLOCK);
    if (fd < 0) { perror("v4l2 open"); return -1; }
    cam->fd = fd;

    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) { perror("VIDIOC_QUERYCAP"); goto fail; }
    fprintf(stderr, "[V4L2] device: driver=%s card=%s bus=%s version=%u\n",
            cap.driver, cap.card, cap.bus_info, cap.version);
    dump_camera_controls(fd);

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
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("VIDIOC_S_FMT MJPEG");
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) { perror("VIDIOC_S_FMT YUYV"); goto fail; }
    }
    cam->w = fmt.fmt.pix.width;
    cam->h = fmt.fmt.pix.height;
    cam->pixfmt = fmt.fmt.pix.pixelformat;

    struct v4l2_streamparm parm;
    memset(&parm, 0, sizeof(parm));
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator   = cam->settings.interval_us;
    parm.parm.capture.timeperframe.denominator = 1000000;
    if (ioctl(fd, VIDIOC_S_PARM, &parm) < 0) {
        perror("VIDIOC_S_PARM");
    } else {
        fprintf(stderr, "[V4L2] timeperframe=%u/%u\n",
                parm.parm.capture.timeperframe.numerator,
                parm.parm.capture.timeperframe.denominator);
    }

    // ── Exposure / gain controls ────────────────────────────────────────────
    // Prefer HW auto exposure (if supported) since our transport wants MJPEG.
    // If HW AE isn't available, force manual exposure + low gain to avoid
    // full-white saturation in bright scenes.
    {
        const int has_exp_auto = v4l2_ctrl_supported(fd, V4L2_CID_EXPOSURE_AUTO);
        const int has_exp_abs  = v4l2_ctrl_supported(fd, V4L2_CID_EXPOSURE_ABSOLUTE);
        const int has_gain     = v4l2_ctrl_supported(fd, V4L2_CID_GAIN);
        const int has_autogain = v4l2_ctrl_supported(fd, V4L2_CID_AUTOGAIN);

        // Keep our ToF-paced timing stable where supported.
        if (v4l2_ctrl_supported(fd, V4L2_CID_EXPOSURE_AUTO_PRIORITY))
            (void)v4l2_try_set_ctrl(fd, V4L2_CID_EXPOSURE_AUTO_PRIORITY, 0,
                                    "[V4L2] EXPOSURE_AUTO_PRIORITY (non-fatal)");

        // Try to enable HW auto exposure first.
        int hw_ae_enabled = 0;
        if (has_exp_auto) {
            if (v4l2_try_set_ctrl(fd, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_APERTURE_PRIORITY,
                                  "[V4L2] EXPOSURE_AUTO=APERTURE_PRIORITY (non-fatal)") == 0) {
                hw_ae_enabled = 1;
            } else if (v4l2_try_set_ctrl(fd, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_AUTO,
                                         "[V4L2] EXPOSURE_AUTO=AUTO (non-fatal)") == 0) {
                hw_ae_enabled = 1;
            }
        }
        fprintf(stderr, "[V4L2] hw_auto_exposure=%s\n", hw_ae_enabled ? "ON" : "OFF");

        // Auto gain tends to help keep highlights under control.
        if (has_autogain) {
            (void)v4l2_try_set_ctrl(fd, V4L2_CID_AUTOGAIN, 1,
                                    "[V4L2] AUTOGAIN (non-fatal)");
        }

        // If HW AE didn't enable, fall back to manual exposure + conservative gain.
        if (!hw_ae_enabled && has_exp_abs) {
            if (has_exp_auto) {
                (void)v4l2_try_set_ctrl(fd, V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL,
                                        "[V4L2] EXPOSURE_AUTO=MANUAL (non-fatal)");
            }
            (void)v4l2_try_set_ctrl(fd, V4L2_CID_EXPOSURE_ABSOLUTE, cam->settings.exposure_abs,
                                    "[V4L2] EXPOSURE_ABSOLUTE (non-fatal)");

            if (has_autogain) {
                (void)v4l2_try_set_ctrl(fd, V4L2_CID_AUTOGAIN, 0,
                                        "[V4L2] AUTOGAIN=0 (non-fatal)");
            }
            if (has_gain) {
                // Conservative default; camera-specific ranges vary.
                (void)v4l2_try_set_ctrl(fd, V4L2_CID_GAIN, 0,
                                        "[V4L2] GAIN=0 (non-fatal)");
            }
            v4l2_try_set_ctrl(fd, V4L2_CID_EXPOSURE_ABSOLUTE, 60, "Exposure");
            v4l2_try_set_ctrl(fd, V4L2_CID_BRIGHTNESS, -50, "Brightness"); // Crucial for this sensor
            v4l2_try_set_ctrl(fd, V4L2_CID_CONTRAST, 60, "Contrast");
            v4l2_try_set_ctrl(fd, V4L2_CID_SATURATION, 50, "Saturation");
            v4l2_try_set_ctrl(fd, V4L2_CID_GAMMA, 300, "Gamma");

            struct v4l2_streamparm parm;
            memset(&parm, 0, sizeof(parm));
            parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            parm.parm.capture.timeperframe.numerator = 1;
            parm.parm.capture.timeperframe.denominator = 120; // Force 120fps window
            ioctl(fd, VIDIOC_S_PARM, &parm);
        }


        // AWB is usually fine to leave enabled.
        if (v4l2_ctrl_supported(fd, V4L2_CID_AUTO_WHITE_BALANCE))
            (void)v4l2_try_set_ctrl(fd, V4L2_CID_AUTO_WHITE_BALANCE, 1,
                                    "[V4L2] AUTO_WHITE_BALANCE (non-fatal)");

        // Software AEC is only meaningful for raw luma formats.
        if (cam->pixfmt != V4L2_PIX_FMT_YUYV) {
            cam->aec_enabled = 0;
        }

        // For cameras without HW-AE, bias exposure toward a low-safe value
        // and reduce contrast/brightness/backlight to avoid blown highlights.
        if (!v4l2_ctrl_supported(fd, V4L2_CID_EXPOSURE_AUTO)) {
            // Exposure absolute already set above; nudge brightness/contrast
            if (v4l2_ctrl_supported(fd, V4L2_CID_BRIGHTNESS))
                (void)v4l2_try_set_ctrl(fd, V4L2_CID_BRIGHTNESS, 0, "[V4L2] BRIGHTNESS=0");
            if (v4l2_ctrl_supported(fd, V4L2_CID_CONTRAST))
                (void)v4l2_try_set_ctrl(fd, V4L2_CID_CONTRAST, 40, "[V4L2] CONTRAST=40");
            if (v4l2_ctrl_supported(fd, V4L2_CID_SATURATION))
                (void)v4l2_try_set_ctrl(fd, V4L2_CID_SATURATION, 40, "[V4L2] SATURATION=40");
            if (v4l2_ctrl_supported(fd, V4L2_CID_BACKLIGHT_COMPENSATION))
                (void)v4l2_try_set_ctrl(fd, V4L2_CID_BACKLIGHT_COMPENSATION, 0, "[V4L2] BACKLIGHT=0");
        }

        if (has_exp_auto) {
            struct v4l2_control c;
            memset(&c, 0, sizeof(c));
            c.id = V4L2_CID_EXPOSURE_AUTO;
            if (ioctl(fd, VIDIOC_G_CTRL, &c) == 0)
                fprintf(stderr, "[V4L2] exposure_auto=%d (0=MANUAL 1=AUTO 2=SHUTTER 3=APERTURE)\n", c.value);
        }
        if (has_autogain) {
            struct v4l2_control c;
            memset(&c, 0, sizeof(c));
            c.id = V4L2_CID_AUTOGAIN;
            if (ioctl(fd, VIDIOC_G_CTRL, &c) == 0)
                fprintf(stderr, "[V4L2] autogain=%d\n", c.value);
        }
        if (has_gain) {
            struct v4l2_control c;
            memset(&c, 0, sizeof(c));
            c.id = V4L2_CID_GAIN;
            if (ioctl(fd, VIDIOC_G_CTRL, &c) == 0)
                fprintf(stderr, "[V4L2] gain=%d\n", c.value);
        }

        if (has_exp_abs) {
            struct v4l2_control ctrl;
            memset(&ctrl, 0, sizeof(ctrl));
            ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
            if (ioctl(fd, VIDIOC_G_CTRL, &ctrl) == 0)
                fprintf(stderr, "[V4L2] exposure_absolute=%d (100us units = %.1f ms)\n",
                        ctrl.value, ctrl.value / 10.0f);
        }
    }
    // ───────────────────────────────────────────────────────────────────────

    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count  = V4L2_NB_BUFFERS;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) { perror("VIDIOC_REQBUFS"); goto fail; }
    cam->nbuffers = req.count;

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

    for (uint32_t bi = 0; bi < cam->nbuffers; bi++)
        fprintf(stderr, "[V4L2] buffer %u length=%u\n", bi, (unsigned)cam->mmap_lens[bi]);

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

    pthread_mutex_lock(&cam->mutex);
    cam->running = 0;
    pthread_cond_broadcast(&cam->cond);
    pthread_mutex_unlock(&cam->mutex);
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
    fast_aec_deinit(&cam->aec_ctx);
    free(cam->frame_buf);
    cam->frame_buf = NULL;
    pthread_cond_destroy(&cam->cond);
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

int v4l2_camera_snapshot_wait(V4L2Camera *cam, uint8_t *dst, uint32_t dst_cap,
                              uint32_t *out_len, uint64_t *out_ts_us,
                              uint64_t min_ts_us, uint32_t timeout_ms) {
    *out_len = 0;
    *out_ts_us = 0;
    if (!cam || !cam->started || !dst || !dst_cap) return 0;

    pthread_mutex_lock(&cam->mutex);

    // If caller doesn't care about timestamp alignment, just grab what's available.
    if (min_ts_us == 0 || timeout_ms == 0) {
        uint32_t n = cam->frame_len;
        if (cam->frame_ready && n > 0) {
            if (n > dst_cap) n = dst_cap;
            memcpy(dst, cam->frame_buf, n);
            *out_len   = n;
            *out_ts_us = cam->frame_ts_us;
            cam->frame_ready = 0;
            pthread_mutex_unlock(&cam->mutex);
            return 1;
        }
        pthread_mutex_unlock(&cam->mutex);
        return 0;
    }

    const struct timespec deadline = mono_deadline_ms(timeout_ms);
    while (cam->started) {
        if (cam->frame_ready && cam->frame_len > 0 && cam->frame_ts_us >= min_ts_us) break;

        int rc = pthread_cond_timedwait(&cam->cond, &cam->mutex, &deadline);
        if (rc == ETIMEDOUT) break;
    }

    // On timeout, return the latest frame if we have one (even if it's older than min_ts_us).
    if (!cam->frame_ready || cam->frame_len == 0) {
        pthread_mutex_unlock(&cam->mutex);
        return 0;
    }

    uint32_t n = cam->frame_len;
    if (n > dst_cap) n = dst_cap;
    memcpy(dst, cam->frame_buf, n);
    *out_len   = n;
    *out_ts_us = cam->frame_ts_us;
    cam->frame_ready = 0;
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
    usleep(10000);
    return v4l2_camera_start(cam, s->w, s->h);
}