#include "mlx90640_driver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <errno.h>
#include "MLX90640_I2C_Driver.h"
#include "mlx90640_linux_hal.h"

static int16_t to_centi_c(float c) {
    float v = c * 100.0f;
    if (v >  32767.0f) v =  32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)lrintf(v);
}

// ── Background capture thread ──────────────────────────────────────────────────

static void *mlx_thread(void *arg) {
    Mlx90640Driver *d = arg;

    while (d->running) {
        // Poll status register (bit 3 = new data ready).
        // Sleep 4 ms between checks — fast enough for 16 Hz, cheap on CPU.
        i2c_bus_lock(d->bus);
        MLX90640_IDF_SetFd(d->bus->fd);
        uint16_t status = 0;
        int rc = MLX90640_I2CRead(d->addr, 0x8000, 1, &status);
        i2c_bus_unlock(d->bus);

        if (rc != MLX90640_NO_ERROR) {
            // I2C error: back off and attempt bus recovery on next iteration
            fprintf(stderr, "MLX: status read error %d, backing off\n", rc);
            usleep(20000);
            continue;
        }
        if (!(status & 0x0008u)) {
            usleep(4000);
            continue;
        }

        // New subpage ready
        i2c_bus_lock(d->bus);
        MLX90640_IDF_SetFd(d->bus->fd);
        uint16_t frameData[834];
        int subpage = MLX90640_GetFrameData(d->addr, frameData);
        float ta = 0.0f;
        if (subpage == 0 || subpage == 1)
            ta = MLX90640_GetTa(frameData, &d->params);
        i2c_bus_unlock(d->bus);  // <-- release before heavy CPU work

        if (subpage == 0 || subpage == 1) {
            MLX90640_CalculateTo(frameData, &d->params, 0.95f, ta - 8.0f, d->frame_f);
            d->got_subpage[subpage] = 1;
        }

        if (subpage < 0 || subpage > 1) {
            fprintf(stderr, "MLX: bad subpage %d, skipping\n", subpage);
            usleep(4000);
            continue;
        }

        // Need both subpages before publishing a full frame
        if (!d->got_subpage[0] || !d->got_subpage[1])
            continue;

        // Build output
        MlxDataV1 out;
        memset(&out, 0, sizeof(out));
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        out.ts_us           = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
        out.cfg.w           = MLX_W;
        out.cfg.h           = MLX_H;
        out.cfg.mode        = d->settings.mode;
        out.cfg.resolution  = d->settings.resolution;
        out.cfg.refresh     = d->settings.refresh;
        out.cfg.reserved    = 0;
        out.ta_cC           = to_centi_c(ta);
        out.vdd_mV          = 0;
        for (uint16_t i = 0; i < MLX_PIXELS; i++)
            out.frame_cC[i] = to_centi_c(d->frame_f[i]);

        pthread_mutex_lock(&d->mutex);
        d->latest      = out;
        d->frame_ready = 1;
        d->frame_gen++;
        pthread_cond_broadcast(&d->cond);
        pthread_mutex_unlock(&d->mutex);
    }
    return NULL;
}

// ── Public API ─────────────────────────────────────────────────────────────────

int mlx_begin(Mlx90640Driver *d) {
    uint16_t eeData[MLX90640_EEPROM_DUMP_NUM];
    int found = 0;

    fprintf(stderr, "MLX: Attempting to detect sensor...\n");
    i2c_bus_lock(d->bus);
    MLX90640_IDF_SetFd(d->bus->fd);

    for (int attempt = 0; attempt < 4 && !found; attempt++) {
        uint8_t addrs[] = { 0x33, 0x32 };
        for (int ai = 0; ai < 2 && !found; ai++) {
            fprintf(stderr, "MLX: Trying address 0x%02X (attempt %d/4)...\n",
                    addrs[ai], attempt + 1);
            if (MLX90640_DumpEE(addrs[ai], eeData) == MLX90640_NO_ERROR) {
                d->addr = addrs[ai];
                found   = 1;
                fprintf(stderr, "MLX: Found at 0x%02X\n", addrs[ai]);
            }
        }
        if (!found) usleep(40000);
    }
    i2c_bus_unlock(d->bus);

    if (!found) { fprintf(stderr, "MLX: not found\n"); return -1; }

    int rc = MLX90640_ExtractParameters(eeData, &d->params);
    if (rc != MLX90640_NO_ERROR) {
        fprintf(stderr, "MLX: ExtractParameters failed: %d\n", rc);
        return -1;
    }

    MlxSettings def = { MLX90640_CHESS, MLX90640_ADC_18BIT, MLX90640_16_HZ };
    d->settings = def;
    memset(d->frame_f, 0, sizeof(d->frame_f));
    d->got_subpage[0] = 0;
    d->got_subpage[1] = 0;
    d->frame_ready    = 0;
    d->frame_gen      = 0;
    d->running        = 0;

    fprintf(stderr, "MLX: Successfully initialized at 0x%02X\n", d->addr);
    return mlx_apply_settings(d, &def);
}

int mlx_start_thread(Mlx90640Driver *d) {
    pthread_mutex_init(&d->mutex, NULL);
    {
        pthread_condattr_t attr;
        pthread_condattr_init(&attr);
        pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
        pthread_cond_init(&d->cond, &attr);
        pthread_condattr_destroy(&attr);
    }
    d->running = 1;
    if (pthread_create(&d->thread, NULL, mlx_thread, d) != 0) {
        d->running = 0;
        return -1;
    }
    fprintf(stderr, "MLX: capture thread started\n");
    return 0;
}

void mlx_stop_thread(Mlx90640Driver *d) {
    if (!d->running) return;
    d->running = 0;
    pthread_cond_broadcast(&d->cond);
    pthread_join(d->thread, NULL);
    pthread_cond_destroy(&d->cond);
    pthread_mutex_destroy(&d->mutex);
    fprintf(stderr, "MLX: capture thread stopped\n");
}

// Called from loop_thread instead of the old mlx_read_frame
int mlx_consume(Mlx90640Driver *d, MlxDataV1 *out) {
    pthread_mutex_lock(&d->mutex);
    if (!d->frame_ready) {
        pthread_mutex_unlock(&d->mutex);
        return 0;
    }
    *out           = d->latest;
    d->frame_ready = 0;
    pthread_mutex_unlock(&d->mutex);
    return 1;
}

void mlx_get_settings(Mlx90640Driver *d, MlxSettings *s) {
    *s = d->settings;
}

int mlx_apply_settings(Mlx90640Driver *d, const MlxSettings *s) {
    i2c_bus_lock(d->bus);
    MLX90640_IDF_SetFd(d->bus->fd);

    if (s->mode == MLX90640_CHESS)
        MLX90640_SetChessMode(d->addr);
    else
        MLX90640_SetInterleavedMode(d->addr);

    MLX90640_SetResolution(d->addr, s->resolution);
    MLX90640_SetRefreshRate(d->addr, s->refresh);

    i2c_bus_unlock(d->bus);
    d->settings = *s;
    return 0;
}