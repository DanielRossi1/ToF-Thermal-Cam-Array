#include "mlx90640_driver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <stdbool.h>
#include "MLX90640_I2C_Driver.h"
#include "mlx90640_linux_hal.h"

static int16_t to_centi_c(float c) {
    float v = c * 100.0f;
    if (v > 32767.0f)  v = 32767.0f;
    if (v < -32768.0f) v = -32768.0f;
    return (int16_t)lrintf(v);
}

int mlx_begin(Mlx90640Driver *d) {
    uint16_t eeData[MLX90640_EEPROM_DUMP_NUM];
    int found = 0;

    fprintf(stderr, "MLX: Attempting to detect sensor...\n");
    i2c_bus_lock(d->bus);
    MLX90640_IDF_SetFd(d->bus->fd);

    for (int attempt = 0; attempt < 4 && !found; attempt++) {
        uint8_t addrs[] = { 0x33, 0x32 };
        for (int ai = 0; ai < 2 && !found; ai++) {
            fprintf(stderr, "MLX: Trying address 0x%02X (attempt %d/4)...\n", addrs[ai], attempt + 1);
            if (MLX90640_DumpEE(addrs[ai], eeData) == MLX90640_NO_ERROR) {
                d->addr = addrs[ai];
                found = 1;
                fprintf(stderr, "MLX: Found at 0x%02X\n", addrs[ai]);
            }
        }
        if (!found) usleep(40000);
    }
    i2c_bus_unlock(d->bus);

    if (!found) {
        fprintf(stderr, "MLX: not found at 0x33 / 0x32\n");
        return -1;
    }

    int rc = MLX90640_ExtractParameters(eeData, &d->params);
    if (rc != MLX90640_NO_ERROR) {
        fprintf(stderr, "MLX: ExtractParameters failed: %d\n", rc);
        return -1;
    }

    MlxSettings def = { MLX90640_CHESS, MLX90640_ADC_18BIT, MLX90640_16_HZ };
    d->settings = def;
    fprintf(stderr, "MLX: Successfully initialized at 0x%02X\n", d->addr);
    return mlx_apply_settings(d, &def);
}

int mlx_read_frame(Mlx90640Driver *d, MlxDataV1 *out) {
    i2c_bus_lock(d->bus);
    MLX90640_IDF_SetFd(d->bus->fd);

    float frame[MLX_PIXELS];
    memset(frame, 0, sizeof(frame));   // ← initialize

    uint16_t frameData[834];
    bool got[2] = {false, false};

    // Loop until we have one read of each subpage
    int safety = 8;
    while ((!got[0] || !got[1]) && safety-- > 0) {
        int subpage = MLX90640_GetFrameData(d->addr, frameData);
        if (subpage < 0) { i2c_bus_unlock(d->bus); return 0; }
        if (subpage > 1)  continue;
        float ta = MLX90640_GetTa(frameData, &d->params);
        MLX90640_CalculateTo(frameData, &d->params, 0.95f, ta - 8.0f, frame);
        got[subpage] = true;
    }

    const float ta = MLX90640_GetTa(frameData, &d->params);
    i2c_bus_unlock(d->bus);

    memset(out, 0, sizeof(*out));

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    out->ts_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;

    out->cfg.w          = MLX_W;
    out->cfg.h          = MLX_H;
    out->cfg.mode       = d->settings.mode;
    out->cfg.resolution = d->settings.resolution;
    out->cfg.refresh    = d->settings.refresh;
    out->cfg.reserved   = 0;
    out->ta_cC          = to_centi_c(ta);
    out->vdd_mV         = 0;

    for (uint16_t i = 0; i < MLX_PIXELS; i++)
        out->frame_cC[i] = to_centi_c(frame[i]);

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