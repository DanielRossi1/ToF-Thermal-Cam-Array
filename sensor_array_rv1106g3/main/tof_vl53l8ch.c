#include "tof_vl53l8ch.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#define VL53L8CX_I2C_ADDR  0x29

uint8_t vl53l8cx_idf_write(void *handle, uint16_t reg, uint8_t *data, uint32_t size) {
    return VL53L8CH_io_write(handle, reg, data, size);
}

uint8_t vl53l8cx_idf_read(void *handle, uint16_t reg, uint8_t *data, uint32_t size) {
    return VL53L8CH_io_read(handle, reg, data, size);
}

uint8_t vl53l8cx_idf_wait(void *handle, uint32_t ms) {
    return VL53L8CH_io_wait(handle, ms);
}

int tof_begin(TofVl53l8ch *tof) {
    I2CBus *bus    = tof->bus;
    int     lpn_pin = tof->lpn_pin;
    memset(tof, 0, sizeof(*tof));
    tof->bus     = bus;
    tof->lpn_pin = lpn_pin;
    tof->ranging_started = 0;

    // Default settings
    tof->settings.side               = 8;
    tof->settings.ranging_hz         = 15;
    tof->settings.integration_time_ms = 50;
    tof->settings.continuous         = 1;

    // Optional: toggle LPn for hardware reset via GPIO sysfs
    if (tof->lpn_pin >= 0) {
        char dir_path[64], val_path[64];
        snprintf(dir_path, sizeof(dir_path),
                 "/sys/class/gpio/gpio%d/direction", tof->lpn_pin);
        snprintf(val_path, sizeof(val_path),
                 "/sys/class/gpio/gpio%d/value", tof->lpn_pin);

        // Export if not already exported
        if (access(dir_path, F_OK) != 0) {
            int efd = open("/sys/class/gpio/export", O_WRONLY);
            if (efd >= 0) {
                char buf[8];
                int n = snprintf(buf, sizeof(buf), "%d", tof->lpn_pin);
                if (write(efd, buf, (size_t)n) < 0) {
                    fprintf(stderr, "ToF: GPIO export write failed");
                }
                close(efd);
                usleep(50000);
            } else {
                fprintf(stderr, "ToF: Cannot open GPIO export");
            }
        }

        // Set direction to output
        int dfd = open(dir_path, O_WRONLY);
        if (dfd >= 0) {
            if (write(dfd, "out", 3) < 0) {
                fprintf(stderr, "ToF: GPIO direction write failed");
            }
            close(dfd);
        } else {
            fprintf(stderr, "ToF: Cannot open GPIO direction");
        }

        // Toggle LOW -> HIGH with proper boot delay
        int vfd = open(val_path, O_WRONLY);
        if (vfd >= 0) {
            ssize_t w0 = write(vfd, "0", 1);      // 10ms LOW  (hard reset)
            (void)w0;
            usleep(10000);
            ssize_t w1 = write(vfd, "1", 1);      // HIGH = wake up
            (void)w1;
            close(vfd);
            usleep(100000);           // 100ms boot time
            fprintf(stderr, "ToF: LPn pin toggled for reset");
        } else {
            fprintf(stderr, "ToF: Cannot open GPIO value (LPn)");
        }
    }

    // Set up platform
    tof->fd = tof->bus->fd;

    tof->dev.platform.address = VL53L8CX_I2C_ADDR;
    tof->dev.platform.Write   = vl53l8cx_idf_write;
    tof->dev.platform.Read    = vl53l8cx_idf_read;
    tof->dev.platform.Wait    = vl53l8cx_idf_wait;
    tof->dev.platform.handle  = tof->bus;

    i2c_bus_lock(tof->bus);
    fprintf(stderr, "ToF: Attempting to initialize sensor at 0x%02X\n", VL53L8CX_I2C_ADDR);
    uint8_t status = vl53l8cx_init(&tof->dev);
    i2c_bus_unlock(tof->bus);

    if (status != VL53L8CX_STATUS_OK) {
        fprintf(stderr, "ToF: vl53l8cx_init failed: %u\n", (unsigned)status);
        return -1;
    }
    fprintf(stderr, "ToF: Initialization successful, applying settings\n");
    int ret = tof_apply_settings(tof, &tof->settings);
    if (ret != 0) {
        fprintf(stderr, "ToF: Failed to apply settings\n");
        return -1;
    }
    fprintf(stderr, "ToF: Sensor ready and ranging started\n");
    return 0;
}

int tof_read(TofVl53l8ch *tof, TofDataV1 *out) {
    i2c_bus_lock(tof->bus);

    uint8_t ready = 0;
    uint8_t st = vl53l8cx_check_data_ready(&tof->dev, &ready);
    if (st != VL53L8CX_STATUS_OK || !ready) {
        i2c_bus_unlock(tof->bus);
        return 0;
    }

    VL53L8CX_ResultsData res;
    st = vl53l8cx_get_ranging_data(&tof->dev, &res);
    i2c_bus_unlock(tof->bus);
    if (st != VL53L8CX_STATUS_OK) return 0;

    memset(out, 0, sizeof(*out));

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    out->ts_us = (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;

    out->cfg.side               = tof->settings.side;
    out->cfg.targets_per_zone   = VL53L8CX_NB_TARGET_PER_ZONE;
    out->cfg.ranging_hz         = tof->settings.ranging_hz;
    out->cfg.integration_time_ms = tof->settings.integration_time_ms;
    out->cfg.reserved           = 0;
    out->silicon_temp_degc      = (uint8_t)res.silicon_temp_degc;

    const uint8_t tpz   = VL53L8CX_NB_TARGET_PER_ZONE;
    const uint8_t zones = TOF_ZONES;

    for (uint8_t z = 0; z < zones; z++) {
        out->nb_target_detected[z] = res.nb_target_detected[z];
        out->nb_spads_enabled[z]   = (uint8_t)res.nb_spads_enabled[z];
        out->ambient_per_spad[z]   = res.ambient_per_spad[z];

        for (uint8_t t = 0; t < tpz && t < TOF_MAX_TARGETS_PER_ZONE; t++) {
            int src = (int)z * tpz + t;
            int dst = (int)z * TOF_MAX_TARGETS_PER_ZONE + t;
            out->distance_mm[dst]     = res.distance_mm[src];
            out->range_sigma_mm[dst]  = res.range_sigma_mm[src];
            out->target_status[dst]   = res.target_status[src];
            out->reflectance[dst]     = res.reflectance[src];
            out->signal_per_spad[dst] = res.signal_per_spad[src];
        }
        for (uint8_t t = tpz; t < TOF_MAX_TARGETS_PER_ZONE; t++) {
            int dst = (int)z * TOF_MAX_TARGETS_PER_ZONE + t;
            out->distance_mm[dst]     = 0;
            out->range_sigma_mm[dst]  = 0;
            out->target_status[dst]   = 0;
            out->reflectance[dst]     = 0;
            out->signal_per_spad[dst] = 0;
        }
    }
    return 1;
}

void tof_get_settings(TofVl53l8ch *tof, TofSettings *s) {
    *s = tof->settings;
}

int tof_apply_settings(TofVl53l8ch *tof, const TofSettings *s) {
    TofSettings next = *s;
    if (next.side != 4 && next.side != 8) next.side = 8;
    if (next.ranging_hz == 0)             next.ranging_hz = 10;
    if (next.integration_time_ms == 0)    next.integration_time_ms = 20;

    i2c_bus_lock(tof->bus);

    if (tof->ranging_started) {
        vl53l8cx_stop_ranging(&tof->dev);
        tof->ranging_started = 0;
    }

    uint8_t res  = (next.side == 4) ? VL53L8CX_RESOLUTION_4X4
                                     : VL53L8CX_RESOLUTION_8X8;
    uint8_t mode = next.continuous ? VL53L8CX_RANGING_MODE_CONTINUOUS
                                   : VL53L8CX_RANGING_MODE_AUTONOMOUS;
    uint8_t st;

    st = vl53l8cx_set_resolution(&tof->dev, res);
    if (st) { fprintf(stderr, "ToF: set_resolution failed: %u\n", st);
               i2c_bus_unlock(tof->bus); return -1; }

    st = vl53l8cx_set_ranging_frequency_hz(&tof->dev, (uint8_t)next.ranging_hz);
    if (st) { fprintf(stderr, "ToF: set_frequency failed: %u\n", st);
               i2c_bus_unlock(tof->bus); return -1; }

    st = vl53l8cx_set_ranging_mode(&tof->dev, mode);
    if (st) { fprintf(stderr, "ToF: set_mode failed: %u\n", st);
               i2c_bus_unlock(tof->bus); return -1; }

    st = vl53l8cx_set_integration_time_ms(&tof->dev, next.integration_time_ms);
    if (st) { fprintf(stderr, "ToF: set_integration failed: %u\n", st);
               i2c_bus_unlock(tof->bus); return -1; }

    st = vl53l8cx_start_ranging(&tof->dev);
    i2c_bus_unlock(tof->bus);

    if (st != VL53L8CX_STATUS_OK) {
        fprintf(stderr, "ToF: start_ranging failed: %u\n", st);
        return -1;
    }

    tof->ranging_started = 1;
    tof->settings = next;
    return 0;
}