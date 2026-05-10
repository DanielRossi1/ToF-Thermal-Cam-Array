/**
 * VL53L8CH platform layer for Linux (Rockchip RV1106).
 *
 * Implements I2C via Linux /dev/i2c-* using I2C_RDWR ioctl
 * which sends the 7-bit address in every transaction — required
 * because VL53L8CX uses 16-bit register addressing.
 * The VL53LMZ API calls these callbacks through the platform struct.
 */

#include "platform.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define VL53_I2C_ADDR  0x29

// ── I2C handle ──────────────────────────────────────────────────────────────
// Stored in platform.handle as a pointer to an int (the fd).

static int get_fd(void *handle) {
    if (!handle) return -1;
    return *(int *)handle;
}

// ── Read / Write primitives (use I2C_RDWR for proper 16-bit reg addressing) ──

uint8_t VL53L8CH_io_write(void *handle, uint16_t reg,
                          uint8_t *data, uint32_t size) {
    int fd = get_fd(handle);
    if (fd < 0) return 1;

    uint32_t offset = 0;
    while (offset < size) {
        uint32_t chunk = (size - offset > 32) ? 32 : (size - offset);
        uint8_t buf[34];
        buf[0] = (uint8_t)((reg + offset) >> 8);
        buf[1] = (uint8_t)((reg + offset) & 0xFF);
        memcpy(buf + 2, data + offset, chunk);

        struct i2c_msg msgs[1] = {
            { .addr = VL53_I2C_ADDR, .flags = 0,
              .len = (__u16)(chunk + 2), .buf = buf },
        };
        struct i2c_rdwr_ioctl_data ioctl_data = { .msgs = msgs, .nmsgs = 1 };
        if (ioctl(fd, I2C_RDWR, &ioctl_data) < 0)
            return 1;
        offset += chunk;
    }
    return 0;
}

uint8_t VL53L8CH_io_read(void *handle, uint16_t reg,
                         uint8_t *data, uint32_t size) {
    int fd = get_fd(handle);
    if (fd < 0) return 1;

    uint32_t offset = 0;
    while (offset < size) {
        uint32_t chunk = (size - offset > 32) ? 32 : (size - offset);
        uint8_t reg_buf[2];
        reg_buf[0] = (uint8_t)((reg + offset) >> 8);
        reg_buf[1] = (uint8_t)((reg + offset) & 0xFF);

        struct i2c_msg msgs[2] = {
            { .addr = VL53_I2C_ADDR, .flags = 0,
              .len = 2, .buf = reg_buf },
            { .addr = VL53_I2C_ADDR, .flags = I2C_M_RD,
              .len = (__u16)chunk, .buf = data + offset },
        };
        struct i2c_rdwr_ioctl_data ioctl_data = { .msgs = msgs, .nmsgs = 2 };
        if (ioctl(fd, I2C_RDWR, &ioctl_data) < 0)
            return 1;
        offset += chunk;
    }
    return 0;
}

uint8_t VL53L8CH_io_wait(void *handle, uint32_t ms) {
    (void)handle;
    struct timespec ts = { .tv_sec = ms / 1000, .tv_nsec = (long)(ms % 1000) * 1000000L };
    nanosleep(&ts, NULL);
    return 0;
}

// ── Platform helpers used by VL53LMZ API ────────────────────────────────────

uint8_t RdByte(VL53L8CH_Platform *p, uint16_t reg, uint8_t *val) {
    return p->Read(p->handle, reg, val, 1);
}

uint8_t WrByte(VL53L8CH_Platform *p, uint16_t reg, uint8_t val) {
    return p->Write(p->handle, reg, &val, 1);
}

uint8_t WrMulti(VL53L8CH_Platform *p, uint16_t reg, uint8_t *data, uint32_t size) {
    return p->Write(p->handle, reg, data, size);
}

uint8_t RdMulti(VL53L8CH_Platform *p, uint16_t reg, uint8_t *data, uint32_t size) {
    return p->Read(p->handle, reg, data, size);
}

uint8_t WaitMs(VL53L8CH_Platform *p, uint32_t ms) {
    return p->Wait(p->handle, ms);
}

void SwapBuffer(uint8_t *buffer, uint16_t size) {
    uint16_t i;
    uint8_t tmp[4];
    for (i = 0; i < size; i += 4) {
        tmp[0] = buffer[i + 3];
        tmp[1] = buffer[i + 2];
        tmp[2] = buffer[i + 1];
        tmp[3] = buffer[i];
        memcpy(&buffer[i], tmp, 4);
    }
}