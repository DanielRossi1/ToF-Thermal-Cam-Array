/*
 * MLX90640 I2C HAL for Linux (Buildroot / Rockchip RV1106).
 * Implements the Melexis driver I/O interface using Linux /dev/i2c-*.
 */
#include "MLX90640_I2C_Driver.h"
#include "MLX90640_API.h"
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

static int s_fd = -1;

void MLX90640_IDF_SetFd(int fd) {
    s_fd = fd;
}

void MLX90640_I2CInit(void) {
    // I2C is already initialised by the bus owner.
}

void MLX90640_I2CFreqSet(int freq) {
    (void)freq;
}

int MLX90640_I2CGeneralReset(void) {
    uint8_t buf[1] = { 0x06 };
    struct i2c_msg msgs[1] = {
        { .addr = 0x00, .flags = 0, .len = 1, .buf = buf },
    };
    struct i2c_rdwr_ioctl_data ioctl_data = { .msgs = msgs, .nmsgs = 1 };
    if (ioctl(s_fd, I2C_RDWR, &ioctl_data) < 0) return -1;
    return 0;
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress,
                     uint16_t nMemAddressRead, uint16_t *data) {
    if (!data || nMemAddressRead == 0) return -1;
    if (s_fd < 0) return -1;

    const size_t byte_count = (size_t)nMemAddressRead * 2u;
    uint8_t reg[2] = { (uint8_t)(startAddress >> 8), (uint8_t)(startAddress & 0xFF) };

    struct i2c_msg msgs[2] = {
        { .addr = (__u16)slaveAddr, .flags = 0, .len = 2, .buf = reg },
        { .addr = (__u16)slaveAddr, .flags = I2C_M_RD, .len = (__u16)byte_count, .buf = (uint8_t *)data },
    };
    struct i2c_rdwr_ioctl_data ioctl_data = { .msgs = msgs, .nmsgs = 2 };
    if (ioctl(s_fd, I2C_RDWR, &ioctl_data) < 0) return MLX90640_I2C_NACK_ERROR;

    for (uint16_t i = 0; i < nMemAddressRead; i++) {
        uint8_t *w = (uint8_t *)&data[i];
        uint8_t tmp = w[0];
        w[0] = w[1];
        w[1] = tmp;
    }
    return MLX90640_NO_ERROR;
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data) {
    if (s_fd < 0) return -1;
    uint8_t buf[4] = {
        (uint8_t)(writeAddress >> 8),
        (uint8_t)(writeAddress & 0xFF),
        (uint8_t)(data >> 8),
        (uint8_t)(data & 0xFF),
    };
    struct i2c_msg msgs[1] = {
        { .addr = (__u16)slaveAddr, .flags = 0, .len = 4, .buf = buf },
    };
    struct i2c_rdwr_ioctl_data ioctl_data = { .msgs = msgs, .nmsgs = 1 };
    if (ioctl(s_fd, I2C_RDWR, &ioctl_data) < 0) return MLX90640_I2C_WRITE_ERROR;

    usleep(2000);
    uint16_t check = 0;
    int rc = MLX90640_I2CRead(slaveAddr, writeAddress, 1, &check);
    if (rc != MLX90640_NO_ERROR) return rc;
    if (check != data) return MLX90640_I2C_WRITE_ERROR;
    return MLX90640_NO_ERROR;
}