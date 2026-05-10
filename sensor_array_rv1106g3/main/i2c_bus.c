#include "i2c_bus.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

int i2c_bus_open(I2CBus *bus, const char *path) {
    memset(bus, 0, sizeof(*bus));
    bus->fd = open(path, O_RDWR);
    if (bus->fd < 0) {
        perror("i2c open");
        return -1;
    }
    pthread_mutex_init(&bus->mutex, NULL);
    return 0;
}

void i2c_bus_close(I2CBus *bus) {
    if (bus->fd >= 0) {
        close(bus->fd);
        bus->fd = -1;
    }
    pthread_mutex_destroy(&bus->mutex);
}

int i2c_bus_lock(I2CBus *bus) {
    return pthread_mutex_lock(&bus->mutex);
}

void i2c_bus_unlock(I2CBus *bus) {
    pthread_mutex_unlock(&bus->mutex);
}

int i2c_bus_probe(I2CBus *bus, uint8_t dev_addr7) {
    if (ioctl(bus->fd, I2C_SLAVE, dev_addr7) < 0) return 0;
    uint8_t dummy;
    return (read(bus->fd, &dummy, 0) >= 0) ? 1 : 0;
}

int i2c_bus_write_reg16(I2CBus *bus, uint8_t dev_addr7, uint16_t reg,
                        const uint8_t *data, size_t len) {
    uint8_t *buf = (uint8_t *)malloc(len + 2);
    if (!buf) return -1;
    buf[0] = (uint8_t)(reg >> 8);
    buf[1] = (uint8_t)(reg & 0xFF);
    if (data && len) memcpy(buf + 2, data, len);

    struct i2c_msg msgs[1] = {
        { .addr = dev_addr7, .flags = 0, .len = (__u16)(len + 2), .buf = buf },
    };
    struct i2c_rdwr_ioctl_data ioctl_data = { .msgs = msgs, .nmsgs = 1 };
    int ret = ioctl(bus->fd, I2C_RDWR, &ioctl_data);
    free(buf);
    return (ret < 0) ? -1 : 0;
}

int i2c_bus_read_reg16(I2CBus *bus, uint8_t dev_addr7, uint16_t reg,
                       uint8_t *data, size_t len) {
    if (!data || len == 0) return -1;
    uint8_t reg_buf[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };

    struct i2c_msg msgs[2] = {
        { .addr = dev_addr7, .flags = 0, .len = 2, .buf = reg_buf },
        { .addr = dev_addr7, .flags = I2C_M_RD, .len = (__u16)len, .buf = data },
    };
    struct i2c_rdwr_ioctl_data ioctl_data = { .msgs = msgs, .nmsgs = 2 };
    return (ioctl(bus->fd, I2C_RDWR, &ioctl_data) < 0) ? -1 : 0;
}

int i2c_bus_write_reg16_word(I2CBus *bus, uint8_t dev_addr7, uint16_t reg,
                             uint16_t word) {
    uint8_t buf[2] = { (uint8_t)(word >> 8), (uint8_t)(word & 0xFF) };
    return i2c_bus_write_reg16(bus, dev_addr7, reg, buf, 2);
}

int i2c_bus_read_reg16_words(I2CBus *bus, uint8_t dev_addr7, uint16_t reg,
                             uint16_t *data, size_t word_count) {
    if (!data || word_count == 0) return -1;
    int ret = i2c_bus_read_reg16(bus, dev_addr7, reg, (uint8_t *)data, word_count * 2);
    if (ret < 0) return ret;
    for (size_t i = 0; i < word_count; i++) {
        uint8_t *p = (uint8_t *)&data[i];
        uint8_t tmp = p[0];
        p[0] = p[1];
        p[1] = tmp;
    }
    return 0;
}

int i2c_bus_raw_rdwr(I2CBus *bus, uint8_t addr, const uint8_t *wbuf, size_t wlen,
                     uint8_t *rbuf, size_t rlen) {
    struct i2c_msg msgs[2];
    int nmsgs = 0;

    if (wbuf && wlen) {
        msgs[nmsgs].addr  = addr;
        msgs[nmsgs].flags = 0;
        msgs[nmsgs].len   = (__u16)wlen;
        msgs[nmsgs].buf   = (uint8_t *)wbuf;
        nmsgs++;
    }
    if (rbuf && rlen) {
        msgs[nmsgs].addr  = addr;
        msgs[nmsgs].flags = I2C_M_RD;
        msgs[nmsgs].len   = (__u16)rlen;
        msgs[nmsgs].buf   = rbuf;
        nmsgs++;
    }
    struct i2c_rdwr_ioctl_data ioctl_data = { .msgs = msgs, .nmsgs = nmsgs };
    return (ioctl(bus->fd, I2C_RDWR, &ioctl_data) < 0) ? -1 : 0;
}