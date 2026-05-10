#pragma once

#include <stdint.h>
#include <stddef.h>
#include <pthread.h>
#include "hub_config.h"

// Linux I2C bus abstraction using /dev/i2c-N
typedef struct {
    int         fd;
    pthread_mutex_t mutex;
} I2CBus;

int  i2c_bus_open(I2CBus *bus, const char *path);
void i2c_bus_close(I2CBus *bus);

int  i2c_bus_lock(I2CBus *bus);
void i2c_bus_unlock(I2CBus *bus);

// Raw I2C operations
int  i2c_bus_probe(I2CBus *bus, uint8_t dev_addr7);
int  i2c_bus_write_reg16(I2CBus *bus, uint8_t dev_addr7, uint16_t reg,
                         const uint8_t *data, size_t len);
int  i2c_bus_read_reg16(I2CBus *bus, uint8_t dev_addr7, uint16_t reg,
                        uint8_t *data, size_t len);
int  i2c_bus_write_reg16_word(I2CBus *bus, uint8_t dev_addr7, uint16_t reg,
                              uint16_t word);
int  i2c_bus_read_reg16_words(I2CBus *bus, uint8_t dev_addr7, uint16_t reg,
                              uint16_t *data, size_t word_count);

// I2C raw i2c_rdwr for block transfers
int  i2c_bus_raw_rdwr(I2CBus *bus, uint8_t addr, const uint8_t *wbuf, size_t wlen,
                      uint8_t *rbuf, size_t rlen);