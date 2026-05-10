#ifndef PLATFORM_H_
#define PLATFORM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include "platform_config.h"

#define VL53L8CH_COMMS_CHUNK_SIZE  4096
#define SPI_WRITE_MASK(x)          (uint16_t)(x | 0x8000)
#define SPI_READ_MASK(x)           (uint16_t)(x & ~0x8000)

#ifndef DEFAULT_I2C_BUFFER_LEN
#define DEFAULT_I2C_BUFFER_LEN     32
#endif

typedef uint8_t (*VL53L8CH_wait_Func)(void *, uint32_t);
typedef uint8_t (*VL53L8CH_write_Func)(void *, uint16_t, uint8_t *, uint32_t);
typedef uint8_t (*VL53L8CH_read_Func)(void *, uint16_t, uint8_t *, uint32_t);

typedef struct {
    uint16_t address;
    VL53L8CH_write_Func Write;
    VL53L8CH_read_Func Read;
    VL53L8CH_wait_Func Wait;
    void *handle;
} VL53L8CH_Platform;

typedef VL53L8CH_Platform VL53LMZ_Platform;

uint8_t RdByte(VL53L8CH_Platform *p_platform, uint16_t RegisterAddress, uint8_t *p_value);
uint8_t WrByte(VL53L8CH_Platform *p_platform, uint16_t RegisterAddress, uint8_t value);
uint8_t WrMulti(VL53L8CH_Platform *p_platform, uint16_t RegisterAddress, uint8_t *p_values, uint32_t size);
uint8_t RdMulti(VL53L8CH_Platform *p_platform, uint16_t RegisterAddress, uint8_t *p_values, uint32_t size);
uint8_t WaitMs(VL53L8CH_Platform *p_platform, uint32_t TimeMs);
void SwapBuffer(uint8_t *buffer, uint16_t size);

// Linux platform I/O callbacks used by tof_vl53l8ch driver
uint8_t VL53L8CH_io_write(void *handle, uint16_t reg, uint8_t *data, uint32_t size);
uint8_t VL53L8CH_io_read(void *handle, uint16_t reg, uint8_t *data, uint32_t size);
uint8_t VL53L8CH_io_wait(void *handle, uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif