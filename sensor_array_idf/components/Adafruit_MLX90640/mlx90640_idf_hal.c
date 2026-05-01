/*
 * MLX90640 I2C HAL for ESP-IDF.
 * Implements the Melexis driver I/O interface using the ESP-IDF legacy I2C API.
 */

#include "MLX90640_I2C_Driver.h"
#include "MLX90640_API.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

// These are set once before the first use via MLX90640_IDF_SetContext().
static i2c_port_t s_port = I2C_NUM_0;

void MLX90640_IDF_SetPort(i2c_port_t port) {
    s_port = port;
}

void MLX90640_I2CInit(void) {
    // I2C is already initialised by the bus owner; nothing to do here.
}

void MLX90640_I2CFreqSet(int freq) {
    (void)freq;  // frequency is managed by the I2CBus owner
}

int MLX90640_I2CGeneralReset(void) {
    // Send general-call reset (address 0x00, data 0x06).
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, 0x00 | I2C_MASTER_WRITE, true);
    uint8_t reset_byte = 0x06;
    i2c_master_write_byte(cmd, reset_byte, false);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_port, cmd, pdMS_TO_TICKS(10));
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK) ? 0 : -1;
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress,
                     uint16_t nMemAddressRead, uint16_t *data) {
    if (!data || nMemAddressRead == 0) return -1;

    const size_t byte_count = (size_t)nMemAddressRead * 2u;
    uint8_t reg[2] = {(uint8_t)(startAddress >> 8), (uint8_t)(startAddress & 0xFF)};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ((uint8_t)slaveAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, reg, 2, true);
    i2c_master_start(cmd);  // repeated start
    i2c_master_write_byte(cmd, ((uint8_t)slaveAddr << 1) | I2C_MASTER_READ, true);
    uint8_t *p = (uint8_t *)data;
    if (byte_count > 1) {
        i2c_master_read(cmd, p, byte_count - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, p + byte_count - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_port, cmd, pdMS_TO_TICKS(500));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return MLX90640_I2C_NACK_ERROR;

    // Sensor sends big-endian 16-bit words; convert to host byte order.
    for (uint16_t i = 0; i < nMemAddressRead; i++) {
        uint8_t *w = (uint8_t *)&data[i];
        uint8_t tmp = w[0];
        w[0] = w[1];
        w[1] = tmp;
    }
    return MLX90640_NO_ERROR;
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data) {
    uint8_t buf[4];
    buf[0] = (uint8_t)(writeAddress >> 8);
    buf[1] = (uint8_t)(writeAddress & 0xFF);
    buf[2] = (uint8_t)(data >> 8);
    buf[3] = (uint8_t)(data & 0xFF);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ((uint8_t)slaveAddr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buf, 4, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) return MLX90640_I2C_WRITE_ERROR;

    // Verify echo after write (1 ms settle)
    vTaskDelay(pdMS_TO_TICKS(1));
    uint16_t check = 0;
    int rc = MLX90640_I2CRead(slaveAddr, writeAddress, 1, &check);
    if (rc != MLX90640_NO_ERROR) return rc;
    if (check != data) return MLX90640_I2C_WRITE_ERROR;
    return MLX90640_NO_ERROR;
}
