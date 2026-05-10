#include <Adafruit_MLX90640.h>
#include <algorithm>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*!
 *    @brief  Instantiates a new MLX90640 class
 */
Adafruit_MLX90640::Adafruit_MLX90640(void) {}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_addr
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_MLX90640::begin(uint8_t i2c_addr, i2c_port_t i2c_port) {
  i2c_addr_ = i2c_addr;
  i2c_port_ = i2c_port;

  MLX90640_I2CRead(0, MLX90640_DEVICEID1, 3, serialNumber);

  uint16_t eeMLX90640[832];
  if (MLX90640_DumpEE(0, eeMLX90640) != 0) {
    return false;
  }
#ifdef MLX90640_DEBUG
  for (int i = 0; i < 832; i++) {
    Serial.printf("0x%x, ", eeMLX90640[i]);
  }
  Serial.println();
#endif

  MLX90640_ExtractParameters(eeMLX90640, &_params);
  // whew!
  return true;
}

/*!
 *    @brief  Read nMemAddressRead words from I2C startAddress into data
 *    @param  slaveAddr Not used - kept to maintain backcompatible API
 *    @param  startAddress I2C memory address to start reading
 *    @param  nMemAddressRead 16-bit words to read
 *    @param  data Location to place data read
 *    @return 0 on success
 */
int Adafruit_MLX90640::MLX90640_I2CRead(uint8_t slaveAddr,
                                        uint16_t startAddress,
                                        uint16_t nMemAddressRead,
                                        uint16_t *data) {
  while (nMemAddressRead > 0) {
    uint16_t toRead16 = std::min(nMemAddressRead, (uint16_t)kI2CChunkWords);
    uint8_t cmd[2] = {uint8_t(startAddress >> 8), uint8_t(startAddress & 0xFF)};
    esp_err_t err = i2c_master_write_read_device(
        i2c_port_, slaveAddr ? slaveAddr : i2c_addr_, cmd, sizeof(cmd),
        reinterpret_cast<uint8_t *>(data), toRead16 * sizeof(uint16_t),
        pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
      return -1;
    }
    // we now have to swap every two bytes
    for (int i = 0; i < toRead16; i++) {
      data[i] = __builtin_bswap16(data[i]);
    }
    // advance buffer
    data += toRead16;
    // advance address
    startAddress += toRead16;
    // reduce remaining to read
    nMemAddressRead -= toRead16;
  }
  // success!
  return 0;
}

int Adafruit_MLX90640::MLX90640_I2CWrite(uint8_t slaveAddr,
                                         uint16_t writeAddress, uint16_t data) {
  uint8_t cmd[4];

  cmd[0] = writeAddress >> 8;
  cmd[1] = writeAddress & 0x00FF;
  cmd[2] = data >> 8;
  cmd[3] = data & 0x00FF;

  esp_err_t err = i2c_master_write_to_device(i2c_port_,
                                             slaveAddr ? slaveAddr : i2c_addr_,
                                             cmd, sizeof(cmd),
                                             pdMS_TO_TICKS(100));
  if (err != ESP_OK) {
    return -1;
  }
  vTaskDelay(pdMS_TO_TICKS(1));
  return 0;
}

/*!
 *    @brief Get the frame-read mode
 *    @return Chess or interleaved mode
 */
mlx90640_mode_t Adafruit_MLX90640::getMode(void) {
  return (mlx90640_mode_t)MLX90640_GetCurMode(0);
}

/*!
 *    @brief Set the frame-read mode
 *    @param mode Chess or interleaved mode
 */
void Adafruit_MLX90640::setMode(mlx90640_mode_t mode) {
  if (mode == MLX90640_CHESS) {
    MLX90640_SetChessMode(0);
  } else {
    MLX90640_SetInterleavedMode(0);
  }
}

/*!
 *    @brief  Get resolution for temperature precision
 *    @returns The desired resolution (bits)
 */
mlx90640_resolution_t Adafruit_MLX90640::getResolution(void) {
  return (mlx90640_resolution_t)MLX90640_GetCurResolution(0);
}

/*!
 *    @brief  Set resolution for temperature precision
 *    @param res The desired resolution (bits)
 */
void Adafruit_MLX90640::setResolution(mlx90640_resolution_t res) {
  MLX90640_SetResolution(0, (int)res);
}

/*!
 *    @brief  Get max refresh rate
 *    @returns How many pages per second to read (2 pages per frame)
 */
mlx90640_refreshrate_t Adafruit_MLX90640::getRefreshRate(void) {
  return (mlx90640_refreshrate_t)MLX90640_GetRefreshRate(0);
}

/*!
 *    @brief  Set max refresh rate - too fast and we can't read the
 *    the pages in time, start low and then increment while speeding
 *    up I2C!
 *    @param rate How many pages per second to read (2 pages per frame)
 */
void Adafruit_MLX90640::setRefreshRate(mlx90640_refreshrate_t rate) {
  MLX90640_SetRefreshRate(0, (int)rate);
}

/*!
 *    @brief  Read 2 pages, calculate temperatures and place into framebuf
 *    @param  framebuf 24*32 floating point memory buffer
 *    @return 0 on success
 */
int Adafruit_MLX90640::getFrame(float *framebuf) {
  float emissivity = 0.95;
  float tr = 23.15;
  uint16_t mlx90640Frame[834];
  int status;

  for (uint8_t page = 0; page < 2; page++) {
    status = MLX90640_GetFrameData(i2c_addr_, mlx90640Frame);

#ifdef MLX90640_DEBUG
    Serial.printf("Page%d = [", page);
    for (int i = 0; i < 834; i++) {
      Serial.printf("0x%x, ", mlx90640Frame[i]);
    }
    Serial.println("]");
#endif

    if (status < 0) {
      return status;
    }

    ta = MLX90640_GetTa(mlx90640Frame, &_params); // Store ambient temp locally
    tr = ta - OPENAIR_TA_SHIFT; // For a MLX90640 in the open air the shift is
                                // -8 degC.
#ifdef MLX90640_DEBUG
    Serial.print("Tr = ");
    Serial.println(tr, 8);
#endif
    MLX90640_CalculateTo(mlx90640Frame, &_params, emissivity, tr, framebuf);
  }
  return 0;
}

/*!
 *    @brief  Return ambient temperature of the TO39 package.
 *    @param  newFrame If true, will also capture a new data frame. If false,
 * return the value from the last data frame read.
 *    @return Ambient temperature as a float in degrees Celsius.
 */
float Adafruit_MLX90640::getTa(bool newFrame) {
  if (!newFrame) {
    return ta;
  }
  uint16_t mlx90640Frame[834];
  MLX90640_GetFrameData(i2c_addr_, mlx90640Frame);
  return MLX90640_GetTa(mlx90640Frame, &_params);
}
