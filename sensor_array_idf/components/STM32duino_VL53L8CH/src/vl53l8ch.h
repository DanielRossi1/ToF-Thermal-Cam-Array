/**
 ******************************************************************************
 * @file    vl53l8ch.h
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    17 July 2024
 * @brief   Abstract class of a VL53L8CH Time of Flight(TOF) sensor.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2024 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Prevent recursive inclusion -----------------------------------------------*/
#ifndef __VL53L8CH_H
#define __VL53L8CH_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "platform.h"
#include "vl53lmz_api.h"
#include "vl53lmz_plugin_cnh.h"
#include "vl53lmz_plugin_detection_thresholds.h"
#include "vl53lmz_plugin_motion_indicator.h"
#include "vl53lmz_plugin_xtalk.h"

#define DEFAULT_I2C_BUFFER_LEN 128

/* Class Declaration ---------------------------------------------------------*/
class VL53L8CH {
  public:
    VL53L8CH(i2c_port_t i2c_port, int lpn_pin, int i2c_rst_pin = -1);
    virtual ~VL53L8CH(void);
    virtual int begin(void);
    virtual int end(void);
    virtual void on(void);
    virtual void off(void);
    virtual void i2c_reset(void);
    uint8_t is_alive(uint8_t *p_is_alive);
    uint8_t init(void);
    uint8_t set_i2c_address(uint16_t i2c_address);
    uint8_t get_power_mode(uint8_t *p_power_mode);
    uint8_t set_power_mode(uint8_t power_mode);
    uint8_t start_ranging(void);
    uint8_t stop_ranging(void);
    uint8_t check_data_ready(uint8_t *p_isReady);
    uint8_t get_ranging_data(VL53LMZ_ResultsData *p_results);
    uint8_t get_resolution(uint8_t *p_resolution);
    uint8_t set_resolution(uint8_t resolution);
    uint8_t get_ranging_frequency_hz(uint8_t *p_frequency_hz);
    uint8_t set_ranging_frequency_hz(uint8_t frequency_hz);
    uint8_t get_integration_time_ms(uint32_t *p_time_ms);
    uint8_t set_integration_time_ms(uint32_t integration_time_ms);
    uint8_t get_sharpener_percent(uint8_t *p_sharpener_percent);
    uint8_t set_sharpener_percent(uint8_t sharpener_percent);
    uint8_t get_target_order(uint8_t *p_target_order);
    uint8_t set_target_order(uint8_t target_order);
    uint8_t get_ranging_mode(uint8_t *p_ranging_mode);
    uint8_t set_ranging_mode(uint8_t ranging_mode);
    uint8_t enable_internal_cp(void);
    uint8_t disable_internal_cp(void);
    uint8_t get_external_sync_pin_enable(uint8_t *p_is_sync_pin_enabled);
    uint8_t set_external_sync_pin_enable(uint8_t enable_sync_pin);
    uint8_t get_glare_filter_cfg(uint8_t *p_threshold_pc_x10, int16_t *p_max_range);
    uint8_t set_glare_filter_cfg(uint8_t threshold_pc_x10, int16_t max_range);
    uint8_t dci_read_data(uint8_t *data, uint32_t index, uint16_t data_size);
    uint8_t dci_write_data(uint8_t *data, uint32_t index, uint16_t data_size);
    uint8_t dci_replace_data(uint8_t *data, uint32_t index, uint16_t data_size, uint8_t *new_data, uint16_t new_data_size, uint16_t new_data_pos);
    uint8_t create_output_config(void);
    uint8_t send_output_config_and_start(void);
    uint8_t add_output_block(uint32_t block_header);
    uint8_t disable_output_block(uint32_t block_header);
    uint8_t results_extract_block(uint32_t blk_index, uint8_t *p_data, uint16_t data_size);
    uint8_t get_detection_thresholds_enable(uint8_t *p_enabled);
    uint8_t set_detection_thresholds_enable(uint8_t enabled);
    uint8_t get_detection_thresholds(VL53LMZ_DetectionThresholds *p_thresholds);
    uint8_t set_detection_thresholds(VL53LMZ_DetectionThresholds *p_thresholds);
    uint8_t get_detection_thresholds_auto_stop(uint8_t *p_auto_stop);
    uint8_t set_detection_thresholds_auto_stop(uint8_t auto_stop);
    uint8_t motion_indicator_init(VL53LMZ_Motion_Configuration *p_motion_config, uint8_t resolution);
    uint8_t motion_indicator_set_distance_motion(VL53LMZ_Motion_Configuration  *p_motion_config, uint16_t distance_min_mm, uint16_t distance_max_mm);
    uint8_t motion_indicator_set_resolution(VL53LMZ_Motion_Configuration *p_motion_config, uint8_t resolution);
    uint8_t calibrate_xtalk(uint16_t reflectance_percent, uint8_t nb_samples, uint16_t distance_mm);
    uint8_t get_caldata_xtalk(uint8_t *p_xtalk_data);
    uint8_t set_caldata_xtalk(uint8_t *p_xtalk_data);
    uint8_t get_xtalk_margin(uint32_t *p_xtalk_margin);
    uint8_t set_xtalk_margin(uint32_t xtalk_margin);
    uint8_t cnh_init_config(VL53LMZ_Motion_Configuration *p_mi_config, int16_t start_bin, int16_t num_bins, int16_t sub_sample);
    uint8_t cnh_create_agg_map(VL53LMZ_Motion_Configuration *p_mi_config, int16_t resolution, int16_t start_x, int16_t start_y, int16_t merge_x, int16_t merge_y, int16_t cols, int16_t rows);
    uint8_t cnh_calc_required_memory(VL53LMZ_Motion_Configuration *p_mi_config, uint32_t *p_mem_size);
    uint8_t cnh_calc_min_max_distance(VL53LMZ_Motion_Configuration *p_mi_config, int16_t *p_min_distance, int16_t *p_max_distance);
    uint8_t cnh_send_config(VL53LMZ_Motion_Configuration *p_mi_config);
    uint8_t cnh_get_block_addresses(VL53LMZ_Motion_Configuration *p_mi_config, int32_t agg_id, cnh_data_buffer_t mi_persistent_array, int32_t **p_hist, int8_t  **p_hist_scaler, int32_t **p_ambient, int8_t **p_ambient_scaler);
    uint32_t cnh_get_ref_residual(cnh_data_buffer_t mi_persistent_array);
    uint8_t get_ranging_frequency_x256(VL53LMZ_Configuration *p_dev, uint16_t *p_frequency_x256);
    uint8_t set_ranging_frequency_x256(VL53LMZ_Configuration *p_dev, uint16_t p_frequency_x256);
    uint8_t get_VHV_repeat_count(VL53LMZ_Configuration *p_dev, uint32_t *p_repeat_count);
    uint8_t set_VHV_repeat_count(VL53LMZ_Configuration *p_dev, uint32_t p_repeat_count);

    /**
     * @brief Utility function to read data.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  p_values: pointer to data to be read.
     * @param  size: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint16_t RegisterAddress, uint8_t *p_values, uint32_t size)
    {
      uint8_t reg[2];
      reg[0] = (uint8_t)(RegisterAddress >> 8);
      reg[1] = (uint8_t)(RegisterAddress & 0xFF);

      esp_err_t err = i2c_master_write_read_device(dev_i2c, (_dev.platform.address >> 1) & 0x7F,
                                                   reg, 2,
                                                   p_values, size,
                                                   pdMS_TO_TICKS(100));
      return (err == ESP_OK) ? 0 : 1;
    }

    /**
     * @brief Utility function to write data.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  p_values: pointer to data to be written.
     * @param  size: number of bytes to be written.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint16_t RegisterAddress, uint8_t *p_values, uint32_t size)
    {
      uint8_t buffer[DEFAULT_I2C_BUFFER_LEN + 2];
      uint32_t i = 0;
      
      while (i < size) {
        size_t current_write_size = (size - i > DEFAULT_I2C_BUFFER_LEN ? DEFAULT_I2C_BUFFER_LEN : size - i);
        buffer[0] = (uint8_t)((RegisterAddress + i) >> 8);
        buffer[1] = (uint8_t)((RegisterAddress + i) & 0xFF);
        for(size_t j = 0; j < current_write_size; ++j) {
            buffer[2 + j] = p_values[i + j];
        }

        esp_err_t err = i2c_master_write_to_device(dev_i2c, (_dev.platform.address >> 1) & 0x7F,
                                                   buffer, current_write_size + 2,
                                                   pdMS_TO_TICKS(100));
        if (err != ESP_OK) {
          return 1;
        }
        i += current_write_size;
      }

      return 0;
    }

    /**
     * @brief Utility function to wait.
     * @param  ms: milliseconds to wait.
     * @retval 0
     */
    uint8_t IO_Wait(uint32_t ms)
    {
      vTaskDelay(pdMS_TO_TICKS(ms));
      return 0;
    }

  private:

    /* Helper classes. */
    i2c_port_t dev_i2c;

    /* Configuration */
    int  lpn_pin;
    int i2c_rst_pin;

    VL53LMZ_Configuration _dev;
    VL53LMZ_Configuration *p_dev;
};

#ifdef __cplusplus
extern "C" {
#endif

uint8_t VL53L8CH_io_write(void *handle, uint16_t RegisterAddress, uint8_t *p_values, uint32_t size);
uint8_t VL53L8CH_io_read(void *handle, uint16_t RegisterAddress, uint8_t *p_values, uint32_t size);
uint8_t VL53L8CH_io_wait(void *handle, uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __VL53L8CH_H */
