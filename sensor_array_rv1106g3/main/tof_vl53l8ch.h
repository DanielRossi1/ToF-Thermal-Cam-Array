#pragma once

#include <stdint.h>
#include "hub_frame.h"
#include "i2c_bus.h"

// Compatibility defs mapping VL53L8CX -> VL53LMZ
#define VL53L8CX_RESOLUTION_4X4    16
#define VL53L8CX_RESOLUTION_8X8    64
#define VL53L8CX_NB_TARGET_PER_ZONE 4
#define VL53L8CX_STATUS_OK         0
#define VL53L8CX_STATUS_ERROR      255

#define VL53L8CX_RANGING_MODE_CONTINUOUS  1
#define VL53L8CX_RANGING_MODE_AUTONOMOUS  3

#include "vl53lmz_api.h"

typedef VL53LMZ_Configuration  VL53L8CX_Configuration;
typedef VL53LMZ_ResultsData    VL53L8CX_ResultsData;

#define vl53l8cx_init               vl53lmz_init
#define vl53l8cx_start_ranging      vl53lmz_start_ranging
#define vl53l8cx_stop_ranging       vl53lmz_stop_ranging
#define vl53l8cx_check_data_ready   vl53lmz_check_data_ready
#define vl53l8cx_get_ranging_data   vl53lmz_get_ranging_data
#define vl53l8cx_set_resolution     vl53lmz_set_resolution
#define vl53l8cx_set_ranging_frequency_hz  vl53lmz_set_ranging_frequency_hz
#define vl53l8cx_set_ranging_mode   vl53lmz_set_ranging_mode
#define vl53l8cx_set_integration_time_ms   vl53lmz_set_integration_time_ms

typedef struct {
    uint8_t  side;
    uint16_t ranging_hz;
    uint16_t integration_time_ms;
    int      continuous;
} TofSettings;

typedef struct {
    I2CBus              *bus;
    int                  lpn_pin;
    VL53L8CX_Configuration dev;
    TofSettings          settings;
    int                  ranging_started;
    int                  fd;  // I2C fd for platform
} TofVl53l8ch;

int  tof_begin(TofVl53l8ch *tof);
int  tof_read(TofVl53l8ch *tof, TofDataV1 *out);
void tof_get_settings(TofVl53l8ch *tof, TofSettings *s);
int  tof_apply_settings(TofVl53l8ch *tof, const TofSettings *s);

// Platform callbacks (use void* as expected by VL53LMZ API)
uint8_t vl53l8cx_idf_write(void *handle, uint16_t Address, uint8_t *p_values, uint32_t size);
uint8_t vl53l8cx_idf_read(void *handle, uint16_t Address, uint8_t *p_values, uint32_t size);
uint8_t vl53l8cx_idf_wait(void *handle, uint32_t TimeMs);