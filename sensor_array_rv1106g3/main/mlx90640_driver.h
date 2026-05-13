#pragma once

#include <stdint.h>
#include <pthread.h>
#include "hub_frame.h"
#include "i2c_bus.h"
#include "MLX90640_API.h"

#define MLX90640_0_5_HZ  0
#define MLX90640_1_HZ    1
#define MLX90640_2_HZ    2
#define MLX90640_4_HZ    3
#define MLX90640_8_HZ    4
#define MLX90640_16_HZ   5
#define MLX90640_32_HZ   6
#define MLX90640_64_HZ   7

#define MLX90640_ADC_16BIT 0
#define MLX90640_ADC_17BIT 1
#define MLX90640_ADC_18BIT 2
#define MLX90640_ADC_19BIT 3

#define MLX90640_CHESS       1
#define MLX90640_INTERLEAVED 0

typedef struct {
    uint8_t mode;
    uint8_t resolution;
    uint8_t refresh;
} MlxSettings;

typedef struct {
    I2CBus         *bus;
    paramsMLX90640  params;
    uint8_t         addr;
    MlxSettings     settings;

    // Background capture thread
    pthread_t       thread;
    pthread_mutex_t mutex;
    pthread_cond_t  cond;
    int             running;

    // Published frame (written by thread, consumed by loop_thread)
    MlxDataV1       latest;
    int             frame_ready;
    uint32_t        frame_gen;

    // Internal accumulation across subpages
    float           frame_f[MLX_PIXELS];
    uint8_t         got_subpage[2];
} Mlx90640Driver;

int  mlx_begin(Mlx90640Driver *d);
int  mlx_start_thread(Mlx90640Driver *d);
void mlx_stop_thread(Mlx90640Driver *d);
int  mlx_consume(Mlx90640Driver *d, MlxDataV1 *out);  // called from loop_thread
void mlx_get_settings(Mlx90640Driver *d, MlxSettings *s);
int  mlx_apply_settings(Mlx90640Driver *d, const MlxSettings *s);