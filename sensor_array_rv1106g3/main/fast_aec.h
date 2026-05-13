#pragma once

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    void* tj_handle;          // libjpeg-turbo decompressor handle
    uint8_t* gray_buf;          // 1/8th scale luma buffer (usually 80x60 = 4.8KB)
    uint32_t buf_size;          // Allocated size of gray_buf

    // PID Controller State
    int32_t  target_luma;       // Target brightness (0-255), e.g., 100
    float    kp, ki, kd;        // PID tuning parameters
    float    integral_err;      // Accumulated error
    float    prev_err;          // Previous error (for derivative)
    
    // Hardware Limits
    int32_t  exp_min;           // Minimum absolute exposure (e.g., 50)
    int32_t  exp_max;           // Maximum absolute exposure (e.g., 2000)
    int32_t  current_exp;       // The last calculated exposure
} FastAecContext;

// Initialize the AEC context. Returns 0 on success, -1 on failure.
int fast_aec_init(FastAecContext *ctx, int32_t target_luma, int32_t exp_min, int32_t exp_max, int32_t start_exp);

// Process an MJPEG frame and calculate the new exposure.
// roi_x, roi_y: 0.0f to 1.0f (normalized coordinates of the heat signature).
// roi_weight: 0.0f (ignore ROI, use global average) to 1.0f (100% focus on ROI).
// Returns the new exposure value to apply via V4L2.
int32_t fast_aec_process_frame(FastAecContext *ctx, const uint8_t *jpg_data, uint32_t jpg_len, 
                               float roi_x, float roi_y, float roi_weight);

// Cleanup
void fast_aec_deinit(FastAecContext *ctx);

#ifdef __cplusplus
}
#endif