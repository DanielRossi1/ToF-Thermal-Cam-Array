#include "fast_aec.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <turbojpeg.h>

int fast_aec_init(FastAecContext *ctx, int32_t target_luma, int32_t exp_min, int32_t exp_max, int32_t start_exp) {
    if (!ctx) return -1;
    
    ctx->tj_handle = tjInitDecompress();
    if (!ctx->tj_handle) {
        fprintf(stderr, "[AEC] tjInitDecompress failed: %s\n", tjGetErrorStr());
        return -1;
    }

    // Allocate buffer for 1/8th scale of a 1080p image max (to be safe)
    // 1920/8 = 240, 1080/8 = 135 -> 240 * 135 = ~32KB
    ctx->buf_size = 240 * 135;
    ctx->gray_buf = (uint8_t*)malloc(ctx->buf_size);
    if (!ctx->gray_buf) {
        tjDestroy(ctx->tj_handle);
        return -1;
    }

    ctx->target_luma  = target_luma;
    ctx->exp_min      = exp_min;
    ctx->exp_max      = exp_max;
    ctx->current_exp  = start_exp;
    
    // SOTA PID Tuning for 30Hz AE polling
    ctx->kp           = 0.45f;
    ctx->ki           = 0.05f;
    ctx->kd           = 0.10f;
    ctx->integral_err = 0.0f;
    ctx->prev_err     = 0.0f;

    return 0;
}

int32_t fast_aec_process_frame(FastAecContext *ctx, const uint8_t *jpg_data, uint32_t jpg_len, 
                               float roi_x, float roi_y, float roi_weight) {
    if (!ctx || !ctx->tj_handle || !jpg_data || jpg_len == 0) return ctx->current_exp;

    int width, height, jpegSubsamp, jpegColorspace;
    
    // 1. Read Header (Very fast)
    if (tjDecompressHeader3(ctx->tj_handle, jpg_data, jpg_len, &width, &height, &jpegSubsamp, &jpegColorspace) < 0) {
        return ctx->current_exp; // Corrupt JPEG, skip frame
    }

    // 2. Setup 1/8 scaling (DC extraction)
    tjscalingfactor sf = {1, 8};
    int scaled_w = TJSCALED(width, sf);
    int scaled_h = TJSCALED(height, sf);

    if ((uint32_t)(scaled_w * scaled_h) > ctx->buf_size) {
        return ctx->current_exp; // Out of bounds safety
    }

    // 3. Decompress to Grayscale using FAST flags
    if (tjDecompress2(ctx->tj_handle, jpg_data, jpg_len, ctx->gray_buf, scaled_w, 0, scaled_h, TJPF_GRAY, TJFLAG_FASTDCT | TJFLAG_FASTUPSAMPLE) < 0) {
        return ctx->current_exp;
    }

    // 4. Calculate Weighted Luma
    double total_weight = 0.0;
    double weighted_luma_sum = 0.0;

    // Pre-calculate ROI pixel coordinates
    int roi_px_x = (int)(roi_x * scaled_w);
    int roi_px_y = (int)(roi_y * scaled_h);
    
    // Max distance for Gaussian falloff calculation
    float max_dist_sq = (scaled_w * scaled_w) + (scaled_h * scaled_h);

    for (int y = 0; y < scaled_h; y++) {
        for (int x = 0; x < scaled_w; x++) {
            uint8_t pixel_luma = ctx->gray_buf[y * scaled_w + x];
            float weight = 1.0f; // Base weight

            // 5. Apply Sensor-Fusion ROI Weighting
            if (roi_weight > 0.01f) {
                float dx = (float)(x - roi_px_x);
                float dy = (float)(y - roi_px_y);
                float dist_sq = (dx*dx + dy*dy);
                
                // Inverse distance weighting (Gaussian-like)
                float proximity_factor = 1.0f - (dist_sq / (max_dist_sq * 0.1f)); 
                if (proximity_factor < 0.0f) proximity_factor = 0.0f;

                // Combine base weight with ROI attention
                weight = 1.0f + (proximity_factor * roi_weight * 5.0f); // Multiply impact of ROI
            }

            weighted_luma_sum += (pixel_luma * weight);
            total_weight += weight;
        }
    }

    float current_luma = (float)(weighted_luma_sum / total_weight);

    // 6. PID Control Math
    float error = (float)ctx->target_luma - current_luma;
    
    ctx->integral_err += error;
    // Anti-windup (prevent integral runaway)
    if (ctx->integral_err >  500.0f) ctx->integral_err =  500.0f;
    if (ctx->integral_err < -500.0f) ctx->integral_err = -500.0f;

    float derivative = error - ctx->prev_err;
    ctx->prev_err = error;

    float adjustment = (ctx->kp * error) + (ctx->ki * ctx->integral_err) + (ctx->kd * derivative);

    // 7. Update Exposure with Dampening
    int32_t new_exp = ctx->current_exp + (int32_t)adjustment;

    // Hardware Clamp
    if (new_exp < ctx->exp_min) new_exp = ctx->exp_min;
    if (new_exp > ctx->exp_max) new_exp = ctx->exp_max;

    ctx->current_exp = new_exp;
    return new_exp;
}

void fast_aec_deinit(FastAecContext *ctx) {
    if (ctx) {
        if (ctx->tj_handle) tjDestroy(ctx->tj_handle);
        if (ctx->gray_buf) free(ctx->gray_buf);
        ctx->tj_handle = NULL;
        ctx->gray_buf = NULL;
    }
}