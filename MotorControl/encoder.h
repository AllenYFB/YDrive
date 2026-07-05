#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#include "utils.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_ERROR_UNSTABLE_GAIN (1UL << 0)

typedef struct {
    int32_t cpr;
    int32_t pole_pairs;
    int32_t offset;
    float offset_float;
    float bandwidth;
    uint32_t use_interpolation;
} EncoderConfig;

typedef struct {
    EncoderConfig config;
    uint32_t enabled;
    uint32_t ready;
    uint32_t error;
    int32_t shadow_count;
    int32_t count_in_cpr;
    int16_t tim_count_sample;
    float pll_kp;
    float pll_ki;
    float pos_estimate_counts;
    float pos_cpr_counts;
    float vel_estimate_counts;
    float interpolation;
    float electrical_phase;
    float electrical_phase_vel;
} Encoder;

void encoder_init(Encoder *encoder);
void encoder_start(Encoder *encoder);
void encoder_sample(Encoder *encoder);
uint32_t encoder_update(Encoder *encoder, float dt);
void encoder_set_cpr(Encoder *encoder, int32_t cpr);
void encoder_set_pole_pairs(Encoder *encoder, int32_t pole_pairs);
void encoder_set_offset(Encoder *encoder, int32_t offset, float offset_float);
void encoder_get_status(const Encoder *encoder, Encoder *status);

#ifdef __cplusplus
}
#endif

#endif
