#include "encoder.h"

#include <math.h>

#include "stm32f4xx_hal.h"
#include "tim.h"

#define ENCODER_DEFAULT_CPR 8192
#define ENCODER_DEFAULT_POLE_PAIRS 7
#define ENCODER_DEFAULT_BANDWIDTH 1000.0f

static void encoder_update_pll_gains(Encoder *encoder);

void encoder_init(Encoder *encoder)
{
    if (encoder == 0) {
        return;
    }

    encoder->config.cpr = ENCODER_DEFAULT_CPR;
    encoder->config.pole_pairs = ENCODER_DEFAULT_POLE_PAIRS;
    encoder->config.offset = 0;
    encoder->config.offset_float = 0.0f;
    encoder->config.bandwidth = ENCODER_DEFAULT_BANDWIDTH;
    encoder->config.use_interpolation = 1U;
    encoder->enabled = 0U;
    encoder->ready = 0U;
    encoder->error = 0U;
    encoder->shadow_count = 0;
    encoder->count_in_cpr = 0;
    encoder->tim_count_sample = 0;
    encoder->pos_estimate_counts = 0.0f;
    encoder->pos_cpr_counts = 0.0f;
    encoder->vel_estimate_counts = 0.0f;
    encoder->interpolation = 0.5f;
    encoder->electrical_phase = 0.0f;
    encoder->electrical_phase_vel = 0.0f;
    encoder_update_pll_gains(encoder);
}

void encoder_start(Encoder *encoder)
{
    if (encoder == 0) {
        return;
    }

    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    encoder->enabled = 1U;
    encoder->ready = 1U;
}

void encoder_sample(Encoder *encoder)
{
    if ((encoder == 0) || (encoder->enabled == 0U)) {
        return;
    }

    encoder->tim_count_sample = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
}

uint32_t encoder_update(Encoder *encoder, float dt)
{
    int32_t delta_enc;
    float delta_pos_counts;
    float delta_pos_cpr_counts;
    float corrected_count;
    float electrical_rad_per_count;

    if ((encoder == 0) || (encoder->enabled == 0U)) {
        return 0U;
    }

    delta_enc = (int32_t)((int16_t)(encoder->tim_count_sample -
                                   (int16_t)encoder->shadow_count));
    encoder->shadow_count += delta_enc;
    encoder->count_in_cpr = mod_i32(encoder->count_in_cpr + delta_enc,
                                    encoder->config.cpr);

    encoder->pos_estimate_counts += dt * encoder->vel_estimate_counts;
    encoder->pos_cpr_counts += dt * encoder->vel_estimate_counts;

    delta_pos_counts = (float)(encoder->shadow_count -
                               (int32_t)floorf(encoder->pos_estimate_counts));
    delta_pos_cpr_counts = (float)(encoder->count_in_cpr -
                                   (int32_t)floorf(encoder->pos_cpr_counts));
    delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts,
                                   0.5f * (float)encoder->config.cpr);

    encoder->pos_estimate_counts += dt * encoder->pll_kp * delta_pos_counts;
    encoder->pos_cpr_counts += dt * encoder->pll_kp * delta_pos_cpr_counts;
    encoder->pos_cpr_counts = fmodf_pos(encoder->pos_cpr_counts,
                                        (float)encoder->config.cpr);
    encoder->vel_estimate_counts += dt * encoder->pll_ki * delta_pos_cpr_counts;

    if (fabsf(encoder->vel_estimate_counts) < 0.5f * dt * encoder->pll_ki) {
        encoder->vel_estimate_counts = 0.0f;
        encoder->interpolation = 0.5f;
    } else if (encoder->config.use_interpolation == 0U) {
        encoder->interpolation = 0.5f;
    } else if (delta_enc > 0) {
        encoder->interpolation = 0.0f;
    } else if (delta_enc < 0) {
        encoder->interpolation = 1.0f;
    } else {
        encoder->interpolation += dt * encoder->vel_estimate_counts;
        encoder->interpolation = clamp_float(encoder->interpolation, 0.0f, 1.0f);
    }

    corrected_count = (float)(encoder->count_in_cpr - encoder->config.offset) +
                      encoder->interpolation -
                      encoder->config.offset_float;
    electrical_rad_per_count = (float)encoder->config.pole_pairs *
                               2.0f * M_PI_F /
                               (float)encoder->config.cpr;
    encoder->electrical_phase = wrap_pm_pi(electrical_rad_per_count *
                                           corrected_count);
    encoder->electrical_phase_vel = electrical_rad_per_count *
                                    encoder->vel_estimate_counts;
    encoder->ready = 1U;

    return 1U;
}

void encoder_set_cpr(Encoder *encoder, int32_t cpr)
{
    if ((encoder == 0) || (cpr <= 0)) {
        return;
    }

    encoder->config.cpr = cpr;
}

void encoder_set_pole_pairs(Encoder *encoder, int32_t pole_pairs)
{
    if ((encoder == 0) || (pole_pairs <= 0)) {
        return;
    }

    encoder->config.pole_pairs = pole_pairs;
}

void encoder_set_offset(Encoder *encoder, int32_t offset, float offset_float)
{
    if (encoder == 0) {
        return;
    }

    encoder->config.offset = mod_i32(offset, encoder->config.cpr);
    encoder->config.offset_float = offset_float;
}

void encoder_get_status(const Encoder *encoder, Encoder *status)
{
    if ((encoder == 0) || (status == 0)) {
        return;
    }

    *status = *encoder;
}

static void encoder_update_pll_gains(Encoder *encoder)
{
    encoder->pll_kp = 2.0f * encoder->config.bandwidth;
    encoder->pll_ki = 0.25f * encoder->pll_kp * encoder->pll_kp;
}
