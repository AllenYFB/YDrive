#include "foc.h"

#include "arm_sin_cos_f32.h"
#include "pwm_adc.h"

static uint32_t foc_controller_enqueue_modulation(FocController *controller,
                                                 float mod_alpha,
                                                 float mod_beta);

void foc_controller_init(FocController *controller)
{
    if (controller == 0) {
        return;
    }

    controller->pwm_a = PWM_CENTER_TICKS;
    controller->pwm_b = PWM_CENTER_TICKS;
    controller->pwm_c = PWM_CENTER_TICKS;
    controller->mod_alpha = 0.0f;
    controller->mod_beta = 0.0f;
    controller->error = 0U;
}

uint32_t foc_controller_apply_voltage(FocController *controller,
                                      float v_d,
                                      float v_q,
                                      float pwm_phase)
{
    if (controller == 0) {
        return 0U;
    }

    float c = arm_cos_f32(pwm_phase);
    float s = arm_sin_f32(pwm_phase);
    float mod_alpha = c * v_d - s * v_q;
    float mod_beta = c * v_q + s * v_d;

    return foc_controller_enqueue_modulation(controller, mod_alpha, mod_beta);
}

void foc_controller_get_status(const FocController *controller, FocController *status)
{
    if ((controller == 0) || (status == 0)) {
        return;
    }

    *status = *controller;
}

static uint32_t foc_controller_enqueue_modulation(FocController *controller,
                                                 float mod_alpha,
                                                 float mod_beta)
{
    float t_a;
    float t_b;
    float t_c;

    if (svm(mod_alpha, mod_beta, &t_a, &t_b, &t_c) != 0) {
        controller->error |= FOC_ERROR_MODULATION_MAGNITUDE;
        pwm_adc_set_gate_enabled(0U);
        pwm_adc_write_pwm_neutral();
        return 0U;
    }

    PwmTicks pwm = {
        .phase_a = (uint32_t)(t_a * (float)PWM_PERIOD_TICKS),
        .phase_b = (uint32_t)(t_b * (float)PWM_PERIOD_TICKS),
        .phase_c = (uint32_t)(t_c * (float)PWM_PERIOD_TICKS),
    };

    controller->mod_alpha = mod_alpha;
    controller->mod_beta = mod_beta;
    controller->pwm_a = pwm.phase_a;
    controller->pwm_b = pwm.phase_b;
    controller->pwm_c = pwm.phase_c;

    pwm_adc_write_pwm(&pwm);
    return 1U;
}
