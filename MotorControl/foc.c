#include "foc.h"

#include "arm_cos_f32.h"
#include "pwm_adc.h"

#define FOC_ERROR_MODULATION_MAGNITUDE (1UL << 0)

static volatile FocStatus foc_status;

static void foc_enqueue_modulation(float mod_alpha, float mod_beta);

void foc_voltage(float v_d, float v_q, float pwm_phase)
{
    float c = our_arm_cos_f32(pwm_phase);
    float s = our_arm_sin_f32(pwm_phase);
    float mod_alpha = c * v_d - s * v_q;
    float mod_beta = c * v_q + s * v_d;

    foc_enqueue_modulation(mod_alpha, mod_beta);
}

void foc_get_status(FocStatus *status)
{
    if (status == 0) {
        return;
    }

    *status = foc_status;
}

static void foc_enqueue_modulation(float mod_alpha, float mod_beta)
{
    float t_a;
    float t_b;
    float t_c;

    if (svm(mod_alpha, mod_beta, &t_a, &t_b, &t_c) != 0) {
        foc_status.error |= FOC_ERROR_MODULATION_MAGNITUDE;
        pwm_adc_write_pwm_neutral();
        return;
    }

    uint32_t pwm_a = (uint32_t)(t_a * (float)FOC_PWM_PERIOD_TICKS);
    uint32_t pwm_b = (uint32_t)(t_b * (float)FOC_PWM_PERIOD_TICKS);
    uint32_t pwm_c = (uint32_t)(t_c * (float)FOC_PWM_PERIOD_TICKS);

    foc_status.mod_alpha = mod_alpha;
    foc_status.mod_beta = mod_beta;
    foc_status.pwm_a = pwm_a;
    foc_status.pwm_b = pwm_b;
    foc_status.pwm_c = pwm_c;

    pwm_adc_write_pwm_ticks(pwm_a, pwm_b, pwm_c);
}
