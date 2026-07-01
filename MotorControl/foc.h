#ifndef FOC_H
#define FOC_H

#include <stdint.h>

#include "utils.h"

#define FOC_PWM_PERIOD_TICKS 3500U
#define FOC_PWM_CENTER_TICKS 1750U

typedef struct {
    uint32_t pwm_a;
    uint32_t pwm_b;
    uint32_t pwm_c;
    float mod_alpha;
    float mod_beta;
    uint32_t error;
} FocStatus;

void foc_voltage(float v_d, float v_q, float pwm_phase);
void foc_get_status(FocStatus *status);

#endif
