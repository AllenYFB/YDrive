#ifndef FOC_H
#define FOC_H

#include <stdint.h>

#include "utils.h"

#define FOC_ERROR_MODULATION_MAGNITUDE (1UL << 0)

typedef struct {
    uint32_t pwm_a;
    uint32_t pwm_b;
    uint32_t pwm_c;
    float mod_alpha;
    float mod_beta;
    uint32_t error;
} FocController;

void foc_controller_init(FocController *controller);
uint32_t foc_controller_apply_voltage(FocController *controller,
                                      float v_d,
                                      float v_q,
                                      float pwm_phase);
void foc_controller_get_status(const FocController *controller, FocController *status);

#endif
