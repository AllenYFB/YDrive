#ifndef CURRENT_CONTROLLER_H
#define CURRENT_CONTROLLER_H

#include <stdint.h>

#include "foc.h"
#include "pwm_adc.h"
#include "utils.h"

#define CURRENT_CONTROLLER_ERROR_CURRENT_LIMIT (1UL << 0)
#define CURRENT_CONTROLLER_ERROR_FOC (1UL << 1)

typedef struct {
    float phase_current_gain;
    float p_gain;
    float i_gain;
    float current_limit;
    float max_voltage_mod;
    Float2D current_setpoint;
    Float2D current_measured;
    Float2D voltage_integrator;
    Float2D voltage_mod;
    uint32_t enabled;
    uint32_t error;
} CurrentController;

void current_controller_init(CurrentController *controller);
void current_controller_set_enabled(CurrentController *controller, uint32_t enable);
void current_controller_set_target(CurrentController *controller, float id_setpoint, float iq_setpoint);
void current_controller_set_gains(CurrentController *controller, float p_gain, float i_gain);
void current_controller_clear_error(CurrentController *controller);
uint32_t current_controller_update(CurrentController *controller,
                                   FocController *foc_controller,
                                   const PhaseCurrentSample *sample,
                                   float current_phase,
                                   float pwm_phase,
                                   float dt);
void current_controller_get_status(const CurrentController *controller,
                                   CurrentController *status);

#endif
