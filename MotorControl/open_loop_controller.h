#ifndef OPEN_LOOP_CONTROLLER_H
#define OPEN_LOOP_CONTROLLER_H

#include <stdint.h>

#include "utils.h"

typedef struct {
    float max_voltage_ramp;
    float max_phase_vel_ramp;
    float target_voltage;
    float target_phase_vel;
    Float2D vdq_setpoint;
    float phase;
    float phase_vel;
    uint32_t enabled;
} OpenLoopStatus;

void open_loop_controller_init(void);
void open_loop_controller_set_target(float voltage, float phase_vel);
void open_loop_controller_enable(uint32_t enable);
void open_loop_controller_update(float dt);
void open_loop_controller_get_status(OpenLoopStatus *status);

#endif
