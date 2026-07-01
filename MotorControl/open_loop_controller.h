#ifndef OPEN_LOOP_CONTROLLER_H
#define OPEN_LOOP_CONTROLLER_H

#include <stdint.h>

#include "foc.h"
#include "utils.h"

typedef struct {
    float max_voltage_mod_ramp;
    float max_electrical_phase_vel_ramp;
    float target_voltage_mod;
    float target_electrical_phase_vel;
    Float2D voltage_dq;
    float electrical_phase;
    float electrical_phase_vel;
    uint32_t enabled;
} OpenLoopController;

void open_loop_controller_init(OpenLoopController *controller);
void open_loop_controller_set_target(OpenLoopController *controller,
                                     float voltage_mod,
                                     float electrical_phase_vel);
void open_loop_controller_set_enabled(OpenLoopController *controller, uint32_t enable);
uint32_t open_loop_controller_update(OpenLoopController *controller,
                                     FocController *foc_controller,
                                     float dt);
void open_loop_controller_get_status(const OpenLoopController *controller,
                                     OpenLoopController *status);

#endif
