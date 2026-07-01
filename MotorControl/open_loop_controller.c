#include "open_loop_controller.h"

#define OPEN_LOOP_DEFAULT_MAX_VOLTAGE_RAMP 0.5f
#define OPEN_LOOP_DEFAULT_MAX_PHASE_VEL_RAMP 40.0f
#define OPEN_LOOP_PWM_PHASE_ADVANCE_PERIODS 1.5f

void open_loop_controller_init(OpenLoopController *controller)
{
    if (controller == 0) {
        return;
    }

    controller->max_voltage_mod_ramp = OPEN_LOOP_DEFAULT_MAX_VOLTAGE_RAMP;
    controller->max_electrical_phase_vel_ramp = OPEN_LOOP_DEFAULT_MAX_PHASE_VEL_RAMP;
    controller->target_voltage_mod = 0.0f;
    controller->target_electrical_phase_vel = 0.0f;
    controller->voltage_dq.d = 0.0f;
    controller->voltage_dq.q = 0.0f;
    controller->electrical_phase = 0.0f;
    controller->electrical_phase_vel = 0.0f;
    controller->enabled = 0U;
}

void open_loop_controller_set_target(OpenLoopController *controller,
                                     float voltage_mod,
                                     float electrical_phase_vel)
{
    if (controller == 0) {
        return;
    }

    controller->target_voltage_mod = clamp_float(voltage_mod, 0.0f, 0.12f);
    controller->target_electrical_phase_vel = clamp_float(electrical_phase_vel, -80.0f, 80.0f);
}

void open_loop_controller_set_enabled(OpenLoopController *controller, uint32_t enable)
{
    if (controller == 0) {
        return;
    }

    controller->enabled = (enable != 0U) ? 1U : 0U;
    if (controller->enabled == 0U) {
        controller->target_voltage_mod = 0.0f;
        controller->target_electrical_phase_vel = 0.0f;
    }
}

uint32_t open_loop_controller_update(OpenLoopController *controller,
                                     FocController *foc_controller,
                                     float dt)
{
    if ((controller == 0) || (foc_controller == 0)) {
        return 0U;
    }

    float prev_vd = controller->voltage_dq.d;
    float prev_vel = controller->electrical_phase_vel;

    if (controller->enabled == 0U) {
        controller->target_voltage_mod = 0.0f;
        controller->target_electrical_phase_vel = 0.0f;
    }

    controller->voltage_dq.d = clamp_float(controller->target_voltage_mod,
                                           prev_vd - controller->max_voltage_mod_ramp * dt,
                                           prev_vd + controller->max_voltage_mod_ramp * dt);
    controller->voltage_dq.q = 0.0f;

    controller->electrical_phase_vel = clamp_float(controller->target_electrical_phase_vel,
                                                   prev_vel - controller->max_electrical_phase_vel_ramp * dt,
                                                   prev_vel + controller->max_electrical_phase_vel_ramp * dt);
    controller->electrical_phase = wrap_pm_pi(controller->electrical_phase +
                                             controller->electrical_phase_vel * dt);

    float pwm_phase = wrap_pm_pi(controller->electrical_phase +
                                 OPEN_LOOP_PWM_PHASE_ADVANCE_PERIODS * dt *
                                     controller->electrical_phase_vel);
    if (foc_controller_apply_voltage(foc_controller,
                                     controller->voltage_dq.d,
                                     controller->voltage_dq.q,
                                     pwm_phase) == 0U) {
        open_loop_controller_set_enabled(controller, 0U);
        return 0U;
    }

    return 1U;
}

void open_loop_controller_get_status(const OpenLoopController *controller,
                                     OpenLoopController *status)
{
    if ((controller == 0) || (status == 0)) {
        return;
    }

    *status = *controller;
}
