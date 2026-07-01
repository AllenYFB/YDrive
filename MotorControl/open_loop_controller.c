#include "open_loop_controller.h"

#include "foc.h"

#define OPEN_LOOP_DEFAULT_MAX_VOLTAGE_RAMP 0.5f
#define OPEN_LOOP_DEFAULT_MAX_PHASE_VEL_RAMP 40.0f
#define OPEN_LOOP_PWM_PHASE_ADVANCE_PERIODS 1.5f

static volatile OpenLoopStatus open_loop_status;

void open_loop_controller_init(void)
{
    open_loop_status.max_voltage_ramp = OPEN_LOOP_DEFAULT_MAX_VOLTAGE_RAMP;
    open_loop_status.max_phase_vel_ramp = OPEN_LOOP_DEFAULT_MAX_PHASE_VEL_RAMP;
    open_loop_status.target_voltage = 0.0f;
    open_loop_status.target_phase_vel = 0.0f;
    open_loop_status.vdq_setpoint.d = 0.0f;
    open_loop_status.vdq_setpoint.q = 0.0f;
    open_loop_status.phase = 0.0f;
    open_loop_status.phase_vel = 0.0f;
    open_loop_status.enabled = 0U;
}

void open_loop_controller_set_target(float voltage, float phase_vel)
{
    open_loop_status.target_voltage = CLAMP(voltage, 0.0f, 0.12f);
    open_loop_status.target_phase_vel = CLAMP(phase_vel, -80.0f, 80.0f);
}

void open_loop_controller_enable(uint32_t enable)
{
    open_loop_status.enabled = (enable != 0U) ? 1U : 0U;
    if (open_loop_status.enabled == 0U) {
        open_loop_status.target_voltage = 0.0f;
        open_loop_status.target_phase_vel = 0.0f;
    }
}

void open_loop_controller_update(float dt)
{
    float prev_vd = open_loop_status.vdq_setpoint.d;
    float prev_vel = open_loop_status.phase_vel;

    if (open_loop_status.enabled == 0U) {
        open_loop_status.target_voltage = 0.0f;
        open_loop_status.target_phase_vel = 0.0f;
    }

    open_loop_status.vdq_setpoint.d = CLAMP(open_loop_status.target_voltage,
                                            prev_vd - open_loop_status.max_voltage_ramp * dt,
                                            prev_vd + open_loop_status.max_voltage_ramp * dt);
    open_loop_status.vdq_setpoint.q = 0.0f;

    open_loop_status.phase_vel = CLAMP(open_loop_status.target_phase_vel,
                                       prev_vel - open_loop_status.max_phase_vel_ramp * dt,
                                       prev_vel + open_loop_status.max_phase_vel_ramp * dt);
    open_loop_status.phase = wrap_pm_pi(open_loop_status.phase + open_loop_status.phase_vel * dt);

    float pwm_phase = wrap_pm_pi(open_loop_status.phase +
                                 OPEN_LOOP_PWM_PHASE_ADVANCE_PERIODS * dt * open_loop_status.phase_vel);
    foc_voltage(open_loop_status.vdq_setpoint.d, open_loop_status.vdq_setpoint.q, pwm_phase);
}

void open_loop_controller_get_status(OpenLoopStatus *status)
{
    if (status == 0) {
        return;
    }

    *status = open_loop_status;
}
