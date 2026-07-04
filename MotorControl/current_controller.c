#include "current_controller.h"

#include <math.h>

#include "arm_sin_cos_f32.h"

#define CURRENT_DEFAULT_PHASE_GAIN 0.01f
#define CURRENT_DEFAULT_P_GAIN 0.002f
#define CURRENT_DEFAULT_I_GAIN 1.0f
#define CURRENT_DEFAULT_LIMIT 5.0f
#define CURRENT_DEFAULT_MAX_MOD 0.08f
#define CURRENT_DEFAULT_SETPOINT_RAMP 0.1f
#define CURRENT_REPORT_FILTER_K 0.1f
#define CURRENT_INTEGRATOR_DECAY 0.99f

static void current_controller_check_limit(CurrentController *controller,
                                           float id,
                                           float iq);
static void current_controller_clarke_park(const PhaseCurrentSample *sample,
                                           float phase_current_gain,
                                           float current_phase,
                                           float *id,
                                           float *iq);
static void current_controller_update_pi(CurrentController *controller,
                                         float id,
                                         float iq,
                                         float dt);
static void current_controller_ramp_setpoint(CurrentController *controller, float dt);

void current_controller_init(CurrentController *controller)
{
    if (controller == 0) {
        return;
    }

    controller->config.phase_current_gain = CURRENT_DEFAULT_PHASE_GAIN;
    controller->config.current_limit = CURRENT_DEFAULT_LIMIT;
    controller->config.max_voltage_mod = CURRENT_DEFAULT_MAX_MOD;
    controller->pi.kp = CURRENT_DEFAULT_P_GAIN;
    controller->pi.ki = CURRENT_DEFAULT_I_GAIN;
    controller->current.setpoint.d = 0.0f;
    controller->current.setpoint.q = 0.0f;
    controller->current.ramped_setpoint.d = 0.0f;
    controller->current.ramped_setpoint.q = 0.0f;
    controller->current.measured.d = 0.0f;
    controller->current.measured.q = 0.0f;
    controller->pi.integral_voltage.d = 0.0f;
    controller->pi.integral_voltage.q = 0.0f;
    controller->pi.output_voltage.d = 0.0f;
    controller->pi.output_voltage.q = 0.0f;
    controller->enabled = 0U;
    controller->error = 0U;
}

void current_controller_set_enabled(CurrentController *controller, uint32_t enable)
{
    if (controller == 0) {
        return;
    }

    controller->enabled = (enable != 0U) ? 1U : 0U;
    if (controller->enabled != 0U) {
        controller->error = 0U;
    }

    if (controller->enabled == 0U) {
        controller->current.setpoint.d = 0.0f;
        controller->current.setpoint.q = 0.0f;
        controller->current.ramped_setpoint.d = 0.0f;
        controller->current.ramped_setpoint.q = 0.0f;
        controller->pi.integral_voltage.d = 0.0f;
        controller->pi.integral_voltage.q = 0.0f;
        controller->pi.output_voltage.d = 0.0f;
        controller->pi.output_voltage.q = 0.0f;
    }
}

void current_controller_set_target(CurrentController *controller,
                                   float id_setpoint,
                                   float iq_setpoint)
{
    if (controller == 0) {
        return;
    }

    controller->current.setpoint.d = clamp_float(id_setpoint,
                                                 -controller->config.current_limit,
                                                 controller->config.current_limit);
    controller->current.setpoint.q = clamp_float(iq_setpoint,
                                                 -controller->config.current_limit,
                                                 controller->config.current_limit);
}

void current_controller_set_gains(CurrentController *controller, float p_gain, float i_gain)
{
    if (controller == 0) {
        return;
    }

    controller->pi.kp = clamp_float(p_gain, 0.0f, 1.0f);
    controller->pi.ki = clamp_float(i_gain, 0.0f, 1000.0f);
}

void current_controller_set_phase_current_gain(CurrentController *controller,
                                               float phase_current_gain)
{
    if (controller == 0) {
        return;
    }

    controller->config.phase_current_gain = clamp_float(phase_current_gain, -1.0f, 1.0f);
}

void current_controller_set_limits(CurrentController *controller,
                                   float current_limit,
                                   float max_voltage_mod)
{
    if (controller == 0) {
        return;
    }

    controller->config.current_limit = clamp_float(current_limit, 0.0f, 200.0f);
    controller->config.max_voltage_mod = clamp_float(max_voltage_mod, 0.0f, 0.95f);
    current_controller_set_target(controller,
                                  controller->current.setpoint.d,
                                  controller->current.setpoint.q);
}

void current_controller_clear_error(CurrentController *controller)
{
    if (controller == 0) {
        return;
    }

    controller->error = 0U;
}

uint32_t current_controller_update(CurrentController *controller,
                                   FocController *foc_controller,
                                   const PhaseCurrentSample *sample,
                                   float current_phase,
                                   float pwm_phase,
                                   float dt)
{
    float id;
    float iq;

    if ((controller == 0) || (foc_controller == 0) || (sample == 0)) {
        return 0U;
    }

    if (controller->enabled == 0U) {
        return 0U;
    }

    current_controller_clarke_park(sample,
                                   controller->config.phase_current_gain,
                                   current_phase,
                                   &id,
                                   &iq);

    current_controller_check_limit(controller, id, iq);
    current_controller_ramp_setpoint(controller, dt);

    controller->current.measured.d += CURRENT_REPORT_FILTER_K *
                                      (id - controller->current.measured.d);
    controller->current.measured.q += CURRENT_REPORT_FILTER_K *
                                      (iq - controller->current.measured.q);

    current_controller_update_pi(controller, id, iq, dt);

    if (foc_controller_apply_voltage(foc_controller,
                                     controller->pi.output_voltage.d,
                                     controller->pi.output_voltage.q,
                                     pwm_phase) == 0U) {
        controller->error |= CURRENT_CONTROLLER_ERROR_FOC;
        current_controller_set_enabled(controller, 0U);
        return 0U;
    }

    return 1U;
}

void current_controller_get_status(const CurrentController *controller,
                                   CurrentController *status)
{
    if ((controller == 0) || (status == 0)) {
        return;
    }

    *status = *controller;
}

static void current_controller_check_limit(CurrentController *controller,
                                           float id,
                                           float iq)
{
    float limit = controller->config.current_limit;

    if ((id * id + iq * iq) > (limit * limit)) {
        controller->error |= CURRENT_CONTROLLER_ERROR_CURRENT_LIMIT;
    } else {
        controller->error &= ~CURRENT_CONTROLLER_ERROR_CURRENT_LIMIT;
    }
}

static void current_controller_clarke_park(const PhaseCurrentSample *sample,
                                           float phase_current_gain,
                                           float current_phase,
                                           float *id,
                                           float *iq)
{
    float ib = (float)sample->ib * phase_current_gain;
    float ic = (float)sample->ic * phase_current_gain;
    float i_alpha = -ib - ic;
    float i_beta = ONE_BY_SQRT3 * (ib - ic);
    float c = arm_cos_f32(current_phase);
    float s = arm_sin_f32(current_phase);

    *id = c * i_alpha + s * i_beta;
    *iq = c * i_beta - s * i_alpha;
}

static void current_controller_update_pi(CurrentController *controller,
                                         float id,
                                         float iq,
                                         float dt)
{
    float err_d = controller->current.ramped_setpoint.d - id;
    float err_q = controller->current.ramped_setpoint.q - iq;
    float vd = controller->pi.integral_voltage.d + err_d * controller->pi.kp;
    float vq = controller->pi.integral_voltage.q + err_q * controller->pi.kp;
    float v_mag = sqrtf(vd * vd + vq * vq);

    if (v_mag > controller->config.max_voltage_mod) {
        float scale = controller->config.max_voltage_mod / v_mag;
        vd *= scale;
        vq *= scale;
        controller->pi.integral_voltage.d *= CURRENT_INTEGRATOR_DECAY;
        controller->pi.integral_voltage.q *= CURRENT_INTEGRATOR_DECAY;
    } else {
        controller->pi.integral_voltage.d += err_d * controller->pi.ki * dt;
        controller->pi.integral_voltage.q += err_q * controller->pi.ki * dt;
    }

    controller->pi.output_voltage.d = vd;
    controller->pi.output_voltage.q = vq;
}

static void current_controller_ramp_setpoint(CurrentController *controller, float dt)
{
    float step = CURRENT_DEFAULT_SETPOINT_RAMP * dt;

    controller->current.ramped_setpoint.d = clamp_float(controller->current.setpoint.d,
                                                        controller->current.ramped_setpoint.d - step,
                                                        controller->current.ramped_setpoint.d + step);
    controller->current.ramped_setpoint.q = clamp_float(controller->current.setpoint.q,
                                                        controller->current.ramped_setpoint.q - step,
                                                        controller->current.ramped_setpoint.q + step);
}
