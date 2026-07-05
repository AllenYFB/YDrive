#include "current_controller.h"

#include <math.h>

#include "arm_sin_cos_f32.h"

#define CURRENT_DEFAULT_P_GAIN 0.002f
#define CURRENT_DEFAULT_I_GAIN 1.0f
#define CURRENT_DEFAULT_LIMIT 5.0f
#define CURRENT_DEFAULT_MAX_MOD 0.08f
#define CURRENT_DEFAULT_SETPOINT_RAMP 0.1f
#define CURRENT_REPORT_FILTER_K 0.1f
#define CURRENT_INTEGRATOR_DECAY 0.99f

static void current_controller_check_current_limit(CurrentController *controller,
                                                  float id,
                                                  float iq);
static void current_controller_measure_current(const CurrentController *controller,
                                               const PhaseCurrentSample *sample,
                                               float current_phase,
                                               float *id,
                                               float *iq);
static void current_controller_run_pi(CurrentController *controller,
                                      float id,
                                      float iq,
                                      float dt);
static void current_controller_ramp_current(CurrentController *controller, float dt);
static void current_controller_reset_state(CurrentController *controller);

void current_controller_init(CurrentController *controller)
{
    if (controller == 0) {
        return;
    }

    controller->current_limit = CURRENT_DEFAULT_LIMIT;
    controller->max_voltage_mod = CURRENT_DEFAULT_MAX_MOD;
    controller->p_gain = CURRENT_DEFAULT_P_GAIN;
    controller->i_gain = CURRENT_DEFAULT_I_GAIN;
    current_controller_reset_state(controller);
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
        current_controller_reset_state(controller);
    }
}

void current_controller_set_target(CurrentController *controller,
                                   float id_setpoint,
                                   float iq_setpoint)
{
    if (controller == 0) {
        return;
    }

    controller->target_current.d = clamp_float(id_setpoint,
                                               -controller->current_limit,
                                               controller->current_limit);
    controller->target_current.q = clamp_float(iq_setpoint,
                                               -controller->current_limit,
                                               controller->current_limit);
}

void current_controller_set_gains(CurrentController *controller, float p_gain, float i_gain)
{
    if (controller == 0) {
        return;
    }

    controller->p_gain = clamp_float(p_gain, 0.0f, 1.0f);
    controller->i_gain = clamp_float(i_gain, 0.0f, 1000.0f);
}

void current_controller_set_limits(CurrentController *controller,
                                   float current_limit,
                                   float max_voltage_mod)
{
    if (controller == 0) {
        return;
    }

    controller->current_limit = clamp_float(current_limit, 0.0f, 200.0f);
    controller->max_voltage_mod = clamp_float(max_voltage_mod, 0.0f, 0.95f);
    current_controller_set_target(controller,
                                  controller->target_current.d,
                                  controller->target_current.q);
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

    current_controller_measure_current(controller, sample, current_phase, &id, &iq);

    current_controller_check_current_limit(controller, id, iq);
    current_controller_ramp_current(controller, dt);

    controller->measured_current.d += CURRENT_REPORT_FILTER_K *
                                      (id - controller->measured_current.d);
    controller->measured_current.q += CURRENT_REPORT_FILTER_K *
                                      (iq - controller->measured_current.q);

    current_controller_run_pi(controller, id, iq, dt);

    if (foc_controller_apply_voltage(foc_controller,
                                     controller->output_voltage.d,
                                     controller->output_voltage.q,
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

static void current_controller_check_current_limit(CurrentController *controller,
                                                  float id,
                                                  float iq)
{
    float limit = controller->current_limit;

    if ((id * id + iq * iq) > (limit * limit)) {
        controller->error |= CURRENT_CONTROLLER_ERROR_CURRENT_LIMIT;
    } else {
        controller->error &= ~CURRENT_CONTROLLER_ERROR_CURRENT_LIMIT;
    }
}

static void current_controller_measure_current(const CurrentController *controller,
                                               const PhaseCurrentSample *sample,
                                               float current_phase,
                                               float *id,
                                               float *iq)
{
    float ib = sample->ib;
    float ic = sample->ic;
    float i_alpha = -ib - ic;
    float i_beta = ONE_BY_SQRT3 * (ib - ic);
    float c = arm_cos_f32(current_phase);
    float s = arm_sin_f32(current_phase);

    *id = c * i_alpha + s * i_beta;
    *iq = c * i_beta - s * i_alpha;
}

static void current_controller_run_pi(CurrentController *controller,
                                      float id,
                                      float iq,
                                      float dt)
{
    float err_d = controller->ramped_current.d - id;
    float err_q = controller->ramped_current.q - iq;
    float vd = controller->integral_voltage.d + err_d * controller->p_gain;
    float vq = controller->integral_voltage.q + err_q * controller->p_gain;
    float v_mag = sqrtf(vd * vd + vq * vq);

    if (v_mag > controller->max_voltage_mod) {
        float scale = controller->max_voltage_mod / v_mag;
        vd *= scale;
        vq *= scale;
        controller->integral_voltage.d *= CURRENT_INTEGRATOR_DECAY;
        controller->integral_voltage.q *= CURRENT_INTEGRATOR_DECAY;
    } else {
        controller->integral_voltage.d += err_d * controller->i_gain * dt;
        controller->integral_voltage.q += err_q * controller->i_gain * dt;
    }

    controller->output_voltage.d = vd;
    controller->output_voltage.q = vq;
}

static void current_controller_ramp_current(CurrentController *controller, float dt)
{
    float step = CURRENT_DEFAULT_SETPOINT_RAMP * dt;

    controller->ramped_current.d = clamp_float(controller->target_current.d,
                                               controller->ramped_current.d - step,
                                               controller->ramped_current.d + step);
    controller->ramped_current.q = clamp_float(controller->target_current.q,
                                               controller->ramped_current.q - step,
                                               controller->ramped_current.q + step);
}

static void current_controller_reset_state(CurrentController *controller)
{
    controller->target_current.d = 0.0f;
    controller->target_current.q = 0.0f;
    controller->ramped_current.d = 0.0f;
    controller->ramped_current.q = 0.0f;
    controller->measured_current.d = 0.0f;
    controller->measured_current.q = 0.0f;
    controller->integral_voltage.d = 0.0f;
    controller->integral_voltage.q = 0.0f;
    controller->output_voltage.d = 0.0f;
    controller->output_voltage.q = 0.0f;
}
