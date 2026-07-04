#include "current_controller.h"

#include <math.h>

#include "arm_sin_cos_f32.h"

#define CURRENT_DEFAULT_PHASE_GAIN 0.01f
#define CURRENT_DEFAULT_P_GAIN 0.002f
#define CURRENT_DEFAULT_I_GAIN 1.0f
#define CURRENT_DEFAULT_LIMIT 5.0f
#define CURRENT_DEFAULT_MAX_MOD 0.08f
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

void current_controller_init(CurrentController *controller)
{
    if (controller == 0) {
        return;
    }

    controller->phase_current_gain = CURRENT_DEFAULT_PHASE_GAIN;
    controller->p_gain = CURRENT_DEFAULT_P_GAIN;
    controller->i_gain = CURRENT_DEFAULT_I_GAIN;
    controller->current_limit = CURRENT_DEFAULT_LIMIT;
    controller->max_voltage_mod = CURRENT_DEFAULT_MAX_MOD;
    controller->current_setpoint.d = 0.0f;
    controller->current_setpoint.q = 0.0f;
    controller->current_measured.d = 0.0f;
    controller->current_measured.q = 0.0f;
    controller->voltage_integrator.d = 0.0f;
    controller->voltage_integrator.q = 0.0f;
    controller->voltage_mod.d = 0.0f;
    controller->voltage_mod.q = 0.0f;
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
        controller->current_setpoint.d = 0.0f;
        controller->current_setpoint.q = 0.0f;
        controller->voltage_integrator.d = 0.0f;
        controller->voltage_integrator.q = 0.0f;
        controller->voltage_mod.d = 0.0f;
        controller->voltage_mod.q = 0.0f;
    }
}

void current_controller_set_target(CurrentController *controller,
                                   float id_setpoint,
                                   float iq_setpoint)
{
    if (controller == 0) {
        return;
    }

    controller->current_setpoint.d = clamp_float(id_setpoint,
                                                 -controller->current_limit,
                                                 controller->current_limit);
    controller->current_setpoint.q = clamp_float(iq_setpoint,
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
                                   controller->phase_current_gain,
                                   current_phase,
                                   &id,
                                   &iq);

    current_controller_check_limit(controller, id, iq);

    controller->current_measured.d += CURRENT_REPORT_FILTER_K *
                                      (id - controller->current_measured.d);
    controller->current_measured.q += CURRENT_REPORT_FILTER_K *
                                      (iq - controller->current_measured.q);

    current_controller_update_pi(controller, id, iq, dt);

    if (foc_controller_apply_voltage(foc_controller,
                                     controller->voltage_mod.d,
                                     controller->voltage_mod.q,
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
    float limit = controller->current_limit;

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
    float err_d = controller->current_setpoint.d - id;
    float err_q = controller->current_setpoint.q - iq;
    float vd = controller->voltage_integrator.d + err_d * controller->p_gain;
    float vq = controller->voltage_integrator.q + err_q * controller->p_gain;
    float v_mag = sqrtf(vd * vd + vq * vq);

    if (v_mag > controller->max_voltage_mod) {
        float scale = controller->max_voltage_mod / v_mag;
        vd *= scale;
        vq *= scale;
        controller->voltage_integrator.d *= CURRENT_INTEGRATOR_DECAY;
        controller->voltage_integrator.q *= CURRENT_INTEGRATOR_DECAY;
    } else {
        controller->voltage_integrator.d += err_d * controller->i_gain * dt;
        controller->voltage_integrator.q += err_q * controller->i_gain * dt;
    }

    controller->voltage_mod.d = vd;
    controller->voltage_mod.q = vq;
}
