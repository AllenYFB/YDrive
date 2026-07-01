#include "control_loop.h"

#include "foc.h"
#include "open_loop_controller.h"
#include "pwm_adc.h"

#define CURRENT_MEAS_TIMEOUT_MS 10U
#define CONTROL_LOOP_PERIOD_SEC 0.000125f

osThreadId_t controlLoopTaskHandle;

static volatile ControlLoopStatus control_loop_status;

static void control_loop_do_updates(void);
static void encoder_update(void);
static void sensorless_estimator_update(void);
static void controller_update(void);
static void motor_update(void);

void control_loop_signal_current_meas_from_isr(void)
{
    if (controlLoopTaskHandle != 0) {
        (void)osThreadFlagsSet(controlLoopTaskHandle, CONTROL_LOOP_CURRENT_MEAS_FLAG);
    }
}

void control_loop_get_status(ControlLoopStatus *status)
{
    if (status == 0) {
        return;
    }

    *status = control_loop_status;
}

void ControlLoopTask(void *argument)
{
    (void)argument;
    open_loop_controller_init();

    for (;;) {
        uint32_t flags = osThreadFlagsWait(CONTROL_LOOP_CURRENT_MEAS_FLAG,
                                           osFlagsWaitAny,
                                           CURRENT_MEAS_TIMEOUT_MS);

        if ((flags & CONTROL_LOOP_CURRENT_MEAS_FLAG) == 0U) {
            control_loop_status.missed_samples++;
            continue;
        }

        PhaseCurrentSample sample;
        pwm_adc_get_phase_current_sample(&sample);

        control_loop_status.latest_ia = sample.ia;
        control_loop_status.latest_ib = sample.ib;
        control_loop_status.latest_ic = sample.ic;

        control_loop_do_updates();
        control_loop_status.loop_count++;
    }
}

static void control_loop_do_updates(void)
{
    encoder_update();
    sensorless_estimator_update();
    controller_update();
    motor_update();
}

static void encoder_update(void)
{
}

static void sensorless_estimator_update(void)
{
}

static void controller_update(void)
{
}

static void motor_update(void)
{
    PwmAdcStatus pwm_status;
    pwm_adc_get_status(&pwm_status);

    if (pwm_status.offset_calibrated == 0U) {
        open_loop_controller_enable(0U);
        pwm_adc_set_gate_enabled(0U);
        pwm_adc_write_pwm_neutral();
        return;
    }

    open_loop_controller_update(CONTROL_LOOP_PERIOD_SEC);

    OpenLoopStatus open_loop;
    FocStatus foc;
    open_loop_controller_get_status(&open_loop);
    foc_get_status(&foc);

    control_loop_status.open_loop_enable = open_loop.enabled;
    control_loop_status.open_loop_phase = open_loop.phase;
    control_loop_status.open_loop_phase_vel = open_loop.phase_vel;
    control_loop_status.open_loop_voltage = open_loop.vdq_setpoint.d;
    control_loop_status.pwm_a = foc.pwm_a;
    control_loop_status.pwm_b = foc.pwm_b;
    control_loop_status.pwm_c = foc.pwm_c;

    if (open_loop.enabled != 0U) {
        pwm_adc_set_gate_enabled(1U);
    } else {
        pwm_adc_set_gate_enabled(0U);
        pwm_adc_write_pwm_neutral();
    }
}
