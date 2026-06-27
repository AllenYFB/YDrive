#include "control_loop.h"

#include "pwm_adc.h"
#include "tim.h"

#define CURRENT_MEAS_TIMEOUT_MS 10U

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
    /* Placeholder for encoder sampling and interpolation. */
}

static void sensorless_estimator_update(void)
{
    /* Placeholder for sensorless estimator update. */
}

static void controller_update(void)
{
    /* Placeholder for position/velocity/current target update. */
}

static void motor_update(void)
{
    /*
     * Placeholder for Park, PI current loop, inverse Park, and SVM.
     * Keep the bridge command at neutral until the bring-up checks pass.
     */
    pwm_adc_write_pwm_neutral();
}
