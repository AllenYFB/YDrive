#include "motor_axis.h"

#include "current_controller.h"
#include "foc.h"
#include "open_loop_controller.h"
#include "pwm_adc.h"

/* Watchdog for a stopped ADC/TIM interrupt chain.
 * This is not the FOC period; normal control timing is driven by ADC flags.
 */
#define CURRENT_MEAS_WATCHDOG_TIMEOUT_MS 10U
#define MOTOR_AXIS_PERIOD_SEC 0.000125f

osThreadId_t motorAxisTaskHandle;

typedef struct {
    volatile MotorAxisStatus status;
    volatile uint8_t output_active;
    volatile uint8_t control_cycle_finished;
    MotorAxisMode mode;
    OpenLoopController open_loop;
    CurrentController current;
    FocController foc;
} MotorAxis;

static MotorAxis motor_axis = {
    .control_cycle_finished = 1U,
    .mode = MOTOR_AXIS_MODE_IDLE,
};

static void motor_axis_init(void);
static void motor_axis_run_once(void);
static uint32_t motor_axis_update_controller(const PhaseCurrentSample *sample);
static uint32_t motor_axis_run_open_loop_voltage(void);
static uint32_t motor_axis_run_open_loop_current(const PhaseCurrentSample *sample);
static void motor_axis_update_output(uint32_t controller_active);
static void motor_axis_arm_output(void);
static void motor_axis_capture_status(void);
static void motor_axis_disarm(void);

void motor_axis_task(void *argument)
{
    (void)argument;
    motor_axis_init();

    for (;;) {
        uint32_t flags = osThreadFlagsWait(MOTOR_AXIS_CURRENT_MEAS_FLAG,
                                           osFlagsWaitAny,
                                           CURRENT_MEAS_WATCHDOG_TIMEOUT_MS);

        if ((flags & MOTOR_AXIS_CURRENT_MEAS_FLAG) == 0U) {
            if (motor_axis.output_active != 0U) {
                motor_axis_disarm();
            }
            continue;
        }

        motor_axis_run_once();
    }
}

void motor_axis_signal_current_meas_from_isr(void)
{
    /* A new real-current sample arrived before the previous sample produced PWM timings.
     * Once output is armed, this is a real control deadline miss and must disarm.
     */
    if ((motor_axis.output_active != 0U) && (motor_axis.control_cycle_finished == 0U)) {
        motor_axis.status.deadline_miss_count++;
        motor_axis_disarm();
    }

    /* This ADC sample now owns the PWM deadline until the control task finishes. */
    motor_axis.control_cycle_finished = 0U;

    if (motorAxisTaskHandle != 0) {
        (void)osThreadFlagsSet(motorAxisTaskHandle, MOTOR_AXIS_CURRENT_MEAS_FLAG);
    }
}

void motor_axis_get_status(MotorAxisStatus *status)
{
    if (status == 0) {
        return;
    }

    *status = motor_axis.status;
}

void motor_axis_set_open_loop_target(float voltage_mod, float electrical_phase_vel)
{
    open_loop_controller_set_target(&motor_axis.open_loop, voltage_mod, electrical_phase_vel);
}

void motor_axis_set_open_loop_enabled(uint32_t enable)
{
    if (enable != 0U) {
        motor_axis.mode = MOTOR_AXIS_MODE_OPEN_LOOP_VOLTAGE;
        open_loop_controller_set_enabled(&motor_axis.open_loop, 1U);
        current_controller_set_enabled(&motor_axis.current, 0U);
        /* The control task arms the output only after it has written the first PWM update.
         * This prevents a USB command from enabling deadline checks mid-sample.
         */
        motor_axis.output_active = 0U;
        motor_axis.control_cycle_finished = 1U;
    } else {
        motor_axis_disarm();
    }
}

void motor_axis_set_open_loop_current_target(float iq_setpoint, float electrical_phase_vel)
{
    open_loop_controller_set_target(&motor_axis.open_loop, 0.0f, electrical_phase_vel);
    current_controller_set_target(&motor_axis.current, 0.0f, iq_setpoint);
}

void motor_axis_set_open_loop_current_enabled(uint32_t enable)
{
    if (enable != 0U) {
        motor_axis.mode = MOTOR_AXIS_MODE_OPEN_LOOP_CURRENT;
        open_loop_controller_set_enabled(&motor_axis.open_loop, 1U);
        current_controller_clear_error(&motor_axis.current);
        current_controller_set_enabled(&motor_axis.current, 1U);
        motor_axis.output_active = 0U;
        motor_axis.control_cycle_finished = 1U;
    } else {
        motor_axis_disarm();
    }
}

static void motor_axis_init(void)
{
    pwm_adc_init();
    pwm_adc_start_timing();
    open_loop_controller_init(&motor_axis.open_loop);
    current_controller_init(&motor_axis.current);
    foc_controller_init(&motor_axis.foc);
}

static void motor_axis_run_once(void)
{
    PhaseCurrentSample sample;
    uint32_t controller_active;

    pwm_adc_get_phase_current_sample(&sample);

    motor_axis.status.latest_ia = sample.ia;
    motor_axis.status.latest_ib = sample.ib;
    motor_axis.status.latest_ic = sample.ic;

    controller_active = motor_axis_update_controller(&sample);
    motor_axis_update_output(controller_active);

    /* PWM timings for this sample have been calculated and written. */
    motor_axis.control_cycle_finished = 1U;
    motor_axis.status.loop_count++;
}

static uint32_t motor_axis_update_controller(const PhaseCurrentSample *sample)
{
    switch (motor_axis.mode) {
    case MOTOR_AXIS_MODE_OPEN_LOOP_VOLTAGE:
        return motor_axis_run_open_loop_voltage();

    case MOTOR_AXIS_MODE_OPEN_LOOP_CURRENT:
        return motor_axis_run_open_loop_current(sample);

    case MOTOR_AXIS_MODE_IDLE:
    default:
        return 0U;
    }
}

static uint32_t motor_axis_run_open_loop_voltage(void)
{
    return open_loop_controller_update(&motor_axis.open_loop,
                                       &motor_axis.foc,
                                       MOTOR_AXIS_PERIOD_SEC);
}

static uint32_t motor_axis_run_open_loop_current(const PhaseCurrentSample *sample)
{
    float current_phase;
    float pwm_phase;

    if (sample == 0) {
        return 0U;
    }

    if (open_loop_controller_update_phase(&motor_axis.open_loop,
                                          MOTOR_AXIS_PERIOD_SEC) == 0U) {
        return 0U;
    }

    current_phase = motor_axis.open_loop.electrical_phase;
    pwm_phase = wrap_pm_pi(current_phase +
                           1.5f * MOTOR_AXIS_PERIOD_SEC *
                               motor_axis.open_loop.electrical_phase_vel);

    return current_controller_update(&motor_axis.current,
                                     &motor_axis.foc,
                                     sample,
                                     current_phase,
                                     pwm_phase,
                                     MOTOR_AXIS_PERIOD_SEC);
}

static void motor_axis_update_output(uint32_t controller_active)
{
    PwmAdcStatus pwm_status;

    if (controller_active == 0U) {
        motor_axis_disarm();
        return;
    }

    pwm_adc_get_status(&pwm_status);

    if (pwm_status.offset_calibrated == 0U) {
        motor_axis_disarm();
        return;
    }

    motor_axis_capture_status();
    motor_axis_arm_output();
}

static void motor_axis_arm_output(void)
{
    pwm_adc_set_gate_enabled(1U);
    motor_axis.output_active = 1U;
}

static void motor_axis_capture_status(void)
{
    OpenLoopController open_loop;
    CurrentController current;
    FocController foc;
    open_loop_controller_get_status(&motor_axis.open_loop, &open_loop);
    current_controller_get_status(&motor_axis.current, &current);
    foc_controller_get_status(&motor_axis.foc, &foc);

    motor_axis.status.mode = motor_axis.mode;
    motor_axis.status.output_active = motor_axis.output_active;
    motor_axis.status.open_loop_enable = open_loop.enabled;
    motor_axis.status.open_loop_electrical_phase = open_loop.electrical_phase;
    motor_axis.status.open_loop_electrical_phase_vel = open_loop.electrical_phase_vel;
    motor_axis.status.open_loop_voltage_mod = open_loop.voltage_dq.d;
    motor_axis.status.current_id_setpoint = current.current_setpoint.d;
    motor_axis.status.current_iq_setpoint = current.current_setpoint.q;
    motor_axis.status.current_id_measured = current.current_measured.d;
    motor_axis.status.current_iq_measured = current.current_measured.q;
    motor_axis.status.current_vd_mod = current.voltage_mod.d;
    motor_axis.status.current_vq_mod = current.voltage_mod.q;
    motor_axis.status.current_error = current.error;
    motor_axis.status.pwm_a = foc.pwm_a;
    motor_axis.status.pwm_b = foc.pwm_b;
    motor_axis.status.pwm_c = foc.pwm_c;
}

static void motor_axis_disarm(void)
{
    motor_axis.output_active = 0U;
    motor_axis.mode = MOTOR_AXIS_MODE_IDLE;
    open_loop_controller_set_enabled(&motor_axis.open_loop, 0U);
    current_controller_set_enabled(&motor_axis.current, 0U);
    pwm_adc_set_gate_enabled(0U);
    pwm_adc_write_pwm_neutral();
    /* No control output is active, so there is no pending realtime PWM deadline. */
    motor_axis.control_cycle_finished = 1U;
    motor_axis_capture_status();
}
