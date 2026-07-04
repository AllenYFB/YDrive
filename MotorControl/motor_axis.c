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
#define MOTOR_AXIS_PWM_PHASE_ADVANCE_PERIODS 1.5f

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
static uint32_t motor_axis_run_voltage_mode(void);
static uint32_t motor_axis_run_current_mode(const PhaseCurrentSample *sample);
static void motor_axis_update_output(uint32_t controller_active);
static uint32_t motor_axis_output_ready(const PwmAdcStatus *pwm_status);
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
    if ((motor_axis.output_active != 0U) &&
        (motor_axis.control_cycle_finished == 0U)) {
        motor_axis.status.runtime.deadline_miss_count++;
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

void motor_axis_start_open_voltage(float voltage_mod, float electrical_phase_vel)
{
    open_loop_controller_set_target(&motor_axis.open_loop, voltage_mod, electrical_phase_vel);
    motor_axis.mode = MOTOR_AXIS_MODE_OPEN_LOOP_VOLTAGE;
    open_loop_controller_set_enabled(&motor_axis.open_loop, 1U);
    current_controller_set_enabled(&motor_axis.current, 0U);
    /* The control task arms the output only after it has written the first PWM update.
     * This prevents a USB command from enabling deadline checks mid-sample.
     */
    motor_axis.output_active = 0U;
    motor_axis.control_cycle_finished = 1U;
}

void motor_axis_start_open_current(float iq_setpoint, float electrical_phase_vel)
{
    open_loop_controller_set_target(&motor_axis.open_loop, 0.0f, electrical_phase_vel);
    current_controller_set_target(&motor_axis.current, 0.0f, iq_setpoint);
    motor_axis.mode = MOTOR_AXIS_MODE_OPEN_LOOP_CURRENT;
    open_loop_controller_set_enabled(&motor_axis.open_loop, 1U);
    current_controller_clear_error(&motor_axis.current);
    current_controller_set_enabled(&motor_axis.current, 1U);
    motor_axis.output_active = 0U;
    motor_axis.control_cycle_finished = 1U;
}

void motor_axis_stop(void)
{
    motor_axis_disarm();
}

void motor_axis_set_current_pi(float p_gain, float i_gain)
{
    current_controller_set_gains(&motor_axis.current, p_gain, i_gain);
    motor_axis_capture_status();
}

void motor_axis_set_current_config(float phase_current_gain,
                                   float current_limit,
                                   float max_voltage_mod)
{
    current_controller_set_phase_current_gain(&motor_axis.current, phase_current_gain);
    current_controller_set_limits(&motor_axis.current, current_limit, max_voltage_mod);
    motor_axis_capture_status();
}

static void motor_axis_init(void)
{
    pwm_adc_init();
    pwm_adc_start_timing();
    open_loop_controller_init(&motor_axis.open_loop);
    current_controller_init(&motor_axis.current);
    foc_controller_init(&motor_axis.foc);
    motor_axis_capture_status();
}

static void motor_axis_run_once(void)
{
    PhaseCurrentSample sample;
    uint32_t controller_active;

    pwm_adc_get_phase_current_sample(&sample);

    motor_axis.status.phase_current.ia = sample.ia;
    motor_axis.status.phase_current.ib = sample.ib;
    motor_axis.status.phase_current.ic = sample.ic;

    controller_active = motor_axis_update_controller(&sample);
    motor_axis_update_output(controller_active);

    /* PWM timings for this sample have been calculated and written. */
    motor_axis.control_cycle_finished = 1U;
    motor_axis.status.runtime.loop_count++;
}

static uint32_t motor_axis_update_controller(const PhaseCurrentSample *sample)
{
    switch (motor_axis.mode) {
    case MOTOR_AXIS_MODE_OPEN_LOOP_VOLTAGE:
        return motor_axis_run_voltage_mode();

    case MOTOR_AXIS_MODE_OPEN_LOOP_CURRENT:
        return motor_axis_run_current_mode(sample);

    case MOTOR_AXIS_MODE_IDLE:
    default:
        return 0U;
    }
}

static uint32_t motor_axis_run_voltage_mode(void)
{
    return open_loop_controller_update(&motor_axis.open_loop,
                                       &motor_axis.foc,
                                       MOTOR_AXIS_PERIOD_SEC);
}

static uint32_t motor_axis_run_current_mode(const PhaseCurrentSample *sample)
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
                           MOTOR_AXIS_PWM_PHASE_ADVANCE_PERIODS *
                               MOTOR_AXIS_PERIOD_SEC *
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

    if (motor_axis_output_ready(&pwm_status) == 0U) {
        motor_axis_disarm();
        return;
    }

    motor_axis_capture_status();
    motor_axis_arm_output();
}

static uint32_t motor_axis_output_ready(const PwmAdcStatus *pwm_status)
{
    if (pwm_status == 0) {
        return 0U;
    }

    return (pwm_status->offset_calibrated == PWM_ADC_OFFSET_CALIBRATED) ? 1U : 0U;
}

static void motor_axis_arm_output(void)
{
    pwm_adc_set_gate_enabled(1U);
    motor_axis.output_active = 1U;
    motor_axis.status.runtime.output_active = motor_axis.output_active;
}

static void motor_axis_capture_status(void)
{
    OpenLoopController open_loop;
    CurrentController current;
    FocController foc;
    open_loop_controller_get_status(&motor_axis.open_loop, &open_loop);
    current_controller_get_status(&motor_axis.current, &current);
    foc_controller_get_status(&motor_axis.foc, &foc);

    motor_axis.status.runtime.mode = motor_axis.mode;
    motor_axis.status.runtime.output_active = motor_axis.output_active;
    motor_axis.status.open_loop.enabled = open_loop.enabled;
    motor_axis.status.open_loop.electrical_phase = open_loop.electrical_phase;
    motor_axis.status.open_loop.electrical_phase_vel = open_loop.electrical_phase_vel;
    motor_axis.status.open_loop.voltage_mod = open_loop.voltage_dq.d;
    motor_axis.status.current.id_setpoint = current.target_current.d;
    motor_axis.status.current.iq_setpoint = current.target_current.q;
    motor_axis.status.current.id_ramped_setpoint = current.ramped_current.d;
    motor_axis.status.current.iq_ramped_setpoint = current.ramped_current.q;
    motor_axis.status.current.id_measured = current.measured_current.d;
    motor_axis.status.current.iq_measured = current.measured_current.q;
    motor_axis.status.current.vd_mod = current.output_voltage.d;
    motor_axis.status.current.vq_mod = current.output_voltage.q;
    motor_axis.status.current.phase_gain = current.phase_current_gain;
    motor_axis.status.current.p_gain = current.p_gain;
    motor_axis.status.current.i_gain = current.i_gain;
    motor_axis.status.current.limit = current.current_limit;
    motor_axis.status.current.max_voltage_mod = current.max_voltage_mod;
    motor_axis.status.current.error = current.error;
    motor_axis.status.pwm.a = foc.pwm_a;
    motor_axis.status.pwm.b = foc.pwm_b;
    motor_axis.status.pwm.c = foc.pwm_c;
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
