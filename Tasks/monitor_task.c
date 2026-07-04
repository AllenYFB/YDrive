#include "monitor_task.h"

#include <stdio.h>
#include <string.h>

#include "motor_axis.h"
#include "gpio.h"
#include "main.h"
#include "pwm_adc.h"
#include "usbd_cdc_if.h"

#define MONITOR_PRINT_PERIOD_MS 1000U

static void monitor_task_send_boot_msg(void);
static void monitor_task_print_once(void);
static void monitor_task_send_line(const char *msg, int len);

void monitor_task(void *argument)
{
    (void)argument;

    monitor_task_send_boot_msg();

    for (;;) {
        monitor_task_print_once();
        osDelay(MONITOR_PRINT_PERIOD_MS);
    }
}

static void monitor_task_send_boot_msg(void)
{
    static const uint8_t boot_msg[] = "YDrive boot\r\n";

    osDelay(1000);
    (void)CDC_Transmit_FS((uint8_t *)boot_msg, sizeof(boot_msg) - 1U);
}

static void monitor_task_print_once(void)
{
    static uint32_t prev_adc_irq_count;
    static uint32_t prev_current_sample_count;
    char msg[160];
    PwmAdcStatus pwm_adc;
    MotorAxisStatus axis;
    GPIO_PinState nfault_pin = HAL_GPIO_ReadPin(nFAULT_GPIO_Port, nFAULT_Pin);

    pwm_adc_get_status(&pwm_adc);
    motor_axis_get_status(&axis);

    uint32_t adc_irq_rate_hz = pwm_adc.adc_irq_count - prev_adc_irq_count;
    uint32_t current_sample_rate_hz = pwm_adc.current_meas_count - prev_current_sample_count;
    prev_adc_irq_count = pwm_adc.adc_irq_count;
    prev_current_sample_count = pwm_adc.current_meas_count;

    int32_t phase_mrad = (int32_t)(axis.open_loop.electrical_phase * 1000.0f);
    int32_t velocity_mrad_s = (int32_t)(axis.open_loop.electrical_phase_vel * 1000.0f);
    int32_t voltage_mod_milli = (int32_t)(axis.open_loop.voltage_mod * 1000.0f);
    int32_t id_measured_ma = (int32_t)(axis.current.id_measured * 1000.0f);
    int32_t iq_measured_ma = (int32_t)(axis.current.iq_measured * 1000.0f);
    int32_t iq_target_ma = (int32_t)(axis.current.iq_setpoint * 1000.0f);
    int32_t iq_ramped_ma = (int32_t)(axis.current.iq_ramped_setpoint * 1000.0f);
    int32_t vd_mod_milli = (int32_t)(axis.current.vd_mod * 1000.0f);
    int32_t vq_mod_milli = (int32_t)(axis.current.vq_mod * 1000.0f);

    int len = snprintf(msg, sizeof(msg),
                       "sys: nfault=%u mode=%lu out=%lu cal=%u loop=%lu miss=%lu adc_hz=%lu cur_hz=%lu\r\n",
                       (nfault_pin == GPIO_PIN_SET) ? 1U : 0U,
                       (unsigned long)axis.runtime.mode,
                       (unsigned long)axis.runtime.output_active,
                       pwm_adc.offset_calibrated,
                       (unsigned long)axis.runtime.loop_count,
                       (unsigned long)axis.runtime.deadline_miss_count,
                       (unsigned long)adc_irq_rate_hz,
                       (unsigned long)current_sample_rate_hz);
    monitor_task_send_line(msg, len);

    len = snprintf(msg, sizeof(msg),
                   "ctrl: open=%lu phase_mrad=%ld vel_mrad_s=%ld v_milli=%ld iq_sp_ma=%ld iq_ramp_ma=%ld id_ma=%ld iq_ma=%ld vd_milli=%ld vq_milli=%ld ierr=%lu\r\n",
                       (unsigned long)axis.open_loop.enabled,
                       (long)phase_mrad,
                       (long)velocity_mrad_s,
                       (long)voltage_mod_milli,
                       (long)iq_target_ma,
                       (long)iq_ramped_ma,
                       (long)id_measured_ma,
                       (long)iq_measured_ma,
                       (long)vd_mod_milli,
                       (long)vq_mod_milli,
                       (unsigned long)axis.current.error);
    monitor_task_send_line(msg, len);

    len = snprintf(msg, sizeof(msg),
                       "adc: dir=%u dc_cal=%lu vbus=%lu raw=%lu/%lu off=%ld/%ld iabc=%ld/%ld/%ld pwm=%lu/%lu/%lu\r\n",
                       pwm_adc.counting_down,
                       (unsigned long)pwm_adc.dc_cal_count,
                       (unsigned long)pwm_adc.vbus_raw,
                       (unsigned long)pwm_adc.ib_raw,
                       (unsigned long)pwm_adc.ic_raw,
                       (long)pwm_adc.ib_offset,
                       (long)pwm_adc.ic_offset,
                       (long)axis.phase_current.ia,
                       (long)axis.phase_current.ib,
                       (long)axis.phase_current.ic,
                       (unsigned long)axis.pwm.a,
                       (unsigned long)axis.pwm.b,
                       (unsigned long)axis.pwm.c);
    monitor_task_send_line(msg, len);
}

static void monitor_task_send_line(const char *msg, int len)
{
    if ((msg == 0) || (len <= 0)) {
        return;
    }

    (void)CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
}
