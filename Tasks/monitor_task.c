#include "monitor_task.h"

#include <stdio.h>
#include <string.h>

#include "motor_axis.h"
#include "gpio.h"
#include "main.h"
#include "power_stage.h"
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
    PowerStageStatus power_stage;
    MotorAxisStatus axis;
    GPIO_PinState nfault_pin = HAL_GPIO_ReadPin(nFAULT_GPIO_Port, nFAULT_Pin);

    power_stage_get_status(&power_stage);
    motor_axis_get_status(&axis);

    uint32_t adc_irq_rate_hz = power_stage.adc_irq_count - prev_adc_irq_count;
    uint32_t current_sample_rate_hz = power_stage.current_meas_count - prev_current_sample_count;
    prev_adc_irq_count = power_stage.adc_irq_count;
    prev_current_sample_count = power_stage.current_meas_count;

    int32_t phase_mrad = (int32_t)(axis.encoder.electrical_phase * 1000.0f);
    int32_t velocity_mrad_s = (int32_t)(axis.encoder.electrical_phase_vel * 1000.0f);
    int32_t voltage_mod_milli = (int32_t)(axis.open_loop.voltage_mod * 1000.0f);
    int32_t id_measured_ma = (int32_t)(axis.current.id_measured * 1000.0f);
    int32_t iq_measured_ma = (int32_t)(axis.current.iq_measured * 1000.0f);
    int32_t iq_target_ma = (int32_t)(axis.current.iq_setpoint * 1000.0f);
    int32_t iq_ramped_ma = (int32_t)(axis.current.iq_ramped_setpoint * 1000.0f);
    int32_t vd_mod_milli = (int32_t)(axis.current.vd_mod * 1000.0f);
    int32_t vq_mod_milli = (int32_t)(axis.current.vq_mod * 1000.0f);
    int32_t ia_ma = (int32_t)(axis.phase_current.ia * 1000.0f);
    int32_t ib_ma = (int32_t)(axis.phase_current.ib * 1000.0f);
    int32_t ic_ma = (int32_t)(axis.phase_current.ic * 1000.0f);
    int32_t ib_offset_ma = (int32_t)(power_stage.ib_offset * 1000.0f);
    int32_t ic_offset_ma = (int32_t)(power_stage.ic_offset * 1000.0f);
    int32_t vbus_mv = (int32_t)(power_stage.vbus_voltage * 1000.0f);

    int len = snprintf(msg, sizeof(msg),
                       "sys: nfault=%u mode=%lu out=%lu cal=%u loop=%lu miss=%lu adc_hz=%lu cur_hz=%lu\r\n",
                       (nfault_pin == GPIO_PIN_SET) ? 1U : 0U,
                       (unsigned long)axis.runtime.mode,
                       (unsigned long)axis.runtime.output_active,
                       power_stage.offset_calibrated,
                       (unsigned long)axis.runtime.loop_count,
                       (unsigned long)axis.runtime.deadline_miss_count,
                       (unsigned long)adc_irq_rate_hz,
                       (unsigned long)current_sample_rate_hz);
    monitor_task_send_line(msg, len);

    len = snprintf(msg, sizeof(msg),
                   "ctrl: open=%lu enc=%lu phase_mrad=%ld vel_mrad_s=%ld v_milli=%ld iq_sp_ma=%ld iq_ramp_ma=%ld id_ma=%ld iq_ma=%ld vd_milli=%ld vq_milli=%ld ierr=%lu\r\n",
                       (unsigned long)axis.open_loop.enabled,
                       (unsigned long)axis.encoder.ready,
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
                       "adc: dir=%u dc_cal=%lu vbus_mv=%ld vbus_raw=%lu raw=%lu/%lu off_ma=%ld/%ld iabc_ma=%ld/%ld/%ld enc=%ld/%ld pwm=%lu/%lu/%lu\r\n",
                       power_stage.counting_down,
                       (unsigned long)power_stage.dc_cal_count,
                       (long)vbus_mv,
                       (unsigned long)power_stage.vbus_raw,
                       (unsigned long)power_stage.ib_raw,
                       (unsigned long)power_stage.ic_raw,
                       (long)ib_offset_ma,
                       (long)ic_offset_ma,
                       (long)ia_ma,
                       (long)ib_ma,
                       (long)ic_ma,
                       (long)axis.encoder.shadow_count,
                       (long)axis.encoder.count_in_cpr,
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
