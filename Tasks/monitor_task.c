#include "monitor_task.h"

#include <stdio.h>
#include <string.h>

#include "motor_axis.h"
#include "gpio.h"
#include "main.h"
#include "pwm_adc.h"
#include "usbd_cdc_if.h"

#define MONITOR_PRINT_PERIOD_MS 1000U

static void monitor_task_init_hardware(void);
static void monitor_task_print_once(void);

void monitor_task(void *argument)
{
    (void)argument;

    monitor_task_init_hardware();

    for (;;) {
        monitor_task_print_once();
        osDelay(MONITOR_PRINT_PERIOD_MS);
    }
}

static void monitor_task_init_hardware(void)
{
    static const uint8_t boot_msg[] = "YDrive boot\r\n";

    HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);
    osDelay(1000);
    (void)CDC_Transmit_FS((uint8_t *)boot_msg, sizeof(boot_msg) - 1U);

    pwm_adc_init();
    pwm_adc_start_timing();
}

static void monitor_task_print_once(void)
{
    static uint32_t last_adc_count;
    static uint32_t last_current_count;
    char msg[224];
    PwmAdcStatus pwm_adc_status;
    MotorAxisStatus control_status;
    GPIO_PinState nfault_state = HAL_GPIO_ReadPin(nFAULT_GPIO_Port, nFAULT_Pin);

    pwm_adc_get_status(&pwm_adc_status);
    motor_axis_get_status(&control_status);

    uint32_t adc_hz = pwm_adc_status.adc_irq_count - last_adc_count;
    uint32_t current_hz = pwm_adc_status.current_meas_count - last_current_count;
    last_adc_count = pwm_adc_status.adc_irq_count;
    last_current_count = pwm_adc_status.current_meas_count;

    int32_t phase_mrad = (int32_t)(control_status.open_loop_electrical_phase * 1000.0f);
    int32_t vel_mrad_s = (int32_t)(control_status.open_loop_electrical_phase_vel * 1000.0f);
    int32_t v_milli = (int32_t)(control_status.open_loop_voltage_mod * 1000.0f);

    int len = snprintf(msg, sizeof(msg),
                       "nFAULT=%u adc_irq=%lu current=%lu dc_cal=%lu dir=%u axis=%lu deadline=%lu mode=%lu out=%lu cal=%u open=%lu phase_mrad=%ld vel_mrad_s=%ld v_milli=%ld pwm=%lu/%lu/%lu vbus=%lu ib=%lu ic=%lu off_b=%ld off_c=%ld ia=%ld ibc=%ld icc=%ld\r\n",
                       (nfault_state == GPIO_PIN_SET) ? 1U : 0U,
                       (unsigned long)adc_hz,
                       (unsigned long)current_hz,
                       (unsigned long)pwm_adc_status.dc_cal_count,
                       pwm_adc_status.counting_down,
                       (unsigned long)control_status.loop_count,
                       (unsigned long)control_status.deadline_miss_count,
                       (unsigned long)control_status.mode,
                       (unsigned long)control_status.output_active,
                       pwm_adc_status.offset_calibrated,
                       (unsigned long)control_status.open_loop_enable,
                       (long)phase_mrad,
                       (long)vel_mrad_s,
                       (long)v_milli,
                       (unsigned long)control_status.pwm_a,
                       (unsigned long)control_status.pwm_b,
                       (unsigned long)control_status.pwm_c,
                       (unsigned long)pwm_adc_status.vbus_raw,
                       (unsigned long)pwm_adc_status.ib_raw,
                       (unsigned long)pwm_adc_status.ic_raw,
                       (long)pwm_adc_status.ib_offset,
                       (long)pwm_adc_status.ic_offset,
                       (long)control_status.latest_ia,
                       (long)control_status.latest_ib,
                       (long)control_status.latest_ic);

    if (len > 0) {
        (void)CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
    }
}

