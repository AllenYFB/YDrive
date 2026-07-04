#include "pwm_adc.h"

#include "adc.h"
#include "motor_axis.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"
#include "utils.h"

#define PWM_MIN_TICKS 1U
#define PWM_MAX_TICKS 3499U
#define DC_CAL_FILTER_SHIFT 11U
#define DC_CAL_READY_SAMPLES 4096U

typedef struct {
    volatile PwmAdcStatus status;
    volatile PhaseCurrentSample latest_current;
    volatile uint8_t ib_raw_ready;
    volatile uint8_t ic_raw_ready;
    int32_t ib_offset_q8;
    int32_t ic_offset_q8;
} PwmAdc;

static PwmAdc pwm_adc;

static uint8_t pwm_adc_is_counting_down(void);
static uint8_t pwm_adc_read_phase_current_raw(ADC_HandleTypeDef *hadc);
static void pwm_adc_handle_current_window(ADC_HandleTypeDef *hadc);
static void pwm_adc_handle_dc_cal_window(ADC_HandleTypeDef *hadc);
static void pwm_adc_update_offset_filter(volatile int32_t *offset,
                                         int32_t *offset_q8,
                                         uint32_t raw_adc);

void pwm_adc_init(void)
{
    pwm_adc_set_gate_enabled(0U);
    pwm_adc_write_pwm_neutral();
}

void pwm_adc_write_pwm_neutral(void)
{
    pwm_adc_write_pwm_ticks(PWM_CENTER_TICKS, PWM_CENTER_TICKS, PWM_CENTER_TICKS);
}

void pwm_adc_write_pwm(const PwmTicks *pwm)
{
    if (pwm == 0) {
        return;
    }

    pwm_adc_write_pwm_ticks(pwm->phase_a, pwm->phase_b, pwm->phase_c);
}

void pwm_adc_set_gate_enabled(uint32_t enable)
{
    HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin,
                      (enable != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void pwm_adc_write_pwm_ticks(uint32_t phase_a_ticks, uint32_t phase_b_ticks, uint32_t phase_c_ticks)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                          clamp_u32(phase_a_ticks, PWM_MIN_TICKS, PWM_MAX_TICKS));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
                          clamp_u32(phase_b_ticks, PWM_MIN_TICKS, PWM_MAX_TICKS));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
                          clamp_u32(phase_c_ticks, PWM_MIN_TICKS, PWM_MAX_TICKS));
}

void pwm_adc_start_timing(void)
{
    pwm_adc_set_gate_enabled(0U);
    pwm_adc_write_pwm_neutral();

    (void)HAL_ADCEx_InjectedStart_IT(&hadc1);
    (void)HAL_ADCEx_InjectedStart_IT(&hadc2);
    (void)HAL_ADCEx_InjectedStart_IT(&hadc3);

    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}

void pwm_adc_get_phase_current_sample(PhaseCurrentSample *sample)
{
    if (sample == 0) {
        return;
    }

    __disable_irq();
    *sample = pwm_adc.latest_current;
    __enable_irq();
}

void pwm_adc_get_status(PwmAdcStatus *status)
{
    if (status == 0) {
        return;
    }

    *status = pwm_adc.status;
}

void pwm_adc_vbus_sense_adc_cb(ADC_HandleTypeDef *hadc, uint8_t injected)
{
    if (injected == 0U) {
        return;
    }

    pwm_adc.status.adc_irq_count++;
    pwm_adc.status.vbus_raw = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
}

void pwm_adc_trig_adc_cb(ADC_HandleTypeDef *hadc, uint8_t injected)
{
    if (injected == 0U) {
        return;
    }

    pwm_adc.status.adc_irq_count++;
    pwm_adc.status.counting_down = pwm_adc_is_counting_down();

    if (pwm_adc_read_phase_current_raw(hadc) == 0U) {
        return;
    }

    if (pwm_adc.status.counting_down == 0U) {
        pwm_adc_handle_current_window(hadc);
    } else {
        pwm_adc_handle_dc_cal_window(hadc);
    }
}

static uint8_t pwm_adc_is_counting_down(void)
{
    return ((htim1.Instance->CR1 & TIM_CR1_DIR) != 0U) ? 1U : 0U;
}

static uint8_t pwm_adc_read_phase_current_raw(ADC_HandleTypeDef *hadc)
{
    uint32_t raw_adc = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);

    if (hadc->Instance == ADC2) {
        pwm_adc.status.ib_raw = raw_adc;
        return 1U;
    }

    if (hadc->Instance == ADC3) {
        pwm_adc.status.ic_raw = raw_adc;
        return 1U;
    }

    return 0U;
}

static void pwm_adc_handle_current_window(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC2) {
        pwm_adc.latest_current.ib = (int32_t)pwm_adc.status.ib_raw - pwm_adc.status.ib_offset;
        pwm_adc.ib_raw_ready = 1U;
        return;
    }

    if (hadc->Instance == ADC3) {
        pwm_adc.latest_current.ic = (int32_t)pwm_adc.status.ic_raw - pwm_adc.status.ic_offset;
        pwm_adc.ic_raw_ready = 1U;
    }

    if ((pwm_adc.ib_raw_ready == 0U) || (pwm_adc.ic_raw_ready == 0U)) {
        return;
    }

    pwm_adc.ib_raw_ready = 0U;
    pwm_adc.ic_raw_ready = 0U;
    pwm_adc.latest_current.ia = -pwm_adc.latest_current.ib - pwm_adc.latest_current.ic;

    pwm_adc.status.current_meas_count++;
    motor_axis_signal_current_meas_from_isr();
}

static void pwm_adc_handle_dc_cal_window(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC2) {
        pwm_adc_update_offset_filter(&pwm_adc.status.ib_offset,
                                     &pwm_adc.ib_offset_q8,
                                     pwm_adc.status.ib_raw);
    } else if (hadc->Instance == ADC3) {
        pwm_adc_update_offset_filter(&pwm_adc.status.ic_offset,
                                     &pwm_adc.ic_offset_q8,
                                     pwm_adc.status.ic_raw);
    }

    if (pwm_adc.status.dc_cal_count < DC_CAL_READY_SAMPLES) {
        pwm_adc.status.dc_cal_count++;
    } else {
        pwm_adc.status.offset_calibrated = PWM_ADC_OFFSET_CALIBRATED;
    }
}

static void pwm_adc_update_offset_filter(volatile int32_t *offset,
                                         int32_t *offset_q8,
                                         uint32_t raw_adc)
{
    int32_t target_q8 = (int32_t)raw_adc << DC_CAL_FILTER_SHIFT;
    *offset_q8 += (target_q8 - *offset_q8) >> DC_CAL_FILTER_SHIFT;
    *offset = *offset_q8 >> DC_CAL_FILTER_SHIFT;
}

