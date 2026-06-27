#include "pwm_adc.h"

#include "adc.h"
#include "control_loop.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"

#define PWM_NEUTRAL_TICKS 1750U
#define DC_CAL_FILTER_SHIFT 8U
#define DC_CAL_READY_SAMPLES 2048U

static volatile PwmAdcStatus pwm_adc_status;
static volatile PhaseCurrentSample latest_phase_current;
static volatile uint8_t ib_raw_ready;
static volatile uint8_t ic_raw_ready;
static int32_t ib_offset_q8;
static int32_t ic_offset_q8;

void pwm_adc_init(void)
{
    HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);
    pwm_adc_write_pwm_neutral();
}

void pwm_adc_write_pwm_neutral(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_NEUTRAL_TICKS);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_NEUTRAL_TICKS);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_NEUTRAL_TICKS);
}

void pwm_adc_start_timing(void)
{
    HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_RESET);
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
    *sample = latest_phase_current;
    __enable_irq();
}

void pwm_adc_get_status(PwmAdcStatus *status)
{
    if (status == 0) {
        return;
    }

    *status = pwm_adc_status;
}

void pwm_adc_vbus_sense_adc_cb(ADC_HandleTypeDef *hadc, uint8_t injected)
{
    if (injected == 0U) {
        return;
    }

    pwm_adc_status.adc_irq_count++;
    pwm_adc_status.vbus_raw = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
}

void pwm_adc_trig_adc_cb(ADC_HandleTypeDef *hadc, uint8_t injected)
{
    if (injected == 0U) {
        return;
    }

    pwm_adc_status.adc_irq_count++;

    uint8_t counting_down = ((htim1.Instance->CR1 & TIM_CR1_DIR) != 0U) ? 1U : 0U;
    uint8_t current_meas_not_dc_cal = (counting_down == 0U) ? 1U : 0U;
    pwm_adc_status.counting_down = counting_down;

    if (hadc->Instance == ADC2) {
        pwm_adc_status.ib_raw = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    } else if (hadc->Instance == ADC3) {
        pwm_adc_status.ic_raw = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    } else {
        return;
    }

    if (current_meas_not_dc_cal != 0U) {
        if (hadc->Instance == ADC2) {
            latest_phase_current.ib = (int32_t)pwm_adc_status.ib_raw - pwm_adc_status.ib_offset;
            ib_raw_ready = 1U;
            return;
        } else if (hadc->Instance == ADC3) {
            latest_phase_current.ic = (int32_t)pwm_adc_status.ic_raw - pwm_adc_status.ic_offset;
            ic_raw_ready = 1U;
        }

        if ((ib_raw_ready == 0U) || (ic_raw_ready == 0U)) {
            return;
        }

        ib_raw_ready = 0U;
        ic_raw_ready = 0U;

        latest_phase_current.ia = -latest_phase_current.ib - latest_phase_current.ic;

        pwm_adc_status.current_meas_count++;
        control_loop_signal_current_meas_from_isr();
    } else {
        if (hadc->Instance == ADC2) {
            int32_t target_q8 = (int32_t)pwm_adc_status.ib_raw << DC_CAL_FILTER_SHIFT;
            ib_offset_q8 += (target_q8 - ib_offset_q8) >> DC_CAL_FILTER_SHIFT;
            pwm_adc_status.ib_offset = ib_offset_q8 >> DC_CAL_FILTER_SHIFT;
        } else if (hadc->Instance == ADC3) {
            int32_t target_q8 = (int32_t)pwm_adc_status.ic_raw << DC_CAL_FILTER_SHIFT;
            ic_offset_q8 += (target_q8 - ic_offset_q8) >> DC_CAL_FILTER_SHIFT;
            pwm_adc_status.ic_offset = ic_offset_q8 >> DC_CAL_FILTER_SHIFT;
        }

        if (pwm_adc_status.dc_cal_count < DC_CAL_READY_SAMPLES) {
            pwm_adc_status.dc_cal_count++;
        } else {
            pwm_adc_status.offset_calibrated = 1U;
        }
    }
}
