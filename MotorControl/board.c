
#include "board.h"

#include "adc.h"
#include "foc.h"
#include "gpio.h"
#include "motor.h"
#include "tim.h"
#include "axis.h"

#include <stdint.h>

#define CURRENT_SENSE_MIN_VOLT 0.3f
#define CURRENT_SENSE_MAX_VOLT 3.0f
#define CURRENT_ADC_LOWER_BOUND ((uint32_t)((float)(1U << 12) * CURRENT_SENSE_MIN_VOLT / 3.3f))
#define CURRENT_ADC_UPPER_BOUND ((uint32_t)((float)(1U << 12) * CURRENT_SENSE_MAX_VOLT / 3.3f))

float vbus_voltage = 12.0f;
volatile uint32_t timestamp_ = 0;

static Iph_ABC_t current0;
static uint8_t counting_down_;
static uint8_t timer_dir_valid_;

static uint8_t adcs_ready(void)
{
    return ((ADC1->SR & ADC_SR_JEOC) != 0U) &&
           ((ADC2->SR & ADC_SR_JEOC) != 0U) &&
           ((ADC3->SR & ADC_SR_JEOC) != 0U);
}

static void vbus_sense_adc_cb(uint32_t adc_value)
{
    /* Vbus=adc*3.3*19/4096 */
    float voltage_scale = 3.3f * VBUS_S_DIVIDER_RATIO / 4096.0f;
    vbus_voltage = (float)adc_value * voltage_scale;
}

static float phase_current_from_adcval(uint32_t adc_value)
{
    if ((adc_value < CURRENT_ADC_LOWER_BOUND) ||
        (adc_value > CURRENT_ADC_UPPER_BOUND)) {
        set_error(ERROR_CURRENT_SENSE_SATURATION);
        return 0.0f;
    }

    /* adc_bal=adc-2048 
    因为电机电流是交流的，双向流动。
    运放输出偏置在 1.65V（ADC 中点 2048），这样正负电流都能被采样。
    减去 2048 后，adcval_bal 的符号就代表了电流方向。*/
    int32_t adcval_bal = (int32_t)adc_value - (1 << 11);

    /* Vamp=3.3/4096*adc_bal */
    float amp_out_volt = (3.3f / (float)(1 << 12)) * (float)adcval_bal;

    /* Vshunt=Vamp/Gain */
    float shunt_volt = amp_out_volt / PHASE_CURRENT_GAIN;

    /* Iphase=Vshunt/Rshunt */
    return shunt_volt / SHUNT_RESISTANCE;
}

static uint8_t fetch_and_reset_adcs(Iph_ABC_t *current)
{
    if (adcs_ready() == 0U) {
        return 0U;
    }

    vbus_sense_adc_cb(ADC1->JDR1);
    current->phB = phase_current_from_adcval(ADC2->JDR1);
    current->phC = phase_current_from_adcval(ADC3->JDR1);
    /* Ia=-Ib-Ic */
    current->phA = -current->phB - current->phC;

    ADC1->SR = ~(ADC_SR_JEOC | ADC_SR_JSTRT | ADC_SR_OVR);
    ADC2->SR = ~(ADC_SR_JEOC | ADC_SR_JSTRT | ADC_SR_OVR);
    ADC3->SR = ~(ADC_SR_JEOC | ADC_SR_JSTRT | ADC_SR_OVR);

    return 1U;
}

static uint8_t wait_for_next_injected_adc(void)
{
    uint32_t timeout = 10000U;

    while ((adcs_ready() == 0U) && (timeout > 0U)) {
        timeout--;
    }

    return adcs_ready();
}

void motor_control_start(void)
{
    HAL_GPIO_WritePin(EN_GATE_GPIO_Port, EN_GATE_Pin, GPIO_PIN_SET);

    TIM1->CCR1 = TIM_1_8_PERIOD_CLOCKS / 2U;
    TIM1->CCR2 = TIM_1_8_PERIOD_CLOCKS / 2U;
    TIM1->CCR3 = TIM_1_8_PERIOD_CLOCKS / 2U;

    (void)HAL_ADCEx_InjectedStart(&hadc1);
    (void)HAL_ADCEx_InjectedStart(&hadc2);
    (void)HAL_ADCEx_InjectedStart(&hadc3);

    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIM_Base_Start_IT(&htim1);

    HAL_NVIC_SetPriority(ControlLoop_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ControlLoop_IRQn);

    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
}

void motor_control_tim1_update(void)
{
    uint8_t counting_down = ((TIM1->CR1 & TIM_CR1_DIR) != 0U) ? 1U : 0U;

    if ((timer_dir_valid_ != 0U) && (counting_down_ == counting_down)) {
        set_error(ERROR_TIMER_UPDATE_MISSED);
        TIM1->CCR1 = TIM_1_8_PERIOD_CLOCKS / 2U;
        TIM1->CCR2 = TIM_1_8_PERIOD_CLOCKS / 2U;
        TIM1->CCR3 = TIM_1_8_PERIOD_CLOCKS / 2U;
        return;
    }

    counting_down_ = counting_down;
    timer_dir_valid_ = 1U;
    timestamp_ += TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1U); /* 3500*(2+1)=10500 */

    if (counting_down == 0U) {
        NVIC->STIR = (uint32_t)ControlLoop_IRQn;
    } else {
        TIM1->CCR1 = TIM_1_8_PERIOD_CLOCKS / 2U;
        TIM1->CCR2 = TIM_1_8_PERIOD_CLOCKS / 2U;
        TIM1->CCR3 = TIM_1_8_PERIOD_CLOCKS / 2U;
    }
}

void motor_control_run_control_loop(void)
{
    uint32_t timestamp = timestamp_;

    if (counting_down_ != 0U) {
        return;
    }

    if (fetch_and_reset_adcs(&current0) == 0U) {
        set_error(ERROR_BAD_TIMING);
        return;
    }

    current_meas_cb(timestamp, &current0);
    axis_control_loop_cb(timestamp);

    if ((wait_for_next_injected_adc() == 0U) ||
        (fetch_and_reset_adcs(&current0) == 0U)) {
        set_error(ERROR_BAD_TIMING);
        return;
    }

    dc_calib_cb(timestamp + TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1U), &current0); /* t+3500*(2+1)=t+10500 */
    pwm_update_cb(timestamp + 3U * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1U));     /* t+3*3500*(2+1)=t+31500 */

    if (timestamp_ != (timestamp + TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1U))) { /* t+3500*(2+1)=t+10500 */
        set_error(ERROR_CONTROL_DEADLINE_MISSED);
    }
}
