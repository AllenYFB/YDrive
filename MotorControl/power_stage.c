#include "power_stage.h"

#include "adc.h"
#include "motor_axis.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"
#include "utils.h"

#define PWM_MIN_TICKS 1U
#define PWM_MAX_TICKS 3499U
#define DC_CAL_READY_SAMPLES 4096U
#define DC_CAL_FILTER_K 0.0005f
#define ADC_ZERO_CURRENT_COUNT 2048.0f
#define ADC_VREF 3.3f
#define ADC_COUNT_FULL_SCALE 4096.0f
#define VBUS_DIVIDER_RATIO 19.0f
#define CURRENT_SHUNT_DEFAULT_OHM 0.0005f
#define CURRENT_AMP_GAIN_DEFAULT 10.0f

typedef struct {
    float amp_per_adc_volt;
} PowerStageCurrentScale;

typedef struct {
    float ib_raw_amp;
    float ic_raw_amp;
    uint8_t ib_ready;
    uint8_t ic_ready;
} PowerStageCurrentSampleBuffer;

typedef struct {
    volatile PowerStageStatus status;
    volatile PhaseCurrentSample latest_current;
    PowerStageCurrentScale current_scale;
    PowerStageCurrentSampleBuffer current_sample;
} PowerStage;

static PowerStage power_stage;

static void power_stage_read_current_adc(ADC_HandleTypeDef *hadc);
static void power_stage_use_current_sample(ADC_HandleTypeDef *hadc);
static void power_stage_use_offset_sample(ADC_HandleTypeDef *hadc);
static void power_stage_publish_current(void);
static uint8_t power_stage_is_current_adc(ADC_HandleTypeDef *hadc);
static uint8_t power_stage_is_offset_window(void);
static float power_stage_adc_to_volt(uint32_t raw_adc);
static float power_stage_adc_to_current(uint32_t raw_adc);
static float power_stage_adc_to_vbus(uint32_t raw_adc);
static void power_stage_filter_offset(volatile float *offset, float raw_amp);

void power_stage_init(void)
{
    power_stage_set_current_scale(CURRENT_SHUNT_DEFAULT_OHM,
                                  CURRENT_AMP_GAIN_DEFAULT);
    power_stage_enable_output(0U);
    power_stage_write_neutral();
}

void power_stage_set_current_scale(float shunt_resistance_ohm, float amp_gain)
{
    if ((shunt_resistance_ohm <= 0.0f) || (amp_gain <= 0.0f)) {
        return;
    }

    power_stage.current_scale.amp_per_adc_volt =
        1.0f / (shunt_resistance_ohm * amp_gain);
}

void power_stage_start_timing(void)
{
    power_stage_write_neutral();

    (void)HAL_ADCEx_InjectedStart_IT(&hadc1);
    (void)HAL_ADCEx_InjectedStart_IT(&hadc2);
    (void)HAL_ADCEx_InjectedStart_IT(&hadc3);

    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    (void)HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

    power_stage_enable_output(0U);
}

void power_stage_start_offset_calibration(void)
{
    __disable_irq();
    power_stage.status.ib_offset = 0.0f;
    power_stage.status.ic_offset = 0.0f;
    power_stage.status.dc_cal_count = 0U;
    power_stage.status.offset_calibrated = POWER_STAGE_OFFSET_NOT_READY;
    power_stage.latest_current.ia = 0.0f;
    power_stage.latest_current.ib = 0.0f;
    power_stage.latest_current.ic = 0.0f;
    power_stage.current_sample.ib_ready = 0U;
    power_stage.current_sample.ic_ready = 0U;
    __enable_irq();
}

void power_stage_enable_output(uint32_t enable)
{
    if (enable != 0U) {
        __HAL_TIM_MOE_ENABLE(&htim1);
    } else {
        __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    }
}

void power_stage_write_neutral(void)
{
    power_stage_write_ticks(POWER_STAGE_PWM_CENTER_TICKS,
                            POWER_STAGE_PWM_CENTER_TICKS,
                            POWER_STAGE_PWM_CENTER_TICKS);
}

void power_stage_write_pwm(const PhasePwm *pwm)
{
    if (pwm == 0) {
        return;
    }

    power_stage_write_ticks(pwm->phase_a, pwm->phase_b, pwm->phase_c);
}

void power_stage_write_ticks(uint32_t phase_a_ticks, uint32_t phase_b_ticks, uint32_t phase_c_ticks)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                          clamp_u32(phase_a_ticks, PWM_MIN_TICKS, PWM_MAX_TICKS));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,
                          clamp_u32(phase_b_ticks, PWM_MIN_TICKS, PWM_MAX_TICKS));
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
                          clamp_u32(phase_c_ticks, PWM_MIN_TICKS, PWM_MAX_TICKS));
}

void power_stage_get_current(PhaseCurrentSample *sample)
{
    if (sample == 0) {
        return;
    }

    __disable_irq();
    *sample = power_stage.latest_current;
    __enable_irq();
}

void power_stage_get_status(PowerStageStatus *status)
{
    if (status == 0) {
        return;
    }

    *status = power_stage.status;
}

void power_stage_vbus_adc_callback(ADC_HandleTypeDef *hadc, uint8_t injected)
{
    uint32_t raw_adc;

    if (injected == 0U) {
        return;
    }

    raw_adc = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);

    power_stage.status.adc_irq_count++;
    power_stage.status.vbus_raw = raw_adc;
    power_stage.status.vbus_voltage = power_stage_adc_to_vbus(raw_adc);
}

void power_stage_current_adc_callback(ADC_HandleTypeDef *hadc, uint8_t injected)
{
    if (injected == 0U) {
        return;
    }

    power_stage.status.adc_irq_count++;
    power_stage.status.counting_down = power_stage_is_offset_window();

    if (power_stage_is_current_adc(hadc) == 0U) {
        return;
    }

    power_stage_read_current_adc(hadc);

    if (power_stage.status.counting_down == 0U) {
        power_stage_use_current_sample(hadc);
    } else {
        power_stage_use_offset_sample(hadc);
    }
}

static void power_stage_read_current_adc(ADC_HandleTypeDef *hadc)
{
    uint32_t raw_adc = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
    float raw_amp = power_stage_adc_to_current(raw_adc);

    if (hadc->Instance == ADC2) {
        power_stage.status.ib_raw = raw_adc;
        power_stage.current_sample.ib_raw_amp = raw_amp;
        return;
    }

    if (hadc->Instance == ADC3) {
        power_stage.status.ic_raw = raw_adc;
        power_stage.current_sample.ic_raw_amp = raw_amp;
    }
}

static void power_stage_use_current_sample(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC2) {
        power_stage.latest_current.ib =
            power_stage.current_sample.ib_raw_amp - power_stage.status.ib_offset;
        power_stage.current_sample.ib_ready = 1U;
        return;
    }

    if (hadc->Instance == ADC3) {
        power_stage.latest_current.ic =
            power_stage.current_sample.ic_raw_amp - power_stage.status.ic_offset;
        power_stage.current_sample.ic_ready = 1U;
    }

    power_stage_publish_current();
}

static void power_stage_use_offset_sample(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC2) {
        power_stage_filter_offset(&power_stage.status.ib_offset,
                                  power_stage.current_sample.ib_raw_amp);
    } else if (hadc->Instance == ADC3) {
        power_stage_filter_offset(&power_stage.status.ic_offset,
                                  power_stage.current_sample.ic_raw_amp);
    }

    if (power_stage.status.dc_cal_count < DC_CAL_READY_SAMPLES) {
        power_stage.status.dc_cal_count++;
    } else {
        power_stage.status.offset_calibrated = POWER_STAGE_OFFSET_READY;
    }
}

static void power_stage_publish_current(void)
{
    if ((power_stage.current_sample.ib_ready == 0U) ||
        (power_stage.current_sample.ic_ready == 0U)) {
        return;
    }

    power_stage.current_sample.ib_ready = 0U;
    power_stage.current_sample.ic_ready = 0U;
    power_stage.latest_current.ia = -power_stage.latest_current.ib - power_stage.latest_current.ic;

    power_stage.status.current_meas_count++;
    motor_axis_signal_current_meas_from_isr();
}

static uint8_t power_stage_is_current_adc(ADC_HandleTypeDef *hadc)
{
    return ((hadc->Instance == ADC2) || (hadc->Instance == ADC3)) ? 1U : 0U;
}

static uint8_t power_stage_is_offset_window(void)
{
    return ((htim1.Instance->CR1 & TIM_CR1_DIR) != 0U) ? 1U : 0U;
}

static float power_stage_adc_to_volt(uint32_t raw_adc)
{
    return (ADC_VREF / ADC_COUNT_FULL_SCALE) * (float)raw_adc;
}

static float power_stage_adc_to_current(uint32_t raw_adc)
{
    float adc_balanced = (float)raw_adc - ADC_ZERO_CURRENT_COUNT;
    float amp_out_volt = (ADC_VREF / ADC_COUNT_FULL_SCALE) * adc_balanced;

    return amp_out_volt * power_stage.current_scale.amp_per_adc_volt;
}

static float power_stage_adc_to_vbus(uint32_t raw_adc)
{
    return power_stage_adc_to_volt(raw_adc) * VBUS_DIVIDER_RATIO;
}

static void power_stage_filter_offset(volatile float *offset, float raw_amp)
{
    *offset += (raw_amp - *offset) * DC_CAL_FILTER_K;
}

