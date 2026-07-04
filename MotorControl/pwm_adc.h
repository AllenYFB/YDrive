#ifndef PWM_ADC_H
#define PWM_ADC_H

#include <stdint.h>
#include "adc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PWM_PERIOD_TICKS 3500U
#define PWM_CENTER_TICKS 1750U
#define PWM_ADC_OFFSET_NOT_CALIBRATED 0U
#define PWM_ADC_OFFSET_CALIBRATED 1U

typedef struct {
    uint32_t adc_irq_count;
    uint32_t current_meas_count;
    uint32_t dc_cal_count;
    uint32_t vbus_raw;
    uint32_t ib_raw;
    uint32_t ic_raw;
    int32_t ib_offset;
    int32_t ic_offset;
    uint8_t counting_down;
    uint8_t offset_calibrated;
} PwmAdcStatus;

typedef struct {
    int32_t ia;
    int32_t ib;
    int32_t ic;
} PhaseCurrentSample;

typedef struct {
    uint32_t phase_a;
    uint32_t phase_b;
    uint32_t phase_c;
} PwmTicks;

void pwm_adc_init(void);
void pwm_adc_start_timing(void);
void pwm_adc_set_gate_enabled(uint32_t enable);
void pwm_adc_write_pwm_neutral(void);
void pwm_adc_write_pwm(const PwmTicks *pwm);
void pwm_adc_write_pwm_ticks(uint32_t phase_a_ticks, uint32_t phase_b_ticks, uint32_t phase_c_ticks);
void pwm_adc_get_phase_current_sample(PhaseCurrentSample *sample);
void pwm_adc_get_status(PwmAdcStatus *status);
void pwm_adc_vbus_sense_adc_cb(ADC_HandleTypeDef *hadc, uint8_t injected);
void pwm_adc_trig_adc_cb(ADC_HandleTypeDef *hadc, uint8_t injected);

#ifdef __cplusplus
}
#endif

#endif
