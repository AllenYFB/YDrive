#ifndef PWM_ADC_H
#define PWM_ADC_H

#include <stdint.h>
#include "adc.h"

#ifdef __cplusplus
extern "C" {
#endif

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

void pwm_adc_init(void);
void pwm_adc_start_timing(void);
void pwm_adc_write_pwm_neutral(void);
void pwm_adc_get_phase_current_sample(PhaseCurrentSample *sample);
void pwm_adc_get_status(PwmAdcStatus *status);
void pwm_adc_vbus_sense_adc_cb(ADC_HandleTypeDef *hadc, uint8_t injected);
void pwm_adc_trig_adc_cb(ADC_HandleTypeDef *hadc, uint8_t injected);

#ifdef __cplusplus
}
#endif

#endif
