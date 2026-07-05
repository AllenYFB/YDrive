#ifndef POWER_STAGE_H
#define POWER_STAGE_H

#include <stdint.h>
#include "adc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define POWER_STAGE_PWM_PERIOD_TICKS 3500U
#define POWER_STAGE_PWM_CENTER_TICKS 1750U
#define POWER_STAGE_OFFSET_NOT_READY 0U
#define POWER_STAGE_OFFSET_READY 1U

typedef struct {
    uint32_t adc_irq_count;
    uint32_t current_meas_count;
    uint32_t dc_cal_count;
    uint32_t vbus_raw;
    float vbus_voltage;
    uint32_t ib_raw;
    uint32_t ic_raw;
    float ib_offset;
    float ic_offset;
    uint8_t counting_down;
    uint8_t offset_calibrated;
} PowerStageStatus;

typedef struct {
    float ia;
    float ib;
    float ic;
} PhaseCurrentSample;

typedef struct {
    uint32_t phase_a;
    uint32_t phase_b;
    uint32_t phase_c;
} PhasePwm;

void power_stage_init(void);
void power_stage_start_timing(void);
void power_stage_set_current_scale(float shunt_resistance_ohm, float amp_gain);
void power_stage_start_offset_calibration(void);
void power_stage_enable_output(uint32_t enable);
void power_stage_write_neutral(void);
void power_stage_write_pwm(const PhasePwm *pwm);
void power_stage_write_ticks(uint32_t phase_a_ticks, uint32_t phase_b_ticks, uint32_t phase_c_ticks);
void power_stage_get_current(PhaseCurrentSample *sample);
void power_stage_get_status(PowerStageStatus *status);
void power_stage_vbus_adc_callback(ADC_HandleTypeDef *hadc, uint8_t injected);
void power_stage_current_adc_callback(ADC_HandleTypeDef *hadc, uint8_t injected);

#ifdef __cplusplus
}
#endif

#endif
