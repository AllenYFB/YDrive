#ifndef MOTOR_AXIS_H
#define MOTOR_AXIS_H

#include <stdint.h>
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_AXIS_CURRENT_MEAS_FLAG (1UL << 0)

typedef enum {
    MOTOR_AXIS_MODE_IDLE = 0,
    MOTOR_AXIS_MODE_OPEN_LOOP_VOLTAGE = 1,
} MotorAxisMode;

typedef struct {
    uint32_t loop_count;
    uint32_t missed_samples;
    uint32_t deadline_miss_count;
    MotorAxisMode mode;
    uint32_t output_active;
    int32_t latest_ia;
    int32_t latest_ib;
    int32_t latest_ic;
    uint32_t open_loop_enable;
    float open_loop_electrical_phase;
    float open_loop_electrical_phase_vel;
    float open_loop_voltage_mod;
    uint32_t pwm_a;
    uint32_t pwm_b;
    uint32_t pwm_c;
} MotorAxisStatus;

extern osThreadId_t motorAxisTaskHandle;

void motor_axis_task(void *argument);
void motor_axis_signal_current_meas_from_isr(void);
void motor_axis_get_status(MotorAxisStatus *status);
void motor_axis_set_open_loop_target(float voltage_mod, float electrical_phase_vel);
void motor_axis_set_open_loop_enabled(uint32_t enable);

#ifdef __cplusplus
}
#endif

#endif
