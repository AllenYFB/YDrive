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
    MOTOR_AXIS_MODE_OPEN_LOOP_CURRENT_ALIGN = 2,
    MOTOR_AXIS_MODE_OPEN_LOOP_CURRENT_RAMP = 3,
    MOTOR_AXIS_MODE_OPEN_LOOP_CURRENT_RUN = 4,
} MotorAxisMode;

typedef struct {
    uint32_t loop_count;
    uint32_t deadline_miss_count;
    MotorAxisMode mode;
    uint32_t output_active;
} MotorAxisRuntimeStatus;

typedef struct {
    int32_t ia;
    int32_t ib;
    int32_t ic;
} MotorAxisPhaseCurrentStatus;

typedef struct {
    uint32_t enabled;
    float electrical_phase;
    float electrical_phase_vel;
    float voltage_mod;
} MotorAxisOpenLoopStatus;

typedef struct {
    float id_setpoint;
    float iq_setpoint;
    float id_ramped_setpoint;
    float iq_ramped_setpoint;
    float id_measured;
    float iq_measured;
    float vd_mod;
    float vq_mod;
    float phase_gain;
    float p_gain;
    float i_gain;
    float limit;
    float max_voltage_mod;
    uint32_t error;
} MotorAxisCurrentStatus;

typedef struct {
    uint32_t a;
    uint32_t b;
    uint32_t c;
} MotorAxisPwmStatus;

typedef struct {
    MotorAxisRuntimeStatus runtime;
    MotorAxisPhaseCurrentStatus phase_current;
    MotorAxisOpenLoopStatus open_loop;
    MotorAxisCurrentStatus current;
    MotorAxisPwmStatus pwm;
} MotorAxisStatus;

extern osThreadId_t motorAxisTaskHandle;

void motor_axis_task(void *argument);
void motor_axis_signal_current_meas_from_isr(void);
void motor_axis_get_status(MotorAxisStatus *status);
void motor_axis_start_open_voltage(float voltage_mod, float electrical_phase_vel);
void motor_axis_start_open_current(float iq_setpoint, float electrical_phase_vel);
void motor_axis_stop(void);
void motor_axis_set_current_pi(float p_gain, float i_gain);
void motor_axis_set_current_config(float phase_current_gain,
                                   float current_limit,
                                   float max_voltage_mod);

#ifdef __cplusplus
}
#endif

#endif
