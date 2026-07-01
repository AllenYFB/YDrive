#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include <stdint.h>
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONTROL_LOOP_CURRENT_MEAS_FLAG (1UL << 0)

typedef struct {
    uint32_t loop_count;
    uint32_t missed_samples;
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
} ControlLoopStatus;

extern osThreadId_t controlLoopTaskHandle;

void control_loop_start_task(void);
void control_loop_signal_current_meas_from_isr(void);
void control_loop_get_status(ControlLoopStatus *status);
void control_loop_set_open_loop_target(float voltage_mod, float electrical_phase_vel);
void control_loop_set_open_loop_enabled(uint32_t enable);

#ifdef __cplusplus
}
#endif

#endif
