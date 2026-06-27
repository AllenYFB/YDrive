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
} ControlLoopStatus;

extern osThreadId_t controlLoopTaskHandle;

void ControlLoopTask(void *argument);
void control_loop_signal_current_meas_from_isr(void);
void control_loop_get_status(ControlLoopStatus *status);

#ifdef __cplusplus
}
#endif

#endif
