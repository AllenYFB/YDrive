#include "tasks.h"

#include "control_loop.h"
#include "monitor_task.h"

osThreadId_t monitorTaskHandle;

static const osThreadAttr_t monitorTask_attributes = {
    .name = "monitor",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

void tasks_init(void)
{
    monitorTaskHandle = osThreadNew(monitor_task, NULL, &monitorTask_attributes);
    control_loop_start_task();
}
