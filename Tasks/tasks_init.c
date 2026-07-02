#include "tasks_init.h"

#include "motor_axis.h"
#include "monitor_task.h"

osThreadId_t monitorTaskHandle;

static const osThreadAttr_t monitorTask_attributes = {
    .name = "monitor",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

static const osThreadAttr_t motorAxisTask_attributes = {
    .name = "motorAxis",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};

void tasks_init(void)
{
    monitorTaskHandle = osThreadNew(monitor_task, NULL, &monitorTask_attributes);
    motorAxisTaskHandle = osThreadNew(motor_axis_task, NULL, &motorAxisTask_attributes);
}

