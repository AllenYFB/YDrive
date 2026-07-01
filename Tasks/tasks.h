#ifndef TASKS_H
#define TASKS_H

#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

extern osThreadId_t monitorTaskHandle;

void tasks_init(void);

#ifdef __cplusplus
}
#endif

#endif
