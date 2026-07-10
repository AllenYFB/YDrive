
#ifndef __BOARD_LIB_H
#define __BOARD_LIB_H

#include "main.h"
#include "utils.h"

#define TIM_TIME_BASE TIM14

#define VBUS_S_DIVIDER_RATIO 18.73f
#define SHUNT_RESISTANCE 0.001f
#define PHASE_CURRENT_GAIN 10.0f

#define CURRENT_MEAS_PERIOD ((float)(2U * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1U)) / (float)TIM_1_8_CLOCK_HZ) /* 2*3500*(2+1)/168000000=0.000125 */
static const float current_meas_period = CURRENT_MEAS_PERIOD;

#define ControlLoop_IRQHandler OTG_HS_IRQHandler
#define ControlLoop_IRQn OTG_HS_IRQn


/****************************************************************************/
extern  float  vbus_voltage;
/****************************************************************************/
void motor_control_start(void);
void motor_control_tim1_update(void);
void motor_control_run_control_loop(void);
/****************************************************************************/

#endif

