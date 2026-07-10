
#ifndef __OPEN_LOOP_CONTROLLER_LIB_H
#define __OPEN_LOOP_CONTROLLER_LIB_H

#include "utils.h"


/****************************************************************************/
typedef struct 
{
	float max_voltage_ramp_; // [V/s]
	float max_phase_vel_ramp_; // [rad/s^2]

	// Inputs
	float target_vel_;
	float target_voltage_;

	// State/Outputs
	uint32_t timestamp_;
	float2D Vdq_setpoint_;
	float phase_;
	float phase_vel_;
	float total_distance_;
} OPENLOOP_struct;

extern  OPENLOOP_struct openloop_controller_;
/****************************************************************************/
void open_loop_controller_start(void);
void open_loop_controller_update(uint32_t timestamp);
/****************************************************************************/

#endif

