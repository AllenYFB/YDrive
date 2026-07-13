
#ifndef __AXIS_LIB_H
#define __AXIS_LIB_H

#include <stdint.h>


/****************************************************************************/
typedef enum
{
	AXIS_STATE_IDLE = 0,
	AXIS_STATE_MOTOR_CALIBRATION,
	AXIS_STATE_ENCODER_INDEX_SEARCH,
	AXIS_STATE_ENCODER_DIR_FIND,
	AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION,
	AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION,
	AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
	AXIS_STATE_LOCKIN_SPIN,
	AXIS_STATE_CLOSED_LOOP_CONTROL,
	AXIS_STATE_UNDEFINED,
} AXIS_State_Type;

extern  AXIS_State_Type  current_state_;
/****************************************************************************/
void run_state_machine_loop(void);
void control_loop_cb(uint32_t timestamp);
/****************************************************************************/

#endif

