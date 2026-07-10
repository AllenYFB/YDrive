#ifndef __MOTOR_LIB_H
#define __MOTOR_LIB_H

#include "utils.h"

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************/
#define MOTOR_TYPE_HIGH_CURRENT 0
#define MOTOR_TYPE_GIMBAL 1

#define MOTOR_TYPE MOTOR_TYPE_HIGH_CURRENT
#define MOTOR_POLE_PAIRS 7
#define MOTOR_CALIBRATION_CURRENT_DEFAULT 2.0f
#define MOTOR_RESISTANCE_CALIB_MAX_VOLTAGE_DEFAULT 1.0f

#define ERROR_NONE 0U
#define ERROR_MODULATION_MAGNITUDE (1U << 0)
#define ERROR_CURRENT_SENSE_SATURATION (1U << 1)
#define ERROR_CURRENT_LIMIT_VIOLATION (1U << 2)
#define ERROR_PHASE_RESISTANCE_OUT_OF_RANGE (1U << 3)
#define ERROR_UNKNOWN_VBUS_VOLTAGE (1U << 4)
#define ERROR_NOT_HIGH_CURRENT_MOTOR (1U << 5)
#define ERROR_UNBALANCED_PHASES (1U << 6)
#define ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE (1U << 7)
#define ERROR_BAD_TIMING (1U << 8)
#define ERROR_TIMER_UPDATE_MISSED (1U << 9)
#define ERROR_CONTROL_DEADLINE_MISSED (1U << 10)
/****************************************************************************/
typedef struct
{
	float calibration_current;             // [A]
	float resistance_calib_max_voltage;    // [V]
	float phase_inductance;
	float phase_resistance;
	int32_t pole_pairs;
	int32_t motor_type;
	float current_control_bandwidth;       // [rad/s]
	float dc_calib_tau;
} MOTOR_CONFIG;

extern MOTOR_CONFIG motor_config;
/****************************************************************************/
void motor_para_init(void);
void motor_setup(void);
bool run_calibration(void);

void arm(void);
void disarm(void);
void current_meas_cb(uint32_t timestamp, Iph_ABC_t *current);
void dc_calib_cb(uint32_t timestamp, Iph_ABC_t *current);
void pwm_update_cb(uint32_t output_timestamp);
/****************************************************************************/

#endif
