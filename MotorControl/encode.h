#ifndef __ENCODE_LIB_H
#define __ENCODE_LIB_H

#include "utils.h"

#include <stdbool.h>
#include <stdint.h>

/****************************************************************************/
#define ENCODER_CPR_DEFAULT 4000
#define ENCODER_BANDWIDTH_DEFAULT 1000.0f
#define ENCODER_CALIB_RANGE_DEFAULT 0.02f
#define ENCODER_CALIB_SCAN_DISTANCE_DEFAULT (16.0f * M_PI)
#define ENCODER_CALIB_SCAN_OMEGA_DEFAULT (4.0f * M_PI)
#define ENCODER_CALIB_SCAN_VOLTAGE_DEFAULT 0.6f
/****************************************************************************/
typedef enum
{
	ENCODER_MODE_INCREMENTAL = 0,
} ENCODER_MODE;

typedef struct
{
	ENCODER_MODE mode;
	int32_t cpr;
	int32_t direction;
	int32_t phase_offset;
	float phase_offset_float;
	float bandwidth;
	float calib_range;
	float calib_scan_distance;
	float calib_scan_omega;
	bool pre_calibrated;
	bool enable_phase_interpolation;
	float phase_;
	float phase_vel_;
} ENCODER_CONFIG;

typedef struct
{
	bool is_ready;
	int32_t shadow_count;
	int32_t count_in_cpr;
	int16_t tim_cnt_sample;
	float interpolation;
	float pos_estimate_counts;
	float pos_cpr_counts;
	float vel_estimate_counts;
	float pll_kp;
	float pll_ki;
	float pos_estimate;
	float vel_estimate;
	float calib_scan_response;
	float calib_scan_expected;
	float calib_scan_error;
	int32_t calib_init_count;
	int32_t calib_final_count;
	bool pos_estimate_valid;
	bool vel_estimate_valid;
} ENCODER_STATE;

extern ENCODER_CONFIG encoder_config;
extern ENCODER_STATE encoder_state;
/****************************************************************************/
void encoder_init(void);
void encoder_sample_now(void);
bool encoder_update(void);
bool encoder_run_offset_calibration(void);
/****************************************************************************/

#endif
