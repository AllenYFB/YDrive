#include "encode.h"

#include "board.h"
#include "foc.h"
#include "motor.h"
#include "open_loop_controller.h"
#include "as5047p.h"
#include "encoder_abz.h"

#include "cmsis_os.h"

#include <math.h>
#include <string.h>

/****************************************************************************/
ENCODER_CONFIG encoder_config;
ENCODER_STATE encoder_state;
/****************************************************************************/
static void encoder_set_error(uint32_t error);
static void encoder_config_init_defaults(void);
static void encoder_state_reset(void);
static int32_t encoder_get_default_cpr(ENCODER_MODE mode);
static void encoder_setup_mode(void);
static void encoder_sample_incremental(void);
static void encoder_sample_spi_as5047p(void);
static bool encoder_get_incremental_delta(int32_t *delta_enc);
static bool encoder_get_spi_as5047p_delta(int32_t *delta_enc, int32_t *pos_abs_latched);
static void encoder_apply_delta(int32_t delta_enc, int32_t pos_abs_latched);
static bool encoder_run_pll(void);
static float encoder_run_interpolation(int32_t delta_enc, bool snap_to_zero_vel);
static void encoder_publish_phase(float interpolated_enc);
static void encoder_update_pll_gains(void);
/****************************************************************************/
static void encoder_set_error(uint32_t error)
{
	encoder_state.vel_estimate_valid = false;
	encoder_state.pos_estimate_valid = false;
	set_error(error);
}
/****************************************************************************/
static void encoder_config_init_defaults(void)
{
	encoder_config.mode = ENCODER_MODE_DEFAULT;
	encoder_config.cpr = encoder_get_default_cpr(encoder_config.mode);
	encoder_config.direction = 1;
	encoder_config.phase_offset = 0;
	encoder_config.phase_offset_float = 0.0f;
	encoder_config.bandwidth = ENCODER_BANDWIDTH_DEFAULT;
	encoder_config.calib_range = ENCODER_CALIB_RANGE_DEFAULT;
	encoder_config.calib_scan_distance = ENCODER_CALIB_SCAN_DISTANCE_DEFAULT;
	encoder_config.calib_scan_omega = ENCODER_CALIB_SCAN_OMEGA_DEFAULT;
	encoder_config.pre_calibrated = false;
	encoder_config.enable_phase_interpolation = true;
	encoder_config.phase_ = 0.0f;
	encoder_config.phase_vel_ = 0.0f;
}
/****************************************************************************/
static void encoder_state_reset(void)
{
	encoder_state.is_ready = false;
	encoder_state.shadow_count = 0;
	encoder_state.count_in_cpr = 0;
	encoder_state.pos_abs = 0;
	encoder_state.abs_spi_raw = 0U;
	encoder_state.abs_spi_error_flags = 0U;
	encoder_state.tim_cnt_sample = 0;
	encoder_state.abs_spi_pos_updated = false;
	encoder_state.spi_error_rate = 0.0f;
	encoder_state.interpolation = 0.5f;
	encoder_state.pos_estimate_counts = 0.0f;
	encoder_state.pos_cpr_counts = 0.0f;
	encoder_state.vel_estimate_counts = 0.0f;
	encoder_state.pos_estimate = 0.0f;
	encoder_state.vel_estimate = 0.0f;
	encoder_state.calib_scan_response = 0.0f;
	encoder_state.calib_scan_expected = 0.0f;
	encoder_state.calib_scan_error = 0.0f;
	encoder_state.calib_init_count = 0;
	encoder_state.calib_final_count = 0;
	encoder_state.pos_estimate_valid = false;
	encoder_state.vel_estimate_valid = false;
}
/****************************************************************************/
static int32_t encoder_get_default_cpr(ENCODER_MODE mode)
{
	switch (mode)
	{
		case ENCODER_MODE_SPI_AS5047P:
			return ENCODER_CPR_SPI_AS5047P_DEFAULT;

		case ENCODER_MODE_INCREMENTAL:
		default:
			return ENCODER_CPR_DEFAULT;
	}
}
/****************************************************************************/
static void encoder_setup_mode(void)
{
	switch (encoder_config.mode)
	{
		case ENCODER_MODE_INCREMENTAL:
			encoder_abz_init();
			break;

		case ENCODER_MODE_SPI_AS5047P:
			as5047p_init();
			encoder_sample_now();
			if (encoder_state.abs_spi_pos_updated)
			{
				encoder_state.count_in_cpr = encoder_state.pos_abs;
				encoder_state.pos_cpr_counts = (float)encoder_state.pos_abs;
			}
			break;

		default:
			encoder_set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
			break;
	}
}
/****************************************************************************/
static void encoder_update_pll_gains(void)
{
	/* kp=2*bw */
	encoder_state.pll_kp = 2.0f * encoder_config.bandwidth;

	/* ki=0.25*kp^2 */
	encoder_state.pll_ki = 0.25f * encoder_state.pll_kp * encoder_state.pll_kp;

	/* Ts*kp<1 */
	if (!(current_meas_period * encoder_state.pll_kp < 1.0f))
	{
		encoder_set_error(ERROR_ENCODER_UNSTABLE_GAIN);
	}
}
/****************************************************************************/
void encoder_init(void)
{
	encoder_config_init_defaults();
	encoder_state_reset();
	encoder_update_pll_gains();
	encoder_setup_mode();

	if (encoder_config.pre_calibrated)
	{
		encoder_state.is_ready = true;
	}
}
/****************************************************************************/
void encoder_sample_now(void)
{
	switch (encoder_config.mode)
	{
		case ENCODER_MODE_INCREMENTAL:
			encoder_sample_incremental();
			break;

		case ENCODER_MODE_SPI_AS5047P:
			encoder_sample_spi_as5047p();
			break;

		default:
			encoder_set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
			break;
	}
}
/****************************************************************************/
static void encoder_sample_incremental(void)
{
	encoder_state.tim_cnt_sample = encoder_abz_sample();
}
/****************************************************************************/
static void encoder_sample_spi_as5047p(void)
{
	AS5047P_SAMPLE sample;
	int32_t pos_abs = 0;

	if (as5047p_read(&sample, &pos_abs))
	{
		encoder_state.pos_abs = pos_abs;
		encoder_state.abs_spi_pos_updated = true;

		if (encoder_config.pre_calibrated)
		{
			encoder_state.is_ready = true;
		}
	}

	encoder_state.abs_spi_raw = sample.raw;
	encoder_state.abs_spi_error_flags = sample.error_flags;
}
/****************************************************************************/
static bool encoder_get_incremental_delta(int32_t *delta_enc)
{
	int16_t delta_enc_16 =
		(int16_t)encoder_state.tim_cnt_sample - (int16_t)encoder_state.shadow_count;

	*delta_enc = (int32_t)delta_enc_16;
	return true;
}
/****************************************************************************/
static bool encoder_get_spi_as5047p_delta(int32_t *delta_enc, int32_t *pos_abs_latched)
{
	*pos_abs_latched = encoder_state.pos_abs;

	if (!encoder_state.abs_spi_pos_updated)
	{
		/* err_rate=err_rate+Ts*(1-err_rate) */
		encoder_state.spi_error_rate +=
			current_meas_period * (1.0f - encoder_state.spi_error_rate);

		if (encoder_state.spi_error_rate > 0.05f)
		{
			encoder_set_error(ERROR_ABS_SPI_COM_FAIL);
			return false;
		}
	}
	else
	{
		/* err_rate=err_rate+Ts*(0-err_rate) */
		encoder_state.spi_error_rate +=
			current_meas_period * (0.0f - encoder_state.spi_error_rate);
	}

	encoder_state.abs_spi_pos_updated = false;
	*delta_enc = *pos_abs_latched - encoder_state.count_in_cpr;
	*delta_enc = mod_int(*delta_enc, encoder_config.cpr);
	if (*delta_enc > (encoder_config.cpr / 2))
	{
		*delta_enc -= encoder_config.cpr;
	}

	return true;
}
/****************************************************************************/
static void encoder_apply_delta(int32_t delta_enc, int32_t pos_abs_latched)
{
	encoder_state.shadow_count += delta_enc;
	encoder_state.count_in_cpr =
		mod_int(encoder_state.count_in_cpr + delta_enc, encoder_config.cpr);

	if (encoder_config.mode == ENCODER_MODE_SPI_AS5047P)
	{
		encoder_state.count_in_cpr = pos_abs_latched;
	}
}
/****************************************************************************/
static bool encoder_run_pll(void)
{
	/* x=x+Ts*v */
	encoder_state.pos_estimate_counts += current_meas_period * encoder_state.vel_estimate_counts;
	encoder_state.pos_cpr_counts += current_meas_period * encoder_state.vel_estimate_counts;

	/* e=xenc-xpll */
	float delta_pos_counts =
		(float)(encoder_state.shadow_count - (int32_t)encoder_state.pos_estimate_counts);
	float delta_pos_cpr_counts =
		(float)(encoder_state.count_in_cpr - (int32_t)encoder_state.pos_cpr_counts);
	delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, (float)encoder_config.cpr);

	/* x=x+Ts*kp*e */
	encoder_state.pos_estimate_counts +=
		current_meas_period * encoder_state.pll_kp * delta_pos_counts;
	encoder_state.pos_cpr_counts +=
		current_meas_period * encoder_state.pll_kp * delta_pos_cpr_counts;
	encoder_state.pos_cpr_counts =
		fmodf_pos(encoder_state.pos_cpr_counts, (float)encoder_config.cpr);

	/* v=v+Ts*ki*e */
	encoder_state.vel_estimate_counts +=
		current_meas_period * encoder_state.pll_ki * delta_pos_cpr_counts;

	bool snap_to_zero_vel = false;
	if (fabsf(encoder_state.vel_estimate_counts) <
	    0.5f * current_meas_period * encoder_state.pll_ki)
	{
		encoder_state.vel_estimate_counts = 0.0f;
		snap_to_zero_vel = true;
	}

	/* pos_turn=counts/cpr */
	encoder_state.pos_estimate =
		encoder_state.pos_estimate_counts / (float)encoder_config.cpr;
	encoder_state.vel_estimate =
		encoder_state.vel_estimate_counts / (float)encoder_config.cpr;
	encoder_state.pos_estimate_valid = true;
	encoder_state.vel_estimate_valid = true;

	return snap_to_zero_vel;
}
/****************************************************************************/
static float encoder_run_interpolation(int32_t delta_enc, bool snap_to_zero_vel)
{
	int32_t corrected_enc = encoder_state.count_in_cpr - encoder_config.phase_offset;

	if (snap_to_zero_vel || !encoder_config.enable_phase_interpolation)
	{
		encoder_state.interpolation = 0.5f;
	}
	else if (delta_enc > 0)
	{
		encoder_state.interpolation = 0.0f;
	}
	else if (delta_enc < 0)
	{
		encoder_state.interpolation = 1.0f;
	}
	else
	{
		/* interp=interp+Ts*v_counts */
		encoder_state.interpolation += current_meas_period * encoder_state.vel_estimate_counts;
		encoder_state.interpolation = clampf(encoder_state.interpolation, 0.0f, 1.0f);
	}

	return (float)corrected_enc + encoder_state.interpolation;
}
/****************************************************************************/
static void encoder_publish_phase(float interpolated_enc)
{

	/* elec_rad_per_count=pole_pairs*2*pi/cpr */
	float elec_rad_per_enc =
		(float)motor_config.pole_pairs * 2.0f * M_PI / (float)encoder_config.cpr;

	/* phase=elec_rad_per_count*(count-phase_offset_float) */
	float phase = elec_rad_per_enc *
	              (interpolated_enc - encoder_config.phase_offset_float);

	if (encoder_state.is_ready)
	{
		encoder_config.phase_ = wrap_pm_pi(phase) * (float)encoder_config.direction;
		encoder_config.phase_vel_ =
			2.0f * M_PI * encoder_state.vel_estimate *
			(float)motor_config.pole_pairs * (float)encoder_config.direction;
	}
}
/****************************************************************************/
bool encoder_update(void)
{
	int32_t delta_enc = 0;
	int32_t pos_abs_latched = encoder_state.pos_abs;

	switch (encoder_config.mode)
	{
		case ENCODER_MODE_INCREMENTAL:
			if (!encoder_get_incremental_delta(&delta_enc))
			{
				return false;
			}
			break;

		case ENCODER_MODE_SPI_AS5047P:
			if (!encoder_get_spi_as5047p_delta(&delta_enc, &pos_abs_latched))
			{
				return false;
			}
			break;

		default:
			encoder_set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
			return false;
	}

	encoder_apply_delta(delta_enc, pos_abs_latched);

	bool snap_to_zero_vel = encoder_run_pll();
	float interpolated_enc = encoder_run_interpolation(delta_enc, snap_to_zero_vel);
	encoder_publish_phase(interpolated_enc);

	return true;
}
/****************************************************************************/
bool encoder_run_offset_calibration(void)
{
	int32_t init_enc_val;
	float expected_encoder_delta;
	uint32_t i;
	uint32_t num_steps = 0U;
	int64_t enc_value_sum = 0;
	const uint32_t start_lock_ms = 1000U;
	const float start_lock_duration = 1.0f;

	encoder_state.is_ready = false;
	encoder_sample_now();
	(void)encoder_update();
	encoder_state.shadow_count = encoder_state.count_in_cpr;
	encoder_state.pos_estimate_counts = (float)encoder_state.shadow_count;
	encoder_state.pos_cpr_counts = (float)encoder_state.count_in_cpr;
	memset(&openloop_controller_, 0, sizeof(OPENLOOP_struct));

	float max_current_ramp = motor_config.calibration_current / start_lock_duration * 2.0f;
	openloop_controller_.max_current_ramp_ = max_current_ramp;
	openloop_controller_.max_voltage_ramp_ = max_current_ramp;
	openloop_controller_.max_phase_vel_ramp_ = INFINITY;
	openloop_controller_.target_current_ =
		(motor_config.motor_type != MOTOR_TYPE_GIMBAL) ? motor_config.calibration_current : 0.0f;
	openloop_controller_.target_voltage_ =
		(motor_config.motor_type != MOTOR_TYPE_GIMBAL) ? 0.0f :
		                                                  motor_config.calibration_current;
	openloop_controller_.target_vel_ = 0.0f;
	openloop_controller_.phase_ = wrap_pm_pi(0.0f - encoder_config.calib_scan_distance * 0.5f);
	openloop_controller_.total_distance_ = 0.0f;
	arm();

	for (i = 0U; i < start_lock_ms; ++i)
	{
		if (!is_armed_)
		{
			break;
		}
		osDelay(1);
	}

	if (!is_armed_)
	{
		disarm();
		return false;
	}

	encoder_sample_now();
	(void)encoder_update();
	init_enc_val = encoder_state.shadow_count;
	encoder_state.calib_init_count = init_enc_val;

	openloop_controller_.target_vel_ = encoder_config.calib_scan_omega;
	openloop_controller_.total_distance_ = 0.0f;

	while (openloop_controller_.total_distance_ < encoder_config.calib_scan_distance)
	{
		if (!is_armed_)
		{
			break;
		}
		enc_value_sum += encoder_state.shadow_count;
		num_steps++;
		osDelay(1);
	}

	encoder_state.calib_final_count = encoder_state.shadow_count;
	encoder_state.calib_scan_response = fabsf((float)(encoder_state.calib_final_count - init_enc_val));
	expected_encoder_delta =
		encoder_config.calib_scan_distance /
		((float)motor_config.pole_pairs * 2.0f * M_PI / (float)encoder_config.cpr);
	encoder_state.calib_scan_expected = expected_encoder_delta;

	if (encoder_state.shadow_count > init_enc_val + 8)
	{
		encoder_config.direction = 1;
	}
	else if (encoder_state.shadow_count < init_enc_val - 8)
	{
		encoder_config.direction = -1;
	}
	else
	{
		set_error(ERROR_ENCODER_NO_RESPONSE);
		disarm();
		return false;
	}

	if (expected_encoder_delta <= 0.0f)
	{
		set_error(ERROR_ENCODER_FAILED);
		disarm();
		return false;
	}

	if (fabsf(encoder_state.calib_scan_response - expected_encoder_delta) /
	        expected_encoder_delta >
	    encoder_config.calib_range)
	{
		encoder_state.calib_scan_error =
			fabsf(encoder_state.calib_scan_response - expected_encoder_delta) /
			expected_encoder_delta;
		set_error(ERROR_ENCODER_FAILED);
		disarm();
		return false;
	}

	encoder_state.calib_scan_error =
		fabsf(encoder_state.calib_scan_response - expected_encoder_delta) /
		expected_encoder_delta;

	openloop_controller_.target_vel_ = -encoder_config.calib_scan_omega;

	while (openloop_controller_.total_distance_ > 0.0f)
	{
		if (!is_armed_)
		{
			break;
		}
		enc_value_sum += encoder_state.shadow_count;
		num_steps++;
		osDelay(1);
	}

	if (!is_armed_ || (num_steps == 0U))
	{
		disarm();
		return false;
	}

	disarm();

	int32_t phase_offset = (int32_t)(enc_value_sum / (int64_t)num_steps);
	int32_t residual = (int32_t)(enc_value_sum - ((int64_t)phase_offset * (int64_t)num_steps));

	encoder_config.phase_offset = mod_int(phase_offset, encoder_config.cpr);
	encoder_config.phase_offset_float = (float)residual / (float)num_steps + 0.5f;
	encoder_state.is_ready = true;
	encoder_config.pre_calibrated = true;

	return true;
}
/****************************************************************************/
