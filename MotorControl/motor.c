#include "motor.h"

#include "board.h"
#include "cmsis_os.h"
#include "foc.h"
#include "tim.h"

#include <math.h>

/****************************************************************************/
typedef enum
{
	MOTOR_CONTROL_MODE_NORMAL = 0,
	MOTOR_CONTROL_MODE_RESISTANCE,
	MOTOR_CONTROL_MODE_INDUCTANCE,
} MOTOR_CONTROL_MODE;

typedef struct
{
	float target_current;
	float max_voltage;
	float test_voltage;
	float beta_current;
	float mod_alpha;
} RESISTANCE_MEASUREMENT;

typedef struct
{
	float test_voltage;
	float last_Ialpha;
	float deltaI;
	uint32_t start_timestamp;
	uint32_t last_input_timestamp;
	bool attached;
} INDUCTANCE_MEASUREMENT;
/****************************************************************************/
static void resistance_reset(float target_current, float max_voltage);
static void resistance_on_measurement(void);
static void resistance_get_alpha_beta_output(void);
static void inductance_reset(float test_voltage);
static void inductance_on_measurement(uint32_t input_timestamp);
static void inductance_get_alpha_beta_output(void);
static void update_current_controller_gains(void);
/****************************************************************************/
MOTOR_CONFIG motor_config;
bool is_armed_ = false;

static Iph_ABC_t DC_calib_;
static float dc_calib_running_since_ = 0.0f;
static float max_dc_calib_ = 0.0f;

static MOTOR_CONTROL_MODE motor_control_mode_ = MOTOR_CONTROL_MODE_NORMAL;
static RESISTANCE_MEASUREMENT resistance_meas_;
static INDUCTANCE_MEASUREMENT inductance_meas_;
/****************************************************************************/
void motor_para_init(void)
{
	motor_config.calibration_current = MOTOR_CALIBRATION_CURRENT_DEFAULT;
	motor_config.resistance_calib_max_voltage = MOTOR_RESISTANCE_CALIB_MAX_VOLTAGE_DEFAULT;
	motor_config.phase_inductance = 0.0f;
	motor_config.phase_resistance = 0.0f;
	motor_config.pole_pairs = MOTOR_POLE_PAIRS;
	motor_config.motor_type = MOTOR_TYPE;
	motor_config.current_control_bandwidth = 1000.0f;
	motor_config.dc_calib_tau = 0.2f;
}
/****************************************************************************/
void motor_setup(void)
{
	const float kMargin = 0.90f;
	const float max_output_swing = 1.35f;
	const float actual_gain = PHASE_CURRENT_GAIN;

	/* Imax=0.9*1.35/Rshunt/Gain */
	float max_unity_gain_current = kMargin * max_output_swing / SHUNT_RESISTANCE;
	float max_allowed_current = max_unity_gain_current / actual_gain;

	max_dc_calib_ = 0.1f * max_allowed_current;
}
/****************************************************************************/
static void resistance_reset(float target_current, float max_voltage)
{
	resistance_meas_.target_current = target_current;
	resistance_meas_.max_voltage = max_voltage;
	resistance_meas_.test_voltage = 0.0f;
	resistance_meas_.beta_current = 0.0f;
	resistance_meas_.mod_alpha = 0.0f;
}
/****************************************************************************/
static void resistance_on_measurement(void)
{
	const float kI = 1.0f;
	const float kIBetaFilt = 80.0f;
	float actual_current = Ialpha_beta[0];

	/* e=Iref-Ialpha */
	/* Vtest=Vtest+kI*Ts*e */
	resistance_meas_.test_voltage += (kI * current_meas_period) *
	                                 (resistance_meas_.target_current - actual_current);

	/* Ibeta_f=Ibeta_f+k*Ts*(Ibeta-Ibeta_f) */
	resistance_meas_.beta_current += (kIBetaFilt * current_meas_period) *
	                                 (Ialpha_beta[1] - resistance_meas_.beta_current);

	if (fabsf(resistance_meas_.test_voltage) > resistance_meas_.max_voltage)
	{
		disarm();
		set_error(ERROR_PHASE_RESISTANCE_OUT_OF_RANGE);
	}
	else if (vbus_voltage <= 6.0f)
	{
		disarm();
		set_error(ERROR_UNKNOWN_VBUS_VOLTAGE);
	}
	else
	{
		/* mod=Vtest/((2/3)*Vbus) */
		float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
		resistance_meas_.mod_alpha = resistance_meas_.test_voltage * vfactor;
	}
}
/****************************************************************************/
static void resistance_get_alpha_beta_output(void)
{
	enqueue_modulation_timings(resistance_meas_.mod_alpha, 0.0f);
}
/****************************************************************************/
static void inductance_reset(float test_voltage)
{
	inductance_meas_.test_voltage = test_voltage;
	inductance_meas_.last_Ialpha = 0.0f;
	inductance_meas_.deltaI = 0.0f;
	inductance_meas_.start_timestamp = 0U;
	inductance_meas_.last_input_timestamp = 0U;
	inductance_meas_.attached = false;
}
/****************************************************************************/
static void inductance_on_measurement(uint32_t input_timestamp)
{
	if (inductance_meas_.attached)
	{
		float sign = (inductance_meas_.test_voltage >= 0.0f) ? 1.0f : -1.0f;

		/* dI=sum(-sign*(Ialpha[n]-Ialpha[n-1])) */
		inductance_meas_.deltaI += -sign * (Ialpha_beta[0] - inductance_meas_.last_Ialpha);
	}
	else
	{
		inductance_meas_.start_timestamp = input_timestamp;
		inductance_meas_.attached = true;
	}

	inductance_meas_.last_Ialpha = Ialpha_beta[0];
	inductance_meas_.last_input_timestamp = input_timestamp;
}
/****************************************************************************/
static void inductance_get_alpha_beta_output(void)
{
	/* Vtest=-Vtest */
	inductance_meas_.test_voltage *= -1.0f;

	/* mod=Vtest/((2/3)*Vbus) */
	float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
	float mod_alpha = inductance_meas_.test_voltage * vfactor;

	enqueue_modulation_timings(mod_alpha, 0.0f);
}
/****************************************************************************/
static void update_current_controller_gains(void)
{
	/* Kp=wc*L */
	/* Ki=(R/L)*Kp=wc*R */
	pi_gains_[0] = motor_config.current_control_bandwidth * motor_config.phase_inductance;
	pi_gains_[1] = motor_config.current_control_bandwidth * motor_config.phase_resistance;
}
/****************************************************************************/
bool measure_phase_resistance(float test_current, float max_voltage)
{
	uint32_t i;

	resistance_reset(test_current, max_voltage);
	motor_control_mode_ = MOTOR_CONTROL_MODE_RESISTANCE;

	arm();

	for (i = 0; i < 3000U; ++i)
	{
		if (!is_armed_)
		{
			break;
		}
		osDelay(1);
	}

	bool success = is_armed_;
	disarm();

	if (success)
	{
		/* R=V/I */
		motor_config.phase_resistance =
			resistance_meas_.test_voltage / resistance_meas_.target_current;
	}

	if (motor_config.phase_resistance > 2.0f)
	{
		set_error(ERROR_NOT_HIGH_CURRENT_MOTOR);
		success = false;
	}

	if ((fabsf(resistance_meas_.beta_current) / test_current) > 0.2f)
	{
		set_error(ERROR_UNBALANCED_PHASES);
		success = false;
	}

	motor_control_mode_ = MOTOR_CONTROL_MODE_NORMAL;
	return success;
}
/****************************************************************************/
bool measure_phase_inductance(float test_voltage)
{
	uint32_t i;

	inductance_reset(test_voltage);
	motor_control_mode_ = MOTOR_CONTROL_MODE_INDUCTANCE;

	arm();

	for (i = 0; i < 1250U; ++i)
	{
		if (!is_armed_)
		{
			break;
		}
		osDelay(1);
	}

	bool success = is_armed_;
	disarm();

	if (success)
	{
		/* dt=(t1-t0)/Ftim */
		float dt = (float)(inductance_meas_.last_input_timestamp -
		                   inductance_meas_.start_timestamp) /
		           (float)TIM_1_8_CLOCK_HZ;

		/* L=abs(V)/(dI/dt) */
		motor_config.phase_inductance =
			fabsf(inductance_meas_.test_voltage) / (inductance_meas_.deltaI / dt);
	}

	if (!(motor_config.phase_inductance >= 2e-6f &&
	      motor_config.phase_inductance <= 4000e-6f))
	{
		set_error(ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE);
		success = false;
	}

	motor_control_mode_ = MOTOR_CONTROL_MODE_NORMAL;
	return success;
}
/****************************************************************************/
bool run_calibration(void)
{
	float resistance_calib_max_voltage = motor_config.resistance_calib_max_voltage;

	if (motor_config.motor_type == MOTOR_TYPE_HIGH_CURRENT)
	{
		if (!measure_phase_resistance(motor_config.calibration_current,
		                              resistance_calib_max_voltage))
		{
			return false;
		}

		if (!measure_phase_inductance(resistance_calib_max_voltage))
		{
			return false;
		}
	}
	else if (motor_config.motor_type == MOTOR_TYPE_GIMBAL)
	{
	}
	else
	{
		return false;
	}

	update_current_controller_gains();

	return true;
}
/****************************************************************************/
void arm(void)
{
	is_armed_ = true;
	TIM1->BDTR |= TIM_BDTR_MOE;
}
/****************************************************************************/
void disarm(void)
{
	is_armed_ = false;
	TIM1->BDTR &= ~TIM_BDTR_MOE;
}
/****************************************************************************/
void current_meas_cb(uint32_t timestamp, Iph_ABC_t *current)
{
	Iph_ABC_t current_meas;

	/* tvalid=7.5*tau 这里用 7.5 * tau 作为校准结果有效的等待时间——对于一阶低通滤波器，
	经过约 7.5 个时间常数 后，滤波器输出基本达到稳态（约 99.95%），
	此时 DC 偏置校准值被认为是可靠的。*/
	bool dc_calib_valid = (dc_calib_running_since_ >= motor_config.dc_calib_tau * 7.5f) &&
	                      (fabsf(DC_calib_.phA) < max_dc_calib_) &&
	                      (fabsf(DC_calib_.phB) < max_dc_calib_) &&
	                      (fabsf(DC_calib_.phC) < max_dc_calib_);

	if (dc_calib_valid)
	{
		/* I=Iraw-Ioffset */
		current_meas.phA = current->phA - DC_calib_.phA;
		current_meas.phB = current->phB - DC_calib_.phB;
		current_meas.phC = current->phC - DC_calib_.phC;
	}
	else
	{
		current_meas.phA = 0.0f;
		current_meas.phB = 0.0f;
		current_meas.phC = 0.0f;
	}

	on_measurement(timestamp, &current_meas);

	if (motor_control_mode_ == MOTOR_CONTROL_MODE_RESISTANCE)
	{
		resistance_on_measurement();
	}
	else if (motor_control_mode_ == MOTOR_CONTROL_MODE_INDUCTANCE)
	{
		inductance_on_measurement(timestamp);
	}
}
/****************************************************************************/
void dc_calib_cb(uint32_t timestamp, Iph_ABC_t *current)
{
	(void)timestamp;

	/* Ts=2*3500*(2+1)/168000000=0.000125 */
	const float dc_calib_period = (float)(2U * TIM_1_8_PERIOD_CLOCKS *
	                                      (TIM_1_8_RCR + 1U)) /
	                              (float)TIM_1_8_CLOCK_HZ;

	/* k=Ts/tau */
	const float calib_filter_k =
		clampf(dc_calib_period / motor_config.dc_calib_tau, 0.0f, 1.0f);

	/* y=y+k*(x-y) *********=====************y_new = (1-k)*y_old + k*x
	这里 tau 是 一阶低通滤波器（指数移动平均）的时间常数 τ。
	滤波系数 k = Ts / tau，其中 Ts 是采样周期。
	滤波器方程为经典的 y_new = (1-k)·y_old + k·x，
	用于平滑地估算三相电流的直流偏置（DC offset）*/
	DC_calib_.phA += (current->phA - DC_calib_.phA) * calib_filter_k;
	DC_calib_.phB += (current->phB - DC_calib_.phB) * calib_filter_k;
	DC_calib_.phC += (current->phC - DC_calib_.phC) * calib_filter_k;
	dc_calib_running_since_ += dc_calib_period;
}
/****************************************************************************/
void pwm_update_cb(uint32_t output_timestamp)
{
	if (motor_control_mode_ == MOTOR_CONTROL_MODE_RESISTANCE)
	{
		resistance_get_alpha_beta_output();
	}
	else if (motor_control_mode_ == MOTOR_CONTROL_MODE_INDUCTANCE)
	{
		inductance_get_alpha_beta_output();
	}
	else
	{
		foc_pwm_update_cb(output_timestamp);
	}
}
/****************************************************************************/
