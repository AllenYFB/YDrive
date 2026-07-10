
#include "motor.h"

#include "board.h"
#include "foc.h"
#include "tim.h"
#include "cmsis_os.h"

#include <math.h>

/****************************************************************************/
MOTOR_CONFIG  motor_config;
static Iph_ABC_t  DC_calib_;
static float dc_calib_running_since_ = 0.0f;

static float max_dc_calib_ = 0.0f; // [A] set in setup()

typedef enum
{
	MOTOR_CONTROL_MODE_NORMAL = 0,
	MOTOR_CONTROL_MODE_RESISTANCE,
	MOTOR_CONTROL_MODE_INDUCTANCE,
} MOTOR_CONTROL_MODE;

static MOTOR_CONTROL_MODE motor_control_mode_ = MOTOR_CONTROL_MODE_NORMAL;
/****************************************************************************/
//参数初始化，官方代码中，直接在定义的时候赋值
void motor_para_init(void)
{
	motor_config.calibration_current = MOTOR_CALIBRATION_CURRENT_DEFAULT;    // [A]
	motor_config.resistance_calib_max_voltage = MOTOR_RESISTANCE_CALIB_MAX_VOLTAGE_DEFAULT; // [V] - You may need to increase this if this voltage isn't sufficient to drive calibration_current through the motor.
	motor_config.phase_inductance = 0.0f;        // to be set by measure_phase_inductance
	motor_config.phase_resistance = 0.0f;        // to be set by measure_phase_resistance
	motor_config.pole_pairs = MOTOR_POLE_PAIRS;
	motor_config.motor_type = MOTOR_TYPE;
	// Read out max_allowed_current to see max supported value for current_lim.
	// float current_lim = 70.0f; //[A]
//	motor_config.torque_lim = std::numeric_limits<float>::infinity();           //[Nm]. 
	// Value used to compute shunt amplifier gains
	//float requested_current_range = 60.0f; //  [A]1mΩ采样电阻对应60A，0.5mΩ采样电阻对应120A
	motor_config.current_control_bandwidth = 1000.0f;  // [rad/s]
	
//	float acim_gain_min_flux = 10; // [A]
//	float acim_autoflux_min_Id = 10; // [A]
//	bool acim_autoflux_enable = false;
//	float acim_autoflux_attack_gain = 10.0f;
//	float acim_autoflux_decay_gain = 1.0f;
	
	
	
	motor_config.dc_calib_tau = 0.2f;
}
/****************************************************************************/
void motor_setup(void)
{
	// Solve for exact gain, then snap down to have equal or larger range as requested
	// or largest possible range otherwise
	const float kMargin = 0.90f;
	const float max_output_swing = 1.35f; // [V] out of amplifier
	
	float max_unity_gain_current = kMargin * max_output_swing * (1/SHUNT_RESISTANCE); // [A]  1215
	//float requested_gain = max_unity_gain_current / config_.requested_current_range; // [V/V]
	const float actual_gain=20.0f;  //固定放大倍数20倍
	
	// Values for current controller
	float phase_current_rev_gain_ = 1.0f / actual_gain;  //0.05
	// Clip all current control to actual usable range
	float max_allowed_current = max_unity_gain_current * phase_current_rev_gain_;  // =1215*0.05=60.75 A

	max_dc_calib_ = 0.1f * max_allowed_current;  //约等于 6A
}
/****************************************************************************/

bool  is_armed_ = false;

float  cali_target_current_;
float  cali_max_voltage_;
float  test_voltage_;
float  I_beta_; // [A] low pass filtered Ibeta response
float  test_mod_;

void arm(void);
void disarm(void);
/****************************************************************************/
void Resistance_reset(void)
{
	test_voltage_ = 0;
	I_beta_ = 0;
}
/*************************************/
// 电阻测量的电流输入侧，在 current_meas_cb() 后执行
void Resistance_on_measurement(void)
{
	const float kI = 1.0f; // [(V/s)/A]
	const float kIBetaFilt = 80.0f;
	
	float actual_current = Ialpha_beta[0];    //Ialpha_beta 是foc.c中clark变换后的值
	test_voltage_ += (kI * current_meas_period) * (cali_target_current_ - actual_current);  //消除误差，保持测量电压的稳定
	I_beta_ += (kIBetaFilt * current_meas_period) * (Ialpha_beta[1] - I_beta_);    //消除误差，获取稳定的测量电流
	if(fabsf(test_voltage_) > cali_max_voltage_)
	{
		disarm();
		set_error(ERROR_PHASE_RESISTANCE_OUT_OF_RANGE);
	}
	else if(vbus_voltage <= 6)   //限制电源电压不能低于6V
	{
		disarm();
		set_error(ERROR_UNKNOWN_VBUS_VOLTAGE);
	}
	else
	{
		float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
		test_mod_ = test_voltage_ * vfactor;
	}
}
/*************************************/
//代码被foc.c文件中的pwm_update_cb()函数调用
void Resistance_get_alpha_beta_output(void)
{
	float mod_alpha = test_mod_;
	float mod_beta = 0;
	
	enqueue_modulation_timings(mod_alpha, mod_beta);
}
/****************************************************************************/
uint32_t  start_timestamp_;
uint32_t  last_input_timestamp_;
float last_Ialpha_;
float deltaI_;
bool  attached_ = false;
/****************************************************************************/
void Inductance_reset(void)
{
	attached_ = 0;
	last_Ialpha_ = 0;
	deltaI_ = 0;
}
/*************************************/
// 电感测量的电流输入侧，在 current_meas_cb() 后执行
void Inductance_on_measurement(uint32_t input_timestamp)
{
	if(attached_)
	{
		float sign = test_voltage_ >= 0.0f ? 1.0f : -1.0f;
		deltaI_ += -sign * (Ialpha_beta[0] - last_Ialpha_);
	}
	else
	{
		start_timestamp_ = input_timestamp;   //第一次进入这个函数时的时间
		attached_ = 1;
	}
	
	last_Ialpha_ = Ialpha_beta[0];
	last_input_timestamp_ = input_timestamp;  //每进入一次记录一次，最后一次记录的就是最后一次的时间
}
/*************************************/
//代码被foc.c文件中的pwm_update_cb()函数调用
void Inductance_get_alpha_beta_output(void)
{
	test_voltage_ *= -1.0f;
	float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
	float mod_alpha = test_voltage_ * vfactor;
	float mod_beta = 0.0f;
	
	enqueue_modulation_timings(mod_alpha, mod_beta);
}
/****************************************************************************/
void update_current_controller_gains(void)
{
	// Calculate current control gains
	float p_gain = motor_config.current_control_bandwidth * motor_config.phase_inductance;
	float plant_pole = motor_config.phase_resistance / motor_config.phase_inductance;
	
	pi_gains_[0] = p_gain;
	pi_gains_[1] = plant_pole * p_gain;
}
/****************************************************************************/
bool measure_phase_resistance(float test_current, float max_voltage)
{
	uint32_t i;
	
	cali_target_current_ = test_current;
	cali_max_voltage_ = max_voltage;
	Resistance_reset();
	motor_control_mode_ = MOTOR_CONTROL_MODE_RESISTANCE;
	
	arm();
	
	for(i = 0; i < 3000; ++i)   //看起来这3秒钟像是什么也没干，实际在TIM1更新中断中运行
	{
		//if (!((axis_->requested_state_ == Axis::AXIS_STATE_UNDEFINED) && axis_->motor_.is_armed_))
		if(!is_armed_)break;
		osDelay(1);
	}
	
	bool success = is_armed_;
	disarm();
	
	if(success)motor_config.phase_resistance = test_voltage_ / test_current;
	
	if(motor_config.phase_resistance > 2)    //MOTOR_TYPE_HIGH_CURRENT的相电阻要小于2Ω，否则就是gimbal
	{
		set_error(ERROR_NOT_HIGH_CURRENT_MOTOR);
		success = 0;
	}
	
	if((fabsf(I_beta_) / test_current) > 0.2f)   //如果实际测量电流与设置电流误差比较大
	{
		set_error(ERROR_UNBALANCED_PHASES);
		success = 0;
	}
	
	motor_control_mode_ = MOTOR_CONTROL_MODE_NORMAL;
	return success;
}
/****************************************************************************/
bool measure_phase_inductance(float test_voltage)
{
	uint32_t i;
	
	test_voltage_ = test_voltage;
	Inductance_reset();
	motor_control_mode_ = MOTOR_CONTROL_MODE_INDUCTANCE;
	
	arm();
	
	for(i = 0; i < 1250; ++i)   //1.25秒，有整有零，为什么是这个时间？
	{
		//if (!((axis_->requested_state_ == Axis::AXIS_STATE_UNDEFINED) && axis_->motor_.is_armed_))
		if(!is_armed_)break;
		osDelay(1);
	}
	
	bool success = is_armed_;
	disarm();
	
	if(success)
	{
		float dt = (float)(last_input_timestamp_ - start_timestamp_) / (float)TIM_1_8_CLOCK_HZ; // at 216MHz this overflows after 19 seconds
		motor_config.phase_inductance = fabsf(test_voltage_) / (deltaI_ / dt);
	}
	
	// TODO arbitrary values set for now
	if (!(motor_config.phase_inductance >= 2e-6f && motor_config.phase_inductance <= 4000e-6f))
	{
		set_error(ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE);
		success = 0;
	}
	
	motor_control_mode_ = MOTOR_CONTROL_MODE_NORMAL;
	return success;
}
/****************************************************************************/
bool run_calibration(void)
{
	float R_calib_max_voltage = motor_config.resistance_calib_max_voltage;
	
	if(motor_config.motor_type == MOTOR_TYPE_HIGH_CURRENT)
	{
		if(!measure_phase_resistance(motor_config.calibration_current, R_calib_max_voltage))return 0;
		if(!measure_phase_inductance(R_calib_max_voltage))return 0;
	}
	else if(motor_config.motor_type == MOTOR_TYPE_GIMBAL)
	{
		// no calibration needed
	}
	else
	{
		return 0;
	}
	
	update_current_controller_gains();
	
	return 1;
}
/****************************************************************************/
void arm(void)
{
	is_armed_ = 1;
	TIM1->BDTR |= TIM_BDTR_MOE;
}
/*************************************/
void disarm(void)
{
	is_armed_ = 0;
	TIM1->BDTR &= ~TIM_BDTR_MOE;
}
/****************************************************************************/
/****************************************************************************/
void current_meas_cb(uint32_t timestamp, Iph_ABC_t *current)
{
	Iph_ABC_t current_meas;
	bool dc_calib_valid = (dc_calib_running_since_ >= motor_config.dc_calib_tau * 7.5f)    //1.5秒，需12000次
												&& (fabsf(DC_calib_.phA) < max_dc_calib_)
												&& (fabsf(DC_calib_.phB) < max_dc_calib_)
												&& (fabsf(DC_calib_.phC) < max_dc_calib_);
	if(dc_calib_valid)
	{
		current_meas.phA = current->phA - DC_calib_.phA;
		current_meas.phB = current->phB - DC_calib_.phB;
		current_meas.phC = current->phC - DC_calib_.phC;
	}
	else
	{
		current_meas.phA = 0;
		current_meas.phB = 0;
		current_meas.phC = 0;
	}
	
	on_measurement(timestamp, &current_meas);

	if(motor_control_mode_ == MOTOR_CONTROL_MODE_RESISTANCE)
	{
		Resistance_on_measurement();
	}
	else if(motor_control_mode_ == MOTOR_CONTROL_MODE_INDUCTANCE)
	{
		Inductance_on_measurement(timestamp);
	}
}
/****************************************************************************/
void dc_calib_cb(uint32_t timestamp, Iph_ABC_t *current)
{
	(void)timestamp;
	const float dc_calib_period = (float)(2U * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR + 1U)) / (float)TIM_1_8_CLOCK_HZ;  /* 2*3500*(2+1)/168000000=0.000125 */
	const float calib_filter_k = clampf(dc_calib_period / motor_config.dc_calib_tau, 0.0f, 1.0f);    //0.000625
	
	DC_calib_.phA += (current->phA - DC_calib_.phA) * calib_filter_k;
	DC_calib_.phB += (current->phB - DC_calib_.phB) * calib_filter_k;
	DC_calib_.phC += (current->phC - DC_calib_.phC) * calib_filter_k;
	dc_calib_running_since_ += dc_calib_period;
}
/****************************************************************************/
void pwm_update_cb(uint32_t output_timestamp)
{
	if(motor_control_mode_ == MOTOR_CONTROL_MODE_RESISTANCE)
	{
		Resistance_get_alpha_beta_output();
	}
	else if(motor_control_mode_ == MOTOR_CONTROL_MODE_INDUCTANCE)
	{
		Inductance_get_alpha_beta_output();
	}
	else
	{
		foc_pwm_update_cb(output_timestamp);
	}
}
/****************************************************************************/
