


#include "foc.h"

#include "arm_cos_f32.h"
#include "board.h"
#include "motor.h"
#include "open_loop_controller.h"
#include "tim.h"


#include <math.h>

/*****************************************************************************/
float  Ialpha_beta[2];
static float Id_measured_;
static float Iq_measured_;
static float v_current_control_integral_d_;
static float v_current_control_integral_q_;
/*****************************************************************************/
bool enqueue_modulation_timings(float mod_alpha, float mod_beta)
{
	float tA, tB, tC;
	if (SVM(mod_alpha, mod_beta, &tA, &tB, &tC) != 0)
	{
		set_error(ERROR_MODULATION_MAGNITUDE);
		return false;
	}
	TIM1->CCR1 = (uint16_t)(tA * (float)TIM_1_8_PERIOD_CLOCKS);
	TIM1->CCR2 = (uint16_t)(tB * (float)TIM_1_8_PERIOD_CLOCKS);
	TIM1->CCR3 = (uint16_t)(tC * (float)TIM_1_8_PERIOD_CLOCKS);
	return true;
}
/*****************************************************************************/
bool enqueue_voltage_timings(float v_alpha, float v_beta)
{
	float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
	float mod_alpha = vfactor * v_alpha;
	float mod_beta = vfactor * v_beta;
	if (!enqueue_modulation_timings(mod_alpha, mod_beta))
		return false;
	return true;
}
/****************************************************************************/
// We should probably make FOC Current call FOC Voltage to avoid duplication.
bool FOC_voltage(float v_d, float v_q, float pwm_phase)  //pwm_phase是park逆变换时的θ
{
	float c = our_arm_cos_f32(pwm_phase);
	float s = our_arm_sin_f32(pwm_phase);
	float v_alpha = c*v_d - s*v_q;
	float v_beta = c*v_q + s*v_d;
	return enqueue_voltage_timings(v_alpha, v_beta);
}
/*****************************************************************************/
static bool FOC_current(float Id_des, float Iq_des, float I_phase, float pwm_phase)
{
	float c_I = our_arm_cos_f32(I_phase);
	float s_I = our_arm_sin_f32(I_phase);
	float Id = c_I * Ialpha_beta[0] + s_I * Ialpha_beta[1];
	float Iq = c_I * Ialpha_beta[1] - s_I * Ialpha_beta[0];

	Id_measured_ = Id;
	Iq_measured_ = Iq;

	float Ierr_d = Id_des - Id;
	float Ierr_q = Iq_des - Iq;
	float Vd = v_current_control_integral_d_ + Ierr_d * pi_gains_[0];
	float Vq = v_current_control_integral_q_ + Ierr_q * pi_gains_[0];

	float mod_to_V = (2.0f / 3.0f) * vbus_voltage;
	float V_to_mod = 1.0f / mod_to_V;
	float mod_d = V_to_mod * Vd;
	float mod_q = V_to_mod * Vq;
	float mod_mag = sqrtf(mod_d * mod_d + mod_q * mod_q);
	float mod_scalefactor = (mod_mag > 0.0f) ? (0.80f * sqrt3_by_2 / mod_mag) : 1.0f;

	if (mod_scalefactor < 1.0f)
	{
		mod_d *= mod_scalefactor;
		mod_q *= mod_scalefactor;
		v_current_control_integral_d_ *= 0.99f;
		v_current_control_integral_q_ *= 0.99f;
	}
	else
	{
		v_current_control_integral_d_ += Ierr_d * (pi_gains_[1] * current_meas_period);
		v_current_control_integral_q_ += Ierr_q * (pi_gains_[1] * current_meas_period);
	}

	float c_p = our_arm_cos_f32(pwm_phase);
	float s_p = our_arm_sin_f32(pwm_phase);
	float mod_alpha = c_p * mod_d - s_p * mod_q;
	float mod_beta = c_p * mod_q + s_p * mod_d;

	return enqueue_modulation_timings(mod_alpha, mod_beta);
}
/*****************************************************************************/
void on_measurement(uint32_t input_timestamp, Iph_ABC_t *current)
{
	(void)input_timestamp;

	// Clark transform
	Ialpha_beta[0] = current->phA;
	Ialpha_beta[1] = one_by_sqrt3 * (current->phB - current->phC);
}
/*****************************************************************************/
void foc_pwm_update_cb(uint32_t output_timestamp)
{
	float dt = (float)(int32_t)(output_timestamp - openloop_controller_.timestamp_) / (float)TIM_1_8_CLOCK_HZ;
	float pwm_phase = openloop_controller_.phase_ + openloop_controller_.phase_vel_ * dt;
	if ((motor_config.motor_type == MOTOR_TYPE_GIMBAL) ||
	    (openloop_controller_.target_current_ == 0.0f))
	{
		FOC_voltage(openloop_controller_.Vdq_setpoint_.d, openloop_controller_.Vdq_setpoint_.q, pwm_phase);
	}
	else
	{
		FOC_current(openloop_controller_.Idq_setpoint_.d, openloop_controller_.Idq_setpoint_.q,
		            openloop_controller_.phase_, pwm_phase);
	}
}
/*****************************************************************************/



