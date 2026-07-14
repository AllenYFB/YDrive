


#include "foc.h"

#include "arm_cos_f32.h"
#include "board.h"
#include "motor.h"
#include "open_loop_controller.h"
#include "tim.h"



/*****************************************************************************/
float  Ialpha_beta[2];
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
	FOC_voltage(openloop_controller_.Vdq_setpoint_.d, openloop_controller_.Vdq_setpoint_.q, pwm_phase);
}
/*****************************************************************************/



