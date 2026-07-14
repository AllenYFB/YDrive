
#include "open_loop_controller.h"

#include "motor.h"

/****************************************************************************/
#define OPEN_LOOP_DEFAULT_TARGET_VOLTAGE 0.6f
#define OPEN_LOOP_DEFAULT_TARGET_VEL 62.8f
#define OPEN_LOOP_DEFAULT_VOLTAGE_RAMP 2.0f
#define OPEN_LOOP_DEFAULT_PHASE_VEL_RAMP 1000.0f
/****************************************************************************/
OPENLOOP_struct openloop_controller_;
/****************************************************************************/
void open_loop_controller_start(void)
{
	openloop_controller_.timestamp_ = 0U;
	openloop_controller_.Idq_setpoint_ = (float2D){0.0f, 0.0f};
	openloop_controller_.Vdq_setpoint_ = (float2D){0.0f, 0.0f};
	openloop_controller_.phase_ = 0.0f;
	openloop_controller_.phase_vel_ = 0.0f;
	openloop_controller_.total_distance_ = 0.0f;

	if(openloop_controller_.target_voltage_ == 0.0f)
	{
		openloop_controller_.target_voltage_ = OPEN_LOOP_DEFAULT_TARGET_VOLTAGE;
	}
	if(openloop_controller_.target_vel_ == 0.0f)
	{
		openloop_controller_.target_vel_ = OPEN_LOOP_DEFAULT_TARGET_VEL;
	}
	openloop_controller_.max_current_ramp_ = motor_config.calibration_current * 2.0f;
	openloop_controller_.max_voltage_ramp_ = OPEN_LOOP_DEFAULT_VOLTAGE_RAMP;
	openloop_controller_.max_phase_vel_ramp_ = OPEN_LOOP_DEFAULT_PHASE_VEL_RAMP;
}
/****************************************************************************/
void open_loop_controller_update(uint32_t timestamp)
{
	float prev_Id = openloop_controller_.Idq_setpoint_.d;
	float prev_Vd = openloop_controller_.Vdq_setpoint_.d;
	float phase = openloop_controller_.phase_;
	float phase_vel = openloop_controller_.phase_vel_;

	if (openloop_controller_.timestamp_ == 0U) {
		openloop_controller_.timestamp_ = timestamp;
	}

	float dt = (float)(timestamp - openloop_controller_.timestamp_) / (float)TIM_1_8_CLOCK_HZ;

	openloop_controller_.Idq_setpoint_.d = clampf(openloop_controller_.target_current_,
	                                             prev_Id - openloop_controller_.max_current_ramp_ * dt,
	                                             prev_Id + openloop_controller_.max_current_ramp_ * dt);
	openloop_controller_.Idq_setpoint_.q = 0.0f;

	openloop_controller_.Vdq_setpoint_.d = clampf(openloop_controller_.target_voltage_,
	                                             prev_Vd - openloop_controller_.max_voltage_ramp_ * dt,
	                                             prev_Vd + openloop_controller_.max_voltage_ramp_ * dt);
	openloop_controller_.Vdq_setpoint_.q = 0.0f;
	
	phase_vel = clampf(openloop_controller_.target_vel_,
	                   phase_vel - openloop_controller_.max_phase_vel_ramp_ * dt,
	                   phase_vel + openloop_controller_.max_phase_vel_ramp_ * dt);
	openloop_controller_.phase_vel_ = phase_vel;
	openloop_controller_.phase_ = wrap_pm_pi(phase + phase_vel * dt);
	openloop_controller_.total_distance_ += phase_vel * dt;
	openloop_controller_.timestamp_ = timestamp;
}
/****************************************************************************/

