#include "axis.h"

#include "encode.h"
#include "foc.h"
#include "motor.h"
#include "open_loop_controller.h"
#include "usb_command.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/*****************************************************************************/
AXIS_State_Type current_state_;
extern uint8_t usb_sndbuff[128];
/*****************************************************************************/
static int32_t float_to_i32_round(float value)
{
	return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}
/*****************************************************************************/
static void axis_send_calib_line(const char *format, long value0, long value1)
{
	int len = snprintf((char *)usb_sndbuff, sizeof(usb_sndbuff), format, value0, value1);

	if ((len > 0) && ((uint32_t)len < sizeof(usb_sndbuff)))
	{
		usb_send(usb_sndbuff, (uint32_t)len);
	}
}
/*****************************************************************************/
static void axis_send_status_line(bool status)
{
	int len = snprintf((char *)usb_sndbuff, sizeof(usb_sndbuff),
	                   "status=%d,error=%lX.\r\n",
	                   (int)status, (unsigned long)motor_error);

	if ((len > 0) && ((uint32_t)len < sizeof(usb_sndbuff)))
	{
		usb_send(usb_sndbuff, (uint32_t)len);
	}
}
/*****************************************************************************/
static void axis_enter_open_loop_control(void)
{
	open_loop_controller_start();
	arm();
}
/*****************************************************************************/
void run_state_machine_loop(void)
{
	bool status = false;
	static AXIS_State_Type previous_state = AXIS_STATE_UNDEFINED;
	bool state_changed = (current_state_ != previous_state);

	switch (current_state_)
	{
		case AXIS_STATE_MOTOR_CALIBRATION: {
			status = run_calibration();

			axis_send_calib_line("motor R_mohm=%ld,L_uH=%ld\r\n",
			                     (long)float_to_i32_round(motor_config.phase_resistance * 1000.0f),
			                     (long)float_to_i32_round(motor_config.phase_inductance * 1000000.0f));
			axis_send_calib_line("P_milli=%ld,I_milli=%ld\r\n",
			                     (long)float_to_i32_round(pi_gains_[0] * 1000.0f),
			                     (long)float_to_i32_round(pi_gains_[1] * 1000.0f));
			axis_send_status_line(status);

			current_state_ = AXIS_STATE_IDLE;
			disarm();
		} break;

		case AXIS_STATE_ENCODER_OFFSET_CALIBRATION: {
			status = encoder_run_offset_calibration();
			axis_send_calib_line("enc init=%ld,final=%ld\r\n",
			                     (long)encoder_state.calib_init_count,
			                     (long)encoder_state.calib_final_count);
			axis_send_calib_line("enc resp=%ld,exp=%ld\r\n",
			                     (long)float_to_i32_round(encoder_state.calib_scan_response),
			                     (long)float_to_i32_round(encoder_state.calib_scan_expected));
			axis_send_calib_line("enc err_pm=%ld,dir=%ld\r\n",
			                     (long)float_to_i32_round(encoder_state.calib_scan_error * 1000.0f),
			                     (long)encoder_config.direction);
			axis_send_status_line(status);
			current_state_ = AXIS_STATE_IDLE;
		} break;

		case AXIS_STATE_LOCKIN_SPIN: {
			if (state_changed)
			{
				axis_enter_open_loop_control();
			}
		} break;

		case AXIS_STATE_IDLE: {
			if (state_changed)
			{
				disarm();
			}
		} break;

		case AXIS_STATE_UNDEFINED: {
		} break;

		default: {
			status = false;
		} break;
	}

	previous_state = current_state_;
}
/*****************************************************************************/
void sampling_cb(void)
{
	encoder_sample_now();
}
/*****************************************************************************/
void control_loop_cb(uint32_t timestamp)
{
	(void)encoder_update();
	open_loop_controller_update(timestamp);
}
/*****************************************************************************/
