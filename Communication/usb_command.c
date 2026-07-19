#include "usb_command.h"

#include "axis.h"
#include "encode.h"
#include "foc.h"
#include "motor.h"
#include "open_loop_controller.h"
#include "tim.h"
#include "usbd_cdc_if.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

uint8_t usb_recbuff[128];
uint8_t usb_sndbuff[256];
uint32_t usb_rcv_count;

static void usb_command_clear_rx(void);
static char *skip_spaces(char *p);
static uint8_t parse_float_arg(char **p, float *value);
static int32_t float_to_i32_round(float value);
static void usb_send_snprintf(int len);

void usb_command_receive(const uint8_t *data, uint32_t length)
{
    if ((data == 0) || (length == 0U)) {
        return;
    }

    if (length >= sizeof(usb_recbuff)) {
        length = sizeof(usb_recbuff) - 1U;
    }

    memcpy(usb_recbuff, data, length);
    usb_recbuff[length] = 0U;
    usb_rcv_count = length;
}

void usb_send(uint8_t *data, uint32_t len)
{
    if ((data == 0) || (len == 0U)) {
        return;
    }

    if (len > 0xFFFFU) {
        len = 0xFFFFU;
    }

    (void)CDC_Transmit_FS(data, (uint16_t)len);
}

void USBcommander_run(void)
{
    int len;

    if (usb_rcv_count == 0U) {
        return;
    }

    switch (usb_recbuff[0]) {
    case 'H':
    case 'h':
    case '?':
        len = snprintf((char *)usb_sndbuff, sizeof(usb_sndbuff),
                       "YDrive\r\n"
                       "H: help\r\n"
                       "C: motor R/L\r\n"
                       "E: encoder\r\n"
                       "A: enc calib\r\n"
                       "X: clear error\r\n"
                       "O [V rad/s]\r\n"
                       "S: stop\r\n");
        if ((len > 0) && ((uint32_t)len < sizeof(usb_sndbuff))) {
            usb_send(usb_sndbuff, (uint32_t)len);
        }
        break;

    case 'C':
    case 'c':
        current_state_ = AXIS_STATE_MOTOR_CALIBRATION;
        len = snprintf((char *)usb_sndbuff, sizeof(usb_sndbuff), "OK: motor calibration start\r\n");
        if ((len > 0) && ((uint32_t)len < sizeof(usb_sndbuff))) {
            usb_send(usb_sndbuff, (uint32_t)len);
        }
        break;

    case 'E':
    case 'e':
        len = snprintf((char *)usb_sndbuff, sizeof(usb_sndbuff),
                       "enc mode=%d cnt=%ld abs=%ld raw=%04X spi_flags=%lX shadow=%ld cpr=%ld pos_mturn=%ld vel_mturn_s=%ld ready=%d spi_err_pm=%ld error=%lX\r\n",
                       (int)encoder_config.mode,
                       (long)(int16_t)TIM3->CNT,
                       (long)encoder_state.pos_abs,
                       (unsigned int)encoder_state.abs_spi_raw,
                       (unsigned long)encoder_state.abs_spi_error_flags,
                       (long)encoder_state.shadow_count,
                       (long)encoder_config.cpr,
                       (long)float_to_i32_round(encoder_state.pos_estimate * 1000.0f),
                       (long)float_to_i32_round(encoder_state.vel_estimate * 1000.0f),
                       (int)encoder_state.is_ready,
                       (long)float_to_i32_round(encoder_state.spi_error_rate * 1000.0f),
                       (unsigned long)motor_error);
        usb_send_snprintf(len);
        break;

    case 'A':
    case 'a':
        current_state_ = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
        len = snprintf((char *)usb_sndbuff, sizeof(usb_sndbuff), "OK: encoder offset calibration start\r\n");
        usb_send_snprintf(len);
        break;

    case 'X':
    case 'x':
        motor_error = ERROR_NONE;
        len = snprintf((char *)usb_sndbuff, sizeof(usb_sndbuff), "OK: error cleared\r\n");
        usb_send_snprintf(len);
        break;

    case 'O':
    case 'o': {
        float voltage = openloop_controller_.target_voltage_;
        float velocity = openloop_controller_.target_vel_;
        char *args = (char *)usb_recbuff + 1;
        uint8_t has_args;

        args = skip_spaces(args);
        has_args = (*args != '\0') ? 1U : 0U;

        if (has_args != 0U) {
            if ((parse_float_arg(&args, &voltage) == 0U) ||
                (parse_float_arg(&args, &velocity) == 0U) ||
                (*skip_spaces(args) != '\0')) {
                len = snprintf((char *)usb_sndbuff, sizeof(usb_sndbuff), "ERR: O volt vel\r\n");
                usb_send_snprintf(len);
                break;
            }

            openloop_controller_.target_voltage_ = voltage;
            openloop_controller_.target_vel_ = velocity;
        }

        current_state_ = AXIS_STATE_LOCKIN_SPIN;
        len = snprintf((char *)usb_sndbuff, sizeof(usb_sndbuff),
                       "OK: open loop voltage_mV=%ld vel_mrad_s=%ld\r\n",
                       (long)float_to_i32_round(openloop_controller_.target_voltage_ * 1000.0f),
                       (long)float_to_i32_round(openloop_controller_.target_vel_ * 1000.0f));
        usb_send_snprintf(len);
    } break;

    case 'S':
    case 's':
        current_state_ = AXIS_STATE_IDLE;
        len = snprintf((char *)usb_sndbuff, sizeof(usb_sndbuff), "OK: stop\r\n");
        if ((len > 0) && ((uint32_t)len < sizeof(usb_sndbuff))) {
            usb_send(usb_sndbuff, (uint32_t)len);
        }
        break;

    default:
        len = snprintf((char *)usb_sndbuff, sizeof(usb_sndbuff), "ERR: unknown command, send H\r\n");
        if ((len > 0) && ((uint32_t)len < sizeof(usb_sndbuff))) {
            usb_send(usb_sndbuff, (uint32_t)len);
        }
        break;
    }

    usb_command_clear_rx();
}

static void usb_command_clear_rx(void)
{
    memset(usb_recbuff, 0, sizeof(usb_recbuff));
    usb_rcv_count = 0U;
}

static char *skip_spaces(char *p)
{
    while ((*p == ' ') || (*p == '\t') || (*p == '\r') || (*p == '\n')) {
        p++;
    }

    return p;
}

static uint8_t parse_float_arg(char **p, float *value)
{
    char *start;
    char *end;

    if ((p == 0) || (*p == 0) || (value == 0)) {
        return 0U;
    }

    start = skip_spaces(*p);
    end = start;
    *value = strtof(start, &end);

    if (end == start) {
        return 0U;
    }

    *p = end;
    return 1U;
}

static int32_t float_to_i32_round(float value)
{
    return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void usb_send_snprintf(int len)
{
    if ((len > 0) && ((uint32_t)len < sizeof(usb_sndbuff))) {
        usb_send(usb_sndbuff, (uint32_t)len);
    }
}
