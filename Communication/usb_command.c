#include "usb_command.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motor_axis.h"
#include "usbd_cdc_if.h"

static void usb_command_send(const char *msg);
static void usb_command_handle_line(char *line);
static void usb_command_handle_open(const UsbCommandLine *command);
static void usb_command_handle_current(const UsbCommandLine *command);
static void usb_command_handle_current_param(const UsbCommandLine *command);
static void usb_command_handle_current_gain(const UsbCommandLine *command);
static void usb_command_handle_stop(void);
static void usb_command_handle_help(void);
static char *skip_spaces(char *p);
static uint32_t parse_float_arg(char **p, float *value);

void usb_command_receive(const uint8_t *data, uint32_t length)
{
    static char line_buf[96];
    static uint32_t line_len;

    for (uint32_t i = 0; i < length; ++i) {
        char ch = (char)data[i];

        if ((ch == '\r') || (ch == '\n')) {
            if (line_len > 0U) {
                line_buf[line_len] = '\0';
                usb_command_handle_line(line_buf);
                line_len = 0U;
            }
            continue;
        }

        if (line_len < (sizeof(line_buf) - 1U)) {
            line_buf[line_len++] = ch;
        } else {
            line_len = 0U;
            usb_command_send("ERR line too long\r\n");
        }
    }
}

static void usb_command_handle_line(char *line)
{
    UsbCommandLine command = usb_command_parse_line(line);

    switch (command.type) {
    case USB_COMMAND_OPEN:
        usb_command_handle_open(&command);
        break;

    case USB_COMMAND_CURRENT:
        usb_command_handle_current(&command);
        break;

    case USB_COMMAND_CURRENT_PARAM:
        usb_command_handle_current_param(&command);
        break;

    case USB_COMMAND_CURRENT_GAIN:
        usb_command_handle_current_gain(&command);
        break;

    case USB_COMMAND_STOP:
    case USB_COMMAND_IDLE:
        usb_command_handle_stop();
        break;

    case USB_COMMAND_HELP:
        usb_command_handle_help();
        break;

    case USB_COMMAND_UNKNOWN:
    default:
        usb_command_send("ERR unknown cmd\r\n");
        break;
    }
}

UsbCommandLine usb_command_parse_line(char *line)
{
    UsbCommandLine command = {
        .type = USB_COMMAND_UNKNOWN,
        .args = line,
    };

    if (line == 0) {
        return command;
    }

    if (strncmp(line, "open", 4U) == 0) {
        command.type = USB_COMMAND_OPEN;
        command.args = line + 4;
        return command;
    }

    if (strncmp(line, "cur", 3U) == 0) {
        command.type = USB_COMMAND_CURRENT;
        command.args = line + 3;
        return command;
    }

    if (strncmp(line, "cparam", 6U) == 0) {
        command.type = USB_COMMAND_CURRENT_PARAM;
        command.args = line + 6;
        return command;
    }

    if (strncmp(line, "cgain", 5U) == 0) {
        command.type = USB_COMMAND_CURRENT_GAIN;
        command.args = line + 5;
        return command;
    }

    if (strcmp(line, "stop") == 0) {
        command.type = USB_COMMAND_STOP;
        command.args = line + 4;
        return command;
    }

    if (strcmp(line, "idle") == 0) {
        command.type = USB_COMMAND_IDLE;
        command.args = line + 4;
        return command;
    }

    if (strcmp(line, "help") == 0) {
        command.type = USB_COMMAND_HELP;
        command.args = line + 4;
        return command;
    }

    return command;
}

static void usb_command_handle_open(const UsbCommandLine *command)
{
    float voltage;
    float electrical_phase_vel;
    char *args;

    if (command == 0) {
        usb_command_send("ERR open usage\r\n");
        return;
    }

    args = command->args;

    if ((parse_float_arg(&args, &voltage) == 0U) ||
        (parse_float_arg(&args, &electrical_phase_vel) == 0U)) {
        usb_command_send("ERR open usage\r\n");
        return;
    }

    args = skip_spaces(args);
    if (*args != '\0') {
        usb_command_send("ERR open args\r\n");
        return;
    }

    motor_axis_start_open_voltage(voltage, electrical_phase_vel);
    usb_command_send("OK open\r\n");
}

static void usb_command_handle_current(const UsbCommandLine *command)
{
    float iq_setpoint;
    float electrical_phase_vel;
    char *args;

    if (command == 0) {
        usb_command_send("ERR cur usage\r\n");
        return;
    }

    args = command->args;

    if ((parse_float_arg(&args, &iq_setpoint) == 0U) ||
        (parse_float_arg(&args, &electrical_phase_vel) == 0U)) {
        usb_command_send("ERR cur usage\r\n");
        return;
    }

    args = skip_spaces(args);
    if (*args != '\0') {
        usb_command_send("ERR cur args\r\n");
        return;
    }

    motor_axis_start_open_current(iq_setpoint, electrical_phase_vel);
    usb_command_send("OK cur\r\n");
}

static void usb_command_handle_current_param(const UsbCommandLine *command)
{
    float phase_current_gain;
    float current_limit;
    float max_voltage_mod;
    char *args;

    if (command == 0) {
        usb_command_send("ERR cparam usage\r\n");
        return;
    }

    args = skip_spaces(command->args);
    if (*args == '\0') {
        MotorAxisStatus status;
        char msg[128];
        motor_axis_get_status(&status);
        (void)snprintf(msg, sizeof(msg),
                       "cparam gain_u=%ld limit_ma=%ld max_mod_milli=%ld\r\n",
                       (long)(status.current.phase_gain * 1000000.0f),
                       (long)(status.current.limit * 1000.0f),
                       (long)(status.current.max_voltage_mod * 1000.0f));
        usb_command_send(msg);
        return;
    }

    if ((parse_float_arg(&args, &phase_current_gain) == 0U) ||
        (parse_float_arg(&args, &current_limit) == 0U) ||
        (parse_float_arg(&args, &max_voltage_mod) == 0U)) {
        usb_command_send("ERR cparam usage\r\n");
        return;
    }

    args = skip_spaces(args);
    if (*args != '\0') {
        usb_command_send("ERR cparam args\r\n");
        return;
    }

    motor_axis_set_current_config(phase_current_gain, current_limit, max_voltage_mod);
    usb_command_send("OK cparam\r\n");
}

static void usb_command_handle_current_gain(const UsbCommandLine *command)
{
    float p_gain;
    float i_gain;
    char *args;

    if (command == 0) {
        usb_command_send("ERR cgain usage\r\n");
        return;
    }

    args = skip_spaces(command->args);
    if (*args == '\0') {
        MotorAxisStatus status;
        char msg[96];
        motor_axis_get_status(&status);
        (void)snprintf(msg, sizeof(msg),
                       "cgain p_u=%ld i_u=%ld\r\n",
                       (long)(status.current.p_gain * 1000000.0f),
                       (long)(status.current.i_gain * 1000000.0f));
        usb_command_send(msg);
        return;
    }

    if ((parse_float_arg(&args, &p_gain) == 0U) ||
        (parse_float_arg(&args, &i_gain) == 0U)) {
        usb_command_send("ERR cgain usage\r\n");
        return;
    }

    args = skip_spaces(args);
    if (*args != '\0') {
        usb_command_send("ERR cgain args\r\n");
        return;
    }

    motor_axis_set_current_pi(p_gain, i_gain);
    usb_command_send("OK cgain\r\n");
}

static void usb_command_handle_stop(void)
{
    motor_axis_stop();
    usb_command_send("OK stop\r\n");
}

static void usb_command_handle_help(void)
{
    usb_command_send("cmd: open <voltage_mod> <electrical_rad_s>, cur <iq_amp> <electrical_rad_s>, cparam [gain limit max_mod], cgain [p i], stop\r\n");
}

static char *skip_spaces(char *p)
{
    while ((*p == ' ') || (*p == '\t')) {
        p++;
    }

    return p;
}

static uint32_t parse_float_arg(char **p, float *value)
{
    char *start = skip_spaces(*p);
    char *end = start;
    float parsed = strtof(start, &end);

    if (end == start) {
        return 0U;
    }

    *value = parsed;
    *p = end;
    return 1U;
}

static void usb_command_send(const char *msg)
{
    (void)CDC_Transmit_FS((uint8_t *)msg, (uint16_t)strlen(msg));
}

