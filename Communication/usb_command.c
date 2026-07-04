#include "usb_command.h"

#include <stdlib.h>
#include <string.h>

#include "motor_axis.h"
#include "usbd_cdc_if.h"

static void usb_command_send(const char *msg);
static void usb_command_handle_line(char *line);
static void usb_command_handle_open(const UsbCommandLine *command);
static void usb_command_handle_current(const UsbCommandLine *command);
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

    motor_axis_set_open_loop_target(voltage, electrical_phase_vel);
    motor_axis_set_open_loop_enabled(1U);
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

    motor_axis_set_open_loop_current_target(iq_setpoint, electrical_phase_vel);
    motor_axis_set_open_loop_current_enabled(1U);
    usb_command_send("OK cur\r\n");
}

static void usb_command_handle_stop(void)
{
    motor_axis_set_open_loop_enabled(0U);
    usb_command_send("OK stop\r\n");
}

static void usb_command_handle_help(void)
{
    usb_command_send("cmd: open <voltage_mod> <electrical_rad_s>, cur <iq_amp> <electrical_rad_s>, stop\r\n");
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

