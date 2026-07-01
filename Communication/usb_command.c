#include "usb_command.h"

#include <stdlib.h>
#include <string.h>

#include "control_loop.h"
#include "usbd_cdc_if.h"

static void usb_command_send(const char *msg);
static void usb_command_handle_line(char *line);
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
    float voltage;
    float electrical_phase_vel;
    char *args;

    if (strncmp(line, "open", 4U) == 0) {
        args = line + 4;
        if ((parse_float_arg(&args, &voltage) != 0U) &&
            (parse_float_arg(&args, &electrical_phase_vel) != 0U)) {
            args = skip_spaces(args);
            if (*args != '\0') {
                usb_command_send("ERR open args\r\n");
                return;
            }

            control_loop_set_open_loop_target(voltage, electrical_phase_vel);
            control_loop_set_open_loop_enabled(1U);
            usb_command_send("OK open\r\n");
            return;
        }

        usb_command_send("ERR open usage\r\n");
        return;
    }

    if ((strcmp(line, "stop") == 0) || (strcmp(line, "idle") == 0)) {
        control_loop_set_open_loop_enabled(0U);
        usb_command_send("OK stop\r\n");
        return;
    }

    if (strcmp(line, "help") == 0) {
        usb_command_send("cmd: open <voltage_mod> <electrical_rad_s>, stop\r\n");
        return;
    }

    usb_command_send("ERR unknown cmd\r\n");
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
