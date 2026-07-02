#ifndef USB_COMMAND_H
#define USB_COMMAND_H

#include <stdint.h>

typedef enum {
    USB_COMMAND_OPEN,
    USB_COMMAND_STOP,
    USB_COMMAND_IDLE,
    USB_COMMAND_HELP,
    USB_COMMAND_UNKNOWN,
} UsbCommandType;

typedef struct {
    UsbCommandType type;
    char *args;
} UsbCommandLine;

void usb_command_receive(const uint8_t *data, uint32_t length);
UsbCommandLine usb_command_parse_line(char *line);

#endif
