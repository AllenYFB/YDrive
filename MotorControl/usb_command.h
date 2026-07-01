#ifndef USB_COMMAND_H
#define USB_COMMAND_H

#include <stdint.h>

void usb_command_receive(const uint8_t *data, uint32_t length);

#endif
