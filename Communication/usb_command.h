#ifndef USB_COMMAND_H
#define USB_COMMAND_H

#include <stdint.h>

extern uint8_t usb_recbuff[128];
extern uint8_t usb_sndbuff[128];
extern uint32_t usb_rcv_count;

void usb_command_receive(const uint8_t *data, uint32_t length);
void USBcommander_run(void);
void usb_send(uint8_t *data, uint32_t len);

#endif
