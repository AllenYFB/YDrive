#ifndef AS5047P_H
#define AS5047P_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	uint16_t raw;
	uint32_t error_flags;
} AS5047P_SAMPLE;

void as5047p_init(void);
bool as5047p_read(AS5047P_SAMPLE *sample, int32_t *pos_abs);

#endif
