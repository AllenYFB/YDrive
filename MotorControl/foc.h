
#ifndef __FOC_LIB_H
#define __FOC_LIB_H

#include "utils.h"

#include <stdbool.h>
#include <stdint.h>


/****************************************************************************/
extern  uint32_t  motor_error;
extern  float  Ialpha_beta[2];

extern  float  pi_gains_[2];
/****************************************************************************/
bool enqueue_modulation_timings(float mod_alpha, float mod_beta);
void on_measurement(uint32_t input_timestamp, Iph_ABC_t *current);
void foc_pwm_update_cb(uint32_t output_timestamp);
/****************************************************************************/
static inline void set_error(uint32_t error)
{
	motor_error |= error;
}
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/


#endif

