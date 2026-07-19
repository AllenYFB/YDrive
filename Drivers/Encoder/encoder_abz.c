#include "encoder_abz.h"

#include "tim.h"

void encoder_abz_init(void)
{
	TIM3->CNT = 0U;
	(void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

int16_t encoder_abz_sample(void)
{
	return (int16_t)TIM3->CNT;
}
