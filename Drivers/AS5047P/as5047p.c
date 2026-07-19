#include "as5047p.h"

#include "main.h"
#include "spi.h"

#define AS5047P_NCS_GPIO_Port GPIO_5_GPIO_Port
#define AS5047P_NCS_Pin GPIO_5_Pin

static void as5047p_ncs_low(void)
{
	AS5047P_NCS_GPIO_Port->BSRR = ((uint32_t)AS5047P_NCS_Pin << 16U);
}

static void as5047p_ncs_high(void)
{
	AS5047P_NCS_GPIO_Port->BSRR = AS5047P_NCS_Pin;
}

static bool as5047p_spi3_transfer16(uint16_t tx, uint16_t *rx, uint32_t *error_flags)
{
	uint32_t timeout;
	uint16_t dummy;

	if ((SPI3->CR1 & SPI_CR1_SPE) == 0U)
	{
		SPI3->CR1 |= SPI_CR1_SPE;
	}

	if ((SPI3->SR & SPI_SR_OVR) != 0U)
	{
		dummy = *(__IO uint16_t *)&SPI3->DR;
		dummy = (uint16_t)SPI3->SR;
		(void)dummy;
	}

	if ((SPI3->SR & SPI_SR_RXNE) != 0U)
	{
		dummy = *(__IO uint16_t *)&SPI3->DR;
		(void)dummy;
	}

	timeout = 1000U;
	while (((SPI3->SR & SPI_SR_TXE) == 0U) && (timeout > 0U))
	{
		timeout--;
	}
	if (timeout == 0U)
	{
		*error_flags = 1U;
		return false;
	}

	*(__IO uint16_t *)&SPI3->DR = tx;

	timeout = 1000U;
	while (((SPI3->SR & SPI_SR_RXNE) == 0U) && (timeout > 0U))
	{
		timeout--;
	}
	if (timeout == 0U)
	{
		*error_flags = 8U;
		return false;
	}

	*rx = *(__IO uint16_t *)&SPI3->DR;

	timeout = 1000U;
	while (((SPI3->SR & SPI_SR_BSY) != 0U) && (timeout > 0U))
	{
		timeout--;
	}
	if (timeout == 0U)
	{
		*error_flags = 16U;
		return false;
	}

	return true;
}

static uint8_t as5047p_ams_parity(uint16_t value)
{
	value ^= value >> 8;
	value ^= value >> 4;
	value ^= value >> 2;
	value ^= value >> 1;

	return (uint8_t)(value & 1U);
}

void as5047p_init(void)
{
	GPIO_InitTypeDef gpio_init = {0};

	__HAL_RCC_GPIOC_CLK_ENABLE();

	gpio_init.Pin = AS5047P_NCS_Pin;
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull = GPIO_NOPULL;
	gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(AS5047P_NCS_GPIO_Port, &gpio_init);

	as5047p_ncs_high();
}

bool as5047p_read(AS5047P_SAMPLE *sample, int32_t *pos_abs)
{
	uint16_t raw_value = 0U;
	uint32_t error_flags = 0U;

	as5047p_ncs_low();
	bool ok = as5047p_spi3_transfer16(0xFFFFU, &raw_value, &error_flags);
	as5047p_ncs_high();

	sample->raw = raw_value;
	sample->error_flags = error_flags;

	if (!ok)
	{
		return false;
	}

	if ((as5047p_ams_parity(raw_value) != 0U) ||
	    (((raw_value >> 14U) & 1U) != 0U))
	{
		sample->error_flags =
			((as5047p_ams_parity(raw_value) != 0U) ? 2U : 0U) |
			((((raw_value >> 14U) & 1U) != 0U) ? 4U : 0U);
		return false;
	}

	sample->error_flags = 0U;
	*pos_abs = (int32_t)(raw_value & 0x3FFFU);
	return true;
}
