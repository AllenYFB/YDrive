#include "drv8301.h"

#include "main.h"
#include "cmsis_os.h"

#define DRV8301_RW_READ (1U << 15)
#define DRV8301_RW_WRITE 0U
#define DRV8301_REG_SHIFT 11U
#define DRV8301_DATA_MASK 0x07FFU

#define DRV8301_STATUS1_FAULT (1U << 10)

#define DRV8301_CTRL1_GATE_CURRENT_0P25A (2U << 0)
#define DRV8301_CTRL1_GATE_RESET_NORMAL (0U << 2)
#define DRV8301_CTRL1_PWM_MODE_SIX_INPUTS (0U << 3)
#define DRV8301_CTRL1_OC_MODE_LATCH_SHUTDOWN (1U << 4)
#define DRV8301_CTRL1_OC_ADJ_0P730V (21U << 6)

#define DRV8301_CTRL2_OCTW_BOTH (0U << 0)
#define DRV8301_CTRL2_GAIN_SHIFT 2U
#define DRV8301_CTRL2_DC_CAL_CH1_LOAD (0U << 4)
#define DRV8301_CTRL2_DC_CAL_CH2_LOAD (0U << 5)
#define DRV8301_CTRL2_OC_TOFF_NORMAL (0U << 6)

static uint16_t drv8301_transfer(Drv8301 *drv, uint16_t tx_word);
static float drv8301_gain_to_float(Drv8301ShuntGain gain);

void drv8301_init(Drv8301 *drv)
{
    if (drv == 0) {
        return;
    }

    drv->spi = &hspi3;
    drv->enable_port = EN_GATE_GPIO_Port;
    drv->enable_pin = EN_GATE_Pin;
    drv->ncs_port = M0_nCS_GPIO_Port;
    drv->ncs_pin = M0_nCS_Pin;
    drv->nfault_port = nFAULT_GPIO_Port;
    drv->nfault_pin = nFAULT_Pin;
    drv->shunt_gain = DRV8301_SHUNT_GAIN_10;
    drv->shunt_gain_v_per_v = drv8301_gain_to_float(drv->shunt_gain);
    drv->status_1 = 0U;
    drv->status_2 = 0U;
    drv->control_1 = 0U;
    drv->control_2 = 0U;

    HAL_GPIO_WritePin(drv->ncs_port, drv->ncs_pin, GPIO_PIN_SET);
}

uint32_t drv8301_setup(Drv8301 *drv)
{
    if (drv == 0) {
        return 0U;
    }

    HAL_GPIO_WritePin(drv->enable_port, drv->enable_pin, GPIO_PIN_SET);
    osDelay(10U);

    drv->status_1 = drv8301_read_register(drv, DRV8301_REG_STATUS_1);
    if ((drv->status_1 & DRV8301_STATUS1_FAULT) != 0U) {
        return 0U;
    }

    drv->control_1 = DRV8301_CTRL1_GATE_CURRENT_0P25A |
                     DRV8301_CTRL1_GATE_RESET_NORMAL |
                     DRV8301_CTRL1_PWM_MODE_SIX_INPUTS |
                     DRV8301_CTRL1_OC_MODE_LATCH_SHUTDOWN |
                     DRV8301_CTRL1_OC_ADJ_0P730V;
    drv8301_write_register(drv, DRV8301_REG_CONTROL_1, drv->control_1);

    drv->shunt_gain = DRV8301_SHUNT_GAIN_10;
    drv->shunt_gain_v_per_v = drv8301_gain_to_float(drv->shunt_gain);
    drv->control_2 = DRV8301_CTRL2_OCTW_BOTH |
                     ((uint16_t)drv->shunt_gain << DRV8301_CTRL2_GAIN_SHIFT) |
                     DRV8301_CTRL2_DC_CAL_CH1_LOAD |
                     DRV8301_CTRL2_DC_CAL_CH2_LOAD |
                     DRV8301_CTRL2_OC_TOFF_NORMAL;
    drv8301_write_register(drv, DRV8301_REG_CONTROL_2, drv->control_2);

    drv->status_1 = drv8301_read_register(drv, DRV8301_REG_STATUS_1);
    drv->status_2 = drv8301_read_register(drv, DRV8301_REG_STATUS_2);
    drv->control_1 = drv8301_read_register(drv, DRV8301_REG_CONTROL_1);
    drv->control_2 = drv8301_read_register(drv, DRV8301_REG_CONTROL_2);

    return ((drv->status_1 & DRV8301_STATUS1_FAULT) == 0U) ? 1U : 0U;
}

uint32_t drv8301_check_fault(Drv8301 *drv)
{
    if (drv == 0) {
        return 1U;
    }

    if (HAL_GPIO_ReadPin(drv->nfault_port, drv->nfault_pin) == GPIO_PIN_RESET) {
        drv->status_1 = drv8301_read_register(drv, DRV8301_REG_STATUS_1);
        drv->status_2 = drv8301_read_register(drv, DRV8301_REG_STATUS_2);
        return 1U;
    }

    return 0U;
}

float drv8301_get_shunt_gain(const Drv8301 *drv)
{
    if (drv == 0) {
        return 10.0f;
    }

    return drv->shunt_gain_v_per_v;
}

uint16_t drv8301_read_register(Drv8301 *drv, Drv8301Register reg)
{
    uint16_t command;

    if (drv == 0) {
        return 0U;
    }

    command = DRV8301_RW_READ |
              ((uint16_t)reg << DRV8301_REG_SHIFT);

    (void)drv8301_transfer(drv, command);
    return drv8301_transfer(drv, command) & DRV8301_DATA_MASK;
}

void drv8301_write_register(Drv8301 *drv, Drv8301Register reg, uint16_t value)
{
    uint16_t command;

    if (drv == 0) {
        return;
    }

    command = DRV8301_RW_WRITE |
              ((uint16_t)reg << DRV8301_REG_SHIFT) |
              (value & DRV8301_DATA_MASK);

    (void)drv8301_transfer(drv, command);
}

static uint16_t drv8301_transfer(Drv8301 *drv, uint16_t tx_word)
{
    uint16_t rx_word = 0U;

    HAL_GPIO_WritePin(drv->ncs_port, drv->ncs_pin, GPIO_PIN_RESET);
    (void)HAL_SPI_TransmitReceive(drv->spi,
                                  (uint8_t *)&tx_word,
                                  (uint8_t *)&rx_word,
                                  1U,
                                  10U);
    HAL_GPIO_WritePin(drv->ncs_port, drv->ncs_pin, GPIO_PIN_SET);

    return rx_word;
}

static float drv8301_gain_to_float(Drv8301ShuntGain gain)
{
    switch (gain) {
    case DRV8301_SHUNT_GAIN_20:
        return 20.0f;

    case DRV8301_SHUNT_GAIN_40:
        return 40.0f;

    case DRV8301_SHUNT_GAIN_80:
        return 80.0f;

    case DRV8301_SHUNT_GAIN_10:
    default:
        return 10.0f;
    }
}
