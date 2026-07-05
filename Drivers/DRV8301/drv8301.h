#ifndef DRV8301_H
#define DRV8301_H

#include <stdint.h>

#include "gpio.h"
#include "spi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DRV8301_REG_STATUS_1 = 0U,
    DRV8301_REG_STATUS_2 = 1U,
    DRV8301_REG_CONTROL_1 = 2U,
    DRV8301_REG_CONTROL_2 = 3U,
} Drv8301Register;

typedef enum {
    DRV8301_SHUNT_GAIN_10 = 0U,
    DRV8301_SHUNT_GAIN_20 = 1U,
    DRV8301_SHUNT_GAIN_40 = 2U,
    DRV8301_SHUNT_GAIN_80 = 3U,
} Drv8301ShuntGain;

typedef struct {
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *enable_port;
    uint16_t enable_pin;
    GPIO_TypeDef *ncs_port;
    uint16_t ncs_pin;
    GPIO_TypeDef *nfault_port;
    uint16_t nfault_pin;
    Drv8301ShuntGain shunt_gain;
    float shunt_gain_v_per_v;
    uint16_t status_1;
    uint16_t status_2;
    uint16_t control_1;
    uint16_t control_2;
} Drv8301;

void drv8301_init(Drv8301 *drv);
uint32_t drv8301_setup(Drv8301 *drv);
uint32_t drv8301_check_fault(Drv8301 *drv);
float drv8301_get_shunt_gain(const Drv8301 *drv);
uint16_t drv8301_read_register(Drv8301 *drv, Drv8301Register reg);
void drv8301_write_register(Drv8301 *drv, Drv8301Register reg, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif
