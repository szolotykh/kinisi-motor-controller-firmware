#pragma once
#include "../../test_motor_control/stubs/stm32f4xx_hal.h"
uint32_t HAL_GetTick(void);
void test_spi2_clock_enable(void);
#define __HAL_RCC_SPI2_CLK_ENABLE() test_spi2_clock_enable()
