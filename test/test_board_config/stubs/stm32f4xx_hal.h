#pragma once
#include "../../test_motor_control/stubs/stm32f4xx_hal.h"
extern GPIO_TypeDef unused_ports[2];
#define GPIOF (&unused_ports[0])
#define GPIOG (&unused_ports[1])
#define GPIO_PULLUP 1
#define GPIO_PULLDOWN 2
#define __HAL_RCC_GPIOA_IS_CLK_ENABLED() 0
#define __HAL_RCC_GPIOB_IS_CLK_ENABLED() 0
#define __HAL_RCC_GPIOC_IS_CLK_ENABLED() 0
#define __HAL_RCC_GPIOD_IS_CLK_ENABLED() 0
#define __HAL_RCC_GPIOE_IS_CLK_ENABLED() 0
#define __HAL_RCC_GPIOF_IS_CLK_ENABLED() 0
#define __HAL_RCC_GPIOG_IS_CLK_ENABLED() 0
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOB_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOD_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOE_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOF_CLK_ENABLE() ((void)0)
#define __HAL_RCC_GPIOG_CLK_ENABLE() ((void)0)
void HAL_GPIO_TogglePin(GPIO_TypeDef *, uint16_t);
