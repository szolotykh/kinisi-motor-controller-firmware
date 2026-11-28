#pragma once
#include "stm32f4xx.h"
int HAL_GPIO_ReadPin(void *port, uint16_t pin);
void Error_Handler(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
