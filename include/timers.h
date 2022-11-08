//------------------------------------------------------------
// File name: timers.h
//------------------------------------------------------------

#pragma once

#include "stm32f4xx_hal.h"

TIM_HandleTypeDef* get_timer_handeler(TIM_TypeDef * timTypeDef);

uint32_t get_alternate_function_mapping(TIM_TypeDef * timTypeDef);

void rcc_gpio_clk_enable(GPIO_TypeDef* port);

void rcc_tim_clk_enable(TIM_TypeDef * timTypeDef);
