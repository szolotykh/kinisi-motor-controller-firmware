//------------------------------------------------------------
// File name: timers.h
//------------------------------------------------------------

#pragma once

#include "stm32f4xx_hal.h"

// STM32F405 timer kernel clock from the active RCC configuration; 0 for
// unknown timers. GPIO/channel availability remains board-specific.
uint32_t hw_timer_input_clock_hz(const TIM_TypeDef *timer);

TIM_HandleTypeDef* get_timer_handeler(TIM_TypeDef * timTypeDef);

uint32_t get_alternate_function_mapping(TIM_TypeDef * timTypeDef);

void rcc_gpio_clk_enable(GPIO_TypeDef* port);

void rcc_tim_clk_enable(TIM_TypeDef * timTypeDef);
