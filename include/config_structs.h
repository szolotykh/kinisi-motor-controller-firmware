//------------------------------------------------------------
// File name: config_structs.h
//------------------------------------------------------------

#pragma once

#include "stm32f4xx_hal.h"

// PWM channel configuration structure
typedef struct pwm_channel_info_t
{
	uint32_t timerChannel;
	GPIO_TypeDef* port;
	uint16_t pin;
}pwm_channel_info_t;

// Motor configuration structure
typedef struct motor_info_t
{
	TIM_TypeDef* timer;
	pwm_channel_info_t pwmChannel1;
	pwm_channel_info_t pwmChannel2;

}motor_info_t;

// Encoder configuration structure
typedef struct encoder_info_t
{
	TIM_TypeDef* timer;

	GPIO_TypeDef* portA;
	uint16_t pinA;

	GPIO_TypeDef* portB;
	uint16_t pinB;
}encoder_info_t;

// GPIO configuration structure
typedef struct gpio_info_t{
	GPIO_TypeDef* port;
	uint16_t pin;
}gpio_info_t;
