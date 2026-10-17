//------------------------------------------------------------
// File name: config_structs.h
//------------------------------------------------------------

#pragma once

#include "stm32f4xx_hal.h"
#include <stdbool.h>

// Numeric selectors are also usable in #if; C enum names alone are not.
#define MOTOR_CONTROL_MODEL_PWM 0
#define MOTOR_CONTROL_MODEL_ENABLE_DIRECTION 1
typedef enum {
	MOTOR_CONTROL_PWM = MOTOR_CONTROL_MODEL_PWM,
	MOTOR_CONTROL_ENABLE_DIRECTION = MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
} motor_control_model_t;

typedef struct gpio_info_t {
	GPIO_TypeDef* port;
	uint16_t pin;
} gpio_info_t;

// PWM channel configuration structure
typedef struct pwm_channel_info_t
{
	uint32_t timerChannel;
	GPIO_TypeDef* port;
	uint16_t pin;
	bool complementary;
}pwm_channel_info_t;

// Motor configuration structure
typedef struct motor_info_t
{
	motor_control_model_t controlModel;
	TIM_TypeDef* timer;
	// Dual-PWM drivers use these two channels.
	pwm_channel_info_t pwmChannel1;
	pwm_channel_info_t pwmChannel2;
	// Enable/direction drivers use one PWM output and a direction GPIO.
	pwm_channel_info_t enable;
	gpio_info_t direction;
	// DRV8873S control/status pins (used by the v4 driver helper).
	gpio_info_t disable;
	gpio_info_t sleep;
	gpio_info_t fault;
	gpio_info_t chipSelect;

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
