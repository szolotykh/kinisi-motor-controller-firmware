//------------------------------------------------------------
// File name: hw_config.h
//------------------------------------------------------------

#pragma once

#include "stm32f4xx_hal.h"
#include "hw_config_structs.h"

// Board current version
#define MP_V3

// Boards configurations
//------------------------------------------------------------
#ifdef MP_V3
	// Motor configurations
	#define NUMBER_MOTORS 4
	#define NUMBER_ENCODERS 4

	static const motor_info_t motor_info[NUMBER_MOTORS] = {
		{
			.timer = TIM12,
			.pwmChannel1 = {
							.timerChannel = TIM_CHANNEL_1,
							.port = GPIOB,
							.pin = GPIO_PIN_14,
						   },
			.pwmChannel2 = {
							.timerChannel = TIM_CHANNEL_2,
							.port = GPIOB,
							.pin = GPIO_PIN_15,
						   },
		},
		{
			.timer = TIM8,
			.pwmChannel1 = {
							.timerChannel = TIM_CHANNEL_1,
							.port = GPIOC,
							.pin = GPIO_PIN_6,
						   },
			.pwmChannel2 = {
							.timerChannel = TIM_CHANNEL_2,
							.port = GPIOC,
							.pin = GPIO_PIN_7,
						   },
		},
		{
			.timer = TIM8,
			.pwmChannel1 = {
							.timerChannel = TIM_CHANNEL_3,
							.port = GPIOC,
							.pin = GPIO_PIN_8,
						   },
			.pwmChannel2 = {
							.timerChannel = TIM_CHANNEL_4,
							.port = GPIOC,
							.pin = GPIO_PIN_9,
						   },
		},
		{
			.timer = TIM1,
			.pwmChannel1 = {
							.timerChannel = TIM_CHANNEL_1,
							.port = GPIOA,
							.pin = GPIO_PIN_8,
						   },
			.pwmChannel2 = {
							.timerChannel = TIM_CHANNEL_2,
							.port = GPIOA,
							.pin = GPIO_PIN_9,
						   },
		}
	};

	// Encoder configurations
	#define NUMBER_ENCODERS 4

	static const encoder_info_t encoder_info[NUMBER_ENCODERS] = {
		{
			.timer = TIM3,
			.portA = GPIOA,
			.pinA = GPIO_PIN_6,
			.portB = GPIOA,
			.pinB = GPIO_PIN_7
		},
		{
			.timer = TIM5,
			.portA = GPIOA,
			.pinA = GPIO_PIN_0,
			.portB = GPIOA,
			.pinB = GPIO_PIN_1
		},
		{
			.timer = TIM4,
			.portA = GPIOB,
			.pinA = GPIO_PIN_6,
			.portB = GPIOB,
			.pinB = GPIO_PIN_7
		},
		{
			.timer = TIM2,
			.portA = GPIOA,
			.pinA = GPIO_PIN_15,
			.portB = GPIOB,
			.pinB = GPIO_PIN_3
		}
	};

	// GPIO Configurations
	#define NUMBER_GPIO_PINS 10

	static const gpio_info_t gpio_info[NUMBER_GPIO_PINS] = {
		{.port = GPIOB, .pin = GPIO_PIN_13},
		{.port = GPIOB, .pin = GPIO_PIN_12},
		{.port = GPIOC, .pin = GPIO_PIN_10},
		{.port = GPIOC, .pin = GPIO_PIN_11},
		{.port = GPIOB, .pin = GPIO_PIN_5},
		{.port = GPIOB, .pin = GPIO_PIN_4},
		{.port = GPIOD, .pin = GPIO_PIN_2},
		{.port = GPIOC, .pin = GPIO_PIN_12},
		{.port = GPIOB, .pin = GPIO_PIN_9},
		{.port = GPIOB, .pin = GPIO_PIN_8}
	};

	// Status pin configurations
	#define STATUS_LED_PORT GPIOA
	#define STATUS_LED_PIN GPIO_PIN_2

#endif
//------------------------------------------------------------
#ifdef MP_V2
	// Motor configurations
	#define NUMBER_MOTORS 4
	#define NUMBER_ENCODERS 4

	static const motor_info_t motor_info[NUMBER_MOTORS] = {
		{
			.timer = TIM12,
			.pwmChannel1 = {
							.timerChannel = TIM_CHANNEL_1,
							.port = GPIOB,
							.pin = GPIO_PIN_14,
						   },
			.pwmChannel2 = {
							.timerChannel = TIM_CHANNEL_2,
							.port = GPIOB,
							.pin = GPIO_PIN_15,
						   },
		},
		{
			.timer = TIM8,
			.pwmChannel1 = {
							.timerChannel = TIM_CHANNEL_1,
							.port = GPIOC,
							.pin = GPIO_PIN_6,
						   },
			.pwmChannel2 = {
							.timerChannel = TIM_CHANNEL_2,
							.port = GPIOC,
							.pin = GPIO_PIN_7,
						   },
		},
		{
			.timer = TIM8,
			.pwmChannel1 = {
							.timerChannel = TIM_CHANNEL_3,
							.port = GPIOC,
							.pin = GPIO_PIN_8,
						   },
			.pwmChannel2 = {
							.timerChannel = TIM_CHANNEL_4,
							.port = GPIOC,
							.pin = GPIO_PIN_9,
						   },
		},
		{
			.timer = TIM1,
			.pwmChannel1 = {
							.timerChannel = TIM_CHANNEL_1,
							.port = GPIOA,
							.pin = GPIO_PIN_8,
						   },
			.pwmChannel2 = {
							.timerChannel = TIM_CHANNEL_2,
							.port = GPIOA,
							.pin = GPIO_PIN_9,
						   },
		}
	};

	// Encoder configurations
	#define NUMBER_ENCODERS 4

	static const encoder_info_t encoder_info[NUMBER_ENCODERS] = {
		{
			.timer = TIM2,
			.portA = GPIOA,
			.pinA = GPIO_PIN_15,
			.portB = GPIOB,
			.pinB = GPIO_PIN_3
		},
		{
			.timer = TIM3,
			.portA = GPIOA,
			.pinA = GPIO_PIN_6,
			.portB = GPIOA,
			.pinB = GPIO_PIN_7
		},
		{
			.timer = TIM4,
			.portA = GPIOB,
			.pinA = GPIO_PIN_6,
			.portB = GPIOB,
			.pinB = GPIO_PIN_7
		},
		{
			.timer = TIM5,
			.portA = GPIOA,
			.pinA = GPIO_PIN_0,
			.portB = GPIOA,
			.pinB = GPIO_PIN_1
		}
	};

	// GPIO Configurations
	#define NUMBER_GPIO_PINS 4

	static gpio_info_t gpio_info[NUMBER_GPIO_PINS] = {
		{.port = GPIOA, .pin = GPIO_PIN_2},
		{.port = GPIOA, .pin = GPIO_PIN_3},
		{.port = GPIOA, .pin = GPIO_PIN_4},
		{.port = GPIOA, .pin = GPIO_PIN_5}
	};

	// Status pin configurations
	#define STATUS_LED_PORT GPIOC
	#define STATUS_LED_PIN GPIO_PIN_12

#endif

//------------------------------------------------------------
#ifdef MP_V1

	// Motor configurations
	#define NUMBER_MOTORS 4
	#define NUMBER_ENCODERS 4

	static const motor_info_t motor_info[NUMBER_MOTORS] = {
		{
			.timer = TIM5,
			.pwmChannel1 = {
							.timerChannel = TIM_CHANNEL_1,
							.port = GPIOA,
							.pin = GPIO_PIN_0,
						   },
			.pwmChannel2 = {
							.timerChannel = TIM_CHANNEL_2,
							.port = GPIOA,
							.pin = GPIO_PIN_1,
						   },
		},
		{
			.timer = TIM5,
			.pwmChannel1 = {
							.timerChannel = TIM_CHANNEL_3,
							.port = GPIOA,
							.pin = GPIO_PIN_2,
						   },
			.pwmChannel2 = {
							.timerChannel = TIM_CHANNEL_4,
							.port = GPIOA,
							.pin = GPIO_PIN_3,
						   },
		},
		{
			.timer = TIM2,
			.pwmChannel1 = {
							.timerChannel = TIM_CHANNEL_1,
							.port = GPIOA,
							.pin = GPIO_PIN_5,
						   },
			.pwmChannel2 = {
							.timerChannel = TIM_CHANNEL_2,
							.port = GPIOB,
							.pin = GPIO_PIN_3,
						   },
		},
		{
			.timer = TIM2,
			.pwmChannel1 = {
							.timerChannel = TIM_CHANNEL_3,
							.port = GPIOB,
							.pin = GPIO_PIN_10,
						   },
			.pwmChannel2 = {
							.timerChannel = TIM_CHANNEL_4,
							.port = GPIOB,
							.pin = GPIO_PIN_11,
						   },
		}
	};

	// Encoder configurations
	#define NUMBER_ENCODERS 1

	static const encoder_info_t encoder_info[NUMBER_ENCODERS] = {
			{
				.timer = TIM4,
				.portA = GPIOB,
				.pinA = GPIO_PIN_6
				.portB = GPIOB,
				.pinB = GPIO_PIN_7
			}
	};

	// GPIO Configurations
	#define NUMBER_GPIO_PINS 4

	static gpio_info_t gpio_info[NUMBER_GPIO_PINS] = {
		{.port = GPIOA, .pin = GPIO_PIN_2},
		{.port = GPIOA, .pin = GPIO_PIN_3},
		{.port = GPIOA, .pin = GPIO_PIN_4},
		{.port = GPIOA, .pin = GPIO_PIN_5}
	};

	// Status pin configurations
	#define STATUS_LED_PORT GPIOC
	#define STATUS_LED_PIN GPIO_PIN_12
#endif
