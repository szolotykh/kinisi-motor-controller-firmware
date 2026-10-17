//------------------------------------------------------------
// File name: hw_config.h
// Description: Select board hardware mappings and shared motor/encoder configuration.
//------------------------------------------------------------

#pragma once

#include "stm32f4xx_hal.h"
#include "hw_config_structs.h"

// Board current version
#include "board_revision.h"

// Shared carrier frequency for every motor on every supported board.
#define MOTOR_PWM_FREQUENCY_HZ 100000U

#ifdef MP_V4
#define MOTOR_CONTROL_MODEL MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
// IC1 SR[2:0]: 000 selects 53.2 V/us to reduce switching delay at 100 kHz.
#define DRV8873_SLEW_RATE_BITS 0U
#define NUMBER_MOTORS 4
#define NUMBER_ENCODERS 4
#define NUMBER_GPIO_PINS 8
#define STATUS_LED_PORT GPIOB
#define STATUS_LED_PIN GPIO_PIN_14
static const motor_info_t motor_info[NUMBER_MOTORS] = {
    {
        .controlModel = MOTOR_CONTROL_ENABLE_DIRECTION, .timer = TIM1,
        .enable = {TIM_CHANNEL_4, GPIOE, GPIO_PIN_14, false},
        .direction = {GPIOD, GPIO_PIN_14}, .disable = {GPIOD, GPIO_PIN_13},
        .sleep = {GPIOB, GPIO_PIN_15}, .fault = {GPIOD, GPIO_PIN_8},
        .chipSelect = {GPIOD, GPIO_PIN_15}
    },
    {
        .controlModel = MOTOR_CONTROL_ENABLE_DIRECTION, .timer = TIM1,
        .enable = {TIM_CHANNEL_3, GPIOE, GPIO_PIN_12, true},
        .direction = {GPIOD, GPIO_PIN_0}, .disable = {GPIOA, GPIO_PIN_15},
        .sleep = {GPIOD, GPIO_PIN_2}, .fault = {GPIOD, GPIO_PIN_3},
        .chipSelect = {GPIOD, GPIO_PIN_1}
    },
    {
        .controlModel = MOTOR_CONTROL_ENABLE_DIRECTION, .timer = TIM1,
        .enable = {TIM_CHANNEL_2, GPIOE, GPIO_PIN_10, true},
        .direction = {GPIOD, GPIO_PIN_5}, .disable = {GPIOD, GPIO_PIN_4},
        .sleep = {GPIOB, GPIO_PIN_4}, .fault = {GPIOB, GPIO_PIN_5},
        .chipSelect = {GPIOD, GPIO_PIN_6}
    },
    {
        .controlModel = MOTOR_CONTROL_ENABLE_DIRECTION, .timer = TIM1,
        .enable = {TIM_CHANNEL_1, GPIOE, GPIO_PIN_8, true},
        .direction = {GPIOE, GPIO_PIN_5}, .disable = {GPIOE, GPIO_PIN_4},
        .sleep = {GPIOB, GPIO_PIN_8}, .fault = {GPIOB, GPIO_PIN_9},
        .chipSelect = {GPIOE, GPIO_PIN_6}
    }
};
static const encoder_info_t encoder_info[4] = {
    {TIM3, GPIOA, GPIO_PIN_6, GPIOA, GPIO_PIN_7},
    {TIM5, GPIOA, GPIO_PIN_0, GPIOA, GPIO_PIN_1},
    {TIM4, GPIOB, GPIO_PIN_6, GPIOB, GPIO_PIN_7},
    {TIM8, GPIOC, GPIO_PIN_6, GPIOC, GPIO_PIN_7}
};
static const gpio_info_t gpio_info[8] = {
    {GPIOE, GPIO_PIN_15}, {GPIOE, GPIO_PIN_13},
    {GPIOE, GPIO_PIN_11}, {GPIOE, GPIO_PIN_9},
    {GPIOE, GPIO_PIN_7}, {GPIOB, GPIO_PIN_1},
    {GPIOB, GPIO_PIN_0}, {GPIOA, GPIO_PIN_5}
};
#endif

// Boards configurations
//------------------------------------------------------------
#ifdef MP_V3
#define MOTOR_CONTROL_MODEL MOTOR_CONTROL_MODEL_PWM
	// Motor configurations
	#define NUMBER_MOTORS 4
	#define NUMBER_ENCODERS 4

	// Status pin configurations
	#define STATUS_LED_PORT GPIOA
	#define STATUS_LED_PIN GPIO_PIN_2

	static const motor_info_t motor_info[NUMBER_MOTORS] = {
		{
			.controlModel = MOTOR_CONTROL_PWM,
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
			.controlModel = MOTOR_CONTROL_PWM,
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
			.controlModel = MOTOR_CONTROL_PWM,
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
			.controlModel = MOTOR_CONTROL_PWM,
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

#endif
//------------------------------------------------------------
#ifdef MP_V2
#define MOTOR_CONTROL_MODEL MOTOR_CONTROL_MODEL_PWM
	// Motor configurations
	#define NUMBER_MOTORS 4
	#define NUMBER_ENCODERS 4

	// Status pin configurations
	#define STATUS_LED_PORT GPIOC
	#define STATUS_LED_PIN GPIO_PIN_12

	static const motor_info_t motor_info[NUMBER_MOTORS] = {
		{
			.controlModel = MOTOR_CONTROL_PWM,
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
			.controlModel = MOTOR_CONTROL_PWM,
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
			.controlModel = MOTOR_CONTROL_PWM,
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
			.controlModel = MOTOR_CONTROL_PWM,
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

#endif

//------------------------------------------------------------
#ifdef MP_V1
#define MOTOR_CONTROL_MODEL MOTOR_CONTROL_MODEL_PWM

	// Motor configurations
	#define NUMBER_MOTORS 4
	#define NUMBER_ENCODERS 1

	// Status pin configurations
	#define STATUS_LED_PORT GPIOA
	#define STATUS_LED_PIN GPIO_PIN_2

	static const motor_info_t motor_info[NUMBER_MOTORS] = {
		{
			.controlModel = MOTOR_CONTROL_PWM,
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
			.controlModel = MOTOR_CONTROL_PWM,
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
			.controlModel = MOTOR_CONTROL_PWM,
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
			.controlModel = MOTOR_CONTROL_PWM,
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
	static const encoder_info_t encoder_info[NUMBER_ENCODERS] = {
			{
				.timer = TIM4,
				.portA = GPIOB,
				.pinA = GPIO_PIN_6,
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

#endif
