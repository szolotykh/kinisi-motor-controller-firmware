//------------------------------------------------------------
// File name: motor.c
//------------------------------------------------------------

#include "math.h"
#include "hw_motor.h"
#include "hw_timers.h"
#include "hw_config.h"
#include "stm32f4xx_hal.h"
#include "stdbool.h"

void set_motor_channel(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t speed);
void init_channel(TIM_HandleTypeDef *htim, const pwm_channel_info_t *channel_info, TIM_TypeDef * timTypeDef);
void init_motor_timer(const motor_info_t *motorInfo);

typedef struct
{
	bool isInitialized;
	uint8_t isReversed;
}motor_status_t;

static motor_status_t motor_status[NUMBER_MOTORS];

void initialize_motor(motorIndex motorIndex, bool isReversed)
{
	if(!motor_status[motorIndex].isInitialized)
	{
		init_motor_timer(&motor_info[motorIndex]);
		motor_status[motorIndex].isInitialized = true;
		motor_status[motorIndex].isReversed = isReversed;
	}
}

uint8_t motor_is_reversed(motorIndex motorIndex)
{
	return motor_status[motorIndex].isReversed;
}

uint8_t motor_is_initialized(motorIndex motorIndex)
{
	return motor_status[motorIndex].isInitialized;
}

void set_motor_speed(motorIndex motorIndex, double pwm)
{
	bool direction = pwm > 0;
	if (pwm > 100.0) pwm = 100.0;
	if (pwm < -100.0) pwm = -100.0;
	uint16_t speed = fabs(pwm)*MOTOR_MAX_SPEED/100.0;

	if(motor_status[motorIndex].isInitialized)
	{
		TIM_HandleTypeDef *htim = get_timer_handeler(motor_info[motorIndex].timer);
		direction = direction ^ motor_status[motorIndex].isReversed;
		speed = MOTOR_MAX_SPEED - speed;
		if(direction)
		{
			set_motor_channel(htim, motor_info[motorIndex].pwmChannel1.timerChannel, speed);
			set_motor_channel(htim, motor_info[motorIndex].pwmChannel2.timerChannel, MOTOR_MAX_SPEED);
		}
		else
		{
			set_motor_channel(htim, motor_info[motorIndex].pwmChannel1.timerChannel, MOTOR_MAX_SPEED);
			set_motor_channel(htim, motor_info[motorIndex].pwmChannel2.timerChannel, speed);
		}
	}
}

void stop_motor(motorIndex motorIndex)
{
	if(motor_status[motorIndex].isInitialized)
	{
		TIM_HandleTypeDef *htim = get_timer_handeler(motor_info[motorIndex].timer);
		// Both channels are set to low
		set_motor_channel(htim, motor_info[motorIndex].pwmChannel1.timerChannel, 0);
		set_motor_channel(htim, motor_info[motorIndex].pwmChannel2.timerChannel, 0);
	}
}

void brake_motor(motorIndex motorIndex)
{
	if(motor_status[motorIndex].isInitialized)
	{
		TIM_HandleTypeDef *htim = get_timer_handeler(motor_info[motorIndex].timer);
		// Both channels are set to high
		set_motor_channel(htim, motor_info[motorIndex].pwmChannel1.timerChannel, MOTOR_MAX_SPEED);
		set_motor_channel(htim, motor_info[motorIndex].pwmChannel2.timerChannel, MOTOR_MAX_SPEED);
	}
}

void set_motor_channel(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t speed){
	switch(channel)
	{
		case TIM_CHANNEL_1:
			htim->Instance->CCR1 = (uint32_t)speed;
		break;

		case TIM_CHANNEL_2:
			htim->Instance->CCR2 = (uint32_t)speed;
		break;

		case TIM_CHANNEL_3:
			htim->Instance->CCR3 = (uint32_t)speed;
		break;

		case TIM_CHANNEL_4:
			htim->Instance->CCR4 = (uint32_t)speed;
		break;
	}
}

void init_motor_timer(const motor_info_t *motorInfo)
{
	TIM_HandleTypeDef *htim = get_timer_handeler(motorInfo->timer);

	if(HAL_TIM_Base_GetState(htim) != HAL_TIM_STATE_READY){
		rcc_tim_clk_enable(motorInfo->timer);

		TIM_ClockConfigTypeDef sClockSourceConfig = {0};
		TIM_MasterConfigTypeDef sMasterConfig = {0};

		htim->Instance = motorInfo->timer;
		htim->Init.Prescaler = 1-1;
		htim->Init.CounterMode = TIM_COUNTERMODE_UP;
		htim->Init.Period = 840-1;
		htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
		if (HAL_TIM_Base_Init(htim) != HAL_OK)
		{
			Error_Handler();
		}
		sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
		if (HAL_TIM_ConfigClockSource(htim, &sClockSourceConfig) != HAL_OK)
		{
			Error_Handler();
		}
		if (HAL_TIM_PWM_Init(htim) != HAL_OK)
		{
			Error_Handler();
		}
		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK)
		{
			Error_Handler();
		}
	}

	init_channel(htim, &motorInfo->pwmChannel1, motorInfo->timer);
	init_channel(htim, &motorInfo->pwmChannel2, motorInfo->timer);

	HAL_TIM_PWM_Start(htim, motorInfo->pwmChannel1.timerChannel);
	HAL_TIM_PWM_Start(htim, motorInfo->pwmChannel2.timerChannel);
}

void init_channel(TIM_HandleTypeDef *htim, const pwm_channel_info_t *channel_info, TIM_TypeDef * timTypeDef)
{
	// Configure channels
	TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(htim, &sConfigOC, channel_info->timerChannel) != HAL_OK)
	{
		Error_Handler();
	}

	GPIO_InitTypeDef GPIO_InitStruct = {0};

	rcc_gpio_clk_enable(channel_info->port);

    GPIO_InitStruct.Pin = channel_info->pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = get_alternate_function_mapping(timTypeDef);

    HAL_GPIO_Init(channel_info->port, &GPIO_InitStruct);
}
