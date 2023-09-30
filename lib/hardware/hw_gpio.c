//------------------------------------------------------------
// File name: gpio.c
//------------------------------------------------------------

#include "hw_gpio.h"
#include "hw_config.h"
#include "stm32f4xx_hal.h"

void rcc_gpiox_clk_enable(GPIO_TypeDef* gpiox);

void initialize_gpio_pin(uint8_t pin, uint8_t mode)
{
	// Check if pin is valid for seclected board
	if (pin < NUMBER_GPIO_PINS)
	{
		if(mode == GPIO_MODE_INPUT || mode == GPIO_MODE_INPUT_PULLUP || mode == GPIO_MODE_INPUT_NOPULL){
			rcc_gpiox_clk_enable(gpio_info[pin].port);

			GPIO_InitTypeDef PinInitStruct = {0};
			PinInitStruct.Pin = gpio_info[pin].pin;
			PinInitStruct.Mode = GPIO_MODE_INPUT;
			PinInitStruct.Pull = GPIO_PULLDOWN;
			if(mode == GPIO_MODE_INPUT_PULLUP){
				PinInitStruct.Pull = GPIO_PULLUP;
			}else if(mode == GPIO_MODE_INPUT_NOPULL){
				PinInitStruct.Pull = GPIO_NOPULL;
			}

			PinInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(gpio_info[pin].port, &PinInitStruct);
		}
		else if (mode == GPIO_MODE_OUTPUT)
		{
			rcc_gpiox_clk_enable(gpio_info[pin].port);

			GPIO_InitTypeDef PinInitStruct = {0};
			PinInitStruct.Pin = gpio_info[pin].pin;
			PinInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			PinInitStruct.Pull = GPIO_NOPULL;
			PinInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(gpio_info[pin].port, &PinInitStruct);

			// Set pin to low
			HAL_GPIO_WritePin(gpio_info[pin].port, gpio_info[pin].pin, GPIO_PIN_RESET);
		}
		// If pin is not valid, do nothing
	}
}

uint8_t get_gpio_pin_state(uint8_t pin)
{
	return HAL_GPIO_ReadPin(gpio_info[pin].port, gpio_info[pin].pin);
}

void set_gpio_pin_state(uint8_t pin, uint8_t state)
{
	HAL_GPIO_WritePin(gpio_info[pin].port, gpio_info[pin].pin, state);
}

void toggle_gpio_pin_state(uint8_t pin)
{
	HAL_GPIO_TogglePin (gpio_info[pin].port, gpio_info[pin].pin);
}

void initialize_status_led()
{
	// Configure Status LED pin
	// GPIO Ports Clock Enable
	rcc_gpiox_clk_enable(STATUS_LED_PORT);
	HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_RESET);
	GPIO_InitTypeDef StatusLEDPinInitStruct = {0};
	StatusLEDPinInitStruct.Pin = STATUS_LED_PIN;
	StatusLEDPinInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	StatusLEDPinInitStruct.Pull = GPIO_NOPULL;
	StatusLEDPinInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(STATUS_LED_PORT, &StatusLEDPinInitStruct);
}

void set_status_led_state(uint8_t state)
{
	HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, state);
}

void toggle_status_led_state()
{
	HAL_GPIO_TogglePin (STATUS_LED_PORT, STATUS_LED_PIN);
}

void rcc_gpiox_clk_enable(GPIO_TypeDef* gpiox)
{
	if(gpiox == GPIOA && !__HAL_RCC_GPIOA_IS_CLK_ENABLED()){
		__HAL_RCC_GPIOA_CLK_ENABLE();
	}else if(gpiox == GPIOB && !__HAL_RCC_GPIOB_IS_CLK_ENABLED()){
		__HAL_RCC_GPIOB_CLK_ENABLE();
	}else if(gpiox == GPIOC && !__HAL_RCC_GPIOC_IS_CLK_ENABLED()){
		__HAL_RCC_GPIOC_CLK_ENABLE();
	}else if(gpiox == GPIOD && !__HAL_RCC_GPIOD_IS_CLK_ENABLED()){
		__HAL_RCC_GPIOD_CLK_ENABLE();
	}else if(gpiox == GPIOE && !__HAL_RCC_GPIOE_IS_CLK_ENABLED()){
		__HAL_RCC_GPIOE_CLK_ENABLE();
	}else if(gpiox == GPIOF && !__HAL_RCC_GPIOF_IS_CLK_ENABLED()){
		__HAL_RCC_GPIOF_CLK_ENABLE();
	}else if(gpiox == GPIOG && !__HAL_RCC_GPIOG_IS_CLK_ENABLED()){
		__HAL_RCC_GPIOG_CLK_ENABLE();
	}
}
