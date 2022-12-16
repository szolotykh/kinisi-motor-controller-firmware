//------------------------------------------------------------
// File name: gpio.c
//------------------------------------------------------------

#include "hw_gpio.h"
#include "hw_config.h"
#include "stm32f4xx_hal.h"

void rcc_gpiox_clk_enable(GPIO_TypeDef* gpiox);

void initialize_gpio()
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

	// GPIO Pins
	for(uint8_t pin = 0; pin < NUMBER_GPIO_PINS; pin++){
		// GPIO Ports Clock Enable
		rcc_gpiox_clk_enable(gpio_info[pin].port);
		HAL_GPIO_WritePin(gpio_info[pin].port, gpio_info[pin].pin, GPIO_PIN_RESET);

		GPIO_InitTypeDef PinInitStruct = {0};
		PinInitStruct.Pin = gpio_info[pin].pin;
		PinInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		PinInitStruct.Pull = GPIO_NOPULL;
		PinInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(gpio_info[pin].port, &PinInitStruct);
	}
}

void gpio_toggle_pin(uint8_t pin)
{
	HAL_GPIO_TogglePin (gpio_info[pin].port, gpio_info[pin].pin);
}


void gpio_toggle_status_led()
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
