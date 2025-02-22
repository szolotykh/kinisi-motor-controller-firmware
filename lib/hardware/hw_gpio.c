//------------------------------------------------------------
// File name: gpio.c
//------------------------------------------------------------

#include "hw_gpio.h"
#include "hw_config.h"
#include "stm32f4xx_hal.h"

// Static helper function declarations
static void rcc_gpiox_clk_enable(GPIO_TypeDef* gpiox);

// Static implementation functions
static void stm32_initialize_pin(gpio_pin_t pin, uint8_t mode)
{
    if (pin < NUMBER_GPIO_PINS)
    {
        rcc_gpiox_clk_enable(gpio_info[pin].port);
        GPIO_InitTypeDef PinInitStruct = {0};
        PinInitStruct.Pin = gpio_info[pin].pin;
        PinInitStruct.Speed = GPIO_SPEED_FREQ_LOW;

        if(mode == GPIO_MODE_INPUT_PULLDOWN || mode == GPIO_MODE_INPUT_PULLUP || mode == GPIO_MODE_INPUT_NOPULL) {
            PinInitStruct.Mode = GPIO_MODE_INPUT;
            PinInitStruct.Pull = GPIO_PULLDOWN;
            if(mode == GPIO_MODE_INPUT_PULLUP) {
                PinInitStruct.Pull = GPIO_PULLUP;
            } else if(mode == GPIO_MODE_INPUT_NOPULL) {
                PinInitStruct.Pull = GPIO_NOPULL;
            }
        }
        else if (mode == GPIO_MODE_OUTPUT) {
            PinInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            PinInitStruct.Pull = GPIO_NOPULL;
            HAL_GPIO_WritePin(gpio_info[pin].port, gpio_info[pin].pin, GPIO_PIN_RESET);
        }

        HAL_GPIO_Init(gpio_info[pin].port, &PinInitStruct);
    }
}

static uint8_t stm32_get_pin_state(gpio_pin_t pin)
{
    if (pin < NUMBER_GPIO_PINS) {
        return HAL_GPIO_ReadPin(gpio_info[pin].port, gpio_info[pin].pin);
    }
    return 0;
}

static void stm32_set_pin_state(gpio_pin_t pin, uint8_t state)
{
    if (pin < NUMBER_GPIO_PINS) {
        HAL_GPIO_WritePin(gpio_info[pin].port, gpio_info[pin].pin, state);
    }
}

static void stm32_toggle_pin(gpio_pin_t pin)
{
    if (pin < NUMBER_GPIO_PINS) {
        HAL_GPIO_TogglePin(gpio_info[pin].port, gpio_info[pin].pin);
    }
}

static void stm32_init_status_led(void)
{
    rcc_gpiox_clk_enable(STATUS_LED_PORT);
    
    GPIO_InitTypeDef StatusLEDPinInitStruct = {0};
    StatusLEDPinInitStruct.Pin = STATUS_LED_PIN;
    StatusLEDPinInitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    StatusLEDPinInitStruct.Pull = GPIO_NOPULL;
    StatusLEDPinInitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    HAL_GPIO_Init(STATUS_LED_PORT, &StatusLEDPinInitStruct);
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_RESET);
}

static void stm32_set_status_led(uint8_t state)
{
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, state);
}

static void stm32_toggle_status_led(void)
{
    HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
}

// STM32 GPIO interface implementation
static const gpio_interface_t stm32_gpio_interface = {
    .initialize_pin = stm32_initialize_pin,
    .get_state = stm32_get_pin_state,
    .set_state = stm32_set_pin_state,
    .toggle = stm32_toggle_pin,
    .init_status_led = stm32_init_status_led,
    .set_status_led = stm32_set_status_led,
    .toggle_status_led = stm32_toggle_status_led
};

// Public interface getter
const gpio_interface_t* get_gpio_interface(void) {
    return &stm32_gpio_interface;
}

// Helper function implementation
static void rcc_gpiox_clk_enable(GPIO_TypeDef* gpiox)
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
