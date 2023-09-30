//------------------------------------------------------------
// File name: gpio.h
//------------------------------------------------------------

#pragma once

#include "stdint.h"

// GPIO Pins
#define GPIO_0 0
#define GPIO_1 1
#define GPIO_2 2
#define GPIO_3 3
#define GPIO_4 4
#define GPIO_5 5
#define GPIO_6 6
#define GPIO_7 7
#define GPIO_8 8
#define GPIO_9 9

#define LOW 0
#define HIGH 1

#define GPIO_MODE_INPUT 0
#define GPIO_MODE_INPUT_PULLDOWN 0
#define GPIO_MODE_INPUT_PULLUP 1
#define GPIO_MODE_INPUT_NOPULL 2
#define GPIO_MODE_OUTPUT 3

// GPIO
void initialize_gpio_pin(uint8_t pin, uint8_t mode);
uint8_t get_gpio_pin_state(uint8_t pin);
void set_gpio_pin_state(uint8_t pin, uint8_t state);
void toggle_gpio_pin_state(uint8_t pin);


// Status LED
void initialize_status_led();
void set_status_led_state(uint8_t state);
void toggle_status_led();
