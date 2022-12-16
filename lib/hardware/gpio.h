//------------------------------------------------------------
// File name: gpio.h
//------------------------------------------------------------

#pragma once

#include "stdint.h"

#define GPIO_0 0
#define GPIO_1 1
#define GPIO_2 2
#define GPIO_3 3

#define LOW 0
#define HIGH 1

void initialize_gpio();
void gpio_toggle_pin(uint8_t pin);
void gpio_toggle_status_led();
