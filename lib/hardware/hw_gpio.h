//------------------------------------------------------------
// File name: gpio.h
//------------------------------------------------------------

#pragma once

#include "stdint.h"

typedef uint8_t gpio_pin_t;

// Pin states
#define GPIO_LOW     0
#define GPIO_HIGH    1

// Pin modes
#define GPIO_MODE_INPUT_PULLDOWN  0
#define GPIO_MODE_INPUT_PULLUP    1
#define GPIO_MODE_INPUT_NOPULL    2
#define GPIO_MODE_OUTPUT          3

// Interface functions
typedef void (*initialize_pin_fn)(gpio_pin_t pin, uint8_t mode);
typedef uint8_t (*get_pin_state_fn)(gpio_pin_t pin);
typedef void (*set_pin_state_fn)(gpio_pin_t pin, uint8_t state);
typedef void (*toggle_pin_fn)(gpio_pin_t pin);
typedef void (*init_status_led_fn)(void);
typedef void (*set_status_led_fn)(uint8_t state);
typedef void (*toggle_status_led_fn)(void);

// Interface structure
typedef struct {
    initialize_pin_fn initialize_pin;
    get_pin_state_fn get_state;
    set_pin_state_fn set_state;
    toggle_pin_fn toggle;
    init_status_led_fn init_status_led;
    set_status_led_fn set_status_led;
    toggle_status_led_fn toggle_status_led;
} gpio_interface_t;

// Get the GPIO interface implementation
const gpio_interface_t* get_gpio_interface(void);
