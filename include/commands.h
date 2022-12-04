#pragma once

// Commands
// Motor commands
#define INITIALIZE_MOTOR 0x01
#define SET_MOTOR_SPEED 0x02

// Encoder commands
#define INITIALIZE_ENCODER 0x11
#define GET_ENCODER_VALUE 0x12

// GPIO
#define PIN_LOW 0x20
#define PIN_HIGH 0x21
#define PIN_TOGGLE 0x22

// Status LED
#define STATUS_LED_OFF 0x23
#define STATUS_LED_ON 0x24
#define STATUS_LED_TOGGLE 0x25

// Platform
#define PLATFORM_INITIALIZE 0x30
#define PLATFORM_SET_VELOCITY_INPUT 0x31
