//------------------------------------------------------------
// File name: motor.h
//------------------------------------------------------------

#pragma once

#include "stdbool.h"
#include "stdint.h"

// Motor indexes
#define MOTOR0 0
#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3

#define MOTOR_MAX_SPEED 840

typedef unsigned char motorIndex;

// Function pointer types for mocking
typedef void (*initialize_motor_fn)(motorIndex motorIndex, bool isReversed);
typedef uint8_t (*motor_is_reversed_fn)(motorIndex motorIndex);
typedef uint8_t (*motor_is_initialized_fn)(motorIndex motorIndex);
typedef void (*set_motor_speed_fn)(motorIndex motorIndex, double pwm);
typedef void (*stop_motor_fn)(motorIndex motorIndex);
typedef void (*brake_motor_fn)(motorIndex motorIndex);

// Interface structure
typedef struct {
    initialize_motor_fn initialize;
    motor_is_reversed_fn is_reversed;
    motor_is_initialized_fn is_initialized;
    set_motor_speed_fn set_speed;
    stop_motor_fn stop;
    brake_motor_fn brake;
} hw_motor_interface_t;

// Global interface instance
extern hw_motor_interface_t controller_motors;

// Initialize the interface with default implementations
void hw_motor_init(void);

// Allow setting mock implementations
void hw_motor_set_interface(hw_motor_interface_t interface);

// Default implementations
void initialize_motor(motorIndex motorIndex, bool isReversed);

/* Check if motor is reversed
Parameters:
    motorIndex: Motor index
*/
uint8_t motor_is_reversed(motorIndex motorIndex);

/* Check if motor is initialized
Parameters:
    motorIndex: Motor index
*/
uint8_t motor_is_initialized(motorIndex motorIndex);

// Set motor speed in PWM
void set_motor_speed(motorIndex motorIndex, double pwm);
void stop_motor(motorIndex motorIndex);
void brake_motor(motorIndex motorIndex);