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

// Get the motor interface implementation
const hw_motor_interface_t* get_motor_interface(void);

// Set mock interface for testing
void hw_motor_set_interface(const hw_motor_interface_t* interface);

// Initialize motor interface
void hw_motor_init(void);