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