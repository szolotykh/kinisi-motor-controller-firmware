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


typedef unsigned char motorIndex;

extern void initialize_motor(motorIndex motorIndex, bool isReversed);
extern void initialize_motor_all();
extern void set_motor_speed(motorIndex motorIndex, uint8_t direction, uint16_t speed);
