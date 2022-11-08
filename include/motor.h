//------------------------------------------------------------
// File name: motor.h
//------------------------------------------------------------

#pragma once

// Motor indexes
#define MOTOR0 0
#define MOTOR1 1
#define MOTOR2 2
#define MOTOR3 3


typedef unsigned char motorIndex;

extern void initialize_motor(motorIndex motorIndex);
extern void set_motor_speed(motorIndex motorIndex, unsigned short direction, unsigned speed);
