//------------------------------------------------------------
// File name: platform.c
//------------------------------------------------------------
#include <stdlib.h>
#include <controller.h>
#include "platform.h"
#include "motor.h"


#define SPEED_RESOLUTION 840

void SetMotorVelocity(motorIndex motorIndex, int velocity)
{
    unsigned short direction = velocity >= 0;
    unsigned int speed = abs(velocity) * SPEED_RESOLUTION / 100;
    set_motor_speed(motorIndex, direction, speed);
}

void init_platform()
{
    initialize_motor(MOTOR0);
    initialize_motor(MOTOR1);
    initialize_motor(MOTOR2);
    initialize_motor(MOTOR3);
}

void set_velocity_input(signed char x, signed char y, signed char t)
{
    mecanum_velocity_t velocities = get_mecanum_velocities(x, y, t);
    SetMotorVelocity(MOTOR0, -velocities.motor0);
    SetMotorVelocity(MOTOR1, -velocities.motor1);
    SetMotorVelocity(MOTOR2, velocities.motor2);
    SetMotorVelocity(MOTOR3, velocities.motor3);
}