//------------------------------------------------------------
// File name: platform.c
//------------------------------------------------------------
#include "platform.h"
#include "motor.h"

#define SPEED_RESOLUTION 840

void SetMotorVelocity(motorIndex motorIndex, double velocity)
{
    unsigned short direction = velocity >= 0;
    unsigned int speed = abs(velocity) / 100 * SPEED_RESOLUTION;
    set_motor_speed(motorIndex, direction, speed);
} 

void init_platform(){
    initialize_motor(MOTOR0);
    initialize_motor(MOTOR1);
    initialize_motor(MOTOR2);
    initialize_motor(MOTOR3);
}

void set_velocity_input(char x, char y, char t){
        SetMotorVelocity(MOTOR3, (x - y - t));
        SetMotorVelocity(MOTOR0, (x + y + t));
        SetMotorVelocity(MOTOR2, (x + y - t));
        SetMotorVelocity(MOTOR1, (x - y + t));
}