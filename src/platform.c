//------------------------------------------------------------
// File name: platform.c
//------------------------------------------------------------
#include "platform.h"
#include "motor.h"
#include <stdlib.h>

#define SPEED_RESOLUTION 840

void SetMotorVelocity(motorIndex motorIndex, int velocity)
{
    unsigned short direction = velocity >= 0;
    unsigned int speed = abs(velocity) * SPEED_RESOLUTION / 100;
    set_motor_speed(motorIndex, direction, speed);
}

signed char verify_range(signed char c)
{
    if(c > 100) return 100;
    if(c < -100) return -100;
    return c;
}

int sing(int c)
{
    return (c > 0) - (c < 0);
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
    // Adjust if needed velocity input components
     int nx = verify_range(x);
    int ny = verify_range(y);
    int nt = verify_range(t);

    // Normalize velocity
    int l = abs(nx) + abs(ny) + abs(nt);
    nx = sing(nx) * nx * nx / l;
    ny = sing(ny) * ny * ny / l;
    nt = sing(nt) * nt * nt / l;

    SetMotorVelocity(MOTOR3, nx - ny - nt);
    SetMotorVelocity(MOTOR0, -(nx + ny + nt));
    SetMotorVelocity(MOTOR2, nx + ny - nt);
    SetMotorVelocity(MOTOR1, -(nx - ny + nt));
}