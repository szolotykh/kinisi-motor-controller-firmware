//------------------------------------------------------------
// File name: controller.h
//------------------------------------------------------------

#pragma once

typedef struct mecanum_velocity_t
{
    int motor0;
    int motor1;
    int motor2;
    int motor3;
} mecanum_velocity_t;

typedef struct Controller
{
    double kp;
    double ki;
    double kd;
} Controller;

mecanum_velocity_t get_mecanum_velocities(signed char x, signed char y, signed char t);