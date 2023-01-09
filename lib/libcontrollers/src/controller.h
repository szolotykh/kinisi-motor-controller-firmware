//------------------------------------------------------------
// File name: controller.h
//------------------------------------------------------------

#pragma once

// TODO: Replace it with some thing configurable
#define  PWM_LIMIT 840

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

    double ierror;
    double motorPWM;
} Controller;

Controller init_pid_controller(double kp, double ki, double kd);
double update_pid_controller(Controller* controller, double currentVelocuty, double targetVelocity);

mecanum_velocity_t get_mecanum_velocities(signed char x, signed char y, signed char t);

