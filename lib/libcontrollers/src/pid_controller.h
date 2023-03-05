//------------------------------------------------------------
// File name: pid_controller.h
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

typedef struct pid_controller
{
    double kp;
    double ki;
    double kd;

    double integrator;
    double previousError;
    double previousVelocity;
    double motorPWM;
    
    double T;
} pid_controller_t;

void pid_controller_init(pid_controller_t* controller, double kp, double ki, double kd);
double pid_controller_update(pid_controller_t* controller, double currentVelocuty, double targetVelocity);

mecanum_velocity_t get_mecanum_velocities(signed char x, signed char y, signed char t);

