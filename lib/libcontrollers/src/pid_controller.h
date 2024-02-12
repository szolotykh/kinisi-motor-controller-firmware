//------------------------------------------------------------
// File name: pid_controller.h
//------------------------------------------------------------

#pragma once

#include "stdbool.h"

typedef struct pid_controller
{
    double kp;
    double ki;
    double kd;

    double integrator;
    double differentiator;
    double previousError;
    double previousSpeed;
    double motorPWM;

    double max_integral;
    double min_integral;
    
    double T;
    double tau;

    double target_speed;
} pid_controller_t;

// Initialize PID controller
// T: Sampling time of the discrete PID controller in seconds
// kp: Proportional gain
// ki: Integral gain
// kd: Derivative gain
// integral_limit: Integral limit must be positive. If negative or zero, integral limit is disabled
void pid_controller_init(pid_controller_t* controller, double T, double kp, double ki, double kd, double integral_limit);
double pid_controller_update(pid_controller_t* controller, double currentSpeed, double targetSpeed);

