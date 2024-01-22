//------------------------------------------------------------
// File name: pid_controller.h
//------------------------------------------------------------

#pragma once

#include "stdbool.h"

typedef struct pid_controller
{
    bool is_reversed;
    double encoder_resolution;

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

    double target_speed
} pid_controller_t;

// Initialize PID controller
// T: Sampling time of the discrete PID controller in seconds
// kp: Proportional gain
// ki: Integral gain
// kd: Derivative gain
// is_reversed: If true, the motor is reversed
// encoder_resolution: Encoder resolution in pulses per revolution
void pid_controller_init(pid_controller_t* controller, double T, double kp, double ki, double kd, bool is_reversed, double encoder_resolution);
double pid_controller_update(pid_controller_t* controller, double currentSpeed, double targetSpeed);

