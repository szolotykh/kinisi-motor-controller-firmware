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
    bool has_previous;
} pid_controller_t;

// Initialize PID controller
// T: Sampling time of the discrete PID controller in seconds
// kp: Proportional gain
// ki: Integral gain
// kd: Derivative gain
// integral_limit: Integral contribution limit in PWM percentage points, 0..100.
// Zero disables integral action. Gains must be finite and nonnegative.
bool pid_controller_settings_valid(double kp, double ki, double kd, double integral_limit);
void pid_controller_init(pid_controller_t* controller, double T, double kp, double ki, double kd, double integral_limit);
double pid_controller_update(pid_controller_t* controller, double currentSpeed, double targetSpeed);

// Reset the runtime state of a PID controller without changing its tuning.
// Clears the integrator (windup), differentiator, previous error/speed history,
// motorPWM output, derivative history validity and target speed to zero. The gains (kp/ki/kd),
// sampling time T, tau and integral limits are preserved, so the controller
// stays configured and running - only its accumulated history is discarded.
void pid_controller_reset(pid_controller_t* controller);

