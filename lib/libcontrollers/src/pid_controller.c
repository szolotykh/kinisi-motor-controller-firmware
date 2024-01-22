//------------------------------------------------------------
// File name: pid_controller.c
//------------------------------------------------------------
#include <stdlib.h>
#include "pid_controller.h"

int sing(int c);
double limit_to_range(double value, double min_value, double max_value);

void pid_controller_init(pid_controller_t* controller, double T, double kp, double ki, double kd, bool is_reversed, double encoder_resolution)
{
    controller->is_reversed = is_reversed;
    controller->encoder_resolution = encoder_resolution;

    controller->kp = kp;
    controller->ki = ki;
    controller->kd = kd;
    // Sampling time of the discrete PID controller in seconds
    controller->T = T;
    controller->tau = 0.01f;

    controller->previousError = 0;
    controller->previousSpeed = 0;
    controller->motorPWM = 0;
    controller->integrator = 0;
    controller->encoder_resolution = encoder_resolution;
    controller->max_integral = 30.0;
    controller->min_integral = -30.0;
}
// Update the PID controller
// currentSpeed: Current speed of the motor in radians per second
// targetSpeed: Target speed of the motor in radians per second
// return: PWM value to be applied to the motor in range -100.0 to 100.0
double pid_controller_update(pid_controller_t* controller, double currentSpeed, double targetSpeed)
{
    // TODO Add mutex
    double kp = controller->kp;
    double ki = controller->ki;
    double kd = controller->kd;
    controller->target_speed = targetSpeed;

    // Calculate error
    double error = targetSpeed - currentSpeed;

    // Proportional
    double proportional = kp * error;

    // Integral
    controller->integrator += error + 0.5f * controller->ki * controller->T * (error + controller->previousError);
    controller->integrator = limit_to_range(controller->integrator, controller->max_integral, controller->min_integral);

    // Derivative
    controller->differentiator = -(2.0f * kd * (currentSpeed - controller->previousSpeed)
                        + (2.0f * controller->tau - controller->T) * controller->differentiator)
                        / (2.0f * controller->tau + controller->T);



    // Calculate motor PWM
    controller->motorPWM += proportional + ki * controller->integrator + controller->differentiator;
    // Limit motor PWM to range -100.0 to 100.0
    controller->motorPWM = limit_to_range(controller->motorPWM, -100.0, 100.0);

    controller->previousError = error;
    controller->previousSpeed = currentSpeed;

    return controller->motorPWM;
}

double limit_to_range(double value, double min_value, double max_value)
{
    if (value >= max_value) return max_value;
    if (value < min_value) return min_value;
    return value;
}