//------------------------------------------------------------
// File name: controller.c
//------------------------------------------------------------
#include <stdlib.h>
#include "controller.h"

signed char verify_range(signed char c);
int sing(int c);
double limit_to_range(double value, double min_value, double max_value);


Controller init_pid_controller(double kp, double ki, double kd)
{
    Controller controller = {
        .kp = kp,
        .ki = ki,
        .kd = kd,
        .previousError = 0,
        .T = 0.2,
        .previousVelocity = 0,
        .motorPWM = 0
        };
    return controller;
}

double update_pid_controller(Controller* controller, double currentVelocuty, double targetVelocity)
{
    double error = targetVelocity - currentVelocuty;
    controller->integrator += error; // + 0.5f * controller->ki * controller->T * (error + controller->previousError);
    controller->integrator = limit_to_range(controller->integrator, -80, 80);

    double differentiator = controller->kd * (currentVelocuty - controller->previousVelocity);

    controller->motorPWM += controller->kp * error + controller->ki * controller->integrator + differentiator;


    controller->motorPWM = limit_to_range(controller->motorPWM, -PWM_LIMIT, PWM_LIMIT);

    controller->previousError = error;
    controller->previousVelocity = currentVelocuty;

    return controller->motorPWM;
}


mecanum_velocity_t get_mecanum_velocities(signed char x, signed char y, signed char t)
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

    mecanum_velocity_t velocities = {
        .motor0 = nx + ny + nt,
        .motor1 = nx - ny + nt,
        .motor2 = nx + ny - nt,
        .motor3 = nx - ny - nt
        };

    return velocities;
};

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

double limit_to_range(double value, double min_value, double max_value)
{
    if (value > max_value)
    {
        return max_value;
    }
    else if (value < min_value)
    {
        return min_value;
    }
    return value;
}