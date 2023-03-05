//------------------------------------------------------------
// File name: pid_controller.c
//------------------------------------------------------------
#include <stdlib.h>
#include "pid_controller.h"

signed char verify_range(signed char c);
int sing(int c);
double limit_to_range(double value, double min_value, double max_value);


void pid_controller_init(pid_controller_t* controller, double kp, double ki, double kd)
{
    controller->kp = kp;
    controller->ki = ki;
    controller->kd = kd;
    controller->previousError = 0;
    controller->T = 0.2;
    controller->previousVelocity = 0;
    controller->motorPWM = 0;
    controller->integrator = 0;
}

double pid_controller_update(pid_controller_t* controller, double currentVelocuty, double targetVelocity)
{
    // TODO Add mutex
    double kp = controller->kp;
    double ki = controller->ki;
    double kd = controller->kd;

    double error = targetVelocity - currentVelocuty;
    controller->integrator += error; // + 0.5f * controller->ki * controller->T * (error + controller->previousError);
    controller->integrator = limit_to_range(controller->integrator, -80, 80);

    double differentiator = kd * (currentVelocuty - controller->previousVelocity);

    controller->motorPWM += kp * error + ki * controller->integrator + differentiator;


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