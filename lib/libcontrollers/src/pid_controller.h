//------------------------------------------------------------
// File name: pid_controller.h
//------------------------------------------------------------

#pragma once

// TODO: Replace it with some thing configurable
#define  PWM_LIMIT 840

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

