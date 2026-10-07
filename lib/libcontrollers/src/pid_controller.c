//------------------------------------------------------------
// Velocity PID: output and integral contribution are PWM percentage points.
//------------------------------------------------------------
#include <math.h>
#include "pid_controller.h"

static double clamp(double value, double limit)
{
    return fmax(-limit, fmin(limit, value));
}

bool pid_controller_settings_valid(double kp, double ki, double kd, double integral_limit)
{
    return isfinite(kp) && kp >= 0 && isfinite(ki) && ki >= 0 &&
        isfinite(kd) && kd >= 0 && isfinite(integral_limit) &&
        integral_limit >= 0 && integral_limit <= 100;
}

void pid_controller_init(pid_controller_t* controller, double T, double kp, double ki, double kd, double integral_limit)
{
    controller->kp = kp;
    controller->ki = ki;
    controller->kd = kd;
    controller->T = T;
    controller->tau = 0.01;
    controller->max_integral = integral_limit;
    controller->min_integral = -integral_limit;
    pid_controller_reset(controller);
}

void pid_controller_reset(pid_controller_t* controller)
{
    controller->integrator = 0;
    controller->differentiator = 0;
    controller->previousError = 0;
    controller->previousSpeed = 0;
    controller->motorPWM = 0;
    controller->target_speed = 0;
    controller->has_previous = false;
}

double pid_controller_update(pid_controller_t* controller, double currentSpeed, double targetSpeed)
{
    if (!isfinite(currentSpeed) || !isfinite(targetSpeed) ||
        !isfinite(controller->T) || controller->T <= 0 ||
        !isfinite(controller->tau) || controller->tau < 0 ||
        !pid_controller_settings_valid(controller->kp, controller->ki, controller->kd, controller->max_integral)) {
        pid_controller_reset(controller);
        return 0;
    }
    // Preserve the API's explicit stop semantics: zero target clears all output.
    if (fabs(targetSpeed) < 1e-6) {
        pid_controller_reset(controller);
        return 0;
    }
    double error = targetSpeed - currentSpeed;
    double proportional = controller->kp * error;
    double previous_error = controller->has_previous ? controller->previousError : error;
    double integral = controller->ki > 0 ? clamp(controller->integrator +
        controller->ki * controller->T * (0.5 * error + 0.5 * previous_error),
        controller->max_integral) : 0;
    // Derivative on measurement avoids target-step kicks. Backward Euler gives
    // a stable 10 ms low-pass filter, including when the loop period changes.
    double derivative = 0;
    if (controller->has_previous && controller->kd > 0) {
        double denominator = controller->tau + controller->T;
        derivative = controller->tau / denominator * controller->differentiator
            - controller->kd / denominator * (currentSpeed - controller->previousSpeed);
    }
    double output = proportional + integral + derivative;
    // Only reject integration that pushes farther into actuator saturation;
    // allow the stored integral to unwind during reversal or recovery.
    if ((output > 100 && integral > controller->integrator) ||
        (output < -100 && integral < controller->integrator)) {
        integral = controller->integrator;
        output = proportional + integral + derivative;
    }
    if (!isfinite(output) || !isfinite(derivative) || !isfinite(error)) {
        pid_controller_reset(controller);
        return 0;
    }
    controller->integrator = integral;
    controller->differentiator = derivative;
    controller->previousError = error;
    controller->previousSpeed = currentSpeed;
    controller->has_previous = true;
    controller->target_speed = targetSpeed;
    controller->motorPWM = clamp(output, 100);
    return controller->motorPWM;
}
