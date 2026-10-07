#include "position_controller.h"
#include <math.h>
#include <float.h>

static double clamp(double value, double limit) { return fmax(-limit, fmin(limit, value)); }
static double wrap(double angle) { return remainder(angle, 2.0 * M_PI); }

bool position_settings_valid(position_settings_t s)
{
    return isfinite(s.kp) && s.kp > 0 && isfinite(s.max_speed) &&
        s.max_speed > 0 && isfinite(s.tolerance) && s.tolerance >= 0 &&
        isfinite(s.ki) && s.ki >= 0 && isfinite(s.kd) && s.kd >= 0 &&
        isfinite(s.integral_limit) && s.integral_limit >= 0;
}

void position_pid_reset(position_pid_state_t *state) { *state = (position_pid_state_t){0}; }

double position_pid_velocity(position_settings_t s, position_pid_state_t *state,
    double error, double dt, bool angular)
{
    if (!isfinite(error)) { position_pid_reset(state); return 0; }
    if (fabs(error) <= s.tolerance) { position_pid_reset(state); return 0; }
    // Never accumulate or differentiate over an invalid/unknown time interval.
    bool timed = isfinite(dt) && dt > 0;
    double derivative = 0;
    if (timed && state->has_previous) {
        double delta = error - state->previous_error;
        if (angular) delta = wrap(delta);
        // A 20 ms low-pass filter reduces encoder-quantization derivative noise.
        derivative = state->derivative + dt / (0.02 + dt) * (delta / dt - state->derivative);
    }
    double integral = timed ? clamp(state->integral + s.ki * error * dt, s.integral_limit) : state->integral;
    double output = s.kp * error + integral + s.kd * derivative;
    // Conditional integration: don't wind up against the speed limit.
    if ((output > s.max_speed && error > 0) || (output < -s.max_speed && error < 0)) {
        integral = state->integral;
        output = s.kp * error + integral + s.kd * derivative;
    }
    if (!isfinite(output) || !isfinite(derivative)) { position_pid_reset(state); return 0; }
    state->integral = integral;
    state->previous_error = error;
    state->derivative = derivative;
    state->has_previous = timed;
    return clamp(output, s.max_speed);
}

platform_velocity_t position_platform_pid_velocity(position_settings_t linear,
    position_settings_t angular, position_platform_state_t *state,
    platform_odometry_t current, platform_odometry_t target, bool differential, double dt)
{
    platform_velocity_t v = {0};
    double dx = target.x - current.x, dy = target.y - current.y;
    double distance = hypot(dx, dy);
    if (!isfinite(distance) || !isfinite(current.t) || !isfinite(target.t)) {
        *state = (position_platform_state_t){0}; return v;
    }
    bool approaching = distance > linear.tolerance;
    if (differential && approaching != state->approaching) position_pid_reset(&state->heading);
    state->approaching = approaching;
    double bearing = wrap(atan2(dy, dx) - current.t);
    double heading_error = differential && approaching ? bearing : wrap(target.t - current.t);
    v.t = position_pid_velocity(angular, &state->heading, heading_error, dt, true);
    if (!approaching) {
        position_pid_reset(&state->x); position_pid_reset(&state->y); return v;
    }
    if (differential) {
        // Rotate toward the point, approach it, then align final heading.
        double speed = position_pid_velocity(linear, &state->x, distance, dt, false);
        double alignment = fmax(0.0, cos(bearing));
        if (alignment < 1e-6) position_pid_reset(&state->x);
        v.x = fmax(0.0, speed) * alignment;
    } else {
        // Independent world-frame X/Y histories share translation gains.
        // Use radial tolerance, so diagonal errors cannot stop too early.
        linear.tolerance = 0;
        position_settings_t axis = linear;
        axis.max_speed = DBL_MAX; // Limit the translation vector, not each component.
        double old_x = state->x.integral, old_y = state->y.integral;
        double vx = position_pid_velocity(axis, &state->x, dx, dt, false);
        double vy = position_pid_velocity(axis, &state->y, dy, dt, false);
        double speed = hypot(vx, vy);
        if (speed > linear.max_speed) {
            if (vx * dx + vy * dy > 0) { state->x.integral = old_x; state->y.integral = old_y; }
            vx *= linear.max_speed / speed; vy *= linear.max_speed / speed;
        }
        v.x = cos(current.t) * vx + sin(current.t) * vy;
        v.y = -sin(current.t) * vx + cos(current.t) * vy;
    }
    return v;
}

// Stateless helpers retained for proportional-only callers.
double position_velocity(position_settings_t s, double error)
{
    position_pid_state_t state = {0};
    return position_pid_velocity(s, &state, error, 0, false);
}
platform_velocity_t position_platform_velocity(position_settings_t linear,
    position_settings_t angular, platform_odometry_t current,
    platform_odometry_t target, bool differential)
{
    position_platform_state_t state = {0};
    return position_platform_pid_velocity(linear, angular, &state, current, target, differential, 0);
}
