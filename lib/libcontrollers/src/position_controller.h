#pragma once
#include <stdbool.h>
#include "platform_types.h"

// Outer position PID: output is velocity, consumed by the existing velocity PID.
typedef struct {
    double kp, max_speed, tolerance;
    double ki, kd, integral_limit; // Integral contribution limit, in velocity units.
} position_settings_t;

typedef struct {
    double integral, previous_error, derivative;
    bool has_previous;
} position_pid_state_t;

typedef struct {
    position_pid_state_t x, y, heading;
    bool approaching;
} position_platform_state_t;

void position_pid_reset(position_pid_state_t *state);
double position_pid_velocity(position_settings_t settings, position_pid_state_t *state,
    double error, double dt, bool angular);
platform_velocity_t position_platform_pid_velocity(position_settings_t linear,
    position_settings_t angular, position_platform_state_t *state,
    platform_odometry_t current, platform_odometry_t target, bool differential, double dt);

bool position_settings_valid(position_settings_t settings);
double position_velocity(position_settings_t settings, double error);
platform_velocity_t position_platform_velocity(position_settings_t linear,
    position_settings_t angular, platform_odometry_t current,
    platform_odometry_t target, bool differential);
