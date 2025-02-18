#pragma once

// Platform velocity
typedef struct{
    double x;
    double y;
    double t;
} platform_velocity_t;

// Platform controller settings
typedef struct{
    double kp;
    double ki;
    double kd;
    double integral_limit; // Integral limit must be positive. If negative or zero, integral limit is disabled
} plaform_controller_settings_t;

// Platform odometry
typedef struct{
    double x;
    double y;
    double t;
} platform_odometry_t;