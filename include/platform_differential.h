#pragma once

#include <stdint.h>
#include "platform.h" // for platform_velocity_t, plaform_controller_settings_t, etc.

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the Differential platform
void initialize_differential_platform(
    uint8_t isReversed0,
    uint8_t isReversed1,
    uint8_t isEncoderReversed0,
    uint8_t isEncoderReversed1,
    double wheel_diameter,
    double wheel_base,
    double encoder_resolution);

// Sets the Differential platform velocity (PWM units)
void set_differential_platform_velocity(platform_velocity_t platform_velocity);

// Starts the Differential platform velocity controller
void differential_platform_start_velocity_controller(plaform_controller_settings_t plaform_controller_settings);

// Stops the Differential platform velocity controller
void differential_platform_stop_velocity_controller();

// Sets target velocity for Differential platform (m/s)
void differential_platform_set_target_velocity(platform_velocity_t platform_target_velocity);

// Updates Differential platform odometry
platform_odometry_t differential_platform_update_odometry(
    uint8_t* motor_indexes,
    double* velocities,
    uint8_t motor_count);

// Starts odometry for Differential platform
void initialize_differential_platform_odometry();

#ifdef __cplusplus
}
#endif