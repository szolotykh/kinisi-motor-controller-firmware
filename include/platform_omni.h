#pragma once

#include <stdint.h>
#include "platform.h" // for platform_velocity_t, plaform_controller_settings_t, etc.

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the Omni platform
void initialize_omni_platform(
    uint8_t isReversed0,
    uint8_t isReversed1,
    uint8_t isReversed2,
    uint8_t isEncoderReversed0,
    uint8_t isEncoderReversed1,
    uint8_t isEncoderReversed2,
    double wheel_diameter,
    double robot_radius,
    double encoder_resolution);

// Sets the Omni platform velocity (PWM units)
void set_omni_platform_velocity(platform_velocity_t platform_velocity);

// Starts the Omni platform velocity controller
void omni_platform_start_velocity_controller(plaform_controller_settings_t plaform_controller_settings);

// Stops the Omni platform velocity controller
void omni_platform_stop_velocity_controller();

// Sets target velocity for Omni platform (m/s)
void omni_platform_set_target_velocity(platform_velocity_t platform_target_velocity);

// Updates Omni platform odometry
platform_odometry_t omni_platform_update_odometry(
    uint8_t* motor_indexes,
    double* velocities,
    uint8_t motor_count);

// Starts odometry for Omni platform
void initialize_omni_platform_odometry();

#ifdef __cplusplus
}
#endif