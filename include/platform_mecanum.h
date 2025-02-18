#pragma once

#include <stdint.h>
#include "platform_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initializes the Mecanum platform
void initialize_mecanum_platform(
    uint8_t isReversed0,
    uint8_t isReversed1,
    uint8_t isReversed2,
    uint8_t isReversed3,
    double length,
    double width,
    double wheel_diameter,
    double encoder_resolution);

// Sets Mecanum platform velocity (PWM units)
void mecaunm_platform_set_velocity(platform_velocity_t platform_velocity);

// Stops velocity controller for Mecanum platform
void mecanum_platform_stop_velocity_controller();

// Starts velocity controller for Mecanum platform
void mecanum_platform_start_velocity_controller(plaform_controller_settings_t plaform_controller_settings);

// Sets target velocity for Mecanum platform (m/s)
void mecanum_platform_set_target_velocity(platform_velocity_t platform_target_velocity);

// Updates Mecanum platform odometry
platform_odometry_t mecanum_platform_update_odometry(
    uint8_t* motor_indexes,
    double* velocities,
    uint8_t motor_count);

// Starts odometry for Mecanum platform
void mecanum_platform_start_odometry();

#ifdef __cplusplus
}
#endif
