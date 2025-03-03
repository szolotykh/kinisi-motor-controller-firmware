//------------------------------------------------------------
// File name: platform.h
//------------------------------------------------------------

#pragma once

#include "platform_types.h"
#include <pid_controller.h>
#include <hw_motor.h>
#include <hw_encoder.h>
#include <controllers_manager.h>
#include "platform_mecanum.h"
#include "platform_omni.h"
#include "platform_differential.h"

#define ENCODER_UPDATE_RATE 100

// Common platform functions

// Set platform velocity
// Parameters:
//     platform_velocity: Platform velocity. x, y and t are in PWM units [-100, 100]
void set_platform_velocity(platform_velocity_t platform_velocity);

// Start velocity controller for current platform
// Parameters:
//     plaform_controller_settings: Platform controller settings
void platform_start_velocity_controller(plaform_controller_settings_t plaform_controller_settings);

// Set target velocity for current platform
// Parameters:
//     platform_target_velocity: Platform target velocity. x, y and t are in meters per second
void platform_set_target_velocity(platform_velocity_t platform_target_velocity);

// Stop velocity controller for current platform
void platform_stop_velocity_controller();

// Start calculating platform odometry
void platform_start_odometry();

// Check if platform odometry calculation is enabled
// Returns:
//     1 if platform odometry calculation is enabled, 0 if not
uint8_t platform_is_odometry_enabled();

// Reset platform odometry
void platform_reset_odometry();

// Stop calculating platform odometry
void platform_stop_odometry();

// Get platform odometry
// Returns:
//     Platform odometry
platform_odometry_t platform_get_odometry();

// Update platform odometry
// Parameters:
//     motor_indexes: Motor indexes
//     velocities: Velocities
//     motor_count: Motor count
// Returns:
//     Platform odometry
platform_odometry_t platform_update_odometry(uint8_t* motor_indexes, double* velocities, uint8_t motor_count);

// Utils functions
double verify_range(double c);
double sing(double c);