//------------------------------------------------------------
// File name: platform.h
// Description: Expose platform setup, motion control, and odometry lifecycle operations.
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

/**
 * @brief Report whether a platform has been configured for dispatch.
 */
uint8_t platform_is_initialized(void);
/**
 * @brief Report whether platform closed-loop control has been started.
 */
uint8_t platform_is_controller_running(void);

// Common platform functions

// Set platform velocity
// Parameters:
//     platform_velocity: Platform velocity. x, y and t are in PWM units [-100, 100]
/**
 * @brief Apply bounded open-loop platform velocity when a platform is initialized.
 */
void set_platform_velocity(platform_velocity_t platform_velocity);

// Start velocity controller for current platform
// Parameters:
//     plaform_controller_settings: Platform controller settings
/**
 * @brief Initialize the configured platform controller and mark it running.
 */
void platform_start_velocity_controller(plaform_controller_settings_t plaform_controller_settings);

// Set target velocity for current platform
// Parameters:
//     platform_target_velocity: Platform target velocity. x, y and t are in meters per second
/**
 * @brief Set platform velocity targets when platform closed-loop control is running.
 */
void platform_set_target_velocity(platform_velocity_t platform_target_velocity);

// Stop velocity controller for current platform
/**
 * @brief Stop the configured platform controller when it is active.
 */
void platform_stop_velocity_controller();

// Actively brake all platform motors (short brake). Stops the velocity
// controller if running so the closed loop does not override the brake.
// The motors resist motion until a new command is issued.
/**
 * @brief Disable platform closed-loop control and brake its owned wheel motors.
 */
void platform_brake();

// Let all platform motors coast freely (high impedance). Stops the velocity
// controller if running so the closed loop does not override the coast.
// The motors spin down without resistance.
/**
 * @brief Disable platform closed-loop control and coast its owned wheel motors.
 */
void platform_coast();

// Returns 1 if the given motor index is currently owned/driven by the active
// platform (i.e. a platform is initialized and this motor is one of its
// wheels), 0 otherwise. Used to protect platform wheels from direct
// single-motor commands.
/**
 * @brief Check whether the initialized platform owns the selected motor.
 */
uint8_t platform_owns_motor(uint8_t motor_index);

// Start calculating platform odometry
/**
 * @brief Start configured platform odometry and invalidate the cached acquisition timestamp.
 */
void platform_start_odometry();

// Check if platform odometry calculation is enabled
// Returns:
//     1 if platform odometry calculation is enabled, 0 if not
/**
 * @brief Report whether platform odometry integration is enabled.
 */
uint8_t platform_is_odometry_enabled();

// Reset platform odometry
/**
 * @brief Reset the accumulated platform pose and cached acquisition timestamp.
 */
void platform_reset_odometry();

// Stop calculating platform odometry
/**
 * @brief Disable platform integration without resetting its accumulated pose.
 */
void platform_stop_odometry();

// Get platform odometry
// Returns:
//     Platform odometry
/**
 * @brief Return the cached platform pose through the odometry manager.
 */
platform_odometry_t platform_get_odometry();

// Update platform odometry
// Parameters:
//     motor_indexes: Motor indexes
//     velocities: Velocities
//     motor_count: Motor count
// Returns:
//     Platform odometry
/**
 * @brief Convert wheel increments through the configured platform kinematics.
 */
platform_odometry_t platform_update_odometry(uint8_t* motor_indexes, double* velocities, uint8_t motor_count);

// Utils functions
/**
 * @brief Clamp a signed open-loop command component to the range -100 through 100.
 */
double verify_range(double c);
/**
 * @brief Return -1, 0, or 1 according to the sign of the supplied value.
 */
double sing(double c);
