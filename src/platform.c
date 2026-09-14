//------------------------------------------------------------
// File name: platform.c
// Description: Dispatch platform operations and expose initialized/controller/odometry state.
//------------------------------------------------------------
#include <stdlib.h>
#include "platform.h"
#include "platform_common.h"
#include "hw_motor.h"
#include "hw_encoder.h"
#include <stdint.h>
#include <math.h>
#include <controllers_manager.h>
#include <odometry_manager.h>

#define SPEED_RESOLUTION 840

// Internal variables
platform_t platform = {
    .is_initialized = 0,
    .is_controller_initialized = 0
};

// ------------------------------------------------------------------------
// Platform functions

/**
 * @brief Report whether a platform has been configured for dispatch.
 */
uint8_t platform_is_initialized(void) { return platform.is_initialized; }
/**
 * @brief Report whether platform closed-loop control has been started.
 */
uint8_t platform_is_controller_running(void) { return platform.is_controller_initialized; }

/**
 * @brief Apply bounded open-loop platform velocity when a platform is initialized.
 */
void set_platform_velocity(platform_velocity_t platform_velocity) {
    // Set platform velocity only if platform is initialized
    if (!platform.is_initialized) {
        return;
    }
    // Verify that velocity is in range [-100, 100] and adjust if needed
    platform_velocity.x = verify_range(platform_velocity.x);
    platform_velocity.y = verify_range(platform_velocity.y);
    platform_velocity.t = verify_range(platform_velocity.t);

    // Set velocity for initialized platform
    platform.set_platform_velocity(platform_velocity);
}

/**
 * @brief Set platform velocity targets when platform closed-loop control is running.
 */
void platform_set_target_velocity(platform_velocity_t platform_target_velocity) {
    if (!platform.is_initialized || !platform.is_controller_initialized) {
        return;
    }

    platform.set_platform_target_velocity(platform_target_velocity);
}

/**
 * @brief Initialize the configured platform controller and mark it running.
 */
void platform_start_velocity_controller(plaform_controller_settings_t plaform_controller_settings) {
    if (!platform.is_initialized) {
        return;
    }
    
    platform.start_platform_velocity_controller(plaform_controller_settings);
    platform.is_controller_initialized = 1;
}

/**
 * @brief Stop the configured platform controller when it is active.
 */
void platform_stop_velocity_controller() {
    if (!platform.is_initialized || !platform.is_controller_initialized) {
        return;
    }

    platform.is_controller_initialized = 0;
    platform.stop_platform_velocity_controller();
}

/**
 * @brief Disable platform closed-loop control and brake its owned wheel motors.
 */
void platform_brake() {
    if (!platform.is_initialized) {
        return;
    }

    // Drop out of closed-loop control so the PID task stops overriding the
    // motor PWM, then actively brake only this platform's own wheel motors.
    // motor_mask excludes any motor used for something outside the platform.
    platform.is_controller_initialized = 0;
    controllers_manager_brake_multiple(platform.motor_mask);
}

/**
 * @brief Disable platform closed-loop control and coast its owned wheel motors.
 */
void platform_coast() {
    if (!platform.is_initialized) {
        return;
    }

    // Drop out of closed-loop control so the PID task stops overriding the
    // motor PWM, then let only this platform's own wheel motors coast freely.
    platform.is_controller_initialized = 0;
    controllers_manager_stop_controller_multiple(platform.motor_mask);
}

/**
 * @brief Check whether the initialized platform owns the selected motor.
 */
uint8_t platform_owns_motor(uint8_t motor_index) {
    if (!platform.is_initialized) {
        return 0;
    }
    return (platform.motor_mask & (1 << motor_index)) ? 1 : 0;
}

/**
 * @brief Convert wheel increments through the configured platform kinematics.
 */
platform_odometry_t platform_update_odometry(uint8_t* motor_indexes, double* velocities, uint8_t motor_count) {
    if (!platform.is_initialized) {
        platform_odometry_t odometry = {
            .x = 0,
            .y = 0,
            .t = 0
        };
        return odometry;
    }

    return platform.update_platform_odometry(motor_indexes, velocities, motor_count);
}

/**
 * @brief Start configured platform odometry and invalidate the cached acquisition timestamp.
 */
void platform_start_odometry() {
    if (!platform.is_initialized || !platform.initialize_platform_odometry) return;
    // Initialize platform hardware for odometry if it is not initialized
    platform.initialize_platform_odometry();

    platform.is_odometry_enabled = 1;
    
    // Initialize odometry manager if it is not initialized
    odometry_manager_initialize();
    odometry_manager_invalidate_platform_sample();
}

/**
 * @brief Report whether platform odometry integration is enabled.
 */
uint8_t platform_is_odometry_enabled() {
    return platform.is_odometry_enabled;
}

/**
 * @brief Reset the accumulated platform pose and cached acquisition timestamp.
 */
void platform_reset_odometry() {
    // Reset odometry in odometry manager
    odometry_manager_reset_platform_odometry();
}

/**
 * @brief Disable platform integration without resetting its accumulated pose.
 */
void platform_stop_odometry() {
    platform.is_odometry_enabled = 0;
}

/**
 * @brief Return the cached platform pose through the odometry manager.
 */
platform_odometry_t platform_get_odometry() {
    // Get current platform odometry from odometry manager
    return odometry_manager_get_platform_odometry();
}

// Utils functions

/**
 * @brief Clamp a signed open-loop command component to the range -100 through 100.
 */
double verify_range(double c) {
    if(c > 100) return 100;
    if(c < -100) return -100;
    return c;
}

/**
 * @brief Return -1, 0, or 1 according to the sign of the supplied value.
 */
double sing(double c) {
    return (c > 0) - (c < 0);
}
