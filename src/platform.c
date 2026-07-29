//------------------------------------------------------------
// File name: platform.c
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

void platform_set_target_velocity(platform_velocity_t platform_target_velocity) {
    if (!platform.is_initialized || !platform.is_controller_initialized) {
        return;
    }

    platform.set_platform_target_velocity(platform_target_velocity);
}

void platform_start_velocity_controller(plaform_controller_settings_t plaform_controller_settings) {
    if (!platform.is_initialized) {
        return;
    }
    
    platform.start_platform_velocity_controller(plaform_controller_settings);
    platform.is_controller_initialized = 1;
}

void platform_stop_velocity_controller() {
    if (!platform.is_initialized || !platform.is_controller_initialized) {
        return;
    }

    platform.is_controller_initialized = 0;
    platform.stop_platform_velocity_controller();
}

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

void platform_coast() {
    if (!platform.is_initialized) {
        return;
    }

    // Drop out of closed-loop control so the PID task stops overriding the
    // motor PWM, then let only this platform's own wheel motors coast freely.
    platform.is_controller_initialized = 0;
    controllers_manager_stop_controller_multiple(platform.motor_mask);
}

uint8_t platform_owns_motor(uint8_t motor_index) {
    if (!platform.is_initialized) {
        return 0;
    }
    return (platform.motor_mask & (1 << motor_index)) ? 1 : 0;
}

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

void platform_start_odometry() {
    // Initialize platform hardware for odometry if it is not initialized
    platform.initialize_platform_odometry();

    platform.is_odometry_enabled = 1;
    
    // Initialize odometry manager if it is not initialized
    odometry_manager_initialize();
}

uint8_t platform_is_odometry_enabled() {
    return platform.is_odometry_enabled;
}

void platform_reset_odometry() {
    // Reset odometry in odometry manager
    odometry_manager_reset_platform_odometry();
}

void platform_stop_odometry() {
    platform.is_odometry_enabled = 0;
}

platform_odometry_t platform_get_odometry() {
    // Get current platform odometry from odometry manager
    return odometry_manager_get_platform_odometry();
}

// Utils functions

double verify_range(double c) {
    if(c > 100) return 100;
    if(c < -100) return -100;
    return c;
}

double sing(double c) {
    return (c > 0) - (c < 0);
}