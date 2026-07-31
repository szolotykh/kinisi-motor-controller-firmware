#pragma once

#include <stdint.h>
#include "platform_types.h"

// Platform settings for each type
typedef struct {
    double wheel_diameter;
    double length;
    double width;
} mecanum_platform_settings_t;

typedef struct {
    double wheel_diameter;
    double robot_radius;
} omni_platform_settings_t;

typedef struct {
    double wheel_diameter;
    double wheel_base;
} differential_platform_settings_t;

// Function types for platform operations
typedef void (*set_platform_velocity_t)(platform_velocity_t);
typedef void (*set_platform_target_velocity_t)(platform_velocity_t);
typedef void (*start_platform_velocity_controller_t)(plaform_controller_settings_t);
typedef void (*stop_platform_velocity_controller_t)();
typedef void (*initialize_platform_odometry_t)();
typedef platform_odometry_t (*update_platform_odometry_t)(uint8_t* motor_indexes, double* velocities, uint8_t motor_count);

// Platform structure
typedef struct {
    uint8_t is_initialized;
    uint8_t is_controller_initialized;

    // Bit mask (BMOTOR0..BMOTOR3) of the motors this platform actually drives.
    // Used by platform_brake()/platform_coast() so they only affect this
    // platform's wheels and never touch a motor used for something else.
    uint8_t motor_mask;
    
    set_platform_velocity_t set_platform_velocity;
    set_platform_target_velocity_t set_platform_target_velocity;
    start_platform_velocity_controller_t start_platform_velocity_controller;
    stop_platform_velocity_controller_t stop_platform_velocity_controller;

    union {
        mecanum_platform_settings_t mecanum;
        omni_platform_settings_t omni;
        differential_platform_settings_t differential;
    } properties;

    uint8_t is_odometry_enabled;
    initialize_platform_odometry_t initialize_platform_odometry;
    update_platform_odometry_t update_platform_odometry;
} platform_t;

// Global platform instance
extern platform_t platform;