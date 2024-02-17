//------------------------------------------------------------
// File name: platform.h
//------------------------------------------------------------

#pragma once

#include <pid_controller.h>
#include <hw_motor.h>
#include <hw_encoder.h>
#include <controllers_manager.h>

#define ENCODER_UPDATE_RATE 100

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
    double encoder_resolution; // Encoder resolution in pulses per revolution
    double integral_limit; // Integral limit must be positive. If negative or zero, integral limit is disabled
}plaform_controller_settings_t;

// Platform odometry
typedef struct{
    double x;
    double y;
    double t;
} platform_odometry_t;

/*
Initialize mecanum platform
Parameters:
    isReversed0: If true, the motor 0 is reversed
    isReversed1: If true, the motor 1 is reversed
    isReversed2: If true, the motor 2 is reversed
    isReversed3: If true, the motor 3 is reversed
    length: Length of the platform in meters
    width: Width of the platform in meters
    wheel_diameter: Wheel diameter in meters
*/
void initialize_mecanum_platform(uint8_t isReversed0, uint8_t isReversed1, uint8_t isReversed2, uint8_t isReversed3, double length, double width, double wheel_diameter);

/*
Initialize omni platform
Parameters:
    isReversed0: If true, the motor 0 is reversed
    isReversed1: If true, the motor 1 is reversed
    isReversed2: If true, the motor 2 is reversed
    wheel_diameter: Wheel diameter in meters
    robot_radius: Robot radius in meters
*/
void initialize_omni_platform(uint8_t isReversed0, uint8_t isReversed1, uint8_t isReversed2, double wheel_diameter, double robot_radius);

/*
Set platform velocity
Parameters:
    platform_velocity: Platform velocity. x, y and t are in PWM units [-100, 100]
*/
void set_platform_velocity(platform_velocity_t platform_velocity);

// Controller functions
/*
Initialize controller for current platform
Parameters:
    plaform_controller_settings: Platform controller settings
*/
void platform_initialize_controller(plaform_controller_settings_t plaform_controller_settings);

/*
Set target velocity for current platform
Parameters:
    platform_target_velocity: Platform target velocity. x, y and t are in meters per second
*/
void platform_set_target_velocity(platform_velocity_t platform_target_velocity);

// Odometry functions
/*
Start calculating platform odometry
*/
void platform_start_odometry();

/*
Check if platform odometry calculation is enabled
Returns:
    1 if platform odometry calculation is enabled, 0 if not
*/
uint8_t platform_is_odometry_enabled();

/*
Reset platform odometry
*/
void platform_reset_odometry();

/*
Stop calculating platform odometry
*/
void platform_stop_odometry();

/*
Get platform odometry
Returns:
    Platform odometry
*/
platform_odometry_t platform_get_odometry();

/*
platform update odometry
Parameters:
    motor_indexes: Motor indexes
    velocities: Velocities
    motor_count: Motor count
Returns:
    Platform odometry
*/
platform_odometry_t platform_update_odometry(uint8_t* motor_indexes, double* velocities, uint8_t motor_count);