#include "platform_omni.h"
#include "platform_common.h"
#include "controllers_manager.h"
#include "hw_motor.h"
#include "hw_encoder.h"
#include "encoder_odometry.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

// Platform configuration
static struct {
    double wheel_radius;     // R = wheel_diameter/2
    double robot_radius;     // L = distance from center to wheels
    uint8_t is_initialized;
} omni_config = {0};

void set_omni_platform_velocity(platform_velocity_t platform_velocity)
{
    if (!omni_config.is_initialized) {
        return;
    }

    double V1 = sqrt(3.0)/2.0 * platform_velocity.x - 0.5 * platform_velocity.y + platform_velocity.t;
    double V2 = -sqrt(3.0)/2.0 * platform_velocity.x - 0.5 * platform_velocity.y + platform_velocity.t;
    double V3 = platform_velocity.y + platform_velocity.t;

    double maxv = fmax(fabs(V1), fmax(fabs(V2), fabs(V3)));
    if (maxv > 100.0)
    {
        V1 *= 100.0 / maxv;
        V2 *= 100.0 / maxv;
        V3 *= 100.0 / maxv;
    }

    const hw_motor_interface_t* motors = get_motor_interface();
    motors->set_speed(MOTOR0, V1);
    motors->set_speed(MOTOR1, V2);
    motors->set_speed(MOTOR2, V3);
}

void omni_platform_start_velocity_controller(plaform_controller_settings_t plaform_controller_settings)
{
    if (!omni_config.is_initialized) {
        return;
    }

    controllers_manager_initialize_controller_multiple(
        BMOTOR0 | BMOTOR1 | BMOTOR2,
        plaform_controller_settings.kp,
        plaform_controller_settings.ki,
        plaform_controller_settings.kd,
        plaform_controller_settings.integral_limit
    );
}

void omni_platform_stop_velocity_controller()
{
    if (!omni_config.is_initialized) {
        return;
    }

    controllers_manager_stop_controller_multiple(BMOTOR0 | BMOTOR1 | BMOTOR2);
}

void omni_platform_set_target_velocity(platform_velocity_t platform_target_velocity)
{
    if (!omni_config.is_initialized || omni_config.wheel_radius == 0) {
        return;
    }

    double V1 = 1.0 / omni_config.wheel_radius * 
        (sqrt(3.0)/2.0 * platform_target_velocity.x - 
         0.5 * platform_target_velocity.y + 
         omni_config.robot_radius * platform_target_velocity.t);

    double V2 = 1.0 / omni_config.wheel_radius * 
        (-sqrt(3.0)/2.0 * platform_target_velocity.x - 
         0.5 * platform_target_velocity.y + 
         omni_config.robot_radius * platform_target_velocity.t);

    double V3 = 1.0 / omni_config.wheel_radius * 
        (platform_target_velocity.y + 
         omni_config.robot_radius * platform_target_velocity.t);
    
    uint8_t motor_indexes[] = {MOTOR0, MOTOR1, MOTOR2};
    double target_speeds[] = {V1, V2, V3};
    controllers_manager_set_target_speed_multiple(motor_indexes, target_speeds, 3);
}

platform_odometry_t omni_platform_update_odometry(uint8_t* motor_indexes, double* velocities, uint8_t motor_count)
{
    platform_odometry_t odometry = {
        .x = 0,
        .y = 0,
        .t = 0
    };

    if(!omni_config.is_initialized || motor_count < 3 || omni_config.wheel_radius == 0)
    {
        return odometry;
    }

    double v1 = velocities[0];
    double v2 = velocities[1];
    double v3 = velocities[2];

    odometry.x = omni_config.wheel_radius * (1/sqrt(3.0) * v1 - 1/sqrt(3.0) * v2);
    odometry.y = omni_config.wheel_radius/3.0 * (-v1 - v2 + 2.0 * v3);
    odometry.t = omni_config.wheel_radius/(3.0 * omni_config.robot_radius) * (v1 + v2 + v3);

    return odometry;
}

void initialize_omni_platform_odometry()
{
    if (!omni_config.is_initialized) {
        return;
    }

    const encoder_odometry_interface_t* odometry = get_encoder_odometry_interface();
    odometry->start(MOTOR0);
    odometry->start(MOTOR1);
    odometry->start(MOTOR2);
}

void initialize_omni_platform(
    uint8_t isReversed0,
    uint8_t isReversed1,
    uint8_t isReversed2,
    uint8_t isEncoderReversed0,
    uint8_t isEncoderReversed1,
    uint8_t isEncoderReversed2,
    double wheel_diameter,
    double robot_radius,
    double encoder_resolution)
{
    // Re-initialization is allowed: calling this again re-applies motor/encoder
    // reverse flags, resolution and dimensions without a board reset. The
    // hardware timer setup inside motor/encoder initialize is separately
    // guarded, so only the settings are updated.
    const hw_motor_interface_t* motors = get_motor_interface();
    const hw_encoder_interface_t* encoders = get_encoder_interface();

    motors->initialize(MOTOR0, isReversed0);
    motors->initialize(MOTOR1, isReversed1);
    motors->initialize(MOTOR2, isReversed2);

    if (encoder_resolution > 0)
    {
        // Encoder direction configured independently of the motor.
        encoders->initialize(MOTOR0, encoder_resolution, isEncoderReversed0);
        encoders->initialize(MOTOR1, encoder_resolution, isEncoderReversed1);
        encoders->initialize(MOTOR2, encoder_resolution, isEncoderReversed2);
    }

    omni_config.wheel_radius = wheel_diameter / 2.0;
    omni_config.robot_radius = robot_radius;
    omni_config.is_initialized = 1;

    // Register this platform's operations into the global platform dispatcher.
    // Without this, platform.is_initialized stays 0 and platform.* function
    // pointers stay NULL, so set_platform_velocity() (and the other dispatched
    // calls in platform.c) silently no-op -- i.e. "set platform velocity does
    // nothing".
    platform.set_platform_velocity = set_omni_platform_velocity;
    platform.set_platform_target_velocity = omni_platform_set_target_velocity;
    platform.start_platform_velocity_controller = omni_platform_start_velocity_controller;
    platform.stop_platform_velocity_controller = omni_platform_stop_velocity_controller;
    platform.initialize_platform_odometry = initialize_omni_platform_odometry;
    platform.update_platform_odometry = omni_platform_update_odometry;
    platform.properties.omni.wheel_diameter = wheel_diameter;
    platform.properties.omni.robot_radius = robot_radius;
    platform.is_initialized = 1;
}
