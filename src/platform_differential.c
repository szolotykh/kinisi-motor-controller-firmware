#include "platform_differential.h"
#include "hw_motor.h"
#include "hw_encoder.h"
#include "encoder_odometry.h"
#include "controllers_manager.h"
#include "platform_common.h"
#include <math.h>

// Platform configuration
static struct {
    double wheel_radius;     // R = wheel_diameter/2
    double wheel_base;       // L = wheel base
    uint8_t is_initialized;
} differential_config = {0};

void set_differential_platform_velocity(platform_velocity_t platform_velocity)
{
    if (!differential_config.is_initialized) {
        return;
    }

    double V0 = platform_velocity.x - platform_velocity.t;
    double V1 = platform_velocity.x + platform_velocity.t;

    const hw_motor_interface_t* motors = get_motor_interface();
    motors->set_speed(MOTOR0, V0);
    motors->set_speed(MOTOR1, V1);
}

void differential_platform_start_velocity_controller(plaform_controller_settings_t plaform_controller_settings)
{
    if (!differential_config.is_initialized) {
        return;
    }

    controllers_manager_initialize_controller_multiple(
        BMOTOR0 | BMOTOR1,
        plaform_controller_settings.kp,
        plaform_controller_settings.ki,
        plaform_controller_settings.kd,
        plaform_controller_settings.integral_limit
    );
}

void differential_platform_stop_velocity_controller()
{
    if (!differential_config.is_initialized) {
        return;
    }

    controllers_manager_stop_controller_multiple(BMOTOR0 | BMOTOR1);
}

void differential_platform_set_target_velocity(platform_velocity_t platform_target_velocity)
{
    if (!differential_config.is_initialized || differential_config.wheel_radius == 0) {
        return;
    }

    double V0 = platform_target_velocity.x - platform_target_velocity.t * differential_config.wheel_base / 2.0;
    double V1 = platform_target_velocity.x + platform_target_velocity.t * differential_config.wheel_base / 2.0;

    const hw_motor_interface_t* motors = get_motor_interface();
    motors->set_speed(MOTOR0, V0);
    motors->set_speed(MOTOR1, V1);
}

platform_odometry_t differential_platform_update_odometry(uint8_t* motor_indexes, double* velocities, uint8_t motor_count)
{
    platform_odometry_t odometry = {
        .x = 0,
        .y = 0,
        .t = 0
    };

    if (!differential_config.is_initialized || motor_count < 2) {
        return odometry;
    }

    double R = differential_config.wheel_radius;
    double L = differential_config.wheel_base;
    double v0 = velocities[0];
    double v1 = velocities[1];

    odometry.x = R / 2.0 * (v0 + v1);
    odometry.t = R / L * (v1 - v0);

    return odometry;
}

void initialize_differential_platform_odometry()
{
    if (!differential_config.is_initialized) {
        return;
    }

    const encoder_odometry_interface_t* odometry = get_encoder_odometry_interface();
    odometry->start(MOTOR0);
    odometry->start(MOTOR1);
}

void initialize_differential_platform(
    uint8_t isReversed0,
    uint8_t isReversed1,
    uint8_t isEncoderReversed0,
    uint8_t isEncoderReversed1,
    double wheel_diameter,
    double wheel_base,
    double encoder_resolution)
{
    const hw_motor_interface_t* motors = get_motor_interface();
    const hw_encoder_interface_t* encoders = get_encoder_interface();

    motors->initialize(MOTOR0, isReversed0);
    motors->initialize(MOTOR1, isReversed1);

    if (encoder_resolution > 0)
    {
        // Encoder direction configured independently of the motor.
        encoders->initialize(MOTOR0, encoder_resolution, isEncoderReversed0);
        encoders->initialize(MOTOR1, encoder_resolution, isEncoderReversed1);
    }

    differential_config.wheel_radius = wheel_diameter / 2.0;
    differential_config.wheel_base = wheel_base;
    differential_config.is_initialized = 1;

    // Register this platform's operations into the global platform dispatcher
    // (see platform.c). Without this, platform.is_initialized stays 0 and the
    // platform.* function pointers stay NULL, so set_platform_velocity() and
    // the other dispatched calls silently no-op. Not hardware-tested yet.
    platform.set_platform_velocity = set_differential_platform_velocity;
    platform.set_platform_target_velocity = differential_platform_set_target_velocity;
    platform.start_platform_velocity_controller = differential_platform_start_velocity_controller;
    platform.stop_platform_velocity_controller = differential_platform_stop_velocity_controller;
    platform.initialize_platform_odometry = initialize_differential_platform_odometry;
    platform.update_platform_odometry = differential_platform_update_odometry;
    platform.properties.differential.wheel_diameter = wheel_diameter;
    platform.properties.differential.wheel_base = wheel_base;
    platform.is_initialized = 1;
}
