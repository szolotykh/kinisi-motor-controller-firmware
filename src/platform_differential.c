#include "platform_differential.h"
#include "platform_common.h"
#include "controllers_manager.h"
#include "hw_motor.h"
#include "hw_encoder.h"
#include "encoder_odometry.h"
#include <math.h>
#include <stdlib.h>

void set_differential_platform_velocity(platform_velocity_t platform_velocity)
{
    double V0 = platform_velocity.x - platform_velocity.t * platform.properties.differential.wheel_base / 2.0;
    double V1 = platform_velocity.x + platform_velocity.t * platform.properties.differential.wheel_base / 2.0;

    set_motor_speed(MOTOR0, V0);
    set_motor_speed(MOTOR1, V1);
}

void differential_platform_start_velocity_controller(plaform_controller_settings_t plaform_controller_settings)
{
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
    controllers_manager_stop_controller_multiple(BMOTOR0 | BMOTOR1);
}

void differential_platform_set_target_velocity(platform_velocity_t platform_target_velocity)
{
    double V0 = platform_target_velocity.x - platform_target_velocity.t * platform.properties.differential.wheel_base / 2.0;
    double V1 = platform_target_velocity.x + platform_target_velocity.t * platform.properties.differential.wheel_base / 2.0;

    set_motor_speed(MOTOR0, V0);
    set_motor_speed(MOTOR1, V1);
}

platform_odometry_t differential_platform_update_odometry(uint8_t* motor_indexes, double* velocities, uint8_t motor_count)
{
    platform_odometry_t odometry = {
        .x = 0,
        .y = 0,
        .t = 0
    };

    if (motor_count < 2)
    {
        return odometry;
    }

    double R = platform.properties.differential.wheel_diameter / 2.0;
    double L = platform.properties.differential.wheel_base;
    double v0 = velocities[0];
    double v1 = velocities[1];

    odometry.x = R / 2.0 * (v0 + v1);
    odometry.t = R / L * (v1 - v0);

    return odometry;
}

void initialize_differential_platform_odometry()
{
    encoder_start_odometry(MOTOR0);
    encoder_start_odometry(MOTOR1);
}

void initialize_differential_platform(
    uint8_t isReversed0,
    uint8_t isReversed1,
    double wheel_diameter,
    double wheel_base,
    double encoder_resolution)
{
    initialize_motor(MOTOR0, isReversed0);
    initialize_motor(MOTOR1, isReversed1);

    if (encoder_resolution > 0)
    {
        initialize_encoder(MOTOR0, encoder_resolution, isReversed0);
        initialize_encoder(MOTOR1, encoder_resolution, isReversed1);
    }
}
