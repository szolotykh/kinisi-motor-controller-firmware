#include "platform_mecanum.h"
#include "platform_common.h"
#include "controllers_manager.h"
#include "hw_motor.h"
#include "hw_encoder.h"
#include "encoder_odometry.h"
#include <math.h>
#include <stdlib.h>

// Platform configuration
static struct {
    double wheel_radius;     // R = wheel_diameter/2
    double length;          // L = platform length
    double width;           // W = platform width
    uint8_t is_initialized;
} mecanum_config = {0};

void mecaunm_platform_set_velocity(platform_velocity_t platform_velocity) {
    if (!mecanum_config.is_initialized) {
        return;
    }

    // Normalize velocity
    int l = abs(platform_velocity.x) + abs(platform_velocity.y) + abs(platform_velocity.t);
    if (l == 0) {
        // If velocities are all zero, just set motors to zero
        const hw_motor_interface_t* motors = get_motor_interface();
        motors->set_speed(MOTOR0, 0);
        motors->set_speed(MOTOR1, 0);
        motors->set_speed(MOTOR2, 0);
        motors->set_speed(MOTOR3, 0);
        return;
    }

    double sing_x = (platform_velocity.x > 0) - (platform_velocity.x < 0);
    double sing_y = (platform_velocity.y > 0) - (platform_velocity.y < 0);
    double sing_t = (platform_velocity.t > 0) - (platform_velocity.t < 0);

    platform_velocity.x = sing_x * platform_velocity.x * platform_velocity.x / l;
    platform_velocity.y = sing_y * platform_velocity.y * platform_velocity.y / l;
    platform_velocity.t = sing_t * platform_velocity.t * platform_velocity.t / l;

    double motor0 = platform_velocity.x + platform_velocity.y + platform_velocity.t;
    double motor1 = platform_velocity.x - platform_velocity.y + platform_velocity.t;
    double motor2 = platform_velocity.x + platform_velocity.y - platform_velocity.t;
    double motor3 = platform_velocity.x - platform_velocity.y - platform_velocity.t;

    const hw_motor_interface_t* motors = get_motor_interface();
    motors->set_speed(MOTOR0, motor0);
    motors->set_speed(MOTOR1, motor1);
    motors->set_speed(MOTOR2, motor2);
    motors->set_speed(MOTOR3, motor3);
}

void mecanum_platform_start_velocity_controller(plaform_controller_settings_t plaform_controller_settings)
{
    if (!mecanum_config.is_initialized) {
        return;
    }

    controllers_manager_initialize_controller_multiple(
        BMOTOR0 | BMOTOR1 | BMOTOR2 | BMOTOR3,
        plaform_controller_settings.kp,
        plaform_controller_settings.ki,
        plaform_controller_settings.kd,
        plaform_controller_settings.integral_limit
    );
}

void mecanum_platform_stop_velocity_controller()
{
    if (!mecanum_config.is_initialized) {
        return;
    }

    controllers_manager_stop_controller_multiple(BMOTOR0 | BMOTOR1 | BMOTOR2 | BMOTOR3);
}

void mecanum_platform_set_target_velocity(platform_velocity_t platform_target_velocity)
{
    if (!mecanum_config.is_initialized || mecanum_config.wheel_radius == 0) {
        return;
    }

    double R = mecanum_config.wheel_radius;
    double L = mecanum_config.length;
    double W = mecanum_config.width;

    double V1 = 1.0 / R * (platform_target_velocity.x + platform_target_velocity.y + (L + W) / 2.0 * platform_target_velocity.t);
    double V2 = 1.0 / R * (platform_target_velocity.x - platform_target_velocity.y + (L + W) / 2.0 * platform_target_velocity.t);
    double V3 = 1.0 / R * (platform_target_velocity.x + platform_target_velocity.y - (L + W) / 2.0 * platform_target_velocity.t);
    double V4 = 1.0 / R * (platform_target_velocity.x - platform_target_velocity.y - (L + W) / 2.0 * platform_target_velocity.t);

    uint8_t motor_indexes[] = {MOTOR0, MOTOR1, MOTOR2, MOTOR3};
    double target_speeds[]  = {V1, V2, V3, V4};
    controllers_manager_set_target_speed_multiple(motor_indexes, target_speeds, 4);
}

platform_odometry_t mecanum_platform_update_odometry(
    uint8_t* motor_indexes,
    double* velocities,
    uint8_t motor_count)
{
    platform_odometry_t odometry = {
        .x = 0,
        .y = 0,
        .t = 0
    };

    if(!mecanum_config.is_initialized || motor_count < 4 || 
       mecanum_config.wheel_radius == 0 || (mecanum_config.length + mecanum_config.width) == 0)
    {
        return odometry;
    }

    double R = mecanum_config.wheel_radius;
    double L = mecanum_config.length;
    double W = mecanum_config.width;

    double v1 = velocities[0];
    double v2 = velocities[1];
    double v3 = velocities[2];
    double v4 = velocities[3];

    odometry.x = R/4.0 * (v1 + v2 + v3 + v4);
    odometry.y = R/4.0 * (v1 - v2 + v3 - v4);
    odometry.t = R/(2.0 * (L + W)) * (v1 + v2 - v3 - v4);

    return odometry;
}

void mecanum_platform_start_odometry()
{
    if (!mecanum_config.is_initialized) {
        return;
    }
    
    const encoder_odometry_interface_t* odometry = get_encoder_odometry_interface();
    odometry->start(MOTOR0);
    odometry->start(MOTOR1);
    odometry->start(MOTOR2);
    odometry->start(MOTOR3);
}

void initialize_mecanum_platform(
    uint8_t isReversed0,
    uint8_t isReversed1,
    uint8_t isReversed2,
    uint8_t isReversed3,
    uint8_t isEncoderReversed0,
    uint8_t isEncoderReversed1,
    uint8_t isEncoderReversed2,
    uint8_t isEncoderReversed3,
    double length,
    double width,
    double wheel_diameter,
    double encoder_resolution)
{
    const hw_motor_interface_t* motors = get_motor_interface();
    const hw_encoder_interface_t* encoders = get_encoder_interface();

    motors->initialize(MOTOR0, isReversed0);
    motors->initialize(MOTOR1, isReversed1);
    motors->initialize(MOTOR2, isReversed2);
    motors->initialize(MOTOR3, isReversed3);

    if (encoder_resolution > 0)
    {
        // Encoder direction configured independently of the motor.
        encoders->initialize(MOTOR0, encoder_resolution, isEncoderReversed0);
        encoders->initialize(MOTOR1, encoder_resolution, isEncoderReversed1);
        encoders->initialize(MOTOR2, encoder_resolution, isEncoderReversed2);
        encoders->initialize(MOTOR3, encoder_resolution, isEncoderReversed3);
    }

    mecanum_config.wheel_radius = wheel_diameter / 2.0;
    mecanum_config.length = length;
    mecanum_config.width = width;
    mecanum_config.is_initialized = 1;

    // Register this platform's operations into the global platform dispatcher
    // (see platform.c). Without this, platform.is_initialized stays 0 and the
    // platform.* function pointers stay NULL, so set_platform_velocity() and
    // the other dispatched calls silently no-op. Not hardware-tested yet.
    platform.set_platform_velocity = mecaunm_platform_set_velocity;
    platform.set_platform_target_velocity = mecanum_platform_set_target_velocity;
    platform.start_platform_velocity_controller = mecanum_platform_start_velocity_controller;
    platform.stop_platform_velocity_controller = mecanum_platform_stop_velocity_controller;
    platform.initialize_platform_odometry = mecanum_platform_start_odometry;
    platform.update_platform_odometry = mecanum_platform_update_odometry;
    platform.motor_mask = BMOTOR0 | BMOTOR1 | BMOTOR2 | BMOTOR3;
    platform.properties.mecanum.wheel_diameter = wheel_diameter;
    platform.properties.mecanum.length = length;
    platform.properties.mecanum.width = width;
    platform.is_initialized = 1;
}
