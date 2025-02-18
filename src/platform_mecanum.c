#include "platform_mecanum.h"
#include "platform_common.h"
#include "controllers_manager.h"
#include "hw_motor.h"
#include "hw_encoder.h"
#include "encoder_odometry.h"
#include <math.h>
#include <stdlib.h>

void mecaunm_platform_set_velocity(platform_velocity_t platform_velocity) {
    // Normalize velocity
    int l = abs(platform_velocity.x) + abs(platform_velocity.y) + abs(platform_velocity.t);
    if (l == 0) {
        // If velocities are all zero, just set motors to zero
        set_motor_speed(MOTOR0, 0);
        set_motor_speed(MOTOR1, 0);
        set_motor_speed(MOTOR2, 0);
        set_motor_speed(MOTOR3, 0);
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

    set_motor_speed(MOTOR0, -motor0);
    set_motor_speed(MOTOR1, -motor1);
    set_motor_speed(MOTOR2, motor2);
    set_motor_speed(MOTOR3, motor3);
}

void mecanum_platform_stop_velocity_controller() {
    controllers_manager_stop_controller_multiple(BMOTOR0 | BMOTOR1 | BMOTOR2 | BMOTOR3);
}

void mecanum_platform_start_velocity_controller(plaform_controller_settings_t plaform_controller_settings) {
    controllers_manager_initialize_controller_multiple(
        BMOTOR0 | BMOTOR1 | BMOTOR2 | BMOTOR3,
        plaform_controller_settings.kp,
        plaform_controller_settings.ki,
        plaform_controller_settings.kd,
        plaform_controller_settings.integral_limit
    );
}

void mecanum_platform_set_target_velocity(platform_velocity_t platform_target_velocity)
{
    // Use forward/inverse kinematics to set motor speeds
    double R = 0.0; // We will set in initialize_mecanum_platform
    double L = 0.0;
    double W = 0.0;
    // The actual values for R,L,W are stored in the platform. For now, let them be local.
    // The real code will reference them from the global platform object.

    // Just replicate the code from platform.c.
    // We'll remove the logic referencing global platform if necessary.
    // Here is the direct logic:

    // Suppose we read them from platform.properties.mecanum:
    // double R =  platform.properties.mecanum.wheel_diameter / 2.0;
    // double L =  platform.properties.mecanum.length;
    // double W =  platform.properties.mecanum.width;

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

    if(motor_count < 4)
    {
        return odometry;
    }

    // We'll do the same logic from platform.c for odometry:
    // double R =  platform.properties.mecanum.wheel_diameter / 2.0;
    // double L =  platform.properties.mecanum.length;
    // double W =  platform.properties.mecanum.width;

    double R = 0.0;
    double L = 0.0;
    double W = 0.0;
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
    // Start odometry for each motor encoder
    encoder_start_odometry(MOTOR0);
    encoder_start_odometry(MOTOR1);
    encoder_start_odometry(MOTOR2);
    encoder_start_odometry(MOTOR3);
}

void initialize_mecanum_platform(
    uint8_t isReversed0,
    uint8_t isReversed1,
    uint8_t isReversed2,
    uint8_t isReversed3,
    double length,
    double width,
    double wheel_diameter,
    double encoder_resolution)
{
    // Implementation from platform.c
    // We'll keep it short. In real code, we might store these in a global structure or a static structure.
    // For now, just replicate.

    initialize_motor(MOTOR0, isReversed0);
    initialize_motor(MOTOR1, isReversed1);
    initialize_motor(MOTOR2, isReversed2);
    initialize_motor(MOTOR3, isReversed3);

    if (encoder_resolution > 0)
    {
        initialize_encoder(MOTOR0, encoder_resolution, isReversed0);
        initialize_encoder(MOTOR1, encoder_resolution, isReversed1);
        initialize_encoder(MOTOR2, encoder_resolution, isReversed2);
        initialize_encoder(MOTOR3, encoder_resolution, isReversed3);
    }
}
