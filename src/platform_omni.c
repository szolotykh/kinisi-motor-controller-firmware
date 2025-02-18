#include "platform_omni.h"
#include "platform_common.h"
#include "controllers_manager.h"
#include "hw_motor.h"
#include "hw_encoder.h"
#include "encoder_odometry.h"
#include <math.h>
#include <stdlib.h>

void set_omni_platform_velocity(platform_velocity_t platform_velocity)
{
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

    set_motor_speed(MOTOR0, V1);
    set_motor_speed(MOTOR1, V2);
    set_motor_speed(MOTOR2, V3);
}

void omni_platform_start_velocity_controller(plaform_controller_settings_t plaform_controller_settings)
{
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
    controllers_manager_stop_controller_multiple(BMOTOR0 | BMOTOR1 | BMOTOR2);
}

void omni_platform_set_target_velocity(platform_velocity_t platform_target_velocity)
{
    double R = 0.0; // for actual code, read from a global or a struct
    double L = 0.0; // referencing platform.properties.omni.

    double V1 = 1.0 / R * (sqrt(3.0)/2.0 * platform_target_velocity.x - 0.5 * platform_target_velocity.y + L * platform_target_velocity.t);
    double V2 = 1.0 / R * (-sqrt(3.0)/2.0 * platform_target_velocity.x - 0.5 * platform_target_velocity.y + L * platform_target_velocity.t);
    double V3 = 1.0 / R * (platform_target_velocity.y + L * platform_target_velocity.t);
    
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

    if(motor_count < 3)
    {
        return odometry;
    }

    double R = 0.0;  // read these from global or store in struct
    double L = 0.0;

    double v1 = velocities[0];
    double v2 = velocities[1];
    double v3 = velocities[2];

    odometry.x = R * (1/sqrt(3.0) * v1 - 1/sqrt(3.0) * v2);
    odometry.y = R/3.0 * (-v1 - v2 + 2.0 * v3);
    odometry.t = R/(3.0 * L) * (v1 + v2 + v3);

    return odometry;
}

void initialize_omni_platform_odometry()
{
    encoder_start_odometry(MOTOR0);
    encoder_start_odometry(MOTOR1);
    encoder_start_odometry(MOTOR2);
}

void initialize_omni_platform(
    uint8_t isReversed0,
    uint8_t isReversed1,
    uint8_t isReversed2,
    double wheel_diameter,
    double robot_radius,
    double encoder_resolution)
{
    initialize_motor(MOTOR0, isReversed0);
    initialize_motor(MOTOR1, isReversed1);
    initialize_motor(MOTOR2, isReversed2);

    if (encoder_resolution > 0)
    {
        initialize_encoder(MOTOR0, encoder_resolution, isReversed0);
        initialize_encoder(MOTOR1, encoder_resolution, isReversed1);
        initialize_encoder(MOTOR2, encoder_resolution, isReversed2);
    }
}
