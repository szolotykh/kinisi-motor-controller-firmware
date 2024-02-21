//------------------------------------------------------------
// File name: platform.c
//------------------------------------------------------------
#include <stdlib.h>
#include "platform.h"
#include "hw_motor.h"
#include "hw_encoder.h"
#include <stdint.h>
#include <math.h>
#include <controllers_manager.h>
#include <odometry_manager.h>

#define SPEED_RESOLUTION 840

// Function types definitions
typedef void (*set_platform_velocity_t)(platform_velocity_t);
typedef void (*set_platform_target_velocity_t)(platform_velocity_t);
typedef void (*start_platform_velocity_controller_t)(plaform_controller_settings_t);
typedef void (*stop_platform_velocity_controller_t)();
typedef void (*initialize_platform_odometry_t)();
typedef platform_odometry_t (*update_platform_odometry_t)(uint8_t* motor_indexes, double* velocities, uint8_t motor_count);

// Mecanum platform settings
typedef struct
{
    double wheel_diameter; // Wheel diameter in meters
    double length; // Distace from front to back wheels in meters
    double width; // Distance between left and right wheels in meters
}  mecanum_platform_settings_t;

// Omni platform settings
typedef struct
{
    double wheel_diameter; // Wheel diameter in meters
    double robot_radius; // Distance from the center of the platform to the wheel in meters
}  omni_platform_settings_t;

// Platform structure
typedef struct
{
    // Indicates if platform is initialized
    uint8_t is_initialized;

    // Indicates if platform controller is initialized
    uint8_t is_controller_initialized;

    // Platform functions
    set_platform_velocity_t set_platform_velocity;
    set_platform_target_velocity_t set_platform_target_velocity;
    start_platform_velocity_controller_t start_platform_velocity_controller;
    stop_platform_velocity_controller_t stop_platform_velocity_controller;

    // Platform properties
    union {
        mecanum_platform_settings_t mecanum;
        omni_platform_settings_t omni;
    } properties;

    // Platform odometry
    uint8_t is_odometry_enabled;
    initialize_platform_odometry_t initialize_platform_odometry;
    update_platform_odometry_t update_platform_odometry;

} platform_t;


typedef struct mecanum_velocity_t
{
    double motor0;
    double motor1;
    double motor2;
    double motor3;
} mecanum_velocity_t;

// Internal variables
static platform_t platform = {
    .is_initialized = 0,
    .is_controller_initialized = 0};

// Function definitions
double verify_range(double c);
double sing(double c);

// ------------------------------------------------------------------------
// Platform functions

void set_platform_velocity(platform_velocity_t platform_velocity){
    // Set platform velocity only if platform is initialized
    if (!platform.is_initialized)
    {
        return;
    }
    // Verify that velocity is in range [-100, 100] and adjust if needed
    platform_velocity.x = verify_range(platform_velocity.x);
    platform_velocity.y = verify_range(platform_velocity.y);
    platform_velocity.t = verify_range(platform_velocity.t);

    // Set velocity for initialized platform
    platform.set_platform_velocity(platform_velocity);
}

void platform_set_target_velocity(platform_velocity_t platform_target_velocity)
{
    if (!platform.is_initialized || !platform.is_controller_initialized)
    {
        return;
    }

    platform.set_platform_target_velocity(platform_target_velocity);
}

void platform_start_velocity_controller(plaform_controller_settings_t plaform_controller_settings)
{
    if (!platform.is_initialized)
    {
        return;
    }
    
    platform.start_platform_velocity_controller(plaform_controller_settings);
    platform.is_controller_initialized = 1;
}

void platform_stop_velocity_controller()
{
    if (!platform.is_initialized || !platform.is_controller_initialized)
    {
        return;
    }

    platform.is_controller_initialized = 0;
    platform.stop_platform_velocity_controller();
}

platform_odometry_t platform_update_odometry(uint8_t* motor_indexes, double* velocities, uint8_t motor_count)
{
    if (!platform.is_initialized)
    {
        platform_odometry_t odometry = {
            .x = 0,
            .y = 0,
            .t = 0
        };
        return odometry;
    }

    return platform.update_platform_odometry(motor_indexes, velocities, motor_count);
}

// ------------------------------------------------------------------------
// Mecanum platform functions
void mecaunm_platform_set_velocity(platform_velocity_t platform_velocity){
    // Normalize velocity
    int l = abs(platform_velocity.x) + abs(platform_velocity.y) + abs(platform_velocity.t);
    platform_velocity.x = sing(platform_velocity.x) * platform_velocity.x * platform_velocity.x / l;
    platform_velocity.y = sing(platform_velocity.y) * platform_velocity.y * platform_velocity.y / l;
    platform_velocity.t = sing(platform_velocity.t) * platform_velocity.t * platform_velocity.t / l;

    const mecanum_velocity_t mecanum_velocity = {
        .motor0 = platform_velocity.x + platform_velocity.y + platform_velocity.t,
        .motor1 = platform_velocity.x - platform_velocity.y + platform_velocity.t,
        .motor2 = platform_velocity.x + platform_velocity.y - platform_velocity.t,
        .motor3 = platform_velocity.x - platform_velocity.y - platform_velocity.t
        };

    set_motor_speed(MOTOR0, -mecanum_velocity.motor0);
    set_motor_speed(MOTOR1, -mecanum_velocity.motor1);
    set_motor_speed(MOTOR2, mecanum_velocity.motor2);
    set_motor_speed(MOTOR3, mecanum_velocity.motor3);
}

void mecanum_platform_stop_velocity_controller()
{
    controllers_manager_stop_controller_multiple(BMOTOR0 | BMOTOR1 | BMOTOR2 | BMOTOR3);
}

void mecanum_platform_start_velocity_controller(plaform_controller_settings_t plaform_controller_settings)
{
    controllers_manager_initialize_controller_multiple(
        BMOTOR0 | BMOTOR1 | BMOTOR2 | BMOTOR3,
         plaform_controller_settings.kp,
         plaform_controller_settings.ki,
         plaform_controller_settings.kd,
         plaform_controller_settings.integral_limit);
}

void mecanum_platform_set_target_velocity(platform_velocity_t platform_target_velocity)
{
    double R =  platform.properties.mecanum.wheel_diameter / 2.0;
    double L =  platform.properties.mecanum.length;
    double W =  platform.properties.mecanum.width;

    double V1 = 1.0 / R * (platform_target_velocity.x + platform_target_velocity.y + (L + W) / 2 * platform_target_velocity.t);
    double V2 = 1.0 / R * (platform_target_velocity.x - platform_target_velocity.y + (L + W) / 2 * platform_target_velocity.t);
    double V3 = 1.0 / R * (platform_target_velocity.x + platform_target_velocity.y - (L + W) / 2 * platform_target_velocity.t);
    double V4 = 1.0 / R * (platform_target_velocity.x - platform_target_velocity.y - (L + W) / 2 * platform_target_velocity.t);

    uint8_t motor_indexes[] = {MOTOR0, MOTOR1, MOTOR2, MOTOR3};
    double target_speeds[] = {V1, V2, V3, V4};
    controllers_manager_set_target_speed_multiple(motor_indexes, target_speeds, 4);

}

platform_odometry_t mecanum_platform_update_odometry(uint8_t* motor_indexes, double* velocities, uint8_t motor_count)
{
    platform_odometry_t odometry = {
        .x = 0,
        .y = 0,
        .t = 0
    };

    // Motor count must be 4
    if(motor_count < 4)
    {
        return odometry;
    }

    // Calculate odometry (forwards kinematics)
    // TODO: Verify direction of y axis
    double R =  platform.properties.mecanum.wheel_diameter / 2.0; // Radius of the wheel in meters
    double L =  platform.properties.mecanum.length; // Distance from front to back wheels in meters
    double W =  platform.properties.mecanum.width; // Distance between left and right wheels in meters
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

void initialize_mecanum_platform(uint8_t isReversed0, uint8_t isReversed1, uint8_t isReversed2, uint8_t isReversed3, double length, double width, double wheel_diameter, double encoder_resolution)
{
    if (platform.is_initialized)
    {
        return;
    }

    platform.properties.mecanum.length = length;
    platform.properties.mecanum.width = width;

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

    platform.is_initialized = true;
    platform.set_platform_velocity = mecaunm_platform_set_velocity;
    platform.start_platform_velocity_controller = mecanum_platform_start_velocity_controller;
    platform.stop_platform_velocity_controller = mecanum_platform_stop_velocity_controller;
    platform.set_platform_target_velocity = mecanum_platform_set_target_velocity;
    platform.initialize_platform_odometry = mecanum_platform_start_odometry;
    platform.update_platform_odometry = mecanum_platform_update_odometry;

    platform.properties.mecanum.wheel_diameter = wheel_diameter;
    platform.properties.mecanum.length = length;
    platform.properties.mecanum.width = width;
}

// ------------------------------------------------------------------------
// Omni platform functions

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
        plaform_controller_settings.integral_limit);
}

void omni_platform_stop_velocity_controller()
{
    controllers_manager_stop_controller_multiple(BMOTOR0 | BMOTOR1 | BMOTOR2);
}

void omni_platform_set_target_velocity(platform_velocity_t platform_target_velocity)
{
    double R =  platform.properties.omni.wheel_diameter / 2.0; // Radius of the wheel in meters
    double L =  platform.properties.omni.robot_radius; // Distance from the center of the platform to the wheel in meters
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

    // Motor count must be more than or equal 3
    if(motor_count < 3)
    {
        return odometry;
    }

    // Calculate odometry (forwards kinematics)
    double R =  platform.properties.omni.wheel_diameter / 2.0; // Radius of the wheel in meters
    double L =  platform.properties.omni.robot_radius; // Distance from the center of the platform to the wheel in meters
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
    // Start odometry for each motor encoder
    encoder_start_odometry(MOTOR0);
    encoder_start_odometry(MOTOR1);
    encoder_start_odometry(MOTOR2);
}

void initialize_omni_platform(uint8_t isReversed0, uint8_t isReversed1, uint8_t isReversed2, double wheel_diameter, double robot_radius, double encoder_resolution)
{
    if (platform.is_initialized)
    {
        return;
    }

    initialize_motor(MOTOR0, isReversed0);
	initialize_motor(MOTOR1, isReversed1);
	initialize_motor(MOTOR2, isReversed2);

    if (encoder_resolution > 0)
    {
        initialize_encoder(MOTOR0, encoder_resolution, isReversed0);
        initialize_encoder(MOTOR1, encoder_resolution, isReversed1);
        initialize_encoder(MOTOR2, encoder_resolution, isReversed2);
    }

    platform.is_initialized = true;
    platform.set_platform_velocity = set_omni_platform_velocity;
    platform.start_platform_velocity_controller = omni_platform_start_velocity_controller;
    platform.stop_platform_velocity_controller = omni_platform_stop_velocity_controller;
    platform.set_platform_target_velocity = omni_platform_set_target_velocity;
    platform.initialize_platform_odometry = initialize_omni_platform_odometry;
    platform.update_platform_odometry = omni_platform_update_odometry;

    platform.properties.omni.wheel_diameter = wheel_diameter;
    platform.properties.omni.robot_radius = robot_radius;
}

// ------------------------------------------------------------------------
// Odometry functions

void platform_start_odometry()
{
    // Initialize platform hardware for odometry if it is not initialized
    platform.initialize_platform_odometry();

    platform.is_odometry_enabled = 1;
    
    // Initialize odometry manager if it is not initialized
    odometry_manager_initialize();
}

uint8_t platform_is_odometry_enabled()
{
    return platform.is_odometry_enabled;
}

void platform_reset_odometry()
{
    // Reset odometry in odometry manager
    odometry_manager_reset_platform_odometry();
}

void platform_stop_odometry()
{
    platform.is_odometry_enabled = 0;
}

platform_odometry_t platform_get_odometry()
{
    // Get current platform odometry from odometry manager
    return odometry_manager_get_platform_odometry();
}
// --------------------------------------------------4----------------------
// Utils functions

double verify_range(double c)
{
    if(c > 100) return 100;
    if(c < -100) return -100;
    return c;
}

double sing(double c)
{
    return (c > 0) - (c < 0);
}