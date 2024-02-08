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

#define SPEED_RESOLUTION 840

// Function types definitions
typedef void (*set_platform_velocity_t)(platform_velocity_t);
typedef void (*set_platform_target_velocity_t)(controllers_manager_t*, platform_velocity_t);
typedef void (*initialize_platform_controller_t)(controllers_manager_t*, plaform_controller_settings_t);

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
    initialize_platform_controller_t initialize_platform_controller;

    // Platform properties
    union {
        mecanum_platform_settings_t mecanum;
        omni_platform_settings_t omni;
    } properties;
} platform_t;

typedef struct
{
    unsigned int value_n0;
    unsigned int value_n1;
    int velocity_n0;
    int velocity_n1;
    int acceleration_n0;
} encoder_state_t;

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

static encoder_state_t encoder_state[4];

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

void platform_set_target_velocity(controllers_manager_t* controllers_manager, platform_velocity_t platform_target_velocity)
{
    if (!platform.is_initialized || !platform.is_controller_initialized)
    {
        return;
    }

    // TODO: Is controller initialized?

    platform.set_platform_target_velocity(controllers_manager, platform_target_velocity);
}

void platform_initialize_controller(controllers_manager_t* controllers_manager, plaform_controller_settings_t plaform_controller_settings)
{
    if (!platform.is_initialized)
    {
        return;
    }
    
    platform.initialize_platform_controller(controllers_manager, plaform_controller_settings);
    platform.is_controller_initialized = 1;
}

// ------------------------------------------------------------------------
// Mecanum platform functions
void set_mecaunm_platform_velocity(platform_velocity_t platform_velocity){
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

void mecanum_platform_initialize_controller(controllers_manager_t* controllers_manager, plaform_controller_settings_t plaform_controller_settings)
{
    controllers_manager_initialize_controller_multiple(
        controllers_manager,
        BMOTOR0 | BMOTOR1 | BMOTOR2 | BMOTOR3,
         plaform_controller_settings.kp,
         plaform_controller_settings.ki,
         plaform_controller_settings.kd,
         plaform_controller_settings.encoder_resolution,
         plaform_controller_settings.integral_limit);
}

void mecanum_platform_set_target_velocity(controllers_manager_t* controllers_manager, platform_velocity_t platform_target_velocity)
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
    controllers_manager_set_target_speed_multiple(controllers_manager, motor_indexes, target_speeds, 4);

}

void initialize_mecanum_platform(uint8_t isReversed0, uint8_t isReversed1, uint8_t isReversed2, uint8_t isReversed3, double length, double width, double wheel_diameter)
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
    platform.is_initialized = true;
    platform.set_platform_velocity = set_mecaunm_platform_velocity;
    platform.initialize_platform_controller = mecanum_platform_initialize_controller;
    platform.set_platform_target_velocity = mecanum_platform_set_target_velocity;

    platform.properties.mecanum.wheel_diameter = wheel_diameter;
    platform.properties.mecanum.length = length;
    platform.properties.mecanum.width = width;
}

// ------------------------------------------------------------------------
// Omni platform functions

void set_omni_platform_velocity(platform_velocity_t platform_velocity)
{
    double V1 = sqrt(3.0)/2 * platform_velocity.x - 1/2 * platform_velocity.y + platform_velocity.t;
    double V2 = -sqrt(3.0)/2 * platform_velocity.x - 1/2 * platform_velocity.y + platform_velocity.t;
    double V3 = -platform_velocity.y + platform_velocity.t;

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

void omni_platform_initialize_controller(controllers_manager_t* controllers_manager, plaform_controller_settings_t plaform_controller_settings)
{
    controllers_manager_initialize_controller_multiple(
        controllers_manager,
        BMOTOR0 | BMOTOR1 | BMOTOR2,
        plaform_controller_settings.kp,
        plaform_controller_settings.ki,
        plaform_controller_settings.kd,
        plaform_controller_settings.encoder_resolution,
        plaform_controller_settings.integral_limit);
}

void omni_platform_set_target_velocity(controllers_manager_t* controllers_manager, platform_velocity_t platform_target_velocity)
{
    double R =  platform.properties.omni.wheel_diameter / 2.0; // Radius of the wheel in meters
    double L =  platform.properties.omni.robot_radius; // Distance from the center of the platform to the wheel in meters
    double V1 = 1.0 / R * (sqrt(3.0)/2.0 * platform_target_velocity.x - 1.0/2.0 * platform_target_velocity.y + L * platform_target_velocity.t);
    double V2 = 1.0 / R * (-sqrt(3.0)/2.0 * platform_target_velocity.x - 1.0/2.0 * platform_target_velocity.y + L * platform_target_velocity.t);
    double V3 = 1.0 / R * (-platform_target_velocity.y + L * platform_target_velocity.t);
    
    uint8_t motor_indexes[] = {MOTOR0, MOTOR1, MOTOR2};
    double target_speeds[] = {V1, V2, V3};
    controllers_manager_set_target_speed_multiple(controllers_manager, motor_indexes, target_speeds, 3);
}

void initialize_omni_platform(uint8_t isReversed0, uint8_t isReversed1, uint8_t isReversed2, double wheel_diameter, double robot_radius)
{
    if (platform.is_initialized)
    {
        return;
    }

    initialize_motor(MOTOR0, isReversed0);
	initialize_motor(MOTOR1, isReversed1);
	initialize_motor(MOTOR2, isReversed2);
    platform.is_initialized = true;
    platform.set_platform_velocity = set_omni_platform_velocity;
    platform.initialize_platform_controller = omni_platform_initialize_controller;
    platform.set_platform_target_velocity = omni_platform_set_target_velocity;

    platform.properties.omni.wheel_diameter = wheel_diameter;
    platform.properties.omni.robot_radius = robot_radius;
}

// ------------------------------------------------------------------------
// Encoder functions

void encoder_update_state(encoder_index_t index, unsigned int value)
{
    encoder_state[index].value_n1 = encoder_state[index].value_n0;
    encoder_state[index].value_n0 = value;
    encoder_state[index].velocity_n1 = encoder_state[index].velocity_n0;
    encoder_state[index].velocity_n0 = encoder_state[index].value_n0 - encoder_state[index].value_n1;
    encoder_state[index].acceleration_n0 = encoder_state[index].velocity_n0 - encoder_state[index].velocity_n1;
}

unsigned int encoder_get_value(encoder_index_t index)
{
    return encoder_state[index].value_n0;
}

int encoder_get_velocity(encoder_index_t index)
{
    return encoder_state[index].velocity_n0;
}

int get_encoder_acceleration(encoder_index_t index)
{
    return encoder_state[index].acceleration_n0;
}

/*
void init_motor_with_encoder(motorIndex mindex, encoder_index eindex)
{
    initialize_motor(index);
    initialize_encoder(index);
    encoder_values[index] = get_encoder_value(index);
}

int get_motor_rps(motorIndex index)
{

}
*/

// ------------------------------------------------------------------------
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