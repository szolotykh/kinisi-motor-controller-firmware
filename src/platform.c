//------------------------------------------------------------
// File name: platform.c
//------------------------------------------------------------
#include <stdlib.h>
#include "platform.h"
#include "hw_motor.h"
#include "hw_encoder.h"
#include <stdint.h>
#include <math.h>

#define SPEED_RESOLUTION 840

// Type definitions
typedef void (*set_platform_velocity_t)(platform_velocity_t);

typedef struct
{
    uint8_t is_initialized;
    set_platform_velocity_t set_platform_velocity;
    
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
    int motor0;
    int motor1;
    int motor2;
    int motor3;
} mecanum_velocity_t;

// Internal variables
static platform_t platform = {.is_initialized = 0};

static encoder_state_t encoder_state[4];

// Function definitions
signed char verify_range(signed char c);
int sing(int c);

void set_platform_velocity(platform_velocity_t platform_velocity){
    // Set platform velocity only if platform is initialized
    if (platform.is_initialized)
    {
        return;
    }
    // Verify that velocity is in range [-100, 100] and adjust if needed
    platform_velocity.x = verify_range(platform_velocity.x);
    platform_velocity.y = verify_range(platform_velocity.y);
    platform_velocity.t = verify_range(platform_velocity.t);

    if(platform.is_initialized){
        platform.set_platform_velocity(platform_velocity);
    }
}

void set_motor_velocity(motorIndex motorIndex, int velocity)
{
    uint8_t direction = velocity >= 0;
    uint16_t speed = abs(velocity) * SPEED_RESOLUTION / 100;
    set_motor_speed(motorIndex, direction, speed);
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

    set_motor_velocity(MOTOR0, -mecanum_velocity.motor0);
    set_motor_velocity(MOTOR1, -mecanum_velocity.motor1);
    set_motor_velocity(MOTOR2, mecanum_velocity.motor2);
    set_motor_velocity(MOTOR3, mecanum_velocity.motor3);
}

void initialize_mecanum_platform(uint8_t isReversed0, uint8_t isReversed1, uint8_t isReversed2, uint8_t isReversed3)
{
    if (platform.is_initialized)
    {
        return;
    }

    initialize_motor(MOTOR0, isReversed0);
	initialize_motor(MOTOR1, isReversed1);
	initialize_motor(MOTOR2, isReversed2);
	initialize_motor(MOTOR3, isReversed3);
    platform.is_initialized = true;
    platform.set_platform_velocity = set_mecaunm_platform_velocity;
}

// ------------------------------------------------------------------------
// Omni platform functions

void set_omni_platform_velocity(platform_velocity_t platform_velocity)
{
    double V1 = sqrt(3.0)/2 * platform_velocity.x - 1/2 * platform_velocity.y + platform_velocity.t;
    double V2 = -sqrt(3.0)/2 * platform_velocity.x - 1/2 * platform_velocity.y + platform_velocity.t;
    double V3 = -platform_velocity.y + platform_velocity.t;

    double k = 100 / fmax(fabs(V1), fmax(fabs(V2), fabs(V3)));

    V1 *= k;
    V2 *= k;
    V3 *= k;

    set_motor_velocity(MOTOR0, V1);
    set_motor_velocity(MOTOR1, V2);
    set_motor_velocity(MOTOR2, V3);
}

void initialize_omni_platform(uint8_t isReversed0, uint8_t isReversed1, uint8_t isReversed2, uint16_t wheel_diameter, uint16_t robot_radius)
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
}






// ------------------------------------------------------------------------

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

signed char verify_range(signed char c)
{
    if(c > 100) return 100;
    if(c < -100) return -100;
    return c;
}

int sing(int c)
{
    return (c > 0) - (c < 0);
}