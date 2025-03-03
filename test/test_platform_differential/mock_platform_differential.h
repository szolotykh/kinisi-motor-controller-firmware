#ifndef MOCK_PLATFORM_DIFFERENTIAL_H
#define MOCK_PLATFORM_DIFFERENTIAL_H

#include "platform_differential.h"
#include "hw_motor.h"
#include "hw_encoder.h"
#include "encoder_odometry.h"
#include <stdbool.h>

// Mock functions for platform_differential
void mock_initialize_differential_platform(
    uint8_t isReversed0,
    uint8_t isReversed1,
    double wheel_diameter,
    double wheel_base,
    double encoder_resolution);

void mock_set_differential_platform_velocity(platform_velocity_t velocity);
void mock_initialize_differential_platform_odometry(void);
void mock_differential_platform_start_velocity_controller(plaform_controller_settings_t settings);
void mock_differential_platform_stop_velocity_controller(void);
void mock_differential_platform_set_target_velocity(platform_velocity_t target);
platform_odometry_t mock_differential_platform_update_odometry(uint8_t* motor_indexes, double* velocities, uint8_t motor_count);

// Mock tracking functions
int mock_get_initialize_motor_calls(void);
int mock_get_set_speed_calls(void);
motorIndex mock_get_last_motor(void);
double mock_get_last_speed(void);
int mock_get_initialize_encoder_calls(void);
int mock_get_start_odometry_calls(void);
int mock_get_controller_init_calls(void);
int mock_get_controller_stop_calls(void);
uint8_t mock_get_last_motor_selection(void);
double mock_get_last_kp(void);
double mock_get_last_ki(void);
double mock_get_last_kd(void);
double mock_get_last_integral_limit(void);
bool mock_get_last_reversed(void);
double mock_get_last_encoder_resolution(void);
encoder_index_t mock_get_last_encoder(void);
void mock_get_last_target_speeds(uint8_t* motor_indexes, double* target_speeds, int* count);

// Interface getter functions
const hw_motor_interface_t* mock_hw_motor_get_interface(void);
const hw_encoder_interface_t* mock_hw_encoder_get_interface(void);
const encoder_odometry_interface_t* mock_encoder_odometry_get_interface(void);

// Reset functions
void mock_hw_motor_reset(void);
void mock_hw_encoder_reset(void);
void mock_encoder_odometry_reset(void);

#endif // MOCK_PLATFORM_DIFFERENTIAL_H