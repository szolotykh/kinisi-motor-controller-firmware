#ifndef MOCK_PLATFORM_MECANUM_H
#define MOCK_PLATFORM_MECANUM_H

#include "platform_mecanum.h"
#include "hw_motor.h"
#include "hw_encoder.h"
#include "encoder_odometry.h"

// Mock functions for platform_mecanum
void mock_initialize_mecanum_platform(
    uint8_t isReversed0,
    uint8_t isReversed1,
    uint8_t isReversed2,
    uint8_t isReversed3,
    double length,
    double width,
    double wheel_diameter,
    double encoder_resolution);
void mock_set_mecanum_platform_velocity(platform_velocity_t velocity);
void mock_initialize_mecanum_platform_odometry(void);
void mock_mecanum_platform_start_velocity_controller(plaform_controller_settings_t settings);
void mock_mecanum_platform_stop_velocity_controller(void);
void mock_mecanum_platform_set_target_velocity(platform_velocity_t target);
platform_odometry_t mock_mecanum_platform_update_odometry(uint8_t* motor_indexes, double* velocities, uint8_t motor_count);

// Mock tracking functions
int mock_get_initialize_motor_calls(void);
int mock_get_set_speed_calls(void);
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
uint8_t mock_get_last_reversed(void);
double mock_get_last_encoder_resolution(void);
uint8_t mock_get_last_encoder(void);
void mock_get_last_target_speeds(uint8_t* motor_indexes, double* target_speeds, int* count);

// Interface getter functions
const hw_motor_interface_t* mock_hw_motor_get_interface(void);
const hw_encoder_interface_t* mock_hw_encoder_get_interface(void);
const encoder_odometry_interface_t* mock_encoder_odometry_get_interface(void);

// Reset functions
void mock_hw_motor_reset(void);
void mock_hw_encoder_reset(void);
void mock_encoder_odometry_reset(void);

// Controllers manager mock functions
void controllers_manager_initialize_controller_multiple(uint8_t motor_selection, double kp, double ki, double kd, double integral_limit);
void controllers_manager_stop_controller_multiple(uint8_t motor_selection);
void controllers_manager_set_target_speed_multiple(uint8_t* motor_indexes, double* target_speeds, uint8_t motor_count);

#endif // MOCK_PLATFORM_MECANUM_H
