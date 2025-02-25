#pragma once

#include "hw_motor.h"
#include "hw_encoder.h"
#include "encoder_odometry.h"
#include <stdbool.h>

// Mock interface getters
hw_motor_interface_t* mock_hw_motor_get_interface(void);
hw_encoder_interface_t* mock_hw_encoder_get_interface(void);
encoder_odometry_interface_t* mock_encoder_odometry_get_interface(void);

// New mock functions to implement
const hw_motor_interface_t* get_motor_interface(void);
const hw_encoder_interface_t* get_encoder_interface(void);
const encoder_odometry_interface_t* get_encoder_odometry_interface(void);

// Reset functions
void mock_hw_motor_reset(void);
void mock_hw_encoder_reset(void);
void mock_encoder_odometry_reset(void);

// Mock state getters for verification
motorIndex mock_get_last_motor(void);
double mock_get_last_speed(void);
bool mock_get_last_reversed(void);
int mock_get_initialize_motor_calls(void);
int mock_get_set_speed_calls(void);
int mock_get_initialize_encoder_calls(void);
int mock_get_start_odometry_calls(void);
encoder_index_t mock_get_last_encoder(void);
double mock_get_last_encoder_resolution(void);

// Controller mock state getters
uint8_t mock_get_last_motor_selection(void);
double mock_get_last_kp(void);
double mock_get_last_ki(void);
double mock_get_last_kd(void);
double mock_get_last_integral_limit(void);
int mock_get_controller_init_calls(void);
int mock_get_controller_stop_calls(void);
void mock_get_last_target_speeds(uint8_t* motor_indexes, double* target_speeds, int* count);