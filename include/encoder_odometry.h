#pragma once

#include <stdint.h>

// Function pointer types for mocking
typedef void (*encoder_start_odometry_fn)(uint8_t encoder_index);
typedef void (*encoder_reset_odometry_fn)(uint8_t encoder_index);
typedef double (*encoder_get_odometry_fn)(uint8_t encoder_index);
typedef void (*encoder_stop_odometry_fn)(uint8_t encoder_index);

// Interface structure
typedef struct {
    encoder_start_odometry_fn start;
    encoder_reset_odometry_fn reset;
    encoder_get_odometry_fn get;
    encoder_stop_odometry_fn stop;
} encoder_odometry_interface_t;

// Get the encoder odometry interface implementation
const encoder_odometry_interface_t* get_encoder_odometry_interface(void);

// Set mock interface for testing
void encoder_odometry_set_interface(const encoder_odometry_interface_t* interface);

// Initialize encoder odometry interface
void encoder_odometry_init(void);

// Default implementations
void encoder_start_odometry(uint8_t encoder_index);
void encoder_reset_odometry(uint8_t encoder_index);
double encoder_get_odometry(uint8_t encoder_index);
void encoder_stop_odometry(uint8_t encoder_index);