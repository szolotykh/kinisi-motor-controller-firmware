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

// Global interface instance
extern encoder_odometry_interface_t encoder_odometry;

// Initialize the interface with default implementations
void encoder_odometry_init(void);

// Allow setting mock implementations
void encoder_odometry_set_interface(encoder_odometry_interface_t interface);

// Default implementations
// Start odometry for an encoder
void encoder_start_odometry(uint8_t encoder_index);

// Reset odometry for an encoder
void encoder_reset_odometry(uint8_t encoder_index);

// Get odometry value from encoder
double encoder_get_odometry(uint8_t encoder_index);

// Stop odometry for an encoder
void encoder_stop_odometry(uint8_t encoder_index);