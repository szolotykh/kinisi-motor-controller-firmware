#pragma once

#include <stdint.h>

// Start odometry for an encoder
void encoder_start_odometry(uint8_t encoder_index);

// Reset odometry for an encoder
void encoder_reset_odometry(uint8_t encoder_index);

// Get odometry value from encoder
double encoder_get_odometry(uint8_t encoder_index);

// Stop odometry for an encoder
void encoder_stop_odometry(uint8_t encoder_index);