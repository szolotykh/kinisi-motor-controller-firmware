//------------------------------------------------------------
// File name: odometry_manager.h
// Description: Expose odometry lifecycle, update frequency, and atomic timestamped snapshots.
//------------------------------------------------------------

#pragma once

#include <stdint.h>
#include <platform.h>

/*
Initialize odometry manager
*/
void odometry_manager_initialize();

/*
Start encoder odometry
Parameters:
    encoder_index: Index of the encoder
*/
void encoder_start_odometry(uint8_t encoder_index);

/*
Reset encoder odometry
Parameters:
    encoder_index: Index of the encoder
*/
void encoder_reset_odometry(uint8_t encoder_index);

/*
Get encoder odometry
Parameters:
    encoder_index: Index of the encoder
Returns:
    Odometry of the encoder
*/
double encoder_get_odometry(uint8_t encoder_index);
// Atomic snapshot getters return RESPONSE_OK or a protocol error code.
/**
 * @brief Copy an encoder angle and its acquisition time under the odometry mutex.
 * @param index Encoder index; must be within the configured encoder count.
 * @param angle Receives the cached angle in radians after acquiring the mutex.
 * @param sample_us Receives the cached monotonic acquisition time in microseconds.
 * @return RESPONSE_OK, INVALID_ARGUMENT, ODOMETRY_NOT_INITIALIZED, SAMPLE_NOT_AVAILABLE, or INTERNAL_ERROR.
 * @note Output values are a valid measurement only when RESPONSE_OK is returned.
 */
uint8_t encoder_get_odometry_sample(uint8_t index, double *angle, uint64_t *sample_us);
/**
 * @brief Copy the cached platform pose and acquisition time under the odometry mutex.
 * @param pose Receives position in meters and heading in radians.
 * @param sample_us Receives monotonic acquisition time in microseconds.
 * @return RESPONSE_OK, ODOMETRY_NOT_INITIALIZED, SAMPLE_NOT_AVAILABLE, or INTERNAL_ERROR.
 * @note Output values are a valid measurement only when RESPONSE_OK is returned.
 */
uint8_t odometry_manager_get_platform_sample(platform_odometry_t *pose, uint64_t *sample_us);

/*
Stop encoder odometry
Parameters:
    encoder_index: Index of the encoder
*/
void encoder_stop_odometry(uint8_t encoder_index);

/*
Is odometry manager initialized
*/
uint8_t odometry_manager_is_not_initialized();

/*
Initialize odometry manager
*/
void odometry_manager_initialize();

/*
Get platform odometry
*/
platform_odometry_t odometry_manager_get_platform_odometry();

/*
Reset platform odometry
*/
void odometry_manager_reset_platform_odometry();
// Invalidate a cached measurement on restart without resetting the accumulated pose.
/**
 * @brief Clear the platform sample timestamp while preserving accumulated pose.
 * @note Restart uses this to require a fresh sample; no-op before manager creation.
 */
void odometry_manager_invalidate_platform_sample(void);

/*
Set the global odometry task update frequency (Hz). A single task integrates all
encoder and platform odometry, so this is global. The value is quantized to the
1 ms RTOS tick (period_ms = 1000 / frequency_hz). Ignored if frequency_hz is 0.
Parameters:
    frequency_hz: Odometry update frequency in Hz
*/
void odometry_manager_set_frequency(uint16_t frequency_hz);

/*
Get the current global odometry task update frequency (Hz).
Returns:
    Odometry update frequency in Hz
*/
uint16_t odometry_manager_get_frequency();
