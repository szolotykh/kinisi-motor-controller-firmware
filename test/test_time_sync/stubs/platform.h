//------------------------------------------------------------
// File name: platform.h
// Description: Declare minimal host-test substitutes for platform.h dependencies.
//------------------------------------------------------------
#pragma once
#include <stdint.h>
#include "platform_types.h"
/**
 * @brief Return the fixture platform integration flag.
 */
uint8_t platform_is_odometry_enabled(void);
/**
 * @brief Produce a known body-frame increment under the odometry lock.
 */
platform_odometry_t platform_update_odometry(uint8_t *indexes, double *deltas, uint8_t count);
