#pragma once
#include <stdint.h>
#include "position_controller.h"

uint8_t platform_position_initialize(position_settings_t linear, position_settings_t angular);
uint8_t platform_position_set(platform_odometry_t target);
uint8_t platform_position_reset(void);
// Cancel and forget tuning; zero wheel targets before releasing the position mutex.
void platform_position_cancel(void);
// Called by the controller task BEFORE acquiring the wheel-controller mutex.
void platform_position_update(void);
