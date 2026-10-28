#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool enabled;
    uint16_t threshold_ma;
} hw_motor_current_limit_t;

// Validate against the selected motor's board configuration without changing hardware.
bool hw_motor_current_limit_allowed(uint8_t index, const hw_motor_current_limit_t *setting);
// Read the actual driver setting; never initialize or wake it.
bool hw_motor_get_current_limit(uint8_t index, hw_motor_current_limit_t *setting);
// Apply on an initialized motor and leave its output stopped, including on failure.
bool hw_motor_set_current_limit(uint8_t index, const hw_motor_current_limit_t *setting);
