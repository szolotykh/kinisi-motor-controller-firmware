#pragma once

// Shared PWM contract for every motor. The hardware API accepts percentage
// commands and converts their magnitude to compare values from 0 through 840.
#define MOTOR_PWM_FREQUENCY_HZ 100000U
#define MOTOR_MAX_SPEED 840U

// Timer clocks are read from RCC rather than stored here. A selected timer
// must support an exact integer divider to this common counter frequency.
#define MOTOR_PWM_COUNTER_HZ (MOTOR_PWM_FREQUENCY_HZ * MOTOR_MAX_SPEED)

#if MOTOR_PWM_FREQUENCY_HZ == 0 || MOTOR_MAX_SPEED == 0 || MOTOR_MAX_SPEED > 65535U
#error "Invalid motor PWM frequency or compare range"
#endif
