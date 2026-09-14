//------------------------------------------------------------
// File name: hw_clock.h
// Description: Declare the board monotonic clock used for sampling and synchronization.
//------------------------------------------------------------
#pragma once
#include <stdint.h>
// Monotonic microseconds derived from the existing 1 MHz TIM14 HAL timebase.
/**
 * @brief Read board monotonic time in microseconds, extending the 32-bit HAL tick.
 * @return Extended milliseconds plus the current TIM14 microsecond fraction.
 * @note Task-context use with the active 1 MHz TIM14/1 ms HAL timebase.
 * Call at least once per HAL tick wrap; IRQs must not span multiple tick overflows.
 */
uint64_t hw_clock_microseconds(void);
