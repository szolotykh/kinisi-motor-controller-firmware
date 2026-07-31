//------------------------------------------------------------
// File name: loop_frequency.h
//------------------------------------------------------------
// Pure helpers to convert between a periodic-task update frequency (Hz) and the
// RTOS tick period (milliseconds). Kept free of RTOS/hardware dependencies so
// they can be unit-tested on the host. Shared by the controller and odometry
// task managers.

#pragma once

#include <stdint.h>

// Convert a loop frequency in Hz to the task period in milliseconds.
// Returns 0 for an invalid (zero) frequency. Otherwise the result is quantized
// to whole milliseconds (period_ms = 1000 / frequency_hz) and clamped to a
// minimum of 1 ms, i.e. a maximum effective rate of 1000 Hz.
uint32_t loop_frequency_hz_to_period_ms(uint16_t frequency_hz);

// Convert a task period in milliseconds to a loop frequency in Hz.
// Returns 0 if period_ms is 0.
uint16_t loop_period_ms_to_frequency_hz(uint32_t period_ms);
