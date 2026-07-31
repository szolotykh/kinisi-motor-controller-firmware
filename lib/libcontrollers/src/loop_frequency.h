//------------------------------------------------------------
// File name: loop_frequency.h
//------------------------------------------------------------
// Pure helpers to convert between a periodic-task update frequency (Hz) and the
// RTOS tick period (milliseconds). Kept free of RTOS/hardware dependencies so
// they can be unit-tested on the host. Shared by the controller and odometry
// task managers.

#pragma once

#include <stdint.h>

// Allowed range for a user-supplied loop frequency, in Hz.
// The maximum is bounded by the FreeRTOS tick (configTICK_RATE_HZ = 1000, i.e.
// a 1 ms tick): the periodic tasks use vTaskDelayUntil and cannot be scheduled
// faster than one tick. The minimum of 1 Hz maps to the longest representable
// period (1000 ms). Values outside this range are clamped (see
// loop_frequency_clamp_hz); 0 remains reserved as "invalid".
#define LOOP_FREQUENCY_MIN_HZ 1u
#define LOOP_FREQUENCY_MAX_HZ 1000u

// Clamp a requested loop frequency to the supported range.
// Returns 0 unchanged (reserved "invalid" sentinel so callers can ignore it);
// any other value is clamped to [LOOP_FREQUENCY_MIN_HZ, LOOP_FREQUENCY_MAX_HZ].
uint16_t loop_frequency_clamp_hz(uint16_t frequency_hz);

// Convert a loop frequency in Hz to the task period in milliseconds.
// Returns 0 for an invalid (zero) frequency. Otherwise the result is quantized
// to whole milliseconds (period_ms = 1000 / frequency_hz) and clamped to a
// minimum of 1 ms, i.e. a maximum effective rate of 1000 Hz.
uint32_t loop_frequency_hz_to_period_ms(uint16_t frequency_hz);

// Convert a task period in milliseconds to a loop frequency in Hz.
// Returns 0 if period_ms is 0.
uint16_t loop_period_ms_to_frequency_hz(uint32_t period_ms);
