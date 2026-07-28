//------------------------------------------------------------
// File name: odometry_integrator.h
//------------------------------------------------------------
//
// Pure, dependency-free odometry integration math. This is intentionally
// separate from odometry_manager.c (which pulls in FreeRTOS/CMSIS) so the
// encoder->pose pipeline can be unit tested natively.
//------------------------------------------------------------

#pragma once

#include <stdint.h>
#include "platform_types.h"

// Convert a raw 16-bit incremental-encoder counter change into wheel rotation
// in radians. The hardware counter is unsigned 16-bit and wraps, so a raw
// change above half range is interpreted as a small backward (negative)
// movement.
// Parameters:
//     previous_value: encoder counter at the previous tick
//     current_value:  encoder counter at the current tick
//     resolution:     encoder resolution in ticks per revolution
// Returns:
//     wheel rotation for this tick, in radians (0 if resolution is 0)
double odometry_integrator_wheel_delta(uint16_t previous_value,
                                       uint16_t current_value,
                                       double resolution);

// Accumulate a body-frame odometry increment into a world-frame pose.
// body_delta.{x,y} are the forward/left translation in the ROBOT frame for
// this tick and body_delta.t is the heading change. The translation is rotated
// by the heading at the MIDPOINT of the tick before being added, which is
// 2nd-order accurate for a simultaneous translate + rotate (an arc). Without
// this rotation, driving while turning corrupts x/y (e.g. a small circle
// unrolls into metres of phantom translation).
// Parameters:
//     pose:       current world-frame pose
//     body_delta: body-frame odometry increment for this tick
// Returns:
//     the updated world-frame pose
platform_odometry_t odometry_integrator_accumulate(platform_odometry_t pose,
                                                    platform_odometry_t body_delta);
