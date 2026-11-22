#pragma once
#include <stdbool.h>
#include "commands.h"

// Used for the live board inhibit and for previously queued requests when a
// different transport wakes the board. Stop/coast, sensors and setup reads
// remain available. A sleeping bridge cannot provide an actual IC4 readback.
static inline bool command_requires_awake_drivers(uint8_t command)
{
    switch (command) {
    case INITIALIZE_MOTOR: case SET_MOTOR_SPEED: case BRAKE_MOTOR:
    case INITIALIZE_MOTOR_CONTROLLER: case SET_MOTOR_TARGET_SPEED:
    case INITIALIZE_MOTOR_POSITION_CONTROLLER: case INITIALIZE_MOTOR_POSITION_PID_CONTROLLER:
    case SET_MOTOR_POSITION:
    case INITIALIZE_DIFFERENTIAL_PLATFORM: case INITIALIZE_OMNI_PLATFORM: case INITIALIZE_MECANUM_PLATFORM:
    case SET_PLATFORM_VELOCITY: case START_PLATFORM_CONTROLLER: case BRAKE_PLATFORM:
    case SET_PLATFORM_TARGET_VELOCITY: case INITIALIZE_PLATFORM_POSITION_CONTROLLER:
    case INITIALIZE_PLATFORM_POSITION_PID_CONTROLLER: case SET_PLATFORM_POSITION:
    case GET_MOTOR_CURRENT_LIMIT: case SET_MOTOR_CURRENT_LIMIT: case CLEAR_MOTOR_FAULTS:
        return true;
    default:
        return false;
    }
}
