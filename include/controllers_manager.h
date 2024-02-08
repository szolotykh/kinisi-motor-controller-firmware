//------------------------------------------------------------
// File name: controllers_manager.h
//------------------------------------------------------------
#pragma once

#include "hw_motor.h"
#include "hw_encoder.h"
#include <pid_controller.h>
#include <cmsis_os.h>
#include "semphr.h"
#include "commands.h"

// Define motor identifiers as bit masks
#define BMOTOR0 0x01 // 0000 0001
#define BMOTOR1 0x02 // 0000 0010
#define BMOTOR2 0x04 // 0000 0100
#define BMOTOR3 0x08 // 0000 1000

typedef struct controller_info_t
{
    enum {
        STOP,
        RUN
    } state;
    motorIndex mIndex;
    encoder_index_t eIndex;
    pid_controller_t controller;
} controller_info_t;

// Motor controller manager state
typedef struct controllers_manager_state
{
    double TargetMotorSpeed[4]; // In radians per second
    controller_info_t ControllerInfo[4];
    uint32_t update_interval_ms;
    SemaphoreHandle_t controller_state_mutex;
} controllers_manager_state_t;

// Motor controller manager
typedef struct controllers_manager
{
    osThreadId_t threadHandler;
    controllers_manager_state_t state;
} controllers_manager_t;

/*
Initialize controller manager
Parameters:
    controllers_manager: Pointer to the controller manager
*/
void controllers_manager_init(controllers_manager_t* controllers_manager);

/*
Check if controller manager is not initialized
Parameters:
    controllers_manager: Pointer to the controller manager
Returns:
    1 if controller manager is not initialized, 0 otherwise
*/
uint8_t controllers_manager_is_not_init(const controllers_manager_t* controllers_manager);

/*
Initialize controller for single motor
Parameters:
    controllers_manager: Pointer to the controller manager
    motor_index: Motor index
    encoder_index: Encoder index
    kp: Proportional gain
    ki: Integral gain
    kd: Derivative gain
    is_reversed: If true, the motor is reversed
    encoder_resolution: Encoder resolution in pulses per revolution
    integral_limit: Integral limit must be positive. If negative or zero, integral limit is disabled
*/
void controllers_manager_initialize_controller(controllers_manager_t* controllers_manager, uint8_t motor_index, uint8_t encoder_index, double kp, double ki, double kd, bool is_reversed, double encoder_resolution, double integral_limit);

/* Initialize controller for multiple motors
Parameters:
    controllers_manager: Pointer to the controller manager
    motor_selection: Bit mask of selected motors
    kp: Proportional gain
    ki: Integral gain
    kd: Derivative gain
    encoder_resolution: Encoder resolution in pulses per revolution
    integral_limit: Integral limit must be positive. If negative or zero, integral limit is disabled
*/
void controllers_manager_initialize_controller_multiple(controllers_manager_t* controllers_manager, uint8_t motor_selection, double kp, double ki, double kd, double encoder_resolution, double integral_limit);

/*
Delete controller for single motor
Parameters:
    controllers_manager: Pointer to the controller manager
    motor_index: Motor index
*/
void controllers_manager_delete_controller(controllers_manager_t* controllers_manager, uint8_t motor_index);

/*
Set target speed for motor in radians per second
Parameters:
    controllers_manager: Pointer to the controller manager
    motor_index: Motor index
    target_speed: Target speed in radians per second
*/
void controllers_manager_set_target_speed(controllers_manager_t* controllers_manager, uint8_t motor_index, double target_speed);


/*
Set target speed for multiple motors in radians per second
Parameters:
    controllers_manager: Pointer to the controller manager
    motor_indexes: Pointer to the array of motor indexes
    target_speeds: Pointer to the array of target speeds
    motor_count: Number of motors
*/
void controllers_manager_set_target_speed_multiple(controllers_manager_t* controllers_manager, uint8_t* motor_indexes, double* target_speeds, uint8_t motor_count);

/*
Get motor controller state
Parameters:
    controllers_manager: Pointer to the controller manager
    motor_index: Motor index
Returns:
    Motor controller state
*/
motor_controller_state controllers_manager_get_motor_controller_state(const controllers_manager_t* controllers_manager, uint8_t motor_index);


