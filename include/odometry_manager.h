//------------------------------------------------------------
// File name: odometry_manager.h
//------------------------------------------------------------

#pragma once

#include <stdint.h>
#include <platform.h>

/*
Initialize odometry manager
*/
void odometry_manager_initialize();

/*
Start encoder odometry
Parameters:
    encoder_index: Index of the encoder
*/
void encoder_start_odometry(uint8_t encoder_index);

/*
Reset encoder odometry
Parameters:
    encoder_index: Index of the encoder
*/
void encoder_reset_odometry(uint8_t encoder_index);

/*
Get encoder odometry
Parameters:
    encoder_index: Index of the encoder
Returns:
    Odometry of the encoder
*/
double encoder_get_odometry(uint8_t encoder_index);

/*
Stop encoder odometry
Parameters:
    encoder_index: Index of the encoder
*/
void encoder_stop_odometry(uint8_t encoder_index);

/*
Is odometry manager initialized
*/
uint8_t odometry_manager_is_not_initialized();

/*
Initialize odometry manager
*/
void odometry_manager_initialize();

/*
Get platform odometry
*/
platform_odometry_t odometry_manager_get_platform_odometry();

/*
Reset platform odometry
*/
void odometry_manager_reset_platform_odometry();