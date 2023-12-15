//------------------------------------------------------------
// File name: platform.h
//------------------------------------------------------------

#pragma once

#include <pid_controller.h>
#include <hw_motor.h>
#include <hw_encoder.h>

#define ENCODER_UPDATE_RATE 100

typedef struct{
    int8_t x;
    int8_t y;
    int8_t t;
} platform_velocity_t;

void initialize_mecanum_platform(uint8_t isReversed0, uint8_t isReversed1, uint8_t isReversed2, uint8_t isReversed3);
void initialize_omni_platform(uint8_t isReversed0, uint8_t isReversed1, uint8_t isReversed2, uint16_t wheel_diameter, uint16_t robot_radius);

void encoder_update_state(encoder_index_t index, unsigned int value);
void set_platform_velocity(platform_velocity_t platform_velocity);

unsigned int encoder_get_value(encoder_index_t index);
int encoder_get_velocity(encoder_index_t index);
int encoder_get_acceleration(encoder_index_t index);
// void init_motor_rps(motorIndex index); 
// int get_motor_rps(motorIndex index);