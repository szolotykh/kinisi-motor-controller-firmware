//------------------------------------------------------------
// File name: platform.h
//------------------------------------------------------------

#pragma once

#include <controller.h>
#include <hw_motor.h>
#include <hw_encoder.h>

#define ENCODER_UPDATE_RATE 100

void init_platform();

void encoder_update_state(encoder_index_t index, unsigned int value);
void set_velocity_input(const mecanum_velocity_t velocity);

unsigned int encoder_get_value(encoder_index_t index);
int encoder_get_velocity(encoder_index_t index);
int encoder_get_acceleration(encoder_index_t index);
// void init_motor_rps(motorIndex index); 
// int get_motor_rps(motorIndex index);