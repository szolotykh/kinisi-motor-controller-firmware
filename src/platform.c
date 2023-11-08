//------------------------------------------------------------
// File name: platform.c
//------------------------------------------------------------
#include <stdlib.h>
#include <pid_controller.h>
#include "platform.h"
#include "hw_motor.h"
#include "hw_encoder.h"
#include <stdint.h>

#define SPEED_RESOLUTION 840

typedef struct
{
    unsigned int value_n0;
    unsigned int value_n1;
    int velocity_n0;
    int velocity_n1;
    int acceleration_n0;
} encoder_state_t;

encoder_state_t encoder_state[4];

void set_motor_velocity(motorIndex motorIndex, int velocity)
{
    unsigned short direction = velocity >= 0;
    unsigned int speed = abs(velocity) * SPEED_RESOLUTION / 100;
    set_motor_speed(motorIndex, direction, speed);
}

void initialize_mecanum_platform(uint8_t isReversed0, uint8_t isReversed1, uint8_t isReversed2, uint8_t isReversed3)
{
    initialize_motor(MOTOR0, isReversed0);
	initialize_motor(MOTOR1, isReversed1);
	initialize_motor(MOTOR2, isReversed2);
	initialize_motor(MOTOR3, isReversed3);
}

void set_velocity_input(const mecanum_velocity_t velocity)
{
    set_motor_velocity(MOTOR0, -velocity.motor0);
    set_motor_velocity(MOTOR1, -velocity.motor1);
    set_motor_velocity(MOTOR2, velocity.motor2);
    set_motor_velocity(MOTOR3, velocity.motor3);
}

void encoder_update_state(encoder_index_t index, unsigned int value)
{
    encoder_state[index].value_n1 = encoder_state[index].value_n0;
    encoder_state[index].value_n0 = value;
    encoder_state[index].velocity_n1 = encoder_state[index].velocity_n0;
    encoder_state[index].velocity_n0 = encoder_state[index].value_n0 - encoder_state[index].value_n1;
    encoder_state[index].acceleration_n0 = encoder_state[index].velocity_n0 - encoder_state[index].velocity_n1;
}

unsigned int encoder_get_value(encoder_index_t index)
{
    return encoder_state[index].value_n0;
}

int encoder_get_velocity(encoder_index_t index)
{
    return encoder_state[index].velocity_n0;
}

int get_encoder_acceleration(encoder_index_t index)
{
    return encoder_state[index].acceleration_n0;
}

/*
void init_motor_with_encoder(motorIndex mindex, encoder_index eindex)
{
    initialize_motor(index);
    initialize_encoder(index);
    encoder_values[index] = get_encoder_value(index);
}

int get_motor_rps(motorIndex index)
{

}
*/