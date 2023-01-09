//------------------------------------------------------------
// File name: platform.c
//------------------------------------------------------------
#include <stdlib.h>
#include <controller.h>
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

void SetMotorVelocity(motorIndex motorIndex, int velocity)
{
    unsigned short direction = velocity >= 0;
    unsigned int speed = abs(velocity) * SPEED_RESOLUTION / 100;
    set_motor_speed(motorIndex, direction, speed);
}

void init_platform()
{
    initialize_motor(MOTOR0);
    initialize_motor(MOTOR1);
    initialize_motor(MOTOR2);
    initialize_motor(MOTOR3);
}

void set_velocity_input(const mecanum_velocity_t velocity)
{
    SetMotorVelocity(MOTOR0, -velocity.motor0);
    SetMotorVelocity(MOTOR1, -velocity.motor1);
    SetMotorVelocity(MOTOR2, velocity.motor2);
    SetMotorVelocity(MOTOR3, velocity.motor3);
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