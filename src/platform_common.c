#include "platform_common.h"
#include "hw_motor.h"
#include "hw_encoder.h"
#include "encoder_odometry.h"

// Static interface references
static const hw_motor_interface_t* current_motor_interface = NULL;
static const hw_encoder_interface_t* current_encoder_interface = NULL;
static const encoder_odometry_interface_t* current_odometry_interface = NULL;

// Function to get the motor interface
const hw_motor_interface_t* get_motor_interface(void) {
    return current_motor_interface;
}

// Function to get the encoder interface
const hw_encoder_interface_t* get_encoder_interface(void) {
    return current_encoder_interface;
}

// Function to get the encoder odometry interface
const encoder_odometry_interface_t* get_encoder_odometry_interface(void) {
    return current_odometry_interface;
}

// Function to set the motor interface (used for testing or initialization)
void set_motor_interface_for_testing(const hw_motor_interface_t* interface) {
    current_motor_interface = interface;
}

// Function to set the encoder interface (used for testing or initialization)
void set_encoder_interface_for_testing(const hw_encoder_interface_t* interface) {
    current_encoder_interface = interface;
}

// Function to set the encoder odometry interface (used for testing or initialization)
void set_encoder_odometry_interface_for_testing(const encoder_odometry_interface_t* interface) {
    current_odometry_interface = interface;
}
