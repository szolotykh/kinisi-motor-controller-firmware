//------------------------------------------------------------
// File name: encoder_odometry.c
//------------------------------------------------------------

#include "encoder_odometry.h"
#include "odometry_manager.h"

// Function declarations

static const encoder_odometry_interface_t default_odometry_interface = {
    .start = encoder_start_odometry,
    .reset = encoder_reset_odometry,
    .get = encoder_get_odometry,
    .stop = encoder_stop_odometry
};

static const encoder_odometry_interface_t* current_odometry_interface = &default_odometry_interface;

const encoder_odometry_interface_t* get_encoder_odometry_interface(void) {
    return current_odometry_interface;
}

void encoder_odometry_set_interface(const encoder_odometry_interface_t* interface) {
    current_odometry_interface = interface;
}

void encoder_odometry_init(void) {
    current_odometry_interface = &default_odometry_interface;
}