// Functions to be mocked:
// - hw_motor_init
// - hw_motor_set_interface
// - hw_encoder_init
// - hw_encoder_set_interface
// - encoder_odometry_init
// - encoder_odometry_set_interface
// - initialize_controller_multiple
// - stop_controller_multiple
// - set_target_speed_multiple
// - controllers_manager_initialize_controller_multiple
// - controllers_manager_stop_controller_multiple
// - controllers_manager_set_target_speed_multiple

#include "mock_platform_mecanum.h"
#include "hw_motor.h"
#include "hw_encoder.h"
#include "encoder_odometry.h"
#include "controllers_manager.h"
#include <string.h>

// Mock tracking variables
static motorIndex last_motor = 0;
static double last_speed = 0;
static bool last_reversed = false;
static int initialize_motor_calls = 0;
static int set_speed_calls = 0;
static int initialize_encoder_calls = 0;
static int start_odometry_calls = 0;
static encoder_index_t last_encoder = 0;
static double last_encoder_resolution = 0;

// Controller mock tracking
static uint8_t last_motor_selection = 0;
static double last_kp = 0;
static double last_ki = 0;
static double last_kd = 0;
static double last_integral_limit = 0;
static int controller_init_calls = 0;
static int controller_stop_calls = 0;
static uint8_t last_target_motor_indexes[4];
static double last_target_speeds[4];
static int last_motor_count = 0;

// Current interface pointers - remove default interface references
static const hw_motor_interface_t* current_motor_interface;
static const hw_encoder_interface_t* current_encoder_interface;
static const encoder_odometry_interface_t* current_odometry_interface;

// Mock functions for hw_motor
static void mock_hw_motor_init(motorIndex motorIndex, bool isReversed) {
    initialize_motor_calls++;
    last_motor = motorIndex;
    last_reversed = isReversed;
}

static void mock_set_motor_speed(motorIndex motorIndex, double speed) {
    set_speed_calls++;
    last_motor = motorIndex;
    last_speed = speed;
}

static uint8_t mock_motor_is_initialized(motorIndex motorIndex) {
    return initialize_motor_calls > 0;
}

static hw_motor_interface_t mock_motor_interface = {
    .initialize = mock_hw_motor_init,
    .set_speed = mock_set_motor_speed,
    .is_initialized = mock_motor_is_initialized
};

// Mock functions for hw_encoder
static void mock_hw_encoder_init(encoder_index_t index, double resolution, uint8_t is_reversed) {
    initialize_encoder_calls++;
    last_encoder = index;
    last_encoder_resolution = resolution;
}

static hw_encoder_interface_t mock_encoder_interface = {
    .initialize = mock_hw_encoder_init
};

// Mock functions for encoder_odometry
static void mock_encoder_odometry_start(uint8_t encoder_index) {
    start_odometry_calls++;
    last_encoder = encoder_index;
}

static encoder_odometry_interface_t mock_odometry_interface = {
    .start = mock_encoder_odometry_start
};

// Mock getter functions
int mock_get_initialize_motor_calls(void) {
    return initialize_motor_calls;
}

int mock_get_set_speed_calls(void) {
    return set_speed_calls;
}

double mock_get_last_speed(void) {
    return last_speed;
}

int mock_get_initialize_encoder_calls(void) {
    return initialize_encoder_calls;
}

int mock_get_start_odometry_calls(void) {
    return start_odometry_calls;
}

int mock_get_controller_init_calls(void) {
    return controller_init_calls;
}

int mock_get_controller_stop_calls(void) {
    return controller_stop_calls;
}

uint8_t mock_get_last_motor_selection(void) {
    return last_motor_selection;
}

double mock_get_last_kp(void) {
    return last_kp;
}

double mock_get_last_ki(void) {
    return last_ki;
}

double mock_get_last_kd(void) {
    return last_kd;
}

double mock_get_last_integral_limit(void) {
    return last_integral_limit;
}

uint8_t mock_get_last_reversed(void) {
    return last_reversed;
}

double mock_get_last_encoder_resolution(void) {
    return last_encoder_resolution;
}

uint8_t mock_get_last_encoder(void) {
    return last_encoder;
}

void mock_get_last_target_speeds(uint8_t* motor_indexes, double* target_speeds, int* count) {
    memcpy(motor_indexes, last_target_motor_indexes, sizeof(uint8_t) * 4);
    memcpy(target_speeds, last_target_speeds, sizeof(double) * 4);
    *count = last_motor_count;
}

// Interface getter functions
const hw_motor_interface_t* mock_hw_motor_get_interface(void) {
    return &mock_motor_interface;
}

const hw_encoder_interface_t* mock_hw_encoder_get_interface(void) {
    return &mock_encoder_interface;
}

const encoder_odometry_interface_t* mock_encoder_odometry_get_interface(void) {
    return &mock_odometry_interface;
}

// Reset functions
void mock_hw_motor_reset(void) {
    initialize_motor_calls = 0;
    set_speed_calls = 0;
    last_motor = 0;
    last_speed = 0;
    last_reversed = false;
}

void mock_hw_encoder_reset(void) {
    initialize_encoder_calls = 0;
    last_encoder = 0;
    last_encoder_resolution = 0;
}

void mock_encoder_odometry_reset(void) {
    start_odometry_calls = 0;
    last_encoder = 0;
}

// Platform mock implementations
void mock_mecanum_platform_start_velocity_controller(plaform_controller_settings_t settings) {
    controller_init_calls++;
    last_motor_selection = BMOTOR0 | BMOTOR1 | BMOTOR2 | BMOTOR3;
    last_kp = settings.kp;
    last_ki = settings.ki;
    last_kd = settings.kd;
    last_integral_limit = settings.integral_limit;
}

void mock_mecanum_platform_stop_velocity_controller(void) {
    controller_stop_calls++;
    last_motor_selection = BMOTOR0 | BMOTOR1 | BMOTOR2 | BMOTOR3;
}

void mock_mecanum_platform_set_target_velocity(platform_velocity_t target) {
    last_motor_count = 4;
    double R = 0.05;  // wheel_radius from test values
    double L = 0.4;   // length from test values
    double W = 0.3;   // width from test values
    
    double V1 = 1.0 / R * (target.x + target.y + (L + W) / 2.0 * target.t);
    double V2 = 1.0 / R * (target.x - target.y + (L + W) / 2.0 * target.t);
    double V3 = 1.0 / R * (target.x + target.y - (L + W) / 2.0 * target.t);
    double V4 = 1.0 / R * (target.x - target.y - (L + W) / 2.0 * target.t);

    last_target_motor_indexes[0] = MOTOR0;
    last_target_motor_indexes[1] = MOTOR1;
    last_target_motor_indexes[2] = MOTOR2;
    last_target_motor_indexes[3] = MOTOR3;
    
    last_target_speeds[0] = V1;
    last_target_speeds[1] = V2;
    last_target_speeds[2] = V3;
    last_target_speeds[3] = V4;
}

platform_odometry_t mock_mecanum_platform_update_odometry(uint8_t* motor_indexes, double* velocities, uint8_t motor_count) {
    platform_odometry_t result = {0};
    
    if (motor_count < 4) {
        return result;
    }

    // Match the actual implementation from platform_mecanum.c
    double R = 0.05;  // wheel_radius from test values
    double L = 0.4;   // length from test values
    double W = 0.3;   // width from test values

    double v1 = velocities[0];
    double v2 = velocities[1];
    double v3 = velocities[2];
    double v4 = velocities[3];

    result.x = R/4.0 * (v1 + v2 + v3 + v4);
    result.y = R/4.0 * (v1 - v2 + v3 - v4);
    result.t = R/(2.0 * (L + W)) * (v1 + v2 - v3 - v4);

    return result;
}

// Mock platform functions implementation
void mock_initialize_mecanum_platform(
    uint8_t isReversed0,
    uint8_t isReversed1,
    uint8_t isReversed2,
    uint8_t isReversed3,
    double length,
    double width,
    double wheel_diameter,
    double encoder_resolution) 
{
    // Reset counters before initialization
    mock_hw_motor_reset();
    mock_hw_encoder_reset();

    mock_hw_motor_init(MOTOR0, isReversed0);
    mock_hw_motor_init(MOTOR1, isReversed1);
    mock_hw_motor_init(MOTOR2, isReversed2);
    mock_hw_motor_init(MOTOR3, isReversed3);

    mock_hw_encoder_init(MOTOR0, encoder_resolution, isReversed0);
    mock_hw_encoder_init(MOTOR1, encoder_resolution, isReversed1);
    mock_hw_encoder_init(MOTOR2, encoder_resolution, isReversed2);
    mock_hw_encoder_init(MOTOR3, encoder_resolution, isReversed3);
}

void mock_set_mecanum_platform_velocity(platform_velocity_t velocity) {
    // Convert linear/angular velocity to wheel velocities
    double R = 0.05;  // wheel_radius
    double L = 0.4;   // length
    double W = 0.3;   // width
    
    double V1 = (velocity.x + velocity.y + (L + W) * velocity.t) / R;
    double V2 = (velocity.x - velocity.y + (L + W) * velocity.t) / R;
    double V3 = (velocity.x + velocity.y - (L + W) * velocity.t) / R;
    double V4 = (velocity.x - velocity.y - (L + W) * velocity.t) / R;

    last_motor_count = 4;
    last_target_motor_indexes[0] = MOTOR0;
    last_target_motor_indexes[1] = MOTOR1;
    last_target_motor_indexes[2] = MOTOR2;
    last_target_motor_indexes[3] = MOTOR3;
    
    last_target_speeds[0] = V1;
    last_target_speeds[1] = V2;
    last_target_speeds[2] = V3;
    last_target_speeds[3] = V4;
}

void mock_initialize_mecanum_platform_odometry(void) {
    mock_encoder_odometry_start(MOTOR0);
    mock_encoder_odometry_start(MOTOR1);
    mock_encoder_odometry_start(MOTOR2);
    mock_encoder_odometry_start(MOTOR3);
}

void hw_motor_set_interface(const hw_motor_interface_t* interface) {
    current_motor_interface = interface;
}

void hw_encoder_set_interface(const hw_encoder_interface_t* interface) {
    current_encoder_interface = interface;
}

void encoder_odometry_set_interface(const encoder_odometry_interface_t* interface) {
    current_odometry_interface = interface;
}

// Modify interface init functions to use mock interfaces directly
void hw_motor_init(void) {
    current_motor_interface = mock_hw_motor_get_interface();
}

void hw_encoder_init(void) {
    current_encoder_interface = mock_hw_encoder_get_interface();
}

void encoder_odometry_init(void) {
    current_odometry_interface = mock_encoder_odometry_get_interface();
}

const hw_motor_interface_t* get_motor_interface(void) {
    return current_motor_interface;
}

const hw_encoder_interface_t* get_encoder_interface(void) {
    return current_encoder_interface;
}

const encoder_odometry_interface_t* get_encoder_odometry_interface(void) {
    return current_odometry_interface;
}

// Controllers manager mock implementations
void controllers_manager_initialize_controller_multiple(uint8_t motor_selection, double kp, double ki, double kd, double integral_limit) {
    controller_init_calls++;
    last_motor_selection = motor_selection;
    last_kp = kp;
    last_ki = ki;
    last_kd = kd;
    last_integral_limit = integral_limit;
}

void controllers_manager_stop_controller_multiple(uint8_t motor_selection) {
    controller_stop_calls++;
    last_motor_selection = motor_selection;
}

void controllers_manager_set_target_speed_multiple(uint8_t* motor_indexes, double* target_speeds, uint8_t motor_count) {
    for(int i = 0; i < motor_count && i < 4; i++) {
        last_target_motor_indexes[i] = motor_indexes[i];
        last_target_speeds[i] = target_speeds[i];
    }
    last_motor_count = motor_count;
}