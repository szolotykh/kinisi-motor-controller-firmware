#include "mock_platform_omni.h"
#include "controllers_manager.h"
#include <string.h>
#include <stdio.h>

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
static uint8_t last_target_motor_indexes[3];
static double last_target_speeds[3];
static int last_motor_count = 0;

// Default interface definitions with empty functions
static const hw_motor_interface_t default_motor_interface = {
    .initialize = NULL,
    .set_speed = NULL,
    .is_initialized = NULL,
    .is_reversed = NULL
};

static const hw_encoder_interface_t default_encoder_interface = {
    .initialize = NULL,
    .get_value = NULL,
    .get_direction = NULL,
    .is_initialized = NULL,
    .get_resolution = NULL
};

static const encoder_odometry_interface_t default_odometry_interface = {
    .start = NULL,
    .reset = NULL,
    .get = NULL,
    .stop = NULL
};

// Current interface pointers
static const hw_motor_interface_t* current_motor_interface = &default_motor_interface;
static const hw_encoder_interface_t* current_encoder_interface = &default_encoder_interface;
static const encoder_odometry_interface_t* current_odometry_interface = &default_odometry_interface;

// Mock functions implementations
static void mock_initialize_motor(motorIndex motorIndex, bool isReversed) {
    initialize_motor_calls++;
    last_motor = motorIndex;
    last_reversed = isReversed;
}

static void mock_set_motor_speed(motorIndex motorIndex, double speed) {
    printf("Before update: last_speed = %f\n", last_speed);
    set_speed_calls++;
    last_motor = motorIndex;
    last_speed = speed;
    printf("After update: last_speed = %f\n", last_speed);
    printf("mock_set_motor_speed called: motorIndex = %d, speed = %f\n", motorIndex, speed);
}

static uint8_t mock_motor_is_initialized(motorIndex motorIndex) {
    return initialize_motor_calls > 0;
}

static uint8_t mock_motor_is_reversed(motorIndex motorIndex) {
    return last_reversed;
}

static void mock_initialize_encoder(encoder_index_t index, double resolution, uint8_t is_reversed) {
    initialize_encoder_calls++;
    last_encoder = index;
    last_encoder_resolution = resolution;
}

static void mock_start_odometry(uint8_t encoder_index) {
    start_odometry_calls++;
    last_encoder = encoder_index;
}

// Interface implementations
static hw_motor_interface_t mock_motor_interface = {
    .initialize = mock_initialize_motor,
    .set_speed = mock_set_motor_speed,
    .is_initialized = mock_motor_is_initialized,
    .is_reversed = mock_motor_is_reversed
};

static hw_encoder_interface_t mock_encoder_interface = {
    .initialize = mock_initialize_encoder
};

static encoder_odometry_interface_t mock_encoder_odometry_interface = {
    .start = mock_start_odometry
};

// Mock interface getters
hw_motor_interface_t* mock_hw_motor_get_interface(void) {
    return &mock_motor_interface;
}

hw_encoder_interface_t* mock_hw_encoder_get_interface(void) {
    return &mock_encoder_interface;
}

encoder_odometry_interface_t* mock_encoder_odometry_get_interface(void) {
    return &mock_encoder_odometry_interface;
}

// New mock functions to implement
const hw_motor_interface_t* get_motor_interface(void) {
    return &mock_motor_interface;
}

const hw_encoder_interface_t* get_encoder_interface(void) {
    return &mock_encoder_interface;
}

const encoder_odometry_interface_t* get_encoder_odometry_interface(void) {
    return &mock_encoder_odometry_interface;
}

// Reset functions
void mock_hw_motor_reset(void) {
    last_motor = 0;
    last_speed = 0;
    last_reversed = false;
    initialize_motor_calls = 0;
    set_speed_calls = 0;
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

// Mock state getters
motorIndex mock_get_last_motor(void) { return last_motor; }
double mock_get_last_speed(void) { return last_speed; }
bool mock_get_last_reversed(void) { return last_reversed; }
int mock_get_initialize_motor_calls(void) { return initialize_motor_calls; }
int mock_get_set_speed_calls(void) { return set_speed_calls; }
int mock_get_initialize_encoder_calls(void) { return initialize_encoder_calls; }
int mock_get_start_odometry_calls(void) { return start_odometry_calls; }
encoder_index_t mock_get_last_encoder(void) { return last_encoder; }
double mock_get_last_encoder_resolution(void) { return last_encoder_resolution; }

// Controller mock state getters
uint8_t mock_get_last_motor_selection(void) { return last_motor_selection; }
double mock_get_last_kp(void) { return last_kp; }
double mock_get_last_ki(void) { return last_ki; }
double mock_get_last_kd(void) { return last_kd; }
double mock_get_last_integral_limit(void) { return last_integral_limit; }
int mock_get_controller_init_calls(void) { return controller_init_calls; }
int mock_get_controller_stop_calls(void) { return controller_stop_calls; }
void mock_get_last_target_speeds(uint8_t* motor_indexes, double* target_speeds, int* count) {
    memcpy(motor_indexes, last_target_motor_indexes, sizeof(uint8_t) * 3);
    memcpy(target_speeds, last_target_speeds, sizeof(double) * 3);
    *count = last_motor_count;
}

// Hardware interface stubs
void hw_motor_set_interface(const hw_motor_interface_t* interface) {
    current_motor_interface = interface;
}

void hw_encoder_set_interface(const hw_encoder_interface_t* interface) {
    current_encoder_interface = interface;
}

void encoder_odometry_set_interface(const encoder_odometry_interface_t* interface) {
    current_odometry_interface = interface;
}

void hw_motor_init(void) {
    current_motor_interface = &default_motor_interface;
}

void hw_encoder_init(void) {
    current_encoder_interface = &default_encoder_interface;
}

void encoder_odometry_init(void) {
    current_odometry_interface = &default_odometry_interface;
}

// Controllers manager implementations
void initialize_controller_multiple(uint8_t motor_selection, double kp, double ki, double kd, double integral_limit) {
    controller_init_calls++;
    last_motor_selection = motor_selection;
    last_kp = kp;
    last_ki = ki;
    last_kd = kd;
    last_integral_limit = integral_limit;
}

void stop_controller_multiple(uint8_t motor_selection) {
    controller_stop_calls++;
    last_motor_selection = motor_selection;
}

void set_target_speed_multiple(uint8_t* motor_indexes, double* target_speeds, uint8_t motor_count) {
    for(int i = 0; i < motor_count && i < 3; i++) {
        last_target_motor_indexes[i] = motor_indexes[i];
        last_target_speeds[i] = target_speeds[i];
    }
    last_motor_count = motor_count;
}

void controllers_manager_initialize_controller_multiple(uint8_t motor_selection, double kp, double ki, double kd, double integral_limit) {
    initialize_controller_multiple(motor_selection, kp, ki, kd, integral_limit);
}

void controllers_manager_stop_controller_multiple(uint8_t motor_selection) {
    stop_controller_multiple(motor_selection);
}

void controllers_manager_set_target_speed_multiple(uint8_t* motor_indexes, double* target_speeds, uint8_t motor_count) {
    set_target_speed_multiple(motor_indexes, target_speeds, motor_count);
}