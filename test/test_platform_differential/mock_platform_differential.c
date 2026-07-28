#include "mock_platform_differential.h"
#include "hw_motor.h"
#include "hw_encoder.h"
#include "encoder_odometry.h"
#include "controllers_manager.h"
#include "platform_common.h"
#include <string.h>

// Global platform instance required by the linked platform_*.c files.
platform_t platform = {0};

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
static uint8_t last_target_motor_indexes[2];
static double last_target_speeds[2];
static int last_motor_count = 0;

// Platform configuration
static struct {
    double wheel_radius;
    double wheel_base;
    uint8_t is_initialized;
} differential_config = {0};

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

static uint8_t mock_motor_is_reversed(motorIndex motorIndex) {
    return last_reversed;
}

static hw_motor_interface_t mock_motor_interface = {
    .initialize = mock_hw_motor_init,
    .set_speed = mock_set_motor_speed,
    .is_initialized = mock_motor_is_initialized,
    .is_reversed = mock_motor_is_reversed
};

// Mock functions for hw_encoder
static void mock_initialize_encoder(encoder_index_t index, double resolution, uint8_t is_reversed) {
    initialize_encoder_calls++;
    last_encoder = index;
    last_encoder_resolution = resolution;
}

static hw_encoder_interface_t mock_encoder_interface = {
    .initialize = mock_initialize_encoder
};

// Mock functions for encoder_odometry
static void mock_start_odometry(uint8_t encoder_index) {
    start_odometry_calls++;
    last_encoder = encoder_index;
}

static encoder_odometry_interface_t mock_odometry_interface = {
    .start = mock_start_odometry
};

// Mock interface getters
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
    memcpy(motor_indexes, last_target_motor_indexes, sizeof(uint8_t) * 2);
    memcpy(target_speeds, last_target_speeds, sizeof(double) * 2);
    *count = last_motor_count;
}

// Mock platform functions implementation
void mock_initialize_differential_platform(
    uint8_t isReversed0,
    uint8_t isReversed1,
    double wheel_diameter,
    double wheel_base,
    double encoder_resolution) 
{
    // Reset counters before initialization
    mock_hw_motor_reset();
    mock_hw_encoder_reset();

    mock_hw_motor_init(MOTOR0, isReversed0);
    mock_hw_motor_init(MOTOR1, isReversed1);

    if (encoder_resolution > 0) {
        mock_initialize_encoder(MOTOR0, encoder_resolution, isReversed0);
        mock_initialize_encoder(MOTOR1, encoder_resolution, isReversed1);
    }

    differential_config.wheel_radius = wheel_diameter / 2.0;
    differential_config.wheel_base = wheel_base;
    differential_config.is_initialized = 1;
}

void mock_set_differential_platform_velocity(platform_velocity_t velocity) {
    if (!differential_config.is_initialized) {
        return;
    }

    double V0 = velocity.x - velocity.t;
    double V1 = velocity.x + velocity.t;

    mock_set_motor_speed(MOTOR0, V0);
    mock_set_motor_speed(MOTOR1, V1);
}

void mock_initialize_differential_platform_odometry(void) {
    mock_start_odometry(MOTOR0);
    mock_start_odometry(MOTOR1);
}

void mock_differential_platform_start_velocity_controller(plaform_controller_settings_t settings) {
    controller_init_calls++;
    last_motor_selection = BMOTOR0 | BMOTOR1;
    last_kp = settings.kp;
    last_ki = settings.ki;
    last_kd = settings.kd;
    last_integral_limit = settings.integral_limit;
}

void mock_differential_platform_stop_velocity_controller(void) {
    controller_stop_calls++;
    last_motor_selection = BMOTOR0 | BMOTOR1;
}

void mock_differential_platform_set_target_velocity(platform_velocity_t target) {
    last_motor_count = 2;
    double V0 = target.x - target.t * differential_config.wheel_base / 2.0;
    double V1 = target.x + target.t * differential_config.wheel_base / 2.0;

    last_target_motor_indexes[0] = MOTOR0;
    last_target_motor_indexes[1] = MOTOR1;
    last_target_speeds[0] = V0;
    last_target_speeds[1] = V1;
}

platform_odometry_t mock_differential_platform_update_odometry(uint8_t* motor_indexes, double* velocities, uint8_t motor_count) {
    platform_odometry_t odometry = {
        .x = 0,
        .y = 0,
        .t = 0
    };

    if (!differential_config.is_initialized || motor_count < 2) {
        return odometry;
    }

    double R = differential_config.wheel_radius;
    double L = differential_config.wheel_base;
    double v0 = velocities[0];
    double v1 = velocities[1];

    odometry.x = R / 2.0 * (v0 + v1);
    odometry.t = R / L * (v1 - v0);

    return odometry;
}

// External function definitions that need to be provided to the linker
// These functions are normally provided by the real implementation
// but for testing, we need to provide mock versions

// Hardware interface accessors for the real platform code
const hw_motor_interface_t* get_motor_interface(void) {
    return &mock_motor_interface;
}

const hw_encoder_interface_t* get_encoder_interface(void) {
    return &mock_encoder_interface;
}

const encoder_odometry_interface_t* get_encoder_odometry_interface(void) {
    return &mock_odometry_interface;
}

// Hardware interface initializers that tests might call
void hw_motor_init(void) {
    // No-op for tests
}

void hw_encoder_init(void) {
    // No-op for tests
}

void encoder_odometry_init(void) {
    // No-op for tests
}

// Interface swappers (normally defined elsewhere but needed for tests)
void hw_motor_set_interface(const hw_motor_interface_t* interface) {
    // No-op for tests, we're using our mock interface
}

void hw_encoder_set_interface(const hw_encoder_interface_t* interface) {
    // No-op for tests, we're using our mock interface
}

void encoder_odometry_set_interface(const encoder_odometry_interface_t* interface) {
    // No-op for tests, we're using our mock interface
}

// Controllers manager implementation needed by platform
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
    if (motor_count > 2) motor_count = 2;
    
    last_motor_count = motor_count;
    for (int i = 0; i < motor_count; i++) {
        last_target_motor_indexes[i] = motor_indexes[i];
        last_target_speeds[i] = target_speeds[i];
    }
}