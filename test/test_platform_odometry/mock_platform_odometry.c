//------------------------------------------------------------
// File name: mock_platform_odometry.c
//------------------------------------------------------------
//
// Minimal stubs to satisfy the symbols pulled in by the real platform_*.c
// files that this test env compiles (platform_omni.c, platform_mecanum.c,
// platform_differential.c) and by odometry_integrator.c. The pipeline test
// only needs the omni platform initialized; the motor/encoder/controller
// calls made during initialization are recorded nowhere -- they just need to
// be non-NULL so the platform code does not dereference a null pointer.
//------------------------------------------------------------

#include "mock_platform_odometry.h"
#include "hw_motor.h"
#include "hw_encoder.h"
#include "encoder_odometry.h"
#include "controllers_manager.h"
#include "platform_common.h"

// Global platform instance required by platform_*.c: each initialize_*_platform
// writes its function pointers here. Defined once for the whole test binary.
platform_t platform = {0};

//------------------------------------------------------------
// Motor interface stubs
//------------------------------------------------------------
static void stub_motor_initialize(motorIndex m, bool r) { (void)m; (void)r; }
static uint8_t stub_motor_is_reversed(motorIndex m) { (void)m; return 0; }
static uint8_t stub_motor_is_initialized(motorIndex m) { (void)m; return 1; }
static void stub_motor_set_speed(motorIndex m, double p) { (void)m; (void)p; }
static void stub_motor_stop(motorIndex m) { (void)m; }
static void stub_motor_brake(motorIndex m) { (void)m; }

static const hw_motor_interface_t mock_motor_interface = {
    .initialize = stub_motor_initialize,
    .is_reversed = stub_motor_is_reversed,
    .is_initialized = stub_motor_is_initialized,
    .set_speed = stub_motor_set_speed,
    .stop = stub_motor_stop,
    .brake = stub_motor_brake
};

//------------------------------------------------------------
// Encoder interface stubs
//------------------------------------------------------------
static void stub_encoder_initialize(encoder_index_t i, double res, uint8_t r) { (void)i; (void)res; (void)r; }
static uint16_t stub_encoder_get_value(encoder_index_t i) { (void)i; return 0; }
static uint8_t stub_encoder_get_direction(encoder_index_t i) { (void)i; return 0; }
static uint8_t stub_encoder_is_initialized(encoder_index_t i) { (void)i; return 1; }
static double stub_encoder_get_resolution(encoder_index_t i) { (void)i; return 0; }

static const hw_encoder_interface_t mock_encoder_interface = {
    .initialize = stub_encoder_initialize,
    .get_value = stub_encoder_get_value,
    .get_direction = stub_encoder_get_direction,
    .is_initialized = stub_encoder_is_initialized,
    .get_resolution = stub_encoder_get_resolution
};

//------------------------------------------------------------
// Encoder-odometry interface stub
//------------------------------------------------------------
static void stub_odom_start(uint8_t i) { (void)i; }
static void stub_odom_reset(uint8_t i) { (void)i; }
static double stub_odom_get(uint8_t i) { (void)i; return 0; }
static void stub_odom_stop(uint8_t i) { (void)i; }

static const encoder_odometry_interface_t mock_odometry_interface = {
    .start = stub_odom_start,
    .reset = stub_odom_reset,
    .get = stub_odom_get,
    .stop = stub_odom_stop
};

//------------------------------------------------------------
// Interface getters referenced by the platform_*.c files
//------------------------------------------------------------
const hw_motor_interface_t* get_motor_interface(void) { return &mock_motor_interface; }
const hw_encoder_interface_t* get_encoder_interface(void) { return &mock_encoder_interface; }
const encoder_odometry_interface_t* get_encoder_odometry_interface(void) { return &mock_odometry_interface; }

// Not used by the pipeline test but installed for completeness.
void mock_odometry_install_interfaces(void)
{
    hw_motor_set_interface(&mock_motor_interface);
    hw_encoder_set_interface(&mock_encoder_interface);
    encoder_odometry_set_interface(&mock_odometry_interface);
}

//------------------------------------------------------------
// Controllers manager stubs (referenced by platform_*.c velocity controllers,
// not exercised here but required for linking).
//------------------------------------------------------------
void controllers_manager_initialize_controller_multiple(uint8_t s, double kp, double ki, double kd, double il)
{ (void)s; (void)kp; (void)ki; (void)kd; (void)il; }
void controllers_manager_stop_controller_multiple(uint8_t s) { (void)s; }
void controllers_manager_set_target_speed_multiple(uint8_t* mi, double* ts, uint8_t c) { (void)mi; (void)ts; (void)c; }
