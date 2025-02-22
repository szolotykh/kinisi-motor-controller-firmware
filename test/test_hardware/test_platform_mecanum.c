#include "unity.h"
#include "platform_mecanum.h"
#include "hw_motor.h"
#include "hw_encoder.h"
#include "encoder_odometry.h"
#include "controllers_manager.h"
#include <math.h>

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

// Mock functions for hw_motor
void mock_initialize_motor(motorIndex motorIndex, bool isReversed) {
    initialize_motor_calls++;
    last_motor = motorIndex;
    last_reversed = isReversed;
}

void mock_set_motor_speed(motorIndex motorIndex, double speed) {
    set_speed_calls++;
    last_motor = motorIndex;
    last_speed = speed;
}

uint8_t mock_motor_is_initialized(motorIndex motorIndex) {
    return initialize_motor_calls > 0;
}

uint8_t mock_motor_is_reversed(motorIndex motorIndex) {
    return last_reversed;
}

// Mock functions for hw_encoder
void mock_initialize_encoder(encoder_index_t index, double resolution, uint8_t is_reversed) {
    initialize_encoder_calls++;
    last_encoder = index;
    last_encoder_resolution = resolution;
}

// Mock functions for encoder_odometry
void mock_start_odometry(uint8_t encoder_index) {
    start_odometry_calls++;
    last_encoder = encoder_index;
}

// Mock functions for controllers_manager
void mock_initialize_controller_multiple(uint8_t motor_selection, double kp, double ki, double kd, double integral_limit) {
    controller_init_calls++;
    last_motor_selection = motor_selection;
    last_kp = kp;
    last_ki = ki;
    last_kd = kd;
    last_integral_limit = integral_limit;
}

void mock_stop_controller_multiple(uint8_t motor_selection) {
    controller_stop_calls++;
    last_motor_selection = motor_selection;
}

void mock_set_target_speed_multiple(uint8_t* motor_indexes, double* target_speeds, uint8_t motor_count) {
    for(int i = 0; i < motor_count && i < 4; i++) {
        last_target_motor_indexes[i] = motor_indexes[i];
        last_target_speeds[i] = target_speeds[i];
    }
    last_motor_count = motor_count;
}

void setUp(void) {
    // Reset all tracking variables
    last_motor = 0;
    last_speed = 0;
    last_reversed = false;
    initialize_motor_calls = 0;
    set_speed_calls = 0;
    initialize_encoder_calls = 0;
    start_odometry_calls = 0;
    last_encoder = 0;
    last_encoder_resolution = 0;
    last_motor_selection = 0;
    last_kp = 0;
    last_ki = 0;
    last_kd = 0;
    last_integral_limit = 0;
    controller_init_calls = 0;
    controller_stop_calls = 0;
    last_motor_count = 0;
    for(int i = 0; i < 4; i++) {
        last_target_motor_indexes[i] = 0;
        last_target_speeds[i] = 0;
    }

    // Set up mock interfaces
    hw_motor_interface_t mock_motor = {
        .initialize = mock_initialize_motor,
        .set_speed = mock_set_motor_speed,
        .is_initialized = mock_motor_is_initialized,
        .is_reversed = mock_motor_is_reversed
    };
    hw_motor_set_interface(mock_motor);

    hw_encoder_interface_t mock_encoder = {
        .initialize = mock_initialize_encoder
    };
    hw_encoder_set_interface(mock_encoder);

    encoder_odometry_interface_t mock_odometry = {
        .start = mock_start_odometry
    };
    encoder_odometry_set_interface(mock_odometry);

    // Initialize platform with test values
    initialize_mecanum_platform(1, 0, 1, 0, 0.4, 0.3, 0.1, 1000.0);
}

void tearDown(void) {
    // Reset interfaces to default implementations
    hw_motor_init();
    hw_encoder_init();
    encoder_odometry_init();
}

void test_initialize_mecanum_platform(void) {
    TEST_ASSERT_EQUAL(4, initialize_motor_calls);
    TEST_ASSERT_EQUAL(4, initialize_encoder_calls);
    
    // Verify motor initialization reversals
    TEST_ASSERT_EQUAL(1, controller_motors.is_reversed(MOTOR0));
    TEST_ASSERT_EQUAL(0, controller_motors.is_reversed(MOTOR1));
    TEST_ASSERT_EQUAL(1, controller_motors.is_reversed(MOTOR2));
    TEST_ASSERT_EQUAL(0, controller_motors.is_reversed(MOTOR3));

    // Verify encoder initialization
    TEST_ASSERT_EQUAL(1000.0, last_encoder_resolution);
}

void test_mecanum_platform_set_velocity(void) {
    platform_velocity_t velocity = {
        .x = 100.0,
        .y = 0.0,
        .t = 0.0
    };

    mecaunm_platform_set_velocity(velocity);

    TEST_ASSERT_EQUAL(4, set_speed_calls);
    
    // For x = 100, y = 0, t = 0, the normalized velocities should be all equal magnitude
    TEST_ASSERT_FLOAT_WITHIN(0.1, 100.0, fabs(last_speed));
}

void test_mecanum_platform_set_velocity_zero(void) {
    platform_velocity_t velocity = {
        .x = 0.0,
        .y = 0.0,
        .t = 0.0
    };

    mecaunm_platform_set_velocity(velocity);

    TEST_ASSERT_EQUAL(4, set_speed_calls);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, last_speed);
}

void test_mecanum_platform_start_odometry(void) {
    mecanum_platform_start_odometry();

    TEST_ASSERT_EQUAL(4, start_odometry_calls);
    TEST_ASSERT_EQUAL(MOTOR3, last_encoder); // Last encoder initialized
}

void test_mecanum_platform_start_velocity_controller(void) {
    plaform_controller_settings_t settings = {
        .kp = 1.0,
        .ki = 0.1,
        .kd = 0.01,
        .integral_limit = 100.0
    };

    mecanum_platform_start_velocity_controller(settings);

    TEST_ASSERT_EQUAL(1, controller_init_calls);
    TEST_ASSERT_EQUAL(BMOTOR0 | BMOTOR1 | BMOTOR2 | BMOTOR3, last_motor_selection);
    TEST_ASSERT_EQUAL_DOUBLE(1.0, last_kp);
    TEST_ASSERT_EQUAL_DOUBLE(0.1, last_ki);
    TEST_ASSERT_EQUAL_DOUBLE(0.01, last_kd);
    TEST_ASSERT_EQUAL_DOUBLE(100.0, last_integral_limit);
}

void test_mecanum_platform_stop_velocity_controller(void) {
    mecanum_platform_stop_velocity_controller();

    TEST_ASSERT_EQUAL(1, controller_stop_calls);
    TEST_ASSERT_EQUAL(BMOTOR0 | BMOTOR1 | BMOTOR2 | BMOTOR3, last_motor_selection);
}

void test_mecanum_platform_set_target_velocity(void) {
    platform_velocity_t target = {
        .x = 1.0,  // 1 m/s
        .y = 0.5,  // 0.5 m/s
        .t = 0.1   // 0.1 rad/s
    };

    mecanum_platform_set_target_velocity(target);

    TEST_ASSERT_EQUAL(4, last_motor_count);
    TEST_ASSERT_EQUAL(MOTOR0, last_target_motor_indexes[0]);
    TEST_ASSERT_EQUAL(MOTOR1, last_target_motor_indexes[1]);
    TEST_ASSERT_EQUAL(MOTOR2, last_target_motor_indexes[2]);
    TEST_ASSERT_EQUAL(MOTOR3, last_target_motor_indexes[3]);

    // With wheel_radius = 0.05m, length = 0.4m, width = 0.3m
    // Expected speeds (rad/s):
    double R = 0.05;  // wheel_radius
    double L = 0.4;   // length
    double W = 0.3;   // width
    
    double V1 = 1.0 / R * (1.0 + 0.5 + (L + W) / 2.0 * 0.1);
    double V2 = 1.0 / R * (1.0 - 0.5 + (L + W) / 2.0 * 0.1);
    double V3 = 1.0 / R * (1.0 + 0.5 - (L + W) / 2.0 * 0.1);
    double V4 = 1.0 / R * (1.0 - 0.5 - (L + W) / 2.0 * 0.1);

    TEST_ASSERT_FLOAT_WITHIN(0.01, V1, last_target_speeds[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01, V2, last_target_speeds[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01, V3, last_target_speeds[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.01, V4, last_target_speeds[3]);
}

void test_mecanum_platform_update_odometry(void) {
    uint8_t motor_indexes[] = {MOTOR0, MOTOR1, MOTOR2, MOTOR3};
    double velocities[] = {1.0, -1.0, 2.0, -2.0};
    
    platform_odometry_t odometry = mecanum_platform_update_odometry(motor_indexes, velocities, 4);

    // With wheel_radius = 0.05m, length = 0.4m, width = 0.3m
    double R = 0.05;
    double L = 0.4;
    double W = 0.3;

    // Expected odometry:
    // x = R/4 * (v1 + v2 + v3 + v4)
    // y = R/4 * (v1 - v2 + v3 - v4)
    // t = R/(2*(L+W)) * (v1 + v2 - v3 - v4)
    double expected_x = R/4.0 * (1.0 + (-1.0) + 2.0 + (-2.0));
    double expected_y = R/4.0 * (1.0 - (-1.0) + 2.0 - (-2.0));
    double expected_t = R/(2.0 * (L + W)) * (1.0 + (-1.0) - 2.0 - (-2.0));

    TEST_ASSERT_FLOAT_WITHIN(0.001, expected_x, odometry.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001, expected_y, odometry.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001, expected_t, odometry.t);
}

void test_mecanum_platform_update_odometry_insufficient_motors(void) {
    uint8_t motor_indexes[] = {MOTOR0, MOTOR1, MOTOR2};
    double velocities[] = {1.0, -1.0, 2.0};
    
    platform_odometry_t odometry = mecanum_platform_update_odometry(motor_indexes, velocities, 3);

    TEST_ASSERT_EQUAL_DOUBLE(0.0, odometry.x);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, odometry.y);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, odometry.t);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_initialize_mecanum_platform);
    RUN_TEST(test_mecanum_platform_set_velocity);
    RUN_TEST(test_mecanum_platform_set_velocity_zero);
    RUN_TEST(test_mecanum_platform_start_odometry);
    RUN_TEST(test_mecanum_platform_start_velocity_controller);
    RUN_TEST(test_mecanum_platform_stop_velocity_controller);
    RUN_TEST(test_mecanum_platform_set_target_velocity);
    RUN_TEST(test_mecanum_platform_update_odometry);
    RUN_TEST(test_mecanum_platform_update_odometry_insufficient_motors);
    return UNITY_END();
}