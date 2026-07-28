#include "unity.h"
#include "unity.h"
#include "platform_mecanum.h"
#include "controllers_manager.h"
#include "mock_platform_mecanum.h"
#include "platform_types.h"
#include <math.h>

void setUp(void) {
    // Reset all mock states
    mock_hw_motor_reset();
    mock_hw_encoder_reset();
    mock_encoder_odometry_reset();
    
    // Set up mock interfaces
    hw_motor_set_interface((const hw_motor_interface_t*)mock_hw_motor_get_interface());
    hw_encoder_set_interface((const hw_encoder_interface_t*)mock_hw_encoder_get_interface());
    encoder_odometry_set_interface((const encoder_odometry_interface_t*)mock_encoder_odometry_get_interface());

    // Initialize platform with test values
    initialize_mecanum_platform(1, 0, 1, 0, 0, 0, 0, 0, 0.4, 0.3, 0.1, 1000.0);
}

void tearDown(void) {
    // Reset interfaces to default implementations
    hw_motor_init();
    hw_encoder_init();
    encoder_odometry_init();
}

void test_initialize_mecanum_platform(void) {
    TEST_ASSERT_EQUAL(4, mock_get_initialize_motor_calls());
    TEST_ASSERT_EQUAL(4, mock_get_initialize_encoder_calls());
    
    // Verify motor initialization using mock functions
    TEST_ASSERT_EQUAL(0, mock_get_last_reversed());
    TEST_ASSERT_EQUAL(1000.0, mock_get_last_encoder_resolution());
}

void test_set_mecanum_platform_velocity(void) {
    // TODO: Implement this test case
    TEST_IGNORE_MESSAGE("Test not implemented yet.");
}

void test_initialize_mecanum_platform_odometry(void) {
    mecanum_platform_start_odometry();  // Changed from initialize_mecanum_platform_odometry

    TEST_ASSERT_EQUAL(4, mock_get_start_odometry_calls());
    TEST_ASSERT_EQUAL(MOTOR3, mock_get_last_encoder()); // Last encoder initialized
}

void test_mecanum_platform_start_velocity_controller(void) {
    plaform_controller_settings_t settings = {
        .kp = 1.0,
        .ki = 0.1,
        .kd = 0.01,
        .integral_limit = 100.0
    };

    mecanum_platform_start_velocity_controller(settings);

    TEST_ASSERT_EQUAL(1, mock_get_controller_init_calls());
    TEST_ASSERT_EQUAL(BMOTOR0 | BMOTOR1 | BMOTOR2 | BMOTOR3, mock_get_last_motor_selection());
    TEST_ASSERT_EQUAL_DOUBLE(1.0, mock_get_last_kp());
    TEST_ASSERT_EQUAL_DOUBLE(0.1, mock_get_last_ki());
    TEST_ASSERT_EQUAL_DOUBLE(0.01, mock_get_last_kd());
    TEST_ASSERT_EQUAL_DOUBLE(100.0, mock_get_last_integral_limit());
}

void test_mecanum_platform_stop_velocity_controller(void) {
    mecanum_platform_stop_velocity_controller();

    TEST_ASSERT_EQUAL(1, mock_get_controller_stop_calls());
    TEST_ASSERT_EQUAL(BMOTOR0 | BMOTOR1 | BMOTOR2 | BMOTOR3, mock_get_last_motor_selection());
}

void test_mecanum_platform_set_target_velocity(void) {
    // TODO: Implement this test case
    TEST_IGNORE_MESSAGE("Test not implemented yet.");
}

void test_mecanum_platform_update_odometry(void) {
    uint8_t motor_indexes[] = {MOTOR0, MOTOR1, MOTOR2, MOTOR3};
    double velocities[] = {1.0, -1.0, 2.0, -2.0};
    
    platform_odometry_t odometry = mecanum_platform_update_odometry(motor_indexes, velocities, 4);

    // With wheel_radius = 0.05m, length = 0.4m, width = 0.3m
    double R = 0.05;
    double L = 0.4;
    double W = 0.3;

    // Expected odometry based on platform_mecanum.c implementation:
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
    uint8_t motor_indexes[] = {MOTOR0, MOTOR1};
    double velocities[] = {1.0, -1.0};
    
    platform_odometry_t odometry = mecanum_platform_update_odometry(motor_indexes, velocities, 2);

    TEST_ASSERT_EQUAL_DOUBLE(0.0, odometry.x);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, odometry.y);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, odometry.t);
}

void test_mecanum_platform_set_velocity(void) {
    // TODO: Implement this test case
    TEST_IGNORE_MESSAGE("Test not implemented yet.");
}

void test_mecanum_platform_set_velocity_zero(void) {
    // TODO: Implement this test case
    TEST_IGNORE_MESSAGE("Test not implemented yet.");
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_initialize_mecanum_platform);
    RUN_TEST(test_mecanum_platform_set_velocity);
    RUN_TEST(test_mecanum_platform_set_velocity_zero);
    RUN_TEST(test_initialize_mecanum_platform_odometry);
    RUN_TEST(test_mecanum_platform_start_velocity_controller);
    RUN_TEST(test_mecanum_platform_stop_velocity_controller);
    RUN_TEST(test_mecanum_platform_set_target_velocity);
    RUN_TEST(test_mecanum_platform_update_odometry);
    RUN_TEST(test_mecanum_platform_update_odometry_insufficient_motors);
    return UNITY_END();
}