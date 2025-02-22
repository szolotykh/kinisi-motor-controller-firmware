#include "unity.h"
#include "platform_omni.h"
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
static uint8_t last_target_motor_indexes[3];
static double last_target_speeds[3];
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
    for(int i = 0; i < motor_count && i < 3; i++) {
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
    for(int i = 0; i < 3; i++) {
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
    initialize_omni_platform(1, 0, 1, 0.1, 0.2, 1000.0);
}

void tearDown(void) {
    // Reset interfaces to default implementations
    hw_motor_init();
    hw_encoder_init();
    encoder_odometry_init();
}

void test_initialize_omni_platform(void) {
    double wheel_diameter = 0.1;
    double robot_radius = 0.2;
    double encoder_resolution = 1000.0;

    initialize_omni_platform(1, 0, 1, wheel_diameter, robot_radius, encoder_resolution);

    TEST_ASSERT_EQUAL(3, initialize_motor_calls);
    TEST_ASSERT_EQUAL(3, initialize_encoder_calls);
    
    // Verify motor initialization
    TEST_ASSERT_EQUAL(1, controller_motors.is_reversed(MOTOR0));
    TEST_ASSERT_EQUAL(0, controller_motors.is_reversed(MOTOR1));
    TEST_ASSERT_EQUAL(1, controller_motors.is_reversed(MOTOR2));

    // Verify encoder initialization
    TEST_ASSERT_EQUAL(1000.0, last_encoder_resolution);
}

void test_set_omni_platform_velocity(void) {
    platform_velocity_t velocity = {
        .x = 100.0,
        .y = 0.0,
        .t = 0.0
    };

    set_omni_platform_velocity(velocity);

    TEST_ASSERT_EQUAL(3, set_speed_calls);
    
    // For x = 100, y = 0, t = 0, the velocities should be:
    // V1 = 86.6 (sqrt(3)/2 * 100)
    // V2 = -86.6 (-sqrt(3)/2 * 100)
    // V3 = 0
    TEST_ASSERT_FLOAT_WITHIN(0.1, 86.6, fabs(last_speed));
}

void test_initialize_omni_platform_odometry(void) {
    initialize_omni_platform_odometry();

    TEST_ASSERT_EQUAL(3, start_odometry_calls);
    TEST_ASSERT_EQUAL(MOTOR2, last_encoder); // Last encoder initialized
}

void test_omni_platform_update_odometry(void) {
    uint8_t motor_indexes[] = {MOTOR0, MOTOR1, MOTOR2};
    double velocities[] = {1.0, -1.0, 2.0};
    
    platform_odometry_t odometry = omni_platform_update_odometry(motor_indexes, velocities, 3);

    // Since R and L are 0 in the current implementation, odometry should be 0
    TEST_ASSERT_EQUAL_DOUBLE(0.0, odometry.x);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, odometry.y);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, odometry.t);
}

void test_omni_platform_update_odometry_insufficient_motors(void) {
    uint8_t motor_indexes[] = {MOTOR0, MOTOR1};
    double velocities[] = {1.0, -1.0};
    
    platform_odometry_t odometry = omni_platform_update_odometry(motor_indexes, velocities, 2);

    TEST_ASSERT_EQUAL_DOUBLE(0.0, odometry.x);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, odometry.y);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, odometry.t);
}

void test_omni_platform_start_velocity_controller(void) {
    plaform_controller_settings_t settings = {
        .kp = 1.0,
        .ki = 0.1,
        .kd = 0.01,
        .integral_limit = 100.0
    };

    omni_platform_start_velocity_controller(settings);

    TEST_ASSERT_EQUAL(1, controller_init_calls);
    TEST_ASSERT_EQUAL(BMOTOR0 | BMOTOR1 | BMOTOR2, last_motor_selection);
    TEST_ASSERT_EQUAL_DOUBLE(1.0, last_kp);
    TEST_ASSERT_EQUAL_DOUBLE(0.1, last_ki);
    TEST_ASSERT_EQUAL_DOUBLE(0.01, last_kd);
    TEST_ASSERT_EQUAL_DOUBLE(100.0, last_integral_limit);
}

void test_omni_platform_stop_velocity_controller(void) {
    omni_platform_stop_velocity_controller();

    TEST_ASSERT_EQUAL(1, controller_stop_calls);
    TEST_ASSERT_EQUAL(BMOTOR0 | BMOTOR1 | BMOTOR2, last_motor_selection);
}

void test_omni_platform_set_target_velocity_with_config(void) {
    platform_velocity_t target = {
        .x = 1.0,  // 1 m/s
        .y = 0.5,  // 0.5 m/s
        .t = 0.1   // 0.1 rad/s
    };

    omni_platform_set_target_velocity(target);

    TEST_ASSERT_EQUAL(3, last_motor_count);
    TEST_ASSERT_EQUAL(MOTOR0, last_target_motor_indexes[0]);
    TEST_ASSERT_EQUAL(MOTOR1, last_target_motor_indexes[1]);
    TEST_ASSERT_EQUAL(MOTOR2, last_target_motor_indexes[2]);

    // With wheel_radius = 0.05m and robot_radius = 0.2m
    // Expected speeds (rad/s):
    // V1 = 1/0.05 * (sqrt(3)/2 * 1.0 - 0.5 * 0.5 + 0.2 * 0.1)
    // V2 = 1/0.05 * (-sqrt(3)/2 * 1.0 - 0.5 * 0.5 + 0.2 * 0.1)
    // V3 = 1/0.05 * (0.5 + 0.2 * 0.1)
    double expected_v1 = (sqrt(3.0)/2.0 * 1.0 - 0.5 * 0.5 + 0.2 * 0.1) / 0.05;
    double expected_v2 = (-sqrt(3.0)/2.0 * 1.0 - 0.5 * 0.5 + 0.2 * 0.1) / 0.05;
    double expected_v3 = (0.5 + 0.2 * 0.1) / 0.05;

    TEST_ASSERT_FLOAT_WITHIN(0.01, expected_v1, last_target_speeds[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01, expected_v2, last_target_speeds[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01, expected_v3, last_target_speeds[2]);
}

void test_omni_platform_update_odometry_with_config(void) {
    uint8_t motor_indexes[] = {MOTOR0, MOTOR1, MOTOR2};
    double velocities[] = {1.0, -1.0, 2.0};
    
    platform_odometry_t odometry = omni_platform_update_odometry(motor_indexes, velocities, 3);

    // With wheel_radius = 0.05m and robot_radius = 0.2m
    // Expected odometry:
    // x = 0.05 * (1/sqrt(3) * 1.0 - 1/sqrt(3) * -1.0)
    // y = 0.05/3 * (-1.0 - (-1.0) + 2.0 * 2.0)
    // t = 0.05/(3 * 0.2) * (1.0 + (-1.0) + 2.0)
    double expected_x = 0.05 * (1.0/sqrt(3.0) - (-1.0)/sqrt(3.0));
    double expected_y = 0.05/3.0 * (-1.0 - (-1.0) + 2.0 * 2.0);
    double expected_t = 0.05/(3.0 * 0.2) * (1.0 + (-1.0) + 2.0);

    TEST_ASSERT_FLOAT_WITHIN(0.001, expected_x, odometry.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001, expected_y, odometry.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001, expected_t, odometry.t);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_initialize_omni_platform);
    RUN_TEST(test_set_omni_platform_velocity);
    RUN_TEST(test_initialize_omni_platform_odometry);
    RUN_TEST(test_omni_platform_update_odometry_insufficient_motors);
    RUN_TEST(test_omni_platform_start_velocity_controller);
    RUN_TEST(test_omni_platform_stop_velocity_controller);
    RUN_TEST(test_omni_platform_set_target_velocity_with_config);
    RUN_TEST(test_omni_platform_update_odometry_with_config);
    return UNITY_END();
}