#include "unity.h"
#include "platform_differential.h"
#include "mock_platform_differential.h"
#include "platform_types.h"
#include <math.h>

// Function declarations for platform_differential.h functions
extern void initialize_differential_platform(uint8_t isReversed0, 
                                           uint8_t isReversed1, 
                                           uint8_t isEncoderReversed0, 
                                           uint8_t isEncoderReversed1, 
                                           double wheel_diameter, 
                                           double wheel_base, 
                                           double encoder_resolution);
extern void set_differential_platform_velocity(platform_velocity_t velocity);
extern void differential_platform_start_velocity_controller(plaform_controller_settings_t settings);
extern void differential_platform_stop_velocity_controller(void);
extern void differential_platform_set_target_velocity(platform_velocity_t target);
extern platform_odometry_t differential_platform_update_odometry(uint8_t* motor_indexes, double* velocities, uint8_t motor_count);
extern void initialize_differential_platform_odometry(void);

// Generic platform functions that our tests will call
extern void platform_start_odometry(void);
extern void platform_stop_odometry(void);
extern void platform_reset_odometry(void);
extern platform_odometry_t platform_get_odometry(void);
extern void set_platform_velocity(platform_velocity_t velocity);

void setUp(void) {
    // Reset all mock states
    mock_hw_motor_reset();
    mock_hw_encoder_reset();
    mock_encoder_odometry_reset();
    
    // Set up mock interfaces
    hw_motor_set_interface(mock_hw_motor_get_interface());
    hw_encoder_set_interface(mock_hw_encoder_get_interface());
    encoder_odometry_set_interface(mock_encoder_odometry_get_interface());

    // Initialize platform with test values
    initialize_differential_platform(0, 0, 0, 0, 0.1, 0.3, 1000.0);
}

void tearDown(void) {
    // Reset interfaces to default implementations
    hw_motor_init();
    hw_encoder_init();
    encoder_odometry_init();
}

void test_initialize_differential_platform(void) {
    // Arrange
    const uint8_t isReversed0 = 1;
    const uint8_t isReversed1 = 0;
    const double wheel_diameter = 0.1;  // 10cm diameter
    const double wheel_base = 0.3;      // 30cm between wheels
    const double encoder_resolution = 1000.0;
    
    // The mock_initialize_differential_platform function resets the counters
    // before making its own calls, so we should reset our expectations
    mock_hw_motor_reset();
    mock_hw_encoder_reset();
    
    // Act
    initialize_differential_platform(
        isReversed0,
        isReversed1,
        0,
        0,
        wheel_diameter,
        wheel_base,
        encoder_resolution
    );
    
    // Assert
    TEST_ASSERT_EQUAL(2, mock_get_initialize_motor_calls());
    TEST_ASSERT_EQUAL(2, mock_get_initialize_encoder_calls());
    // The last motor initialized is MOTOR1, so we should expect isReversed1
    TEST_ASSERT_EQUAL(isReversed1, mock_get_last_reversed());
    TEST_ASSERT_EQUAL(encoder_resolution, mock_get_last_encoder_resolution());
}

void test_differential_platform_set_velocity(void) {
    // Arrange
    platform_velocity_t velocity = {
        .x = 0.5,
        .y = 0,
        .t = 0
    };
    
    // Act - Test case 1: Forward movement
    set_differential_platform_velocity(velocity);
    
    // Assert
    TEST_ASSERT_EQUAL(2, mock_get_set_speed_calls());  // Two motors should be updated
    TEST_ASSERT_TRUE(mock_get_last_speed() > 0);      // Last motor speed should be positive
    
    // Arrange - Test case 2: Rotation only
    velocity.x = 0;
    velocity.y = 0;
    velocity.t = 1.0;
    
    // Act
    mock_hw_motor_reset(); // Reset call counters
    set_differential_platform_velocity(velocity);
    
    // Assert
    TEST_ASSERT_EQUAL(2, mock_get_set_speed_calls());
    
    // Arrange - Test case 3: Combined movement and rotation
    velocity.x = 0.3;
    velocity.y = 0;
    velocity.t = 0.5;
    
    // Act
    mock_hw_motor_reset(); // Reset call counters
    set_differential_platform_velocity(velocity);
    
    // Assert
    TEST_ASSERT_EQUAL(2, mock_get_set_speed_calls());
}

void test_initialize_differential_platform_odometry(void) {
    // Act
    initialize_differential_platform_odometry();
    
    // Assert
    TEST_ASSERT_EQUAL(2, mock_get_start_odometry_calls());
    TEST_ASSERT_EQUAL(MOTOR1, mock_get_last_encoder()); // Last encoder initialized
}

void test_differential_platform_start_velocity_controller(void) {
    // Arrange
    plaform_controller_settings_t settings = {
        .kp = 1.0,
        .ki = 0.1,
        .kd = 0.01,
        .integral_limit = 100.0
    };
    
    // Act
    differential_platform_start_velocity_controller(settings);
    
    // Assert
    TEST_ASSERT_EQUAL(1, mock_get_controller_init_calls());
    TEST_ASSERT_EQUAL(BMOTOR0 | BMOTOR1, mock_get_last_motor_selection());
    TEST_ASSERT_EQUAL_DOUBLE(1.0, mock_get_last_kp());
    TEST_ASSERT_EQUAL_DOUBLE(0.1, mock_get_last_ki());
    TEST_ASSERT_EQUAL_DOUBLE(0.01, mock_get_last_kd());
    TEST_ASSERT_EQUAL_DOUBLE(100.0, mock_get_last_integral_limit());
}

void test_differential_platform_stop_velocity_controller(void) {
    // Act
    differential_platform_stop_velocity_controller();
    
    // Assert
    TEST_ASSERT_EQUAL(1, mock_get_controller_stop_calls());
    TEST_ASSERT_EQUAL(BMOTOR0 | BMOTOR1, mock_get_last_motor_selection());
}

void test_differential_platform_set_target_velocity(void) {
    // TODO: Implement this test
    TEST_IGNORE_MESSAGE("Test not implemented yet.");
}

void test_differential_platform_update_odometry(void) {
    // Arrange
    uint8_t motor_indexes[] = {MOTOR0, MOTOR1};
    double velocities[] = {2.0, 3.0};
    
    // Act
    platform_odometry_t odometry = differential_platform_update_odometry(
        motor_indexes, velocities, 2);
    
    // Assert
    // With wheel_radius = 0.05m and wheel_base = 0.3m
    // Expected x = R/2 * (v0 + v1) = 0.05/2 * (2.0 + 3.0) = 0.125
    // Expected t = R/L * (v1 - v0) = 0.05/0.3 * (3.0 - 2.0) = 0.1667
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.125, odometry.x);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, odometry.y);  // Differential drives don't move sideways
    TEST_ASSERT_FLOAT_WITHIN(0.001, 0.1667, odometry.t);
}

void test_differential_platform_update_odometry_insufficient_motors(void) {
    // Arrange
    uint8_t motor_indexes[] = {MOTOR0};
    double velocities[] = {2.0};
    
    // Act
    platform_odometry_t odometry = differential_platform_update_odometry(
        motor_indexes, velocities, 1);
    
    // Assert
    TEST_ASSERT_EQUAL_DOUBLE(0.0, odometry.x);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, odometry.y);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, odometry.t);
}

int main(void) {
    UNITY_BEGIN();
    
    RUN_TEST(test_initialize_differential_platform);
    RUN_TEST(test_differential_platform_set_velocity);
    RUN_TEST(test_initialize_differential_platform_odometry);
    RUN_TEST(test_differential_platform_start_velocity_controller);
    RUN_TEST(test_differential_platform_stop_velocity_controller);
    RUN_TEST(test_differential_platform_set_target_velocity);
    RUN_TEST(test_differential_platform_update_odometry);
    RUN_TEST(test_differential_platform_update_odometry_insufficient_motors);
    
    return UNITY_END();
}
