#include "unity.h"
#include "commands.h"
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

// Derive rolling speed from the rigid-body velocity at each actual wheel
// center, independently of the production mixer and its inverse.
static void check_roller_layout(double length, double width, uint8_t layout)
{
    initialize_mecanum_platform_oriented(0,0,0,0,0,0,0,0, length,width,0.1,100,layout);
    const double x[] = {length/2, -length/2, -length/2, length/2};
    const double y[] = {-width/2, -width/2, width/2, width/2};
    const double standard[] = {1,-1,1,-1};
    const platform_velocity_t targets[] = {{.x=1},{.y=1},{.t=1},{.x=-0.2,.y=0.3,.t=-0.9},{0}};
    for (unsigned n=0; n<sizeof(targets)/sizeof(targets[0]); ++n) {
        platform_velocity_t v=targets[n];
        uint8_t ids[4];
        double controlled[4], expected[4];
        int count=0;
        mecanum_platform_set_target_velocity(v);
        mock_get_last_target_speeds(ids,controlled,&count);
        TEST_ASSERT_EQUAL_INT(4,count);
        for (unsigned i=0;i<4;i++) {
            double slope=layout==MECANUM_ROLLER_STANDARD ? standard[i] : -standard[i];
            expected[i]=((v.x-y[i]*v.t)+slope*(v.y+x[i]*v.t))/0.05;
            TEST_ASSERT_EQUAL_UINT8(i,ids[i]);
            TEST_ASSERT_DOUBLE_WITHIN(1e-10,expected[i],controlled[i]);
        }
        platform_odometry_t odom=mecanum_platform_update_odometry(ids,expected,4);
        TEST_ASSERT_DOUBLE_WITHIN(1e-10,v.x,odom.x);
        TEST_ASSERT_DOUBLE_WITHIN(1e-10,v.y,odom.y);
        TEST_ASSERT_DOUBLE_WITHIN(1e-10,v.t,odom.t);
        if (n == 3) continue; // Open-loop input uses PWM percent per axis.
        mecaunm_platform_set_velocity((platform_velocity_t){.x=v.x*40,.y=v.y*40,.t=v.t*40});
        for (unsigned i=0;i<4;i++) {
            double expected_duty=expected[i]>0 ? 40 : expected[i]<0 ? -40 : 0;
            TEST_ASSERT_DOUBLE_WITHIN(1e-10, expected_duty, mock_motor_duty(i));
        }
    }
}

void test_motor_and_encoder_geometry(void)
{
    check_roller_layout(.4,.3,MECANUM_ROLLER_STANDARD);
    check_roller_layout(.3,.4,MECANUM_ROLLER_STANDARD);
    check_roller_layout(.4,.3,MECANUM_ROLLER_MIRRORED);
    check_roller_layout(.3,.4,MECANUM_ROLLER_MIRRORED);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_initialize_mecanum_platform);
    RUN_TEST(test_motor_and_encoder_geometry);
    RUN_TEST(test_initialize_mecanum_platform_odometry);
    RUN_TEST(test_mecanum_platform_start_velocity_controller);
    RUN_TEST(test_mecanum_platform_stop_velocity_controller);
    RUN_TEST(test_mecanum_platform_update_odometry);
    RUN_TEST(test_mecanum_platform_update_odometry_insufficient_motors);
    return UNITY_END();
}