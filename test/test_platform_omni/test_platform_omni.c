// File: test_platform_omni.c
// Verify omni platform behavior through recorded hardware-interface calls.
#include "commands.h"
#include "unity.h"
#include "platform_omni.h"
#include "mock_platform_omni.h"
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
    initialize_omni_platform(1, 0, 1, 0, 0, 0, 0.1, 0.2, 1000.0);
}

void tearDown(void) {
    // Reset interfaces to default implementations
    hw_motor_init();
    hw_encoder_init();
    encoder_odometry_init();
}

/** @brief Count only the initialization performed by this test, excluding setUp. */
void test_initialize_omni_platform(void) {
    double wheel_diameter = 0.1;
    double robot_radius = 0.2;
    double encoder_resolution = 1000.0;

    mock_hw_motor_reset();
    mock_hw_encoder_reset();
    initialize_omni_platform(1, 0, 1, 0, 0, 0, wheel_diameter, robot_radius, encoder_resolution);

    TEST_ASSERT_EQUAL(3, mock_get_initialize_motor_calls());
    TEST_ASSERT_EQUAL(3, mock_get_initialize_encoder_calls());
    
    // Verify motor initialization using mock functions
    TEST_ASSERT_EQUAL(1, mock_get_last_reversed());
    TEST_ASSERT_EQUAL(1000.0, mock_get_last_encoder_resolution());
}

// Compute rolling speeds by projecting the rigid-body velocity at each
// independently specified wheel center onto its positive (CCW) tangent.
static void check_layout(uint8_t orientation)
{
    const double radius = 0.16, wheel_radius = 0.045;
    initialize_omni_platform_oriented(0,0,0,0,0,0,2*wheel_radius,radius,512,orientation);
    double sign = orientation == OMNI_WHEEL_FRONT ? -1.0 : 1.0;
    double x[] = {sign*radius/2, sign*radius/2, -sign*radius};
    double y[] = {-sign*sqrt(3.0)*radius/2, sign*sqrt(3.0)*radius/2, 0};
    const platform_velocity_t commands[] = {{.x=1},{.y=1},{.t=1},{.x=-.31,.y=.48,.t=-.72},{0}};
    for (unsigned n=0; n<sizeof(commands)/sizeof(commands[0]); ++n) {
        platform_velocity_t v = commands[n];
        double omega[3], target[3], duty[3], max_duty=100;
        uint8_t ports[3];
        int count=0;
        omni_platform_set_target_velocity(v);
        mock_get_last_target_speeds(ports,target,&count);
        TEST_ASSERT_EQUAL_INT(3,count);
        for (unsigned i=0;i<3;++i) {
            double tx=-y[i]/radius, ty=x[i]/radius;
            omega[i]=(tx*(v.x-v.t*y[i])+ty*(v.y+v.t*x[i]))/wheel_radius;
            duty[i]=100*(tx*v.x+ty*v.y+v.t);
            max_duty=fmax(max_duty,fabs(duty[i]));
            TEST_ASSERT_EQUAL_UINT8(i,ports[i]);
            TEST_ASSERT_DOUBLE_WITHIN(1e-10,omega[i],target[i]);
        }
        set_omni_platform_velocity((platform_velocity_t){100*v.x,100*v.y,100*v.t});
        for (unsigned i=0;i<3;++i)
            TEST_ASSERT_DOUBLE_WITHIN(1e-10,100*duty[i]/max_duty,mock_omni_duty(i));
        platform_odometry_t pose=omni_platform_update_odometry(ports,omega,3);
        TEST_ASSERT_DOUBLE_WITHIN(1e-10,v.x,pose.x);
        TEST_ASSERT_DOUBLE_WITHIN(1e-10,v.y,pose.y);
        TEST_ASSERT_DOUBLE_WITHIN(1e-10,v.t,pose.t);
    }
}

void test_set_omni_platform_velocity(void)
{
    check_layout(OMNI_WHEEL_REAR);
    check_layout(OMNI_WHEEL_FRONT);
}

void test_initialize_omni_platform_odometry(void) {
    initialize_omni_platform_odometry();

    TEST_ASSERT_EQUAL(3, mock_get_start_odometry_calls());
    TEST_ASSERT_EQUAL(MOTOR2, mock_get_last_encoder()); // Last encoder initialized
}

void test_omni_platform_start_velocity_controller(void) {
    plaform_controller_settings_t settings = {
        .kp = 1.0,
        .ki = 0.1,
        .kd = 0.01,
        .integral_limit = 100.0
    };

    omni_platform_start_velocity_controller(settings);

    TEST_ASSERT_EQUAL(1, mock_get_controller_init_calls());
    TEST_ASSERT_EQUAL(BMOTOR0 | BMOTOR1 | BMOTOR2, mock_get_last_motor_selection());
    TEST_ASSERT_EQUAL_DOUBLE(1.0, mock_get_last_kp());
    TEST_ASSERT_EQUAL_DOUBLE(0.1, mock_get_last_ki());
    TEST_ASSERT_EQUAL_DOUBLE(0.01, mock_get_last_kd());
    TEST_ASSERT_EQUAL_DOUBLE(100.0, mock_get_last_integral_limit());
}

void test_omni_platform_stop_velocity_controller(void) {
    omni_platform_stop_velocity_controller();

    TEST_ASSERT_EQUAL(1, mock_get_controller_stop_calls());
    TEST_ASSERT_EQUAL(BMOTOR0 | BMOTOR1 | BMOTOR2, mock_get_last_motor_selection());
}

void test_omni_platform_set_target_velocity(void) {
    platform_velocity_t target = {
        .x = 1.0,  // 1 m/s
        .y = 0.5,  // 0.5 m/s
        .t = 0.1   // 0.1 rad/s
    };

    omni_platform_set_target_velocity(target);

    uint8_t motor_indexes[3];
    double target_speeds[3];
    int count;
    mock_get_last_target_speeds(motor_indexes, target_speeds, &count);

    TEST_ASSERT_EQUAL(3, count);
    TEST_ASSERT_EQUAL(MOTOR0, motor_indexes[0]);
    TEST_ASSERT_EQUAL(MOTOR1, motor_indexes[1]);
    TEST_ASSERT_EQUAL(MOTOR2, motor_indexes[2]);

    // With wheel_radius = 0.05m and robot_radius = 0.2m
    // Expected speeds (rad/s):
    // V1 = 1/0.05 * (sqrt(3)/2 * 1.0 - 0.5 * 0.5 + 0.2 * 0.1)
    double expected_v1 = (sqrt(3.0)/2.0 * 1.0 + 0.5 * 0.5 + 0.2 * 0.1) / 0.05;
    double expected_v2 = (-sqrt(3.0)/2.0 * 1.0 + 0.5 * 0.5 + 0.2 * 0.1) / 0.05;
    double expected_v3 = (-0.5 + 0.2 * 0.1) / 0.05;

    TEST_ASSERT_FLOAT_WITHIN(0.01, expected_v1, target_speeds[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01, expected_v2, target_speeds[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.01, expected_v3, target_speeds[2]);
}

void test_omni_platform_update_odometry(void) {
    uint8_t motor_indexes[] = {MOTOR0, MOTOR1, MOTOR2};
    double velocities[] = {1.0, -1.0, 2.0};
    
    platform_odometry_t odometry = omni_platform_update_odometry(motor_indexes, velocities, 3);

    // With wheel_radius = 0.05m and robot_radius = 0.2m
    // Expected odometry:
    // x = 0.05 * (1/sqrt(3) * 1.0 - 1/sqrt(3) * -1.0)
    double expected_x = 0.05 * (1.0/sqrt(3.0) - (-1.0)/sqrt(3.0));
    double expected_y = 0.05/3.0 * (1.0 + (-1.0) - 2.0 * 2.0);
    double expected_t = 0.05/(3.0 * 0.2) * (1.0 + (-1.0) + 2.0);

    TEST_ASSERT_FLOAT_WITHIN(0.001, expected_x, odometry.x);
    TEST_ASSERT_FLOAT_WITHIN(0.001, expected_y, odometry.y);
    TEST_ASSERT_FLOAT_WITHIN(0.001, expected_t, odometry.t);
}

void test_omni_platform_update_odometry_insufficient_motors(void) {
    uint8_t motor_indexes[] = {MOTOR0, MOTOR1};
    double velocities[] = {1.0, -1.0};
    
    platform_odometry_t odometry = omni_platform_update_odometry(motor_indexes, velocities, 2);

    TEST_ASSERT_EQUAL_DOUBLE(0.0, odometry.x);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, odometry.y);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, odometry.t);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_initialize_omni_platform);
    RUN_TEST(test_set_omni_platform_velocity);
    RUN_TEST(test_initialize_omni_platform_odometry);
    RUN_TEST(test_omni_platform_update_odometry_insufficient_motors);
    RUN_TEST(test_omni_platform_start_velocity_controller);
    RUN_TEST(test_omni_platform_stop_velocity_controller);
    RUN_TEST(test_omni_platform_set_target_velocity);
    RUN_TEST(test_omni_platform_update_odometry);
    return UNITY_END();
}
