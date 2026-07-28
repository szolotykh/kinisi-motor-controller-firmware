//------------------------------------------------------------
// File name: test_platform_odometry.c
//------------------------------------------------------------
//
// End-to-end unit test for the odometry pipeline:
//
//     encoder counter delta
//        -> odometry_integrator_wheel_delta()  (ticks -> radians, wrap-safe)
//        -> omni_platform_update_odometry()    (wheels -> body-frame motion)
//        -> odometry_integrator_accumulate()   (body -> world pose, R(theta))
//
// It drives 3 iterations of synthetic encoder values that correspond to
// "turn +90 degrees in place, then drive forward 0.1 m twice" and checks the
// final WORLD pose is (0, 0.2, pi/2) within tolerance. This is exactly the
// case the old firmware got wrong (it added the body-frame forward increment
// straight into world x instead of rotating it into world y).
//------------------------------------------------------------

#include "unity.h"
#include "platform_omni.h"
#include "platform_types.h"
#include "odometry_integrator.h"
#include "mock_platform_odometry.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Robot / encoder constants used for the scenario.
#define WHEEL_DIAMETER   0.1     // -> wheel_radius R = 0.05 m
#define ROBOT_RADIUS     0.2     // L
#define ENCODER_RES      1000.0  // ticks per revolution

// Number of wheels on the omni platform.
#define WHEELS 3

// Running 16-bit encoder counters (mimic the hardware timers). Deliberately
// start high so the forward-turn iterations overflow the counters, exercising
// the wrap handling in odometry_integrator_wheel_delta.
static uint16_t enc[WHEELS];
static uint16_t enc_prev[WHEELS];

void setUp(void) {
    mock_odometry_install_interfaces();

    // isReversed{0,1,2} = 0, isEncoderReversed{0,1,2} = 0.
    initialize_omni_platform(0, 0, 0, 0, 0, 0,
                             WHEEL_DIAMETER, ROBOT_RADIUS, ENCODER_RES);

    for (int i = 0; i < WHEELS; i++) {
        enc[i] = 65000;      // near the top of the uint16 range
        enc_prev[i] = enc[i];
    }
}

void tearDown(void) {
}

// Advance the synthetic encoder counters by the given per-wheel tick deltas,
// then run one full pipeline iteration and accumulate into pose.
static platform_odometry_t step(platform_odometry_t pose, const int tick_delta[WHEELS]) {
    double wheel_rad[WHEELS];
    for (int i = 0; i < WHEELS; i++) {
        enc_prev[i] = enc[i];
        enc[i] = (uint16_t)(enc[i] + tick_delta[i]);
        wheel_rad[i] = odometry_integrator_wheel_delta(enc_prev[i], enc[i], ENCODER_RES);
    }

    uint8_t motor_indexes[WHEELS] = {MOTOR0, MOTOR1, MOTOR2};
    platform_odometry_t body_delta =
        omni_platform_update_odometry(motor_indexes, wheel_rad, WHEELS);

    return odometry_integrator_accumulate(pose, body_delta);
}

// 3 iterations: turn +90 deg in place, then drive forward 0.1 m twice.
// The tick counts come from the inverse kinematics of those body motions
// (verified numerically): a full +90 deg turn is one wheel revolution on each
// wheel; 0.1 m forward is +276 / -276 / 0 ticks.
void test_pipeline_three_iterations_world_pose(void) {
    const int turn_90[WHEELS]    = {1000, 1000, 1000};
    const int forward_01[WHEELS] = {276, -276, 0};

    platform_odometry_t pose = {0};

    // Iteration 1: rotate in place by +pi/2. World translation stays ~0.
    pose = step(pose, turn_90);
    TEST_ASSERT_DOUBLE_WITHIN(0.002, 0.0,        pose.x);
    TEST_ASSERT_DOUBLE_WITHIN(0.002, 0.0,        pose.y);
    TEST_ASSERT_DOUBLE_WITHIN(0.002, M_PI / 2.0, pose.t);

    // Iteration 2: forward 0.1 m in body frame. Heading is +pi/2, so this must
    // become +0.1 m in WORLD y (not world x -- that was the bug).
    pose = step(pose, forward_01);
    TEST_ASSERT_DOUBLE_WITHIN(0.003, 0.0,        pose.x);
    TEST_ASSERT_DOUBLE_WITHIN(0.003, 0.1,        pose.y);
    TEST_ASSERT_DOUBLE_WITHIN(0.002, M_PI / 2.0, pose.t);

    // Iteration 3: another forward 0.1 m -> world y ~0.2.
    pose = step(pose, forward_01);
    TEST_ASSERT_DOUBLE_WITHIN(0.004, 0.0,        pose.x);
    TEST_ASSERT_DOUBLE_WITHIN(0.004, 0.2,        pose.y);
    TEST_ASSERT_DOUBLE_WITHIN(0.002, M_PI / 2.0, pose.t);
}

// Directly exercise the 16-bit wrap handling of the wheel-delta decoder.
void test_wheel_delta_handles_wrap(void) {
    // 1000 ticks on a 1000-tick encoder = one revolution = 2*pi rad.
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 2.0 * M_PI,
        odometry_integrator_wheel_delta(0, 1000, ENCODER_RES));

    // Forward across the top of the range (overflow): 65500 -> 464 is +500.
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 500.0 / ENCODER_RES * 2.0 * M_PI,
        odometry_integrator_wheel_delta(65500, 464, ENCODER_RES));

    // Backward across zero (underflow): 200 -> 65736&0xFFFF=65200 is -500.
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, -500.0 / ENCODER_RES * 2.0 * M_PI,
        odometry_integrator_wheel_delta(200, (uint16_t)(200 - 500), ENCODER_RES));

    // Zero resolution must not divide by zero.
    TEST_ASSERT_EQUAL_DOUBLE(0.0,
        odometry_integrator_wheel_delta(0, 500, 0.0));
}

// The accumulator must rotate the body-frame increment by heading. Forward in
// body frame while facing +pi/2 must land in world +y.
void test_accumulate_rotates_body_into_world(void) {
    platform_odometry_t pose = {.x = 0, .y = 0, .t = M_PI / 2.0};
    platform_odometry_t body = {.x = 0.1, .y = 0.0, .t = 0.0};

    pose = odometry_integrator_accumulate(pose, body);

    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.0,        pose.x);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, 0.1,        pose.y);
    TEST_ASSERT_DOUBLE_WITHIN(1e-9, M_PI / 2.0, pose.t);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_wheel_delta_handles_wrap);
    RUN_TEST(test_accumulate_rotates_body_into_world);
    RUN_TEST(test_pipeline_three_iterations_world_pose);
    return UNITY_END();
}
