#include <unity.h>
#include "platform_types.h"
#include "platform_common.h"
#include "pid_controller.h"

#include <limits.h>

typedef struct {
    double motor0;
    double motor1;
    double motor2;
    double motor3;
} mecanum_velocity_t;

// Helper function to calculate mecanum velocities
mecanum_velocity_t get_mecanum_velocities(double vx, double vy, double vt) {
    mecanum_velocity_t velocities;
    
    if (vx == 0 && vy == 0 && vt == 0) {
        velocities.motor0 = 0;
        velocities.motor1 = 0;
        velocities.motor2 = 0;
        velocities.motor3 = 0;
        return velocities;
    }

    // Normalize input
    int l = abs(vx) + abs(vy) + abs(vt);
    double sing_x = (vx > 0) - (vx < 0);
    double sing_y = (vy > 0) - (vy < 0);
    double sing_t = (vt > 0) - (vt < 0);

    vx = sing_x * vx * vx / l;
    vy = sing_y * vy * vy / l;
    vt = sing_t * vt * vt / l;

    velocities.motor0 = vx + vy + vt;
    velocities.motor1 = vx - vy + vt;
    velocities.motor2 = vx + vy - vt;
    velocities.motor3 = vx - vy - vt;

    return velocities;
}

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

void test_Velocity_X_100(void)
{
    mecanum_velocity_t velocities = get_mecanum_velocities(100, 0, 0);
    TEST_ASSERT_EQUAL_INT(velocities.motor0, 100);
    TEST_ASSERT_EQUAL_INT(velocities.motor1, 100);
    TEST_ASSERT_EQUAL_INT(velocities.motor2, 100);
    TEST_ASSERT_EQUAL_INT(velocities.motor3, 100);
}

void test_Velocity_Y_100() {
    mecanum_velocity_t velocities = get_mecanum_velocities(0, 100, 0);
    TEST_ASSERT_EQUAL_INT(velocities.motor0, 100);
    TEST_ASSERT_EQUAL_INT(velocities.motor1, -100);
    TEST_ASSERT_EQUAL_INT(velocities.motor2, 100);
    TEST_ASSERT_EQUAL_INT(velocities.motor3, -100);
}

void test_Velocity_T_100() {
    mecanum_velocity_t velocities = get_mecanum_velocities(0, 0, 100);
    TEST_ASSERT_EQUAL_INT(velocities.motor0, 100);
    TEST_ASSERT_EQUAL_INT(velocities.motor1, 100);
    TEST_ASSERT_EQUAL_INT(velocities.motor2, -100);
    TEST_ASSERT_EQUAL_INT(velocities.motor3, -100);
}

void test_Velocity_Zero() {
    mecanum_velocity_t velocities = get_mecanum_velocities(0, 0, 0);
    TEST_ASSERT_EQUAL_INT(velocities.motor0, 0);
    TEST_ASSERT_EQUAL_INT(velocities.motor1, 0);
    TEST_ASSERT_EQUAL_INT(velocities.motor2, 0);
    TEST_ASSERT_EQUAL_INT(velocities.motor3, 0);
}

void test_encoder1(void)
{
  unsigned int p0 = 200;
  unsigned int p1 = UINT_MAX - 100 + 1;
  int v0 = p0 - p1;
  TEST_ASSERT_EQUAL_INT(v0, 300);
}

void test_encoder2(void)
{
  unsigned int p0 = UINT_MAX - 100 + 1;
  unsigned int p1 = 200;
  int v0 = p0 - p1;
  TEST_ASSERT_EQUAL_INT(v0, -300);
}

// Regression: a zero target speed must force a true stop. Previously the
// incremental output (motorPWM += ... + integrator) kept a residual PWM that
// made the motor creep at speeds too low for the encoder to measure, so the
// loop could not sense (and therefore never corrected) the motion.
void test_Pid_Zero_Target_Forces_Stop(void)
{
    pid_controller_t c;
    pid_controller_init(&c, 0.1, 0.8, 0.2, 0.0, 30.0);

    // Wind the controller up with a nonzero target while the wheel reads 0,
    // so motorPWM and the integrator both accumulate to nonzero values.
    for (int i = 0; i < 20; i++) {
        pid_controller_update(&c, 0.0, 5.0);
    }
    TEST_ASSERT_TRUE(c.motorPWM != 0.0);

    // Command a stop while the wheel still creeps below encoder resolution
    // (measured speed quantizes to 0): output and integrator must go to zero.
    double pwm = pid_controller_update(&c, 0.0, 0.0);
    TEST_ASSERT_TRUE(pwm == 0.0);
    TEST_ASSERT_TRUE(c.motorPWM == 0.0);
    TEST_ASSERT_TRUE(c.integrator == 0.0);
}

// The stop must be sticky: repeated zero commands (with the wheel unable to
// report the residual creep) must not let the output ramp back up.
void test_Pid_Zero_Target_Does_Not_Creep(void)
{
    pid_controller_t c;
    pid_controller_init(&c, 0.1, 0.8, 0.2, 0.0, 30.0);
    for (int i = 0; i < 20; i++) {
        pid_controller_update(&c, 0.0, 5.0);
    }
    for (int i = 0; i < 15; i++) {
        double pwm = pid_controller_update(&c, 0.0, 0.0);
        TEST_ASSERT_TRUE(pwm == 0.0);
    }
}

// pid_controller_reset must clear the accumulated runtime state (integrator
// windup, differentiator, error/speed history, output and target) so a wound-up
// controller starts fresh on its next update.
void test_Pid_Reset_Clears_Runtime_State(void)
{
    pid_controller_t c;
    pid_controller_init(&c, 0.1, 0.8, 0.2, 0.1, 30.0);

    // Wind the controller up so its runtime fields accumulate nonzero values.
    for (int i = 0; i < 20; i++) {
        pid_controller_update(&c, 0.0, 5.0);
    }
    TEST_ASSERT_TRUE(c.motorPWM != 0.0);
    TEST_ASSERT_TRUE(c.integrator != 0.0);

    pid_controller_reset(&c);

    TEST_ASSERT_EQUAL_DOUBLE(0.0, c.integrator);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, c.differentiator);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, c.previousError);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, c.previousSpeed);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, c.motorPWM);
    TEST_ASSERT_EQUAL_DOUBLE(0.0, c.target_speed);
}

// pid_controller_reset must keep the controller's tuning (gains, sampling time,
// tau and integral limits) so it stays configured and running after a reset.
void test_Pid_Reset_Preserves_Tuning(void)
{
    pid_controller_t c;
    pid_controller_init(&c, 0.1, 0.8, 0.2, 0.1, 30.0);
    for (int i = 0; i < 20; i++) {
        pid_controller_update(&c, 0.0, 5.0);
    }

    pid_controller_reset(&c);

    TEST_ASSERT_EQUAL_DOUBLE(0.8, c.kp);
    TEST_ASSERT_EQUAL_DOUBLE(0.2, c.ki);
    TEST_ASSERT_EQUAL_DOUBLE(0.1, c.kd);
    TEST_ASSERT_EQUAL_DOUBLE(0.1, c.T);
    TEST_ASSERT_EQUAL_DOUBLE(30.0, c.max_integral);
    TEST_ASSERT_EQUAL_DOUBLE(-30.0, c.min_integral);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_Velocity_X_100);
    RUN_TEST(test_Velocity_Y_100);
    RUN_TEST(test_Velocity_T_100);
    RUN_TEST(test_Velocity_Zero);
    RUN_TEST(test_encoder1);
    RUN_TEST(test_encoder2);
    RUN_TEST(test_Pid_Zero_Target_Forces_Stop);
    RUN_TEST(test_Pid_Zero_Target_Does_Not_Creep);
    RUN_TEST(test_Pid_Reset_Clears_Runtime_State);
    RUN_TEST(test_Pid_Reset_Preserves_Tuning);
    return UNITY_END();
}