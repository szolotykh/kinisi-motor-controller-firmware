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

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_Velocity_X_100);
    RUN_TEST(test_Velocity_Y_100);
    RUN_TEST(test_Velocity_T_100);
    RUN_TEST(test_Velocity_Zero);
    RUN_TEST(test_encoder1);
    RUN_TEST(test_encoder2);
    return UNITY_END();
}