#include "unity.h"
#include "controller.h"

#include <limits.h>

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
    RUN_TEST(test_encoder1);
    RUN_TEST(test_encoder2);
    return UNITY_END();
}