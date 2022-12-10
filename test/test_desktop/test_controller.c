#include "unity.h"
#include "controller.h"

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

void test_Velocity_X_100(void)
{
    mecanum_velocity_t velocities = get_mecanum_velocities(100, 0, 0);
    TEST_ASSERT_EQUAL_INT(velocities.motor0, -100);
    TEST_ASSERT_EQUAL_INT(velocities.motor1, -100);
    TEST_ASSERT_EQUAL_INT(velocities.motor2, 100);
    TEST_ASSERT_EQUAL_INT(velocities.motor3, 100);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_Velocity_X_100);
    return UNITY_END();
}