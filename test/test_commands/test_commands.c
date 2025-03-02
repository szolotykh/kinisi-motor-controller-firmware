#include "unity.h"
#include "commands.h"

void setUp(void) {
  // set stuff up here
}

void tearDown(void) {
  // clean stuff up here
}

void test_set_motor_speed_command(void)
{
    char buffer[10] = {0X02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F};
    controller_command_t* command = (controller_command_t*)(&buffer[0]);
    TEST_ASSERT_EQUAL_INT(command->commandType, SET_MOTOR_SPEED);
    TEST_ASSERT_EQUAL_INT(command->properties.set_motor_speed.motor_index, 2);
    TEST_ASSERT_DOUBLE_WITHIN(0.0001, 1.0, command->properties.set_motor_speed.pwm);
}

void test_set_motor_controller_command(void)
{
    char buffer[] = {
        0x05,               // commandType (INITIALIZE_MOTOR_CONTROLLER)
        0x00,               // motor_index
        0x00,               // is_reversed
        0x00,               // encoder_index
        // encoder_resolution (not testing this value)
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        // kp = 0.3
        0x33, 0x33, 0x33, 0x33, 0x33, 0x33, 0xD3, 0x3F, 
        // ki = 0.2
        0x9A, 0x99, 0x99, 0x99, 0x99, 0x99, 0xC9, 0x3F, 
        // kd = 0.1
        0xCD, 0xCC, 0xCC, 0xCC, 0xCC, 0xCC, 0xB9, 0x3F,
        // integral_limit (not testing this value)
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    controller_command_t* command = (controller_command_t*)(&buffer[0]);
    TEST_ASSERT_EQUAL_INT(command->commandType, INITIALIZE_MOTOR_CONTROLLER);
    TEST_ASSERT_EQUAL_INT(command->properties.initialize_motor_controller.motor_index, 0);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.3, command->properties.initialize_motor_controller.kp);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.2, command->properties.initialize_motor_controller.ki);
    TEST_ASSERT_DOUBLE_WITHIN(0.01, 0.1, command->properties.initialize_motor_controller.kd);
}

int main(void) {
    UNITY_BEGIN(); 
    RUN_TEST(test_set_motor_speed_command);
    RUN_TEST(test_set_motor_controller_command);
    return UNITY_END();
}