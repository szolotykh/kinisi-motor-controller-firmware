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
    char buffer[10] = {0X02, 0x02, 0x01, 0x11, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00};
    controller_command_t* command = (controller_command_t*)(&buffer[0]);
    TEST_ASSERT_EQUAL_INT(command->commandType, SET_MOTOR_SPEED);
    TEST_ASSERT_EQUAL_INT(command->properties.setMotorSpeed.motorIndex, 2);
    TEST_ASSERT_EQUAL_INT(command->properties.setMotorSpeed.direction, 1);
    TEST_ASSERT_EQUAL_INT(command->properties.setMotorSpeed.speed, 43537);
}

void test_set_motor_controller_command(void)
{
    char buffer[] = {0x03,0x00,0x33,0x33,0x33,0x33,0x33,0x33,0xd3,0x3f,0x9a,0x99,0x99,0x99,0x99,0x99,0xc9,0x3f,0x9a,0x99,0x99,0x99,0x99,0x99,0xb9,0x3f};
    controller_command_t* command = (controller_command_t*)(&buffer[0]);
    TEST_ASSERT_EQUAL_INT(command->commandType, MOTOR_SET_CONTROLLER);
    TEST_ASSERT_EQUAL_INT(command->properties.setMotorSpeed.motorIndex, 0);
    TEST_ASSERT_EQUAL_DOUBLE(command->properties.setMotorController.kp, 0.3);
    TEST_ASSERT_EQUAL_DOUBLE(command->properties.setMotorController.ki, 0.2);
    TEST_ASSERT_EQUAL_DOUBLE(command->properties.setMotorController.kd, 0.1);
}

int main(void) {
    UNITY_BEGIN(); 
    RUN_TEST(test_set_motor_speed_command);
    RUN_TEST(test_set_motor_controller_command);
    return UNITY_END();
}