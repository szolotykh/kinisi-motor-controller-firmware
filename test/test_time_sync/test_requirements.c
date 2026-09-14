//------------------------------------------------------------
// File name: test_requirements.c
// Description: Verify command prerequisite errors and execution suppression.
//------------------------------------------------------------
#include "command_requirements.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t owned, motors, encoders, controllers, platform_ready, platform_controller;
static uint8_t reply[255], reply_size;
static unsigned executed;
/**
 * @brief Return the fixture ownership bit for the requested motor.
 */
static uint8_t motor_owned(uint8_t i) { assert(i < 4); return (owned >> i) & 1; }
/**
 * @brief Return the fixture initialization bit for the requested motor.
 */
static uint8_t motor_initialized(uint8_t i) { assert(i < 4); return (motors >> i) & 1; }
/**
 * @brief Return the fixture initialization bit for the requested encoder.
 */
static uint8_t encoder_initialized(uint8_t i) { assert(i < 4); return (encoders >> i) & 1; }
/**
 * @brief Return the fixture running bit for the requested motor controller.
 */
static uint8_t controller_running(uint8_t i) { assert(i < 4); return (controllers >> i) & 1; }
/**
 * @brief Return the fixture platform initialization flag.
 */
static uint8_t platform_initialized(void) { return platform_ready; }
/**
 * @brief Return the fixture platform-controller running flag.
 */
static uint8_t platform_controller_running(void) { return platform_controller; }
static const command_resources_t resources = {
    motor_owned, motor_initialized, encoder_initialized, controller_running,
    platform_initialized, platform_controller_running
};
/**
 * @brief Validate the fixture command and record accepted executions for assertions.
 */
static uint8_t handler(controller_command_t *cmd, protocol_send_fn callback)
{
    (void)callback;
    uint8_t error = command_requirements_check(cmd, &resources);
    if (error == RESPONSE_OK) ++executed;
    return error;
}
/**
 * @brief Copy a response frame into fixture storage for wire assertions.
 */
static void capture(uint8_t *bytes, uint8_t size)
{
    memcpy(reply, bytes, size); reply_size = size;
}
/**
 * @brief Dispatch a command and verify error correlation plus absence of execution on failure.
 */
static void check(uint8_t command, uint8_t index, uint8_t expected)
{
    controller_command_t cmd = {0};
    cmd.commandType = command; cmd.message_id = 321;
    // Each indexed command below has its index as the first payload byte.
    ((uint8_t *)&cmd)[3] = index;
    unsigned before = executed;
    protocol_dispatch((uint8_t *)&cmd, command_request_size(command), handler, capture);
    assert(reply[2] == 65 && reply[3] == 1);
    if (expected == RESPONSE_OK) {
        assert(reply_size == 4 && reply[1] == command && executed == before + 1);
    } else {
        assert(reply_size == 6 && reply[1] == KINISI_MESSAGE_ERROR);
        assert(reply[4] == command && reply[5] == expected && executed == before);
        printf("%u,%u\n", command, expected); // Runner cross-checks the JSON declaration.
    }
}
/**
 * @brief Run this file's assertions and return zero when all checks pass.
 */
int main(void)
{
    check(SET_MOTOR_SPEED, 3, RESPONSE_MOTOR_NOT_INITIALIZED);
    check(GET_ENCODER_VALUE, 3, RESPONSE_ENCODER_NOT_INITIALIZED);
    check(START_ENCODER_ODOMETRY, 3, RESPONSE_ENCODER_NOT_INITIALIZED);
    check(SET_MOTOR_TARGET_SPEED, 3, RESPONSE_CONTROLLER_NOT_INITIALIZED);
    check(START_PLATFORM_ODOMETRY, 0, RESPONSE_PLATFORM_NOT_INITIALIZED);
    check(START_PLATFORM_CONTROLLER, 0, RESPONSE_PLATFORM_NOT_INITIALIZED);
    check(SET_PLATFORM_VELOCITY, 0, RESPONSE_PLATFORM_NOT_INITIALIZED);
    check(SET_PLATFORM_TARGET_VELOCITY, 0, RESPONSE_PLATFORM_NOT_INITIALIZED);
    platform_ready = 1;
    check(SET_PLATFORM_TARGET_VELOCITY, 0, RESPONSE_CONTROLLER_NOT_INITIALIZED);
    motors = encoders = controllers = 1; // Another resource's state must not satisfy index 3.
    check(SET_MOTOR_SPEED, 3, RESPONSE_MOTOR_NOT_INITIALIZED);
    check(GET_ENCODER_VALUE, 3, RESPONSE_ENCODER_NOT_INITIALIZED);
    check(SET_MOTOR_TARGET_SPEED, 3, RESPONSE_CONTROLLER_NOT_INITIALIZED);
    motors = encoders = controllers = 8;
    platform_controller = 1;
    check(SET_MOTOR_SPEED, 3, RESPONSE_OK);
    check(GET_ENCODER_VALUE, 3, RESPONSE_OK);
    check(START_ENCODER_ODOMETRY, 3, RESPONSE_OK);
    check(SET_MOTOR_TARGET_SPEED, 3, RESPONSE_OK);
    check(START_PLATFORM_ODOMETRY, 0, RESPONSE_OK);
    check(SET_PLATFORM_TARGET_VELOCITY, 0, RESPONSE_OK);
    owned = 8; motors = controllers = 0;
    check(SET_MOTOR_SPEED, 3, RESPONSE_MOTOR_OWNED);
    check(SET_MOTOR_TARGET_SPEED, 3, RESPONSE_MOTOR_OWNED);
    check(STOP_MOTOR, 3, RESPONSE_MOTOR_OWNED);
    check(BRAKE_MOTOR, 3, RESPONSE_MOTOR_OWNED);
    check(DELETE_MOTOR_CONTROLLER, 3, RESPONSE_MOTOR_OWNED);
    check(RESET_MOTOR_CONTROLLER, 3, RESPONSE_MOTOR_OWNED);
    owned = 0;
    check(STOP_MOTOR, 3, RESPONSE_OK);
    check(BRAKE_MOTOR, 3, RESPONSE_OK);
    check(DELETE_MOTOR_CONTROLLER, 3, RESPONSE_OK);
    check(RESET_MOTOR_CONTROLLER, 3, RESPONSE_OK);
    return 0;
}
