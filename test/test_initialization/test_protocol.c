//------------------------------------------------------------
// File name: test_protocol.c
// Description: Verify shared protocol framing, validation, and fragmented receive queues.
//------------------------------------------------------------
#include "protocol.h"
#include "initialization.h"
#include "message_queue.h"
#include <assert.h>
#include <string.h>

static uint8_t received[255];
static uint8_t received_length;
static unsigned calls;
static uint8_t mode;
/**
 * @brief Save a framed response and its length for protocol assertions.
 */
static void send_reply(uint8_t *bytes, uint8_t length)
{
    memcpy(received, bytes, length);
    received_length = length;
}
/**
 * @brief Return fixture payloads or errors to exercise shared protocol dispatch.
 */
static uint8_t handle(controller_command_t *command, protocol_send_fn callback)
{
    ++calls;
    if (command->commandType == INIT) {
        uint8_t error = initialization_validate(command);
        if (error) return error;
        init_response response = initialization_response();
        callback((uint8_t *)&response, sizeof(response));
        return RESPONSE_OK;
    }
    if (mode == 1) {
        uint8_t payload[] = {0xff, 0x00};
        callback(payload, sizeof(payload));
    }
    if (mode == 2) return RESPONSE_MOTOR_OWNED;
    if (mode == 3) {
        uint8_t payload[252] = {0};
        callback(payload, sizeof(payload));
    }
    return RESPONSE_OK;
}
/**
 * @brief Verify the captured ERROR frame identifies the expected command and code.
 */
static void expect_error(const uint8_t *data, size_t size, uint8_t status)
{
    protocol_dispatch(data, size, handle, send_reply);
    assert(received[2] == (size >= 3 ? data[1] : 0));
    assert(received[3] == (size >= 3 ? data[2] : 0));
    if (status == RESPONSE_OK) {
        assert(received_length == 4 && received[0] == 3);
        assert(received[1] == data[0]);
    } else {
        assert(received_length == 6 && received[0] == 5);
        assert(received[1] == KINISI_MESSAGE_ERROR);
        assert(received[4] == (size ? data[0] : 0));
        assert(received[5] == status);
    }
}

/**
 * @brief Run this file's assertions and return zero when all checks pass.
 */
int main(void)
{
    assert(protocol_next_message_id(0) == 1);
    assert(protocol_next_message_id(1) == 2);
    assert(protocol_next_message_id(65534) == 65535);
    assert(protocol_next_message_id(65535) == 1);
    uint8_t zero_id[] = {TOGGLE_STATUS_LED_STATE, 0, 0};
    expect_error(zero_id, sizeof(zero_id), RESPONSE_INVALID_ARGUMENT);
    uint8_t wrap_id[] = {TOGGLE_STATUS_LED_STATE, 0xff, 0xff};
    expect_error(wrap_id, sizeof(wrap_id), RESPONSE_OK);
    wrap_id[1] = 1; wrap_id[2] = 0;
    expect_error(wrap_id, sizeof(wrap_id), RESPONSE_OK);
    calls = 0;
    uint8_t unknown[] = {0xfe, 0x34, 0x12};
    expect_error(unknown, sizeof(unknown), RESPONSE_UNKNOWN_COMMAND);
    expect_error(NULL, 0, RESPONSE_INVALID_LENGTH);
    uint8_t short_request[] = {SET_MOTOR_SPEED, 0};
    expect_error(short_request, sizeof(short_request), RESPONSE_INVALID_LENGTH);
    uint8_t long_request[255] = {INIT};
    expect_error(long_request, sizeof(long_request), RESPONSE_INVALID_LENGTH);
    controller_command_t motor = {0};
    motor.commandType = SET_MOTOR_SPEED;
    motor.message_id = 42;
    motor.properties.set_motor_speed.motor_index = 4;
    expect_error((uint8_t *)&motor, 12, RESPONSE_INVALID_ARGUMENT);
    motor.properties.set_motor_speed.motor_index = 0;
    motor.properties.set_motor_speed.pwm = NAN;
    expect_error((uint8_t *)&motor, 12, RESPONSE_INVALID_ARGUMENT);
    assert(calls == 0); // Rejected requests must never reach motor logic.
    uint8_t ack[] = {TOGGLE_STATUS_LED_STATE, 42, 0};
    expect_error(ack, sizeof(ack), RESPONSE_OK);
    assert(calls == 1);
    mode = 1;
    protocol_dispatch(ack, sizeof(ack), handle, send_reply);
    const uint8_t expected[] = {5, TOGGLE_STATUS_LED_STATE, 42, 0, 0xff, 0};
    assert(received_length == sizeof(expected));
    assert(memcmp(received, expected, sizeof(expected)) == 0);
    mode = 2;
    expect_error(ack, sizeof(ack), RESPONSE_MOTOR_OWNED);
    mode = 3;
    expect_error(ack, sizeof(ack), RESPONSE_INTERNAL_ERROR);
    mode = 0;

    uint8_t init[] = {INIT, 0x34, 0x12, 1, 1, 0, 0, 2, 0, 0, 3};
    protocol_dispatch(init, sizeof(init), handle, send_reply);
    assert(received_length == 19);
    assert(received[0] == 18 && received[1] == INIT);
    init[7] = 1;
    protocol_dispatch(init, sizeof(init), handle, send_reply);
    assert(received_length == 6 && received[1] == KINISI_MESSAGE_ERROR);
    assert(received[4] == INIT && received[5] == RESPONSE_INCOMPATIBLE_PROTOCOL);
    uint8_t error_request[] = {KINISI_MESSAGE_ERROR, 42, 0};
    expect_error(error_request, sizeof(error_request), RESPONSE_UNKNOWN_COMMAND);

    // Every possible split of an INIT frame, followed by another command.
    char framed[sizeof(init) + 5];
    framed[0] = sizeof(init);
    memcpy(framed + 1, init, sizeof(init));
    framed[sizeof(init) + 1] = 3;
    framed[sizeof(init) + 2] = TOGGLE_STATUS_LED_STATE;
    framed[sizeof(init) + 3] = 42;
    framed[sizeof(init) + 4] = 0;
    for (unsigned split = 0; split <= sizeof(framed); ++split) {
        message_queue_t queue;
        init_queue(&queue);
        assert(enqueue_multi(&queue, framed, split));
        assert(enqueue_multi(&queue, framed + split, sizeof(framed) - split));
        char data[256]; int length;
        dequeue(&queue, data, &length);
        assert(length == sizeof(init) && !memcmp(data, init, sizeof(init)));
        dequeue(&queue, data, &length);
        assert(length == 3 && data[0] == TOGGLE_STATUS_LED_STATE);
        assert(is_queue_empty(&queue));
    }
    message_queue_t queue;
    init_queue(&queue);
    char maximum[256] = {0}; maximum[0] = (char)255;
    for (unsigned i = 0; i < 256; ++i) assert(enqueue_multi(&queue, maximum + i, 1));
    char data[256]; int length;
    dequeue(&queue, data, &length);
    assert(length == 255);
    char empty[] = {0};
    assert(enqueue_multi(&queue, empty, 1));
    dequeue(&queue, data, &length);
    assert(length == 0);
    for (unsigned i = 0; i < MESSAGE_QUEUE_MAX_SIZE; ++i) assert(enqueue_multi(&queue, empty, 1));
    assert(!enqueue_multi(&queue, framed, sizeof(framed)));
    assert(queue.incomplete_count == 0 && queue.count == MESSAGE_QUEUE_MAX_SIZE);
    return 0;
}
