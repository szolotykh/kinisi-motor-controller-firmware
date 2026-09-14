//------------------------------------------------------------
// File name: protocol.c
// Description: Validate protocol frames and encode correlated payload, ACK, or ERROR replies.
//------------------------------------------------------------
#include "protocol.h"
#include <string.h>

// The existing transport callback length is uint8_t, so the entire frame
// must fit in 255 bytes: length + command + message ID + up to 251 payload bytes.
static uint8_t reply[255];
static uint8_t payload_length;
static uint8_t capture_failed;
static uint8_t captured;

/**
 * @brief Copy one command payload into the shared reply buffer.
 * @note Duplicate captures or oversized/null payloads produce INTERNAL_ERROR.
 */
static void capture_payload(uint8_t *data, uint8_t length)
{
    if (captured || length > 251U || (length && !data)) {
        capture_failed = 1;
        return;
    }
    captured = 1;
    payload_length = length;
    if (length) memcpy(reply + 4, data, length);
}

/**
 * @brief Validate one decoded request and emit its correlated reply.
 * @param request Frame bytes after the length prefix; may be null for an invalid frame.
 * @param length Number of request bytes, excluding the length prefix.
 * @param handler Command executor called only after wire/argument validation.
 * @param send Callback that copies the complete framed response before returning.
 * @note Serialized command-task use only; response capture uses shared static state.
 */
void protocol_dispatch(const uint8_t *request, size_t length,
                       protocol_handler_fn handler, protocol_send_fn send)
{
    if (!send) return;
    payload_length = 0;
    capture_failed = 0;
    captured = 0;
    const uint8_t command_id = (request && length) ? request[0] : 0;
    const uint16_t message_id = (request && length >= 3) ?
        (uint16_t)request[1] | ((uint16_t)request[2] << 8) : 0;
    uint8_t status;
    const size_t expected = command_request_size(command_id);
    if (!request || length < 3) {
        status = RESPONSE_INVALID_LENGTH;
    } else if (!expected) {
        status = RESPONSE_UNKNOWN_COMMAND;
    } else if (length != expected) {
        status = RESPONSE_INVALID_LENGTH;
    } else if (message_id == 0) {
        status = RESPONSE_INVALID_ARGUMENT;
    } else {
        // Never cast a short or unaligned transport buffer into a command.
        controller_command_t command = {0};
        memcpy(&command, request, length);
        status = command_arguments_valid(&command) ? RESPONSE_OK : RESPONSE_INVALID_ARGUMENT;
        if (status == RESPONSE_OK) {
            status = handler ? handler(&command, capture_payload) : RESPONSE_INTERNAL_ERROR;
        }
    }
    if (capture_failed) {
        status = RESPONSE_INTERNAL_ERROR;
        payload_length = 0;
    }
    reply[2] = (uint8_t)message_id;
    reply[3] = (uint8_t)(message_id >> 8);
    if (status != RESPONSE_OK) {
        reply[0] = 5;
        reply[1] = KINISI_MESSAGE_ERROR;
        reply[4] = command_id;
        reply[5] = status;
        send(reply, 6);
    } else {
        reply[0] = payload_length + 3U; // Bytes following length.
        reply[1] = command_id;
        send(reply, payload_length + 4U);
    }
}
