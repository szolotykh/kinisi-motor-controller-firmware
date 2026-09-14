//------------------------------------------------------------
// File name: protocol.h
// Description: Declare shared protocol dispatch, error constants, and message-ID sequencing.
//------------------------------------------------------------
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "commands.h"

// Error codes are generated from commands.json into commands.h.
#define RESPONSE_OK 0U

typedef void (*protocol_send_fn)(uint8_t *, uint8_t);
typedef uint8_t (*protocol_handler_fn)(controller_command_t *, protocol_send_fn);

// Sender-side sequence helper: start with previous=0. Skip outstanding IDs
// before sending. Responses echo the initiating ID rather than incrementing.
/**
 * @brief Advance the initiating message ID, wrapping 65535 to 1.
 * @param previous Last allocated ID; use zero to allocate the first ID.
 * @return Next nonzero ID; the caller must avoid IDs still outstanding.
 */
static inline uint16_t protocol_next_message_id(uint16_t previous)
{
    return previous == UINT16_MAX ? 1U : (uint16_t)(previous + 1U);
}

// Called serially by the command task. Transport send functions must copy
// their data before returning, as the existing USB and I2C callbacks do.
/**
 * @brief Validate one decoded request and emit its correlated reply.
 * @param request Frame bytes after the length prefix; may be null for an invalid frame.
 * @param length Number of request bytes, excluding the length prefix.
 * @param handler Command executor called only after wire/argument validation.
 * @param send Callback that copies the complete framed response before returning.
 * @note Serialized command-task use only; response capture uses shared static state.
 */
void protocol_dispatch(const uint8_t *request, size_t length,
                       protocol_handler_fn handler, protocol_send_fn send);
