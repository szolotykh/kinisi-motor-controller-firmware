//------------------------------------------------------------
// File name: commands_handler.h
// Description: Declare validated command execution and payload response callbacks.
//------------------------------------------------------------

#pragma once

#include "commands.h"

/**
 * @brief Execute a validated command after checking its resource prerequisites.
 * @param cmd Request with a valid command ID, payload length, and argument ranges.
 * @param command_callback Receives response payload bytes and must copy them immediately.
 * @return RESPONSE_OK for accepted operations, or a generated protocol error code.
 * @note Called serially inside a connection dispatch; ACK/error framing is handled above.
 */
uint8_t command_handler(controller_command_t* cmd, void (*command_callback)(uint8_t*, uint8_t));
