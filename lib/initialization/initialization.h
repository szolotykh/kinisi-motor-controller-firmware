//------------------------------------------------------------
// File name: initialization.h
// Description: Declare pure INIT validation, identity construction, and client capabilities.
//------------------------------------------------------------
#pragma once
#include "commands.h"
#include <stddef.h>
#include "protocol.h"

#define INIT_STATUS_OK RESPONSE_OK
#define INIT_STATUS_INCOMPATIBLE RESPONSE_INCOMPATIBLE_PROTOCOL
#define INIT_STATUS_INVALID RESPONSE_INVALID_ARGUMENT
#define CLIENT_CAP_WALL_CLOCK 1U
#define CLIENT_CAP_SUBSCRIPTIONS 2U
#define CLIENT_CAP_SUPPORTED_MASK (CLIENT_CAP_WALL_CLOCK | CLIENT_CAP_SUBSCRIPTIONS)

// No motor, telemetry, clock or session side effects. Safe to retry.
/**
 * @brief Construct board/protocol identity without changing connection or motor state.
 * @return Packed identity payload using the selected board and build definitions.
 */
init_response initialization_response(void);
/**
 * @brief Validate protocol compatibility and supported client metadata.
 * @param command Decoded INIT request, already checked for complete length.
 * @return RESPONSE_OK or an INIT compatibility/argument error.
 */
uint8_t initialization_validate(const controller_command_t *command);
/**
 * @brief Check the INIT size, including command and message ID but excluding length.
 * @return Nonzero when the request contains exactly the expected fields.
 */
int initialization_request_length_valid(size_t length);
