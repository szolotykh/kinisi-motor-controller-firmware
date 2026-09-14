//------------------------------------------------------------
// File name: initialization.c
// Description: Validate INIT metadata and construct the build-selected board identity.
//------------------------------------------------------------
#include "initialization.h"
#include "board_revision.h"

#ifndef KINISI_BUILD_ID_HIGH
#define KINISI_BUILD_ID_HIGH 0U
#define KINISI_BUILD_ID_LOW 0U
#endif

/**
 * @brief Check the INIT size, including command and message ID but excluding length.
 * @return Nonzero when the request contains exactly the expected fields.
 */
int initialization_request_length_valid(size_t length)
{
    return length == 3U + sizeof(((controller_command_t *)0)->properties.init);
}

/**
 * @brief Construct board/protocol identity without changing connection or motor state.
 * @return Packed identity payload using the selected board and build definitions.
 */
init_response initialization_response(void)
{
    init_response response = {0};
    response.board_model = 1; // Kinisi motor controller
    response.board_major = KINISI_BOARD_VERSION_MAJOR;
    response.board_minor = KINISI_BOARD_VERSION_MINOR;
    response.board_patch = KINISI_BOARD_VERSION_PATCH;
    response.protocol_major = KINISI_PROTOCOL_MAJOR;
    response.protocol_minor = KINISI_PROTOCOL_MINOR;
    response.protocol_patch = KINISI_PROTOCOL_PATCH;
    response.firmware_build_high = KINISI_BUILD_ID_HIGH;
    response.firmware_build_low = KINISI_BUILD_ID_LOW;
    return response;
}

/**
 * @brief Validate protocol compatibility and supported client metadata.
 * @param command Decoded INIT request, already checked for complete length.
 * @return RESPONSE_OK or an INIT compatibility/argument error.
 */
uint8_t initialization_validate(const controller_command_t *command)
{
    if (command->properties.init.protocol_major != KINISI_PROTOCOL_MAJOR ||
        command->properties.init.protocol_minor > KINISI_PROTOCOL_MINOR) {
        return INIT_STATUS_INCOMPATIBLE;
    } else if (command->properties.init.sdk_type > 3U ||
               (command->properties.init.client_capabilities & ~CLIENT_CAP_SUPPORTED_MASK) != 0U) {
        return INIT_STATUS_INVALID;
    }
    return INIT_STATUS_OK;
}
