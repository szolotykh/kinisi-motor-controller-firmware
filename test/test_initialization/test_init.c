//------------------------------------------------------------
// File name: test_init.c
// Description: Verify INIT identity and compatibility across board revisions.
//------------------------------------------------------------
#include <assert.h>
#include <string.h>
#include "initialization.h"
#include "board_revision.h"

/**
 * @brief Run this file's assertions and return zero when all checks pass.
 */
int main(void)
{
    // Known wire packet, excluding the existing one-byte framing length.
    const unsigned char packet[] = {0x70, 0x34, 0x12, 1, 2, 3, 4, 2, 0, 7, 1};
    controller_command_t command = {0};
    memcpy(&command, packet, sizeof(packet));
    assert(command.commandType == INIT);
    assert(command.message_id == 0x1234);
    assert(command.properties.init.protocol_patch == 7);
    assert(command.properties.init.client_capabilities == 1);
    assert(command.properties.init.client_version_major == 2);
    assert(command.properties.init.client_version_minor == 3);
    assert(command.properties.init.client_version_patch == 4);
    assert(initialization_request_length_valid(sizeof(packet)));
    assert(!initialization_request_length_valid(0));
    assert(!initialization_request_length_valid(sizeof(packet) - 1));
    assert(!initialization_request_length_valid(sizeof(packet) + 1));

    init_response response = initialization_response();
    const unsigned char expected[] = {
        1, 0, KINISI_BOARD_VERSION_MINOR, 0,
        2, 0, 0, 0x78, 0x56, 0x34, 0x12, 0xef, 0xcd, 0xab, 0x90,
    };
    assert(sizeof(response) == sizeof(expected));
    assert(memcmp(&response, expected, sizeof(expected)) == 0);
    init_response retry = initialization_response();
    assert(memcmp(&response, &retry, sizeof(response)) == 0);

    command.properties.init.sdk_type = 3; // Arduino, no wall clock
    command.properties.init.client_capabilities = 0;
    response = initialization_response();
    assert(initialization_validate(&command) == INIT_STATUS_OK);
    command.properties.init.protocol_major = 3;
    response = initialization_response();
    assert(initialization_validate(&command) == INIT_STATUS_INCOMPATIBLE);
    assert(response.board_minor == KINISI_BOARD_VERSION_MINOR);
    command.properties.init.protocol_major = 2;
    command.properties.init.protocol_minor = 6;
    assert(initialization_validate(&command) == INIT_STATUS_INCOMPATIBLE);
    command.properties.init.protocol_minor = 0;
    assert(initialization_validate(&command) == INIT_STATUS_OK);
    command.properties.init.sdk_type = 255;
    assert(initialization_validate(&command) == INIT_STATUS_INVALID);
    command.properties.init.sdk_type = 0;
    for (unsigned int capabilities = 0; capabilities <= 3; ++capabilities) {
        command.properties.init.client_capabilities = capabilities;
        assert(initialization_validate(&command) == INIT_STATUS_OK);
    }
    command.properties.init.client_capabilities = 4;
    assert(initialization_validate(&command) == INIT_STATUS_INVALID);
    command.properties.init.client_capabilities = 7;
    assert(initialization_validate(&command) == INIT_STATUS_INVALID);
    return 0;
}
