//------------------------------------------------------------
// File name: command_requirements.h
// Description: Declare resource queries used to validate command prerequisites.
//------------------------------------------------------------
#pragma once
#include "protocol.h"

// Queries are evaluated only for the resource used by the given command.
typedef struct {
    uint8_t (*motor_owned)(uint8_t);
    uint8_t (*motor_initialized)(uint8_t);
    uint8_t (*encoder_initialized)(uint8_t);
    uint8_t (*controller_running)(uint8_t);
    uint8_t (*platform_initialized)(void);
    uint8_t (*platform_controller_running)(void);
} command_resources_t;

/**
 * @brief Check command resource availability, giving motor ownership precedence.
 * @param cmd Decoded request whose length, index ranges, and arguments are valid.
 * @param r Complete resource-query interface; queries must not mutate hardware.
 * @return RESPONSE_OK or the first applicable prerequisite error.
 */
uint8_t command_requirements_check(const controller_command_t *, const command_resources_t *);
