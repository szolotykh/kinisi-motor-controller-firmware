//------------------------------------------------------------
// File name: command_requirements.c
// Description: Reject commands whose required motor, encoder, or platform is unavailable.
//------------------------------------------------------------
#include "command_requirements.h"

/**
 * @brief Check command resource availability, giving motor ownership precedence.
 * @param cmd Decoded request whose length, index ranges, and arguments are valid.
 * @param r Complete resource-query interface; queries must not mutate hardware.
 * @return RESPONSE_OK or the first applicable prerequisite error.
 */
uint8_t command_requirements_check(const controller_command_t *cmd, const command_resources_t *r)
{
    // Keep ownership precedence consistent across all direct motor operations.
    uint8_t index;
    switch (cmd->commandType) {
    case INITIALIZE_MOTOR: index = cmd->properties.initialize_motor.motor_index; break;
    case SET_MOTOR_SPEED: index = cmd->properties.set_motor_speed.motor_index; break;
    case STOP_MOTOR: index = cmd->properties.stop_motor.motor_index; break;
    case BRAKE_MOTOR: index = cmd->properties.brake_motor.motor_index; break;
    case INITIALIZE_MOTOR_CONTROLLER: index = cmd->properties.initialize_motor_controller.motor_index; break;
    case DELETE_MOTOR_CONTROLLER: index = cmd->properties.delete_motor_controller.motor_index; break;
    case SET_MOTOR_TARGET_SPEED: index = cmd->properties.set_motor_target_speed.motor_index; break;
    case RESET_MOTOR_CONTROLLER: index = cmd->properties.reset_motor_controller.motor_index; break;
    default: index = UINT8_MAX; break;
    }
    if (index != UINT8_MAX && r->motor_owned(index)) return RESPONSE_MOTOR_OWNED;

    switch (cmd->commandType) {
    case SET_MOTOR_SPEED:
        if (!r->motor_initialized(index)) return RESPONSE_MOTOR_NOT_INITIALIZED;
        break;
    case SET_MOTOR_TARGET_SPEED:
        if (!r->controller_running(index)) return RESPONSE_CONTROLLER_NOT_INITIALIZED;
        break;
    case GET_ENCODER_VALUE:
        if (!r->encoder_initialized(cmd->properties.get_encoder_value.encoder_index)) return RESPONSE_ENCODER_NOT_INITIALIZED;
        break;
    case START_ENCODER_ODOMETRY:
        if (!r->encoder_initialized(cmd->properties.start_encoder_odometry.encoder_index)) return RESPONSE_ENCODER_NOT_INITIALIZED;
        break;
    case START_PLATFORM_ODOMETRY: case SET_PLATFORM_VELOCITY: case START_PLATFORM_CONTROLLER:
        if (!r->platform_initialized()) return RESPONSE_PLATFORM_NOT_INITIALIZED;
        break;
    case SET_PLATFORM_TARGET_VELOCITY:
        if (!r->platform_initialized()) return RESPONSE_PLATFORM_NOT_INITIALIZED;
        if (!r->platform_controller_running()) return RESPONSE_CONTROLLER_NOT_INITIALIZED;
        break;
    default: break;
    }
    return RESPONSE_OK;
}
