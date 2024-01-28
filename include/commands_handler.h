//------------------------------------------------------------
// File name: commands_handler.h
//------------------------------------------------------------

#pragma once

#include "commands.h"

void command_handler(controller_command_t* cmd, void (*command_callback)(uint8_t*, uint8_t));