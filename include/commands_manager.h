//------------------------------------------------------------
// File name: commands_manager.h
//------------------------------------------------------------
#pragma once

#include <message_queue.h>
#include <cmsis_os.h>

extern message_queue_t CommandQueue;

typedef struct commands_manager
{
    osThreadId_t threadHandler;
} commands_manager_t;

void commands_manager_init(commands_manager_t* commands_manager);