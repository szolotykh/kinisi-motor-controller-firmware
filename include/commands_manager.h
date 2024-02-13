//------------------------------------------------------------
// File name: commands_manager.h
//------------------------------------------------------------
#pragma once

#include <message_queue.h>
#include <cmsis_os.h>

extern message_queue_t CommandQueue;
extern message_queue_t I2CCommandQueue;

typedef struct commands_manager
{
    osThreadId_t threadHandler;
} commands_manager_t;

void command_callback_usb(uint8_t* resonse, uint8_t data_len);
void command_callback_i2c(uint8_t* resonse, uint8_t data_len);

void commands_manager_init(commands_manager_t* commands_manager);

