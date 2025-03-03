//------------------------------------------------------------
// File name: commands_manager.h
//------------------------------------------------------------
#pragma once

#include <message_queue.h>
#include <stdint.h>

extern message_queue_t CommandQueue;
extern message_queue_t I2CCommandQueue;

void command_callback_usb(uint8_t* resonse, uint8_t data_len);
void command_callback_i2c(uint8_t* resonse, uint8_t data_len);

void commands_manager_start(void);

