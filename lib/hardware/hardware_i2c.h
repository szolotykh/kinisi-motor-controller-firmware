//------------------------------------------------------------
// File name: hardware_i2c.h
//------------------------------------------------------------

#pragma once
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stm32f4xx_hal_i2c.h>

typedef enum {
  AWAITING_SIZE,
  AWAITING_MESSAGE,
  LISTENING
} i2c_receive_state_t;

uint8_t i2c_message_buffer[256];

void initialize_external_i2c(void);
void send_external_i2c(uint8_t* data, uint16_t data_len);
