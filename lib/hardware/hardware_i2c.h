//------------------------------------------------------------
// File name: hardware_i2c.h
// Description: Declare the external I2C transport and its nonblocking transmit interface.
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

/**
 * @brief Configure I2C2 as a listening slave and reset receive/transmit state.
 */
void initialize_external_i2c(void);

// Nonblocking send: succeeds only during an outstanding master read.
/**
 * @brief Copy a frame and start an interrupt-driven transfer when a master is reading.
 * @param data Complete length-prefixed frame to copy.
 * @param data_len Frame size in bytes.
 * @return Nonzero if accepted; zero when busy, unavailable, oversized, or HAL rejects it.
 * @note Does not wait for a master transaction; preserves the caller interrupt mask.
 */
uint8_t try_send_external_i2c(uint8_t* data, uint16_t data_len);
