#pragma once
#include "stm32f4xx_hal.h"
void initialize_external_i2c(void);
uint8_t try_send_external_i2c(uint8_t *data, uint16_t length);
void mark_pending_external_i2c_requests(void);
