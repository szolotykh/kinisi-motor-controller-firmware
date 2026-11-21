#pragma once

#include <stdint.h>

// Called once at boot, after hardware setup and before starting either transport.
void controller_configuration_restore(void);
uint8_t controller_configuration_save(void);
uint8_t controller_configuration_read(uint8_t field, uint8_t index, double *value);
