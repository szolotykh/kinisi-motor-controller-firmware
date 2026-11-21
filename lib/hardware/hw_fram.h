#pragma once
#include <stdbool.h>
#include <stdint.h>

// Board v0.4 U603. The command-manager task serializes all sensor I2C traffic
// after startup, including USB, external I2C and periodic temperature reads.
bool hw_fram_read(uint16_t address, void *data, uint16_t length);
bool hw_fram_write(uint16_t address, const void *data, uint16_t length);
