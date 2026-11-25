#pragma once
#include <stdbool.h>
#include <stdint.h>

// Called by the command-manager task during startup and command processing.
// The payload is an explicitly versioned, fixed-length configuration only.
#define CONFIG_STORE_MAX_PAYLOAD 192U
bool config_store_load(void *data, uint16_t length);
// Read the newest complete record across configuration payload revisions.
uint16_t config_store_load_latest(void *data, uint16_t capacity);
bool config_store_save(const void *data, uint16_t length);
