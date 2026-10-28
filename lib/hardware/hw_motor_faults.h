#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool sleep_requested;
    bool status_valid;
    bool fault_active;
    bool overtemperature_warning;
    bool undervoltage;
    bool charge_pump_undervoltage;
    bool overcurrent;
    bool overtemperature_shutdown;
    bool open_load;
    bool current_regulation;
} hw_motor_faults_t;

// False means no reporting hardware. Reads never change motor state.
uint32_t hw_motor_fault_event_sequence(uint8_t index);
bool hw_motor_read_faults(uint8_t index, hw_motor_faults_t *faults);
// Clear latched faults on an initialized motor, preserving configuration and leaving it stopped.
// False means absent/uninitialized hardware or an unverified clear/readback.
bool hw_motor_clear_faults(uint8_t index, hw_motor_faults_t *faults);
