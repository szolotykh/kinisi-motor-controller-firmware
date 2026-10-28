#pragma once
#include "hw_motor.h"
#include "hw_motor_faults.h"
#include "hw_motor_current_limit.h"

// Configure all control pins safely before any driver is woken.
bool hw_drv8873_prepare(void);
uint32_t hw_drv8873_event_sequence(motorIndex index);
// Wake and verify PH/EN configuration, leaving the bridge disabled.
bool hw_drv8873_initialize(motorIndex index);
// nFAULT is active low. Invalid indices are treated as faulted.
bool hw_drv8873_fault_active(motorIndex index);
// Read-only diagnostics; unavailable registers are reported with status_valid=false.
void hw_drv8873_read_faults(motorIndex index, hw_motor_faults_t *faults);
// Clear through IC3, preserving its other bits. Requires an awake driver; leaves DISABLE asserted.
bool hw_drv8873_clear_faults(motorIndex index, hw_motor_faults_t *faults);
bool hw_drv8873_get_current_limit(motorIndex index, hw_motor_current_limit_t *setting);
bool hw_drv8873_set_current_limit(motorIndex index, const hw_motor_current_limit_t *setting);
