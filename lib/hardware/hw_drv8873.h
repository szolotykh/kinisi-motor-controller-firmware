#pragma once
#include "hw_motor.h"

// Configure all control pins safely before any driver is woken.
bool hw_drv8873_prepare(void);
// Wake and verify PH/EN configuration, leaving the bridge disabled.
bool hw_drv8873_initialize(motorIndex index);
// nFAULT is active low. Invalid indices are treated as faulted.
bool hw_drv8873_fault_active(motorIndex index);
