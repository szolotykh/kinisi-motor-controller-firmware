#pragma once
#include <stdint.h>

typedef enum {
    HW_TEMPERATURE_OK,
    HW_TEMPERATURE_UNAVAILABLE,
    HW_TEMPERATURE_NOT_READY,
    HW_TEMPERATURE_IO_ERROR
} hw_temperature_status_t;

// Call at startup before the scheduler. Missing sensors do not prevent boot.
// Subsequent reads retry failed initialization and recover a failed I2C bus.
// Serialized command-task use only; all temperature sensors share this bus.
void hw_temperature_init(void);

// Output is in degrees Celsius and is only modified on success. No conversion wait.
// After transfer failure, a later read may clear the bus with a 50 ms time budget.
hw_temperature_status_t hw_temperature_read(uint8_t index, double *celsius);
