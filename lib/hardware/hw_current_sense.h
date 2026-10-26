#pragma once
#include <stdint.h>

typedef enum {
    HW_CURRENT_SENSE_OK,
    HW_CURRENT_SENSE_UNAVAILABLE,
    HW_CURRENT_SENSE_OUT_OF_RANGE,
    HW_CURRENT_SENSE_IO_ERROR
} hw_current_sense_status_t;

// ADC1 belongs to this driver. Initialize before the scheduler, read only from
// the serialized command task. Neither operation enables a motor driver.
void hw_current_sense_init(void);

// Unsigned filtered high-side current estimate in amperes, not signed winding
// or battery current. PWM decay, mirror accuracy and nominal VREF affect it.
// Output changes only on success. A saturated ADC is reported as out of range.
hw_current_sense_status_t hw_current_sense_read(uint8_t motor_index, double *amps);
