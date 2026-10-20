#pragma once
#include <stdint.h>

typedef enum {
    HW_IMU_OK,
    HW_IMU_UNAVAILABLE,
    HW_IMU_NOT_READY,
    HW_IMU_IO_ERROR
} hw_imu_status_t;

typedef struct {
    double acceleration_x, acceleration_y, acceleration_z; // m/s^2, includes gravity
    double angular_velocity_x, angular_velocity_y, angular_velocity_z; // rad/s
} hw_imu_sample_t;

void hw_imu_init(void);
// Latest independent accel/gyro samples in sensor axes; no fusion or axis remapping.
// Call from the serialized command task. Output is unchanged on failure.
hw_imu_status_t hw_imu_read(uint8_t index, hw_imu_sample_t *sample);
