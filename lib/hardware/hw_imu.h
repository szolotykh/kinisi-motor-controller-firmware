#pragma once
#include <stdint.h>

typedef enum {
    HW_IMU_OK,
    HW_IMU_UNAVAILABLE,
    HW_IMU_NOT_READY,
    HW_IMU_IO_ERROR,
    HW_IMU_STALE
} hw_imu_status_t;

typedef struct {
    double acceleration_x, acceleration_y, acceleration_z; // m/s^2, includes gravity
    double angular_velocity_x, angular_velocity_y, angular_velocity_z; // rad/s
} hw_imu_sample_t;

void hw_imu_init(void);
// Latest independent accel/gyro samples in sensor axes; no fusion or axis remapping.
// Call from the serialized command task. Output is unchanged on failure.
hw_imu_status_t hw_imu_read(uint8_t index, hw_imu_sample_t *sample);

// Independent source times (board us). A cached payload retains its acquisition
// times. Zero means the producer has yet to observe an event for that channel.
hw_imu_status_t hw_imu_read_observation(uint8_t index, hw_imu_sample_t *sample,
                                         uint64_t *accel_us, uint64_t *gyro_us);

// Physical BMI088 backend: a single producer polls both channels. The reader
// selects a verified pair whose complete acquisition windows precede encoder_us.
void hw_imu_poll_observations(uint32_t epoch);
hw_imu_status_t hw_imu_read_before(uint32_t epoch, uint64_t encoder_us, hw_imu_sample_t *sample,
                                    uint64_t *accel_us, uint64_t *gyro_us);
