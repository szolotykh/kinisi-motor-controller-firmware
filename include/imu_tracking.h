#pragma once
#include "board_revision.h"
#if BOARD_HAS_IMU_TRACKING
#include <stdbool.h>
#include "hw_imu.h"
typedef hw_imu_status_t (*imu_tracking_reader_t)(uint64_t now, hw_imu_sample_t *raw, uint64_t *accel_us,
                                                 uint64_t *gyro_us);
void imu_tracking_install_producer(bool (*start)(void), void (*begin)(uint32_t), imu_tracking_reader_t read);
uint32_t imu_tracking_epoch(void);
// The fusion reader validates its own epoch. Starting or stopping the passive
// observer must never reset the FIFO underneath an active fusion controller.
static inline uint32_t imu_acquisition_epoch(uint32_t fusion_epoch)
{
    return fusion_epoch ? fusion_epoch : imu_tracking_epoch();
}
void imu_tracking_inhibit(void);
#endif
