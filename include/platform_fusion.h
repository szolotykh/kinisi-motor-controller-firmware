#pragma once
#include "board_revision.h"
#include "platform_types.h"
#include <stdint.h>
#include <stdbool.h>

#if BOARD_HAS_FUSION_EXPERIMENT
#include "hw_imu.h"
// A batch token prevents a pending hardware read from crossing a reset/configuration.
uint32_t platform_fusion_epoch(void);
void platform_fusion_install_ingress_fence(void (*fence)(uint32_t *));
void platform_fusion_dequeued(void);
void platform_fusion_observe(uint32_t epoch, uint64_t previous_us, uint64_t encoder_us,
                             platform_odometry_t delta, platform_odometry_t pose, bool valid, bool stationary, bool moved);
typedef hw_imu_status_t (*platform_imu_reader_t)(uint32_t epoch, uint64_t at, hw_imu_sample_t *,
                                                  uint64_t *, uint64_t *);
void platform_fusion_install_imu(platform_imu_reader_t reader, bool (*start)(void));
uint32_t platform_fusion_imu_epoch(void);
void platform_fusion_discontinuity(bool reset_origin);
void platform_fusion_driver_transition(bool sleeping);
void platform_fusion_observed_imu_loss(uint8_t fault);
uint8_t platform_fusion_configure(uint8_t mode, double yaw);
uint8_t platform_fusion_read(uint8_t command, uint8_t kind, void (*reply)(uint8_t *, uint8_t));
uint8_t platform_fusion_velocity(platform_velocity_t target);
#endif
