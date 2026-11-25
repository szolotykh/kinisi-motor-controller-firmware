#pragma once
#include "board_revision.h"
#if BOARD_HAS_FEEDBACK_SOURCE_SELECTOR
#include "commands.h"
#include "platform_types.h"
#include <stdbool.h>
#include <stdint.h>

// One producer owns each frame and its mutex. The board-wide selection token
// changes on source switches and source discontinuities. Neither frame nor
// token is retained in the configuration journal.
typedef struct {
    platform_odometry_t anchor;
    uint32_t token;
} feedback_frame_t;

typedef struct {
    platform_odometry_t pose, twist;
    uint64_t stamp;
    uint32_t sequence, token;
    uint8_t reason, detail;
} feedback_snapshot_t;

uint8_t feedback_requested(void);
void feedback_restore_preference(uint8_t source);
void feedback_invalidate(uint8_t source);
void feedback_anchor(feedback_frame_t *frame, uint8_t source, uint64_t stamp, platform_odometry_t pose);
void feedback_snapshot(feedback_snapshot_t *out, const feedback_frame_t *frame, uint8_t source,
                       uint32_t sequence, uint64_t stamp, platform_odometry_t pose, platform_odometry_t twist);
void odometry_feedback_sample(feedback_snapshot_t *out);
bool odometry_feedback_at_rest(uint64_t now);
uint8_t passive_fusion_select(bool enable);
uint8_t passive_fusion_mount(double yaw);
feedback_fusion_mount passive_fusion_get_mount(void);
uint32_t passive_fusion_epoch(void);
uint32_t passive_fusion_acquisition_epoch(void);
void passive_fusion_observe(uint32_t epoch, uint64_t previous, uint64_t at, platform_odometry_t delta,
                            bool valid, bool stationary, bool moved);
void passive_fusion_sample(feedback_snapshot_t *out);
void passive_fusion_reset(void);
#if BOARD_HAS_FEEDBACK_SOURCE_HEALTH
#include "hw_imu.h"
typedef hw_imu_status_t (*feedback_imu_reader_t)(uint8_t, hw_imu_sample_t *, uint64_t *, uint64_t *);
void feedback_health_install(feedback_imu_reader_t reader, bool (*encoder_sampling)(uint8_t));
void feedback_health_poll(void);
uint32_t feedback_health_raw_epoch(void);
void feedback_health_power_transition(void);
void odometry_counter_transition(uint8_t changed);
bool feedback_counters_available(void);
void feedback_guard_platform_control(void);
void feedback_install_encoder_fence(void (*fence)(uint8_t),uint8_t (*dequeued)(void));
void feedback_health_reset_comparison(void);
void feedback_health_mount(double yaw);
void feedback_health_observe(uint64_t at, uint64_t previous, platform_odometry_t delta, bool valid, bool stationary);
bool feedback_encoder_sampling(uint8_t index);
void odometry_health_read(feedback_source_health *out);
void passive_health_fault(uint8_t reason,uint8_t detail);
void passive_health_read(feedback_source_health *out);
#endif
#endif
