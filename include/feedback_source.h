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
#endif
