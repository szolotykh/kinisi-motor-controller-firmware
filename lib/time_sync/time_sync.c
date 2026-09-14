//------------------------------------------------------------
// File name: time_sync.c
// Description: Estimate the host clock offset and schedule bounded synchronization bursts.
//------------------------------------------------------------
#include "time_sync.h"
#include <string.h>
#include <limits.h>

/**
 * @brief Reset clock setup for INIT while preserving the controller message-ID sequence.
 * @param clock Zero-initialized or previously initialized clock state.
 * @param wall_clock True to synchronize Unix time; false for immediately ready uptime.
 * @param now Current monotonic board time in microseconds.
 */
void time_sync_init(time_sync_t *clock, bool wall_clock, uint64_t now)
{
    uint16_t sequence = clock->sequence; // Don't reuse an outstanding old ID on re-INIT.
    memset(clock, 0, sizeof(*clock));
    clock->sequence = sequence;
    clock->initialized = true;
    clock->mode = wall_clock ? CLOCK_MODE_WALL : CLOCK_MODE_UPTIME;
    clock->ready = !wall_clock;
    clock->interval_us = TIME_SYNC_DEFAULT_INTERVAL_US;
    clock->next_us = now;
}

/**
 * @brief Expire an unanswered request and start a scheduled burst when due.
 * @param clock Mutable clock/session state.
 * @param now Current monotonic board time in microseconds.
 * @return True if another timing request should be attempted now.
 */
bool time_sync_due(time_sync_t *clock, uint64_t now)
{
    if (!clock->initialized || clock->mode != CLOCK_MODE_WALL) return false;
    if (clock->pending && now - clock->sent_us >= TIME_SYNC_TIMEOUT_US) clock->pending = false;
    if (clock->pending) return false;
    if (!clock->burst) {
        if (now < clock->next_us) return false;
        clock->burst = true;
        clock->attempts = 0;
        clock->have_best = false;
    }
    return clock->attempts < TIME_SYNC_BURST_SIZE;
}

/**
 * @brief Record an accepted timing transmission and consume one burst attempt.
 * @param clock Clock state with an active burst.
 * @param id Nonzero controller-allocated request ID.
 * @param now Monotonic microseconds captured immediately before the successful send.
 * @note Do not call for busy or failed transport sends.
 */
void time_sync_sent(time_sync_t *clock, uint16_t id, uint64_t now)
{
    clock->pending = true;
    clock->pending_id = id;
    clock->sequence = id;
    clock->sent_us = now;
    ++clock->attempts;
}

/**
 * @brief Validate a matching reply and retain the lowest-delay sample in the burst.
 * @param clock Clock state with an outstanding timing request.
 * @param id Echoed controller request ID.
 * @param h2 Host receive timestamp in Unix microseconds.
 * @param h3 Host send timestamp in Unix microseconds.
 * @param now Controller receive timestamp in monotonic microseconds.
 * @return True for an accepted sample; false for mismatched IDs or invalid timing.
 * @note A matching reply consumes the pending request even when its sample is invalid.
 */
bool time_sync_receive(time_sync_t *clock, uint16_t id, uint64_t h2, uint64_t h3, uint64_t now)
{
    if (!clock->pending || id != clock->pending_id) return false;
    clock->pending = false;
    if (now < clock->sent_us || now - clock->sent_us >= TIME_SYNC_TIMEOUT_US ||
        h3 < h2 || h2 > INT64_MAX || h3 > INT64_MAX || now > INT64_MAX) return false;
    uint64_t roundtrip = now - clock->sent_us;
    uint64_t processing = h3 - h2;
    if (processing > roundtrip) return false;
    uint64_t delay = roundtrip - processing;
    if (delay > TIME_SYNC_MAX_DELAY_US) return false;
    // Midpoint form avoids overflowing a sum of two Unix-time offsets.
    uint64_t host_mid = h2 + processing / 2;
    uint64_t local_mid = clock->sent_us + roundtrip / 2;
    int64_t offset = (int64_t)host_mid - (int64_t)local_mid;
    if (!clock->have_best || delay < clock->best_delay_us) {
        clock->have_best = true;
        clock->best_delay_us = delay;
        clock->best_offset_us = offset;
    }
    return true;
}

/**
 * @brief Commit a completed burst and schedule the next periodic attempt.
 * @param clock Mutable clock mapping and burst state.
 * @param now Current monotonic board time in microseconds.
 * @return 1 for success, -1 for a failed burst, or 0 while still running.
 * @note Failure preserves any previously valid mapping.
 */
int time_sync_finish(time_sync_t *clock, uint64_t now)
{
    if (!clock->burst || clock->pending || clock->attempts < TIME_SYNC_BURST_SIZE) return 0;
    clock->burst = false;
    clock->next_us = now + clock->interval_us;
    if (!clock->have_best) return -1;
    clock->offset_us = clock->best_offset_us;
    clock->last_sync_us = now;
    clock->ready = true;
    return 1;
}

/**
 * @brief Report readiness/freshness of the current clock mapping.
 * @param clock Clock state to inspect.
 * @param now Current monotonic board time in microseconds.
 * @return CLOCK_QUALITY_UNREADY, CLOCK_QUALITY_VALID, or CLOCK_QUALITY_STALE.
 */
uint8_t time_sync_quality(const time_sync_t *clock, uint64_t now)
{
    if (!clock->ready) return CLOCK_QUALITY_UNREADY;
    if (clock->mode == CLOCK_MODE_UPTIME) return CLOCK_QUALITY_VALID;
    return now - clock->last_sync_us > clock->interval_us +
        TIME_SYNC_BURST_SIZE * TIME_SYNC_TIMEOUT_US ? CLOCK_QUALITY_STALE : CLOCK_QUALITY_VALID;
}

/**
 * @brief Map a saved board acquisition time into the selected clock domain.
 * @param clock Ready clock mapping; uptime mode uses zero offset.
 * @param local Acquisition time in monotonic board microseconds.
 * @param result Receives mapped microseconds only on success.
 * @return False when unready or the mapped value cannot be represented.
 */
bool time_sync_convert(const time_sync_t *clock, uint64_t local, uint64_t *result)
{
    if (!clock->ready || local > INT64_MAX) return false;
    int64_t offset = clock->mode == CLOCK_MODE_WALL ? clock->offset_us : 0;
    if (offset < 0 && local < (uint64_t)(-offset)) return false;
    if (offset > 0 && local > (uint64_t)(INT64_MAX - offset)) return false;
    *result = (uint64_t)((int64_t)local + offset);
    return true;
}
