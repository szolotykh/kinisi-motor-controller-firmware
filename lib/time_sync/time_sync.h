//------------------------------------------------------------
// File name: time_sync.h
// Description: Declare clock mapping state, timing limits, and synchronization operations.
//------------------------------------------------------------
#pragma once
#include <stdint.h>
#include <stdbool.h>

#define CLOCK_MODE_UPTIME 0U
#define CLOCK_MODE_WALL 1U
#define CLOCK_QUALITY_UNREADY 0U
#define CLOCK_QUALITY_VALID 1U
#define CLOCK_QUALITY_STALE 2U
#define TIME_SYNC_DEFAULT_INTERVAL_US 30000000ULL
#define TIME_SYNC_TIMEOUT_US 1000000ULL
#define TIME_SYNC_MAX_DELAY_US 100000ULL
#define TIME_SYNC_BURST_SIZE 3U

// All times/durations are microseconds; offset maps board monotonic time to host Unix time.
// The owner serializes mutation. Initialize this structure to zero before its first INIT.
typedef struct {
    bool initialized, ready, pending, burst, have_best;
    uint8_t mode, attempts;
    // Sequence survives re-INIT; pending_id correlates the single outstanding exchange.
    uint16_t sequence, pending_id;
    // Deadline, age, and sample-delay values use the board monotonic clock.
    uint64_t sent_us, next_us, last_sync_us, interval_us, best_delay_us;
    // Signed offsets permit either clock to be ahead; a burst keeps its best candidate.
    int64_t offset_us, best_offset_us;
} time_sync_t;

/**
 * @brief Reset clock setup for INIT while preserving the controller message-ID sequence.
 * @param clock Zero-initialized or previously initialized clock state.
 * @param wall_clock True to synchronize Unix time; false for immediately ready uptime.
 * @param now Current monotonic board time in microseconds.
 */
void time_sync_init(time_sync_t *clock, bool wall_clock, uint64_t now);
/**
 * @brief Expire an unanswered request and start a scheduled burst when due.
 * @param clock Mutable clock/session state.
 * @param now Current monotonic board time in microseconds.
 * @return True if another timing request should be attempted now.
 */
bool time_sync_due(time_sync_t *clock, uint64_t now);
/**
 * @brief Record an accepted timing transmission and consume one burst attempt.
 * @param clock Clock state with an active burst.
 * @param id Nonzero controller-allocated request ID.
 * @param now Monotonic microseconds captured immediately before the successful send.
 * @note Do not call for busy or failed transport sends.
 */
void time_sync_sent(time_sync_t *clock, uint16_t id, uint64_t now);
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
bool time_sync_receive(time_sync_t *clock, uint16_t id, uint64_t h2, uint64_t h3, uint64_t now);
// Finish a completed burst. Returns 1 for success, -1 for failure, 0 if running.
/**
 * @brief Commit a completed burst and schedule the next periodic attempt.
 * @param clock Mutable clock mapping and burst state.
 * @param now Current monotonic board time in microseconds.
 * @return 1 for success, -1 for a failed burst, or 0 while still running.
 * @note Failure preserves any previously valid mapping.
 */
int time_sync_finish(time_sync_t *clock, uint64_t now);
/**
 * @brief Report readiness/freshness of the current clock mapping.
 * @param clock Clock state to inspect.
 * @param now Current monotonic board time in microseconds.
 * @return CLOCK_QUALITY_UNREADY, CLOCK_QUALITY_VALID, or CLOCK_QUALITY_STALE.
 */
uint8_t time_sync_quality(const time_sync_t *clock, uint64_t now);
/**
 * @brief Map a saved board acquisition time into the selected clock domain.
 * @param clock Ready clock mapping; uptime mode uses zero offset.
 * @param local Acquisition time in monotonic board microseconds.
 * @param result Receives mapped microseconds only on success.
 * @return False when unready or the mapped value cannot be represented.
 */
bool time_sync_convert(const time_sync_t *clock, uint64_t local, uint64_t *result);
