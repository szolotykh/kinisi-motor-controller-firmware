//------------------------------------------------------------
// File name: connection.h
// Description: Declare per-transport connection state and serialized session operations.
//------------------------------------------------------------
#pragma once
#include "protocol.h"
#include "time_sync.h"
#include <stdbool.h>

// Bound how long an unsent command reply can hold the next request in the RX queue.
#define CONNECTION_REPLY_TIMEOUT_US 1000000ULL
#define CONNECTION_HEARTBEAT_DEFAULT_MS 500U
#define CONNECTION_ODOMETRY_SOURCES 5U

// Runtime services are optional for host tests, installed by the command task.
typedef struct {
    void (*stop)(void);
    uint32_t (*calculation_period_ms)(void);
    bool (*period_allowed)(uint32_t);
} connection_services_t;

typedef struct {
    uint64_t next_us;
    uint32_t interval_ms;
} connection_subscription_t;

// Successful sends copy the complete frame before returning; false means retry later.
typedef bool (*connection_try_send_fn)(uint8_t *, uint8_t);
// One state per transport; access only from the serialized command task.
typedef struct {
    time_sync_t clock;
    protocol_handler_fn handler;
    connection_try_send_fn send;
    uint64_t (*now)(void);
    // One complete framed reply waits here until accepted or its transmit deadline expires.
    uint8_t output[255], output_length;
    uint64_t output_queued_us;
    // READY and initial sync failure refer back to the INIT that opened this session.
    uint16_t init_id;
    bool ready_announced;
    // A busy READY/sync send yields one receive turn before it takes priority again.
    bool yield_receive;
    connection_services_t services;
    heartbeat_config heartbeat;
    uint64_t last_activity_us;
    // Loss is latched so buffered motion cannot resume without a fresh INIT.
    bool lost;
    connection_subscription_t subscriptions[CONNECTION_ODOMETRY_SOURCES];
    uint8_t next_subscription;
    bool telemetry_before_reply;
} connection_t;

/** @brief Invalidate this session after loss without invoking the motor-stop hook. */
void connection_invalidate(connection_t *);
/** @brief Invalidate the session and invoke the configured motor-stop hook once. */
void connection_disconnect(connection_t *);
/** @brief Check a proposed calculation period against every active subscription. */
bool connection_period_allowed(const connection_t *, uint32_t period_ms);

/**
 * @brief Initialize an isolated transport session with no completed INIT.
 * @note The send callback must be nonblocking and copy bytes on success.
 * The clock callback returns monotonic board microseconds. All calls are serialized.
 */
void connection_init(connection_t *, protocol_handler_fn, connection_try_send_fn, uint64_t (*now)(void));
/**
 * @brief Discard session readiness, pending timing state, and queued output.
 * @note Keep transport callbacks and the controller ID sequence across reconnects.
 */
void connection_reset(connection_t *);
/**
 * @brief Check whether the scheduler may dequeue another request.
 * @return False while a bounded reply wait or a control transmit turn takes priority.
 */
bool connection_can_receive(const connection_t *);
/**
 * @brief Dispatch one received frame and capture its reply or timing sample.
 * @param c Transport session being serviced.
 * @param data Frame bytes after the length prefix.
 * @param length Frame length excluding its prefix.
 * @note Call only when connection_can_receive is true; poll expires stalled replies.
 * Successful timing replies have no ACK.
 */
void connection_receive(connection_t *, const uint8_t *, size_t);
/**
 * @brief Service pending output, announce READY, or advance periodic synchronization.
 * @note Call frequently on the command task. Stalled replies expire; busy control
 * sends yield to incoming requests without announcing readiness or starting the RTT.
 */
void connection_poll(connection_t *);
/**
 * @brief Expose the clock of the command currently being dispatched.
 * @return Active session clock, or null outside serialized command dispatch.
 */
const time_sync_t *connection_current_clock(void);
