//------------------------------------------------------------
// File name: connection.h
// Description: Declare per-transport connection state and serialized session operations.
//------------------------------------------------------------
#pragma once
#include "protocol.h"
#include "time_sync.h"
#include <stdbool.h>

// Successful sends copy the complete frame before returning; false means retry later.
typedef bool (*connection_try_send_fn)(uint8_t *, uint8_t);
// One state per transport; access only from the serialized command task.
typedef struct {
    time_sync_t clock;
    protocol_handler_fn handler;
    connection_try_send_fn send;
    uint64_t (*now)(void);
    // One complete framed reply waits here until the transport accepts a copy.
    uint8_t output[255], output_length;
    // READY and initial sync failure refer back to the INIT that opened this session.
    uint16_t init_id;
    bool ready_announced;
} connection_t;

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
 * @return False while a reply, READY, or due sync must be serviced first.
 */
bool connection_can_receive(const connection_t *);
/**
 * @brief Dispatch one received frame and capture its reply or timing sample.
 * @param c Transport session being serviced.
 * @param data Frame bytes after the length prefix.
 * @param length Frame length excluding its prefix.
 * @note A pending output prevents dispatch. Successful timing replies have no ACK.
 */
void connection_receive(connection_t *, const uint8_t *, size_t);
/**
 * @brief Service pending output, announce READY, or advance periodic synchronization.
 * @note Call frequently on the command task; busy sends are retried without blocking.
 */
void connection_poll(connection_t *);
/**
 * @brief Expose the clock of the command currently being dispatched.
 * @return Active session clock, or null outside serialized command dispatch.
 */
const time_sync_t *connection_current_clock(void);
