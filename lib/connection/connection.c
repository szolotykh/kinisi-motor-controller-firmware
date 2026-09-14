//------------------------------------------------------------
// File name: connection.c
// Description: Sequence INIT, clock setup, READY, command replies, and periodic synchronization.
//------------------------------------------------------------
#include "connection.h"
#include "initialization.h"
#include <string.h>

static connection_t *active;
/**
 * @brief Copy the framed reply into the active session's pending transmit slot.
 * @note Called synchronously by protocol_dispatch on the command task.
 */
static void capture(uint8_t *data, uint8_t length)
{
    memcpy(active->output, data, length);
    active->output_length = length;
    active->output_queued_us = active->now();
}

/**
 * @brief Identify odometry start/read operations that require initial clock readiness.
 */
static bool odometry_command(uint8_t cmd)
{
    switch (cmd) {
    case START_ENCODER_ODOMETRY: case GET_ENCODER_ODOMETRY:
    case START_PLATFORM_ODOMETRY: case GET_PLATFORM_ODOMETRY:
        return true;
    default: return false;
    }
}

/**
 * @brief Enforce session readiness and handle clock commands before hardware dispatch.
 * @param cmd Fully decoded and validated request for the active connection.
 * @param reply Callback that captures an unframed response payload.
 * @return RESPONSE_OK or a generated protocol error code.
 */
static uint8_t handle(controller_command_t *cmd, protocol_send_fn reply)
{
    if (odometry_command(cmd->commandType)) {
        if (!active->clock.initialized) return RESPONSE_INIT_REQUIRED;
        if (!active->ready_announced) return RESPONSE_CLOCK_NOT_READY;
    }
    if (cmd->commandType == SET_TIME_SYNC_INTERVAL) {
        if (!active->clock.initialized) return RESPONSE_INIT_REQUIRED;
        active->clock.interval_us = (uint64_t)cmd->properties.set_time_sync_interval.interval_ms * 1000;
        active->clock.next_us = active->now() + active->clock.interval_us;
        return RESPONSE_OK;
    }
    if (cmd->commandType == GET_TIME_STATUS) {
        time_status value = {0};
        value.clock_mode = active->clock.mode;
        value.clock_quality = time_sync_quality(&active->clock, active->now());
        value.interval_ms = active->clock.interval_us / 1000;
        value.last_sync_age_us = active->clock.ready && active->clock.mode == CLOCK_MODE_WALL ?
            active->now() - active->clock.last_sync_us : 0;
        reply((uint8_t *)&value, sizeof(value));
        return RESPONSE_OK;
    }
    uint8_t status = active->handler(cmd, reply);
    if (cmd->commandType == INIT && status == RESPONSE_OK) {
        time_sync_init(&active->clock, (cmd->properties.init.client_capabilities & CLIENT_CAP_WALL_CLOCK) != 0, active->now());
        active->ready_announced = false;
        active->init_id = cmd->message_id;
    }
    return status;
}

/**
 * @brief Queue a shared ERROR frame with the failed command and initiating message ID.
 */
static void error_reply(connection_t *c, uint16_t id, uint8_t command, uint8_t error)
{
    uint8_t data[] = {5, KINISI_MESSAGE_ERROR, (uint8_t)id, (uint8_t)(id >> 8), command, error};
    memcpy(c->output, data, sizeof(data));
    c->output_length = sizeof(data);
    c->output_queued_us = c->now();
}

/**
 * @brief Initialize an isolated transport session with no completed INIT.
 * @note The send callback must be nonblocking and copy bytes on success.
 * The clock callback returns monotonic board microseconds. All calls are serialized.
 */
void connection_init(connection_t *c, protocol_handler_fn handler, connection_try_send_fn send, uint64_t (*now)(void))
{
    memset(c, 0, sizeof(*c));
    c->handler = handler; c->send = send; c->now = now;
}

/**
 * @brief Discard session readiness, pending timing state, and queued output.
 * @note Keep transport callbacks and the controller ID sequence across reconnects.
 */
void connection_reset(connection_t *c)
{
    uint16_t sequence = c->clock.sequence;
    memset(&c->clock, 0, sizeof(c->clock));
    c->clock.sequence = sequence;
    c->output_length = 0;
    c->output_queued_us = 0;
    c->ready_announced = false;
    c->yield_receive = false;
}

/**
 * @brief Check whether the scheduler may dequeue another request.
 * @return False while a bounded reply wait or a control transmit turn takes priority.
 */
bool connection_can_receive(const connection_t *c)
{
    // Poll must discard expired replies before dequeue, so receive never drops a request.
    if (c->output_length) return false;
    if (c->yield_receive) return true;
    if (c->clock.ready && !c->ready_announced) return false;
    // Give scheduled sync a turn even when ordinary requests arrive continuously.
    // Once a request is sent, incoming traffic (including its reply) can resume.
    if (c->clock.initialized && c->clock.mode == CLOCK_MODE_WALL && !c->clock.pending &&
        (c->clock.burst || c->now() >= c->clock.next_us)) return false;
    return true;
}
/**
 * @brief Return the active handler's clock, or null outside command dispatch.
 */
const time_sync_t *connection_current_clock(void) { return active ? &active->clock : NULL; }

/**
 * @brief Dispatch one received frame and capture its reply or timing sample.
 * @param c Transport session being serviced.
 * @param data Frame bytes after the length prefix.
 * @param length Frame length excluding its prefix.
 * @note Call only when connection_can_receive is true; poll expires stalled replies.
 * Successful timing replies have no ACK.
 */
void connection_receive(connection_t *c, const uint8_t *data, size_t length)
{
    if (c->output_length) return;
    c->yield_receive = false;
    uint64_t received_us = c->now();
    active = c;
    if (data && length && data[0] == TIME_SYNC_RESPONSE) {
        uint16_t id = length >= 3 ? (uint16_t)data[1] | ((uint16_t)data[2] << 8) : 0;
        if (length != command_request_size(TIME_SYNC_RESPONSE)) {
            error_reply(c, id, TIME_SYNC_RESPONSE, RESPONSE_INVALID_LENGTH);
        } else {
            controller_command_t cmd = {0};
            memcpy(&cmd, data, length);
            if (!time_sync_receive(&c->clock, id,
                cmd.properties.time_sync_response.host_receive_us,
                cmd.properties.time_sync_response.host_send_us, received_us)) {
                error_reply(c, id, TIME_SYNC_RESPONSE, RESPONSE_INVALID_ARGUMENT);
            }
        }
        // Successful timing replies are consumed, not acknowledged recursively.
    } else {
        protocol_dispatch(data, length, handle, capture);
    }
    active = NULL;
}

/**
 * @brief Service pending output, announce READY, or advance periodic synchronization.
 * @note Call frequently on the command task. Stalled replies expire; busy control
 * sends yield to incoming requests without announcing readiness or starting the RTT.
 */
void connection_poll(connection_t *c)
{
    uint64_t now = c->now();
    if (c->output_length && now - c->output_queued_us >= CONNECTION_REPLY_TIMEOUT_US) {
        if (c->output[1] == INIT) {
            // A lost identity must not be followed by READY or timing requests.
            // Keep commands available, but require the client to retry INIT.
            connection_reset(c);
        } else {
            c->output_length = 0;
        }
    }
    // Advance pending sync timeouts even while ordinary replies occupy the TX slot.
    // Otherwise continuous command traffic can keep an unanswered sync pending forever.
    bool due = time_sync_due(&c->clock, now);
    if (c->output_length) {
        if (c->send(c->output, c->output_length)) c->output_length = 0;
        return; // Preserve INIT response before TIME_SYNC/READY ordering.
    }
    if (c->clock.ready && !c->ready_announced) {
        uint8_t ready[] = {4, KINISI_MESSAGE_READY, (uint8_t)c->init_id,
                          (uint8_t)(c->init_id >> 8), c->clock.mode};
        bool sent = c->send(ready, sizeof(ready));
        if (sent) c->ready_announced = true;
        c->yield_receive = !sent;
        return;
    }
    int finished = time_sync_finish(&c->clock, now);
    if (finished < 0 && !c->clock.ready) {
        error_reply(c, c->init_id, INIT, RESPONSE_TIME_SYNC_FAILED);
        return;
    }
    if (due) {
        uint16_t id = protocol_next_message_id(c->clock.sequence);
        uint8_t request[] = {3, KINISI_MESSAGE_TIME_SYNC_REQUEST, (uint8_t)id, (uint8_t)(id >> 8)};
        // send is nonblocking: failed/busy attempts do not start the RTT timer.
        uint64_t sent_us = c->now();
        bool sent = c->send(request, sizeof(request));
        if (sent) time_sync_sent(&c->clock, id, sent_us);
        c->yield_receive = !sent;
    }
}
