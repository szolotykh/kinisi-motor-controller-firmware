//------------------------------------------------------------
// File name: connection.c
// Description: Sequence INIT, clock setup, READY, command replies, and periodic synchronization.
//------------------------------------------------------------
#include "connection.h"
#include "initialization.h"
#include <string.h>

static connection_t *active;
static uint8_t sample_payload[sizeof(platform_odometry_sample)], sample_length;

/** @brief Copy a cached measurement returned by the regular odometry handler. */
static void capture_sample(uint8_t *data, uint8_t length)
{
    if (length <= sizeof(sample_payload)) {
        memcpy(sample_payload, data, length);
        sample_length = length;
    }
}

/** @brief Read through the same handler as GET, preserving measurement timestamps. */
static uint8_t read_sample(connection_t *c, uint8_t source)
{
    controller_command_t request = {0};
    request.commandType = source == 4 ? GET_PLATFORM_ODOMETRY : GET_ENCODER_ODOMETRY;
    request.properties.get_encoder_odometry.encoder_index = source;
    sample_length = 0;
    connection_t *previous = active;
    active = c;
    uint8_t status = c->handler(&request, capture_sample);
    active = previous;
    return status;
}

/** @brief Permit stop/brake operations even when connection loss is latched. */
static bool safe_after_loss(uint8_t command)
{
    switch (command) {
    case INIT: case STOP_MOTOR: case BRAKE_MOTOR: case COAST_PLATFORM:
    case BRAKE_PLATFORM: case STOP_PLATFORM_CONTROLLER:
        return true;
    default: return false;
    }
}

/** @brief Expire activity before either dequeuing commands or transmitting data. */
static bool check_heartbeat(connection_t *c)
{
    if (c->heartbeat.enabled && c->now() - c->last_activity_us >= (uint64_t)c->heartbeat.timeout_ms * 1000) {
        connection_disconnect(c);
        return true;
    }
    return false;
}
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
    if (active->lost && !safe_after_loss(cmd->commandType)) return RESPONSE_INIT_REQUIRED;
    // protocol_dispatch already checked the command, fields and message ID.
    active->last_activity_us = active->now();
    switch (cmd->commandType) {
    case POLL_TELEMETRY:
        if (!active->clock.initialized) return RESPONSE_INIT_REQUIRED;
        if (!active->ready_announced) return RESPONSE_CLOCK_NOT_READY;
        active->telemetry_before_reply = true;
        return RESPONSE_OK;
    case PING:
        return active->clock.initialized ? RESPONSE_OK : RESPONSE_INIT_REQUIRED;
    case SET_HEARTBEAT_CONFIG:
        if (!active->clock.initialized) return RESPONSE_INIT_REQUIRED;
        if (!active->ready_announced) return RESPONSE_CLOCK_NOT_READY;
        active->heartbeat.enabled = cmd->properties.set_heartbeat_config.enabled;
        active->heartbeat.timeout_ms = cmd->properties.set_heartbeat_config.timeout_ms;
        if (!active->heartbeat.enabled) memset(active->subscriptions, 0, sizeof(active->subscriptions));
        return RESPONSE_OK;
    case GET_HEARTBEAT_CONFIG:
        reply((uint8_t *)&active->heartbeat, sizeof(active->heartbeat));
        return RESPONSE_OK;
    case SUBSCRIBE_ODOMETRY: {
        if (!active->clock.initialized) return RESPONSE_INIT_REQUIRED;
        if (!active->ready_announced) return RESPONSE_CLOCK_NOT_READY;
        if (!active->heartbeat.enabled || !active->services.calculation_period_ms) return RESPONSE_INVALID_ARGUMENT;
        uint32_t interval = cmd->properties.subscribe_odometry.interval_ms;
        if (interval < 2ULL * active->services.calculation_period_ms()) return RESPONSE_INVALID_ARGUMENT;
        uint8_t source = cmd->properties.subscribe_odometry.source;
        uint8_t status = read_sample(active, source);
        if (status != RESPONSE_OK && status != RESPONSE_SAMPLE_NOT_AVAILABLE) return status;
        active->subscriptions[source].interval_ms = interval;
        active->subscriptions[source].next_us = active->now() + (uint64_t)interval * 1000;
        return RESPONSE_OK;
    }
    case UNSUBSCRIBE_ODOMETRY:
        active->subscriptions[cmd->properties.unsubscribe_odometry.source].interval_ms = 0;
        return RESPONSE_OK;
    case SET_ODOMETRY_FREQUENCY:
        if (active->services.period_allowed && !active->services.period_allowed(1000U / cmd->properties.set_odometry_frequency.frequency))
            return RESPONSE_INVALID_ARGUMENT;
        break;
    default: break;
    }
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
        if (active->clock.initialized && active->heartbeat.enabled && active->services.stop) active->services.stop();
        active->lost = false;
        active->heartbeat.enabled = false;
        active->heartbeat.timeout_ms = CONNECTION_HEARTBEAT_DEFAULT_MS;
        memset(active->subscriptions, 0, sizeof(active->subscriptions));
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
    c->heartbeat.timeout_ms = CONNECTION_HEARTBEAT_DEFAULT_MS;
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
    c->heartbeat.enabled = false;
    c->heartbeat.timeout_ms = CONNECTION_HEARTBEAT_DEFAULT_MS;
    memset(c->subscriptions, 0, sizeof(c->subscriptions));
    c->next_subscription = 0;
    c->telemetry_before_reply = false;
}

/** @brief Latch connection loss until a valid INIT is completed. */
void connection_invalidate(connection_t *c)
{
    connection_reset(c);
    c->lost = true;
}

/** @brief Stop motion once per loss and require explicit session recovery. */
void connection_disconnect(connection_t *c)
{
    bool notify = !c->lost;
    connection_invalidate(c);
    if (notify && c->services.stop) c->services.stop();
}

/** @brief Keep existing subscription cadences valid when calculation rate changes. */
bool connection_period_allowed(const connection_t *c, uint32_t period_ms)
{
    for (unsigned i = 0; i < CONNECTION_ODOMETRY_SOURCES; ++i)
        if (c->subscriptions[i].interval_ms && c->subscriptions[i].interval_ms < 2ULL * period_ms) return false;
    return true;
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
    (void)check_heartbeat(c);
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
            } else {
                c->last_activity_us = received_us;
            }
        }
        // Successful timing replies are consumed, not acknowledged recursively.
    } else {
        protocol_dispatch(data, length, handle, capture);
    }
    active = NULL;
}

/** @brief Send one due event; return -1 on backpressure, 0 if unavailable, 1 if sent. */
static int publish_sample(connection_t *c, uint64_t now)
{
    // Round-robin sources; at most one telemetry frame per turn. Never queue history.
    for (unsigned offset = 0; offset < CONNECTION_ODOMETRY_SOURCES; ++offset) {
        uint8_t source = (c->next_subscription + offset) % CONNECTION_ODOMETRY_SOURCES;
        connection_subscription_t *sub = &c->subscriptions[source];
        if (!sub->interval_ms || now < sub->next_us) continue;
        uint8_t status = read_sample(c, source);
        bool encoder = source < 4;
        uint8_t expected = encoder ? sizeof(encoder_odometry_sample) : sizeof(platform_odometry_sample);
        if (status == RESPONSE_OK && sample_length == expected) {
            uint8_t frame[5 + sizeof(platform_odometry_sample)] = {0};
            frame[0] = 3 + expected + encoder;
            frame[1] = encoder ? KINISI_MESSAGE_ENCODER_ODOMETRY_EVENT : KINISI_MESSAGE_PLATFORM_ODOMETRY_EVENT;
            if (encoder) frame[4] = source;
            memcpy(frame + 4 + encoder, sample_payload, expected);
            if (!c->send(frame, frame[0] + 1)) return -1;
        }
        uint64_t interval_us = (uint64_t)sub->interval_ms * 1000;
        sub->next_us += ((now - sub->next_us) / interval_us + 1) * interval_us;
        c->next_subscription = (source + 1) % CONNECTION_ODOMETRY_SOURCES;
        return status == RESPONSE_OK && sample_length == expected ? 1 : 0;
    }
    return 0;
}

/**
 * @brief Service pending output, announce READY, or advance periodic synchronization.
 * @note Call frequently on the command task. Stalled replies expire; busy control
 * sends yield to incoming requests without announcing readiness or starting the RTT.
 */
void connection_poll(connection_t *c)
{
    if (check_heartbeat(c)) return;
    uint64_t now = c->now();
    if (c->output_length && now - c->output_queued_us >= CONNECTION_REPLY_TIMEOUT_US) {
        if (c->output[1] == INIT) {
            // A lost identity must not be followed by READY or timing requests.
            // Keep commands available, but require the client to retry INIT.
            connection_reset(c);
        } else {
            c->output_length = 0;
            c->telemetry_before_reply = false;
        }
    }
    // Advance pending sync timeouts even while ordinary replies occupy the TX slot.
    // Otherwise continuous command traffic can keep an unanswered sync pending forever.
    bool due = time_sync_due(&c->clock, now);
    if (c->output_length) {
        if (c->telemetry_before_reply) {
            int result = publish_sample(c, now);
            if (result < 0) return;
            c->telemetry_before_reply = false;
            if (result > 0) return; // Master reads the event, then the correlated ACK.
        }
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
        return;
    }
    if (!c->ready_announced) return;
    (void)publish_sample(c, now);
}
