//------------------------------------------------------------
// File name: connection.c
// Description: Sequence INIT, clock setup, READY, command replies, and periodic synchronization.
//------------------------------------------------------------
#include "connection.h"
#include "initialization.h"
#include <string.h>

static connection_t *active;
static uint8_t sample_payload[sizeof(telemetry_payload_t)], sample_length;
static bool check_heartbeat(connection_t *c);

/** @brief Copy a cached measurement returned by the regular odometry handler. */
static void capture_sample(uint8_t *data, uint8_t length)
{
    if (length <= sizeof(sample_payload)) {
        memcpy(sample_payload, data, length);
        sample_length = length;
    }
}

/** @brief Read through the same handler as GET, preserving measurement timestamps. */
static uint8_t read_sample(connection_t *c, connection_subscription_t *sub)
{
    bool sensor = sub->topic == TOPIC_TEMPERATURE || sub->topic == TOPIC_IMU || sub->topic == TOPIC_MOTOR_CURRENT || sub->topic == TOPIC_MOTOR_FAULTS;
    if (sensor && sub->sample_length && c->now() < sub->next_read_us) {
        sample_length = sub->sample_length;
        memcpy(sample_payload, sub->sample, sample_length);
        return RESPONSE_OK;
    }
    controller_command_t request = {0};
    switch (sub->topic) {
    case TOPIC_ENCODER_ODOMETRY:
        request.commandType = GET_ENCODER_ODOMETRY;
        request.properties.get_encoder_odometry.encoder_index = sub->index;
        break;
    case TOPIC_PLATFORM_ODOMETRY: request.commandType = GET_PLATFORM_ODOMETRY; break;
    case TOPIC_TEMPERATURE:
        request.commandType = GET_TEMPERATURE;
        request.properties.get_temperature.sensor_index = sub->index;
        break;
    case TOPIC_IMU:
        request.commandType = GET_IMU;
        request.properties.get_imu.sensor_index = sub->index;
        break;
    case TOPIC_MOTOR_FAULTS:
        request.commandType = GET_MOTOR_FAULTS;
        request.properties.get_motor_faults.motor_index = sub->index;
        break;
    case TOPIC_MOTOR_CURRENT:
        request.commandType = GET_MOTOR_CURRENT;
        request.properties.get_motor_current.motor_index = sub->index;
        break;
    default: return RESPONSE_INVALID_ARGUMENT;
    }
    sample_length = 0;
    connection_t *previous = active;
    active = c;
    uint8_t status = c->handler(&request, capture_sample);
    active = previous;
    if (sensor) {
        sub->sample_length = 0;
        if (status != RESPONSE_OK) return status;
        uint8_t expected = sub->topic == TOPIC_IMU ? sizeof(imu_sample) :
            sub->topic == TOPIC_MOTOR_FAULTS ? sizeof(motor_fault_status) : sizeof(double);
        if (sample_length != expected) return RESPONSE_INTERNAL_ERROR;
        uint64_t completed = c->now(), timestamp;
        if (!time_sync_convert(&c->clock, completed, &timestamp)) return RESPONSE_CLOCK_NOT_READY;
        memmove(sample_payload + 10, sample_payload, sample_length);
        memcpy(sample_payload, &timestamp, sizeof(timestamp));
        sample_payload[8] = c->clock.mode;
        sample_payload[9] = time_sync_quality(&c->clock, completed);
        sample_length += 10;
        memcpy(sub->sample, sample_payload, sample_length);
        sub->sample_length = sample_length;
        uint32_t cadence = sub->interval_ms;
        if (sub->configured && sub->max_silence_ms && sub->max_silence_ms < cadence) cadence = sub->max_silence_ms;
        sub->next_read_us = completed + (uint64_t)cadence * 1000;
    }
    return status;
}

/** @brief Odometry has stable slots; sensor slots are bounded and keyed by topic/index. */
static connection_subscription_t *subscription_slot(connection_t *c, uint8_t topic, uint8_t index, bool create)
{
    if (!telemetry_payload_size(topic) || index > telemetry_index_limit(topic)) return NULL;
    if (topic == TOPIC_ENCODER_ODOMETRY) return &c->subscriptions[index];
    if (topic == TOPIC_PLATFORM_ODOMETRY) return &c->subscriptions[4];
    connection_subscription_t *free_slot = NULL;
    for (unsigned i = CONNECTION_ODOMETRY_SOURCES; i < CONNECTION_SUBSCRIPTION_CAPACITY; ++i) {
        connection_subscription_t *sub = &c->subscriptions[i];
        if (sub->interval_ms && sub->topic == topic && sub->index == index) return sub;
        if (!sub->interval_ms && !free_slot) free_slot = sub;
    }
    return create ? free_slot : NULL;
}

static uint8_t subscribe(connection_t *c, uint8_t topic, uint8_t index, uint32_t interval, bool legacy, uint8_t mode)
{
    if (!c->clock.initialized) return RESPONSE_INIT_REQUIRED;
    if (!c->ready_announced) return RESPONSE_CLOCK_NOT_READY;
    if (mode > SUBSCRIPTION_MODE_ON_CHANGE || !c->heartbeat.enabled || !telemetry_payload_size(topic) ||
        index > telemetry_index_limit(topic) || interval < telemetry_min_interval_ms(topic))
        return RESPONSE_INVALID_ARGUMENT;
    bool odometry = topic == TOPIC_ENCODER_ODOMETRY || topic == TOPIC_PLATFORM_ODOMETRY;
    if (odometry && (!c->services.calculation_period_ms ||
                    interval < 2ULL * c->services.calculation_period_ms())) return RESPONSE_INVALID_ARGUMENT;
    connection_subscription_t *slot = subscription_slot(c, topic, index, true);
    if (!slot) return RESPONSE_SUBSCRIPTION_LIMIT;
    connection_subscription_t replacement = {.topic = topic, .index = index, .interval_ms = interval, .legacy = legacy, .mode = mode};
    uint8_t status = read_sample(c, &replacement);
    // Sensor operations may consume time. Never revive a session that expired during a read.
    if (check_heartbeat(c)) return RESPONSE_INIT_REQUIRED;
    if (status != RESPONSE_OK && status != RESPONSE_SAMPLE_NOT_AVAILABLE) return status;
    replacement.next_us = c->now() + (mode == SUBSCRIPTION_MODE_ON_CHANGE ? 0 : (uint64_t)interval * 1000);
    *slot = replacement;
    return RESPONSE_OK;
}

/** @brief Install independent trigger/delivery settings without changing legacy commands. */
static uint8_t subscribe_configured(connection_t *c, uint8_t topic, uint8_t index,
    uint32_t poll_ms, uint8_t trigger, uint8_t delivery, uint32_t silence_ms)
{
    bool event = trigger == SUBSCRIPTION_TRIGGER_ON_EVENT;
    uint32_t minimum = telemetry_min_interval_ms(topic);
    if (trigger > SUBSCRIPTION_TRIGGER_ON_EVENT || delivery > SUBSCRIPTION_DELIVERY_ON_CHANGE ||
        poll_ms > 3600000 || silence_ms > 3600000 ||
        (event ? (poll_ms != 0 || !telemetry_event_supported(topic) || !c->services.event_sequence) : poll_ms < minimum) ||
        (silence_ms && (delivery != SUBSCRIPTION_DELIVERY_ON_CHANGE || silence_ms < minimum)))
        return RESPONSE_INVALID_ARGUMENT;
    // Event diagnostics also reconcile every 100 ms: a second fault may not create an edge.
    uint32_t cadence = event ? 100 : poll_ms;
    uint32_t sequence = c->services.event_sequence ? c->services.event_sequence(topic, index) : 0;
    uint8_t status = subscribe(c, topic, index, cadence, false, delivery);
    if (status != RESPONSE_OK) return status;
    connection_subscription_t *sub = subscription_slot(c, topic, index, false);
    sub->configured = true; sub->trigger = trigger; sub->max_silence_ms = silence_ms;
    sub->observed_sequence = sub->sampled_sequence = sequence;
    sub->next_us = c->now(); // Every configured subscription starts with a snapshot.
    return RESPONSE_OK;
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
    case GET_PWM_FREQUENCY:
    case SET_PWM_FREQUENCY:
    case GET_MOTOR_CURRENT_LIMIT:
    case SET_MOTOR_CURRENT_LIMIT:
    case CLEAR_MOTOR_FAULTS:
        if (!active->clock.initialized) return RESPONSE_INIT_REQUIRED;
        if (!active->ready_announced) return RESPONSE_CLOCK_NOT_READY;
        break;
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
        uint8_t source = cmd->properties.subscribe_odometry.source;
        return subscribe(active, source == 4 ? TOPIC_PLATFORM_ODOMETRY : TOPIC_ENCODER_ODOMETRY,
                         source == 4 ? 0 : source, cmd->properties.subscribe_odometry.interval_ms, true, SUBSCRIPTION_MODE_PERIODIC);
    }
    case SUBSCRIBE:
        return subscribe(active, cmd->properties.subscribe.topic_id, cmd->properties.subscribe.index,
                         cmd->properties.subscribe.interval_ms, false, SUBSCRIPTION_MODE_PERIODIC);
    case SUBSCRIBE_WITH_MODE:
        return subscribe(active, cmd->properties.subscribe_with_mode.topic_id, cmd->properties.subscribe_with_mode.index,
                         cmd->properties.subscribe_with_mode.interval_ms, false, cmd->properties.subscribe_with_mode.mode);
    case SUBSCRIBE_CONFIGURED:
        return subscribe_configured(active, cmd->properties.subscribe_configured.topic_id,
            cmd->properties.subscribe_configured.index, cmd->properties.subscribe_configured.poll_interval_ms,
            cmd->properties.subscribe_configured.trigger, cmd->properties.subscribe_configured.delivery,
            cmd->properties.subscribe_configured.max_silence_ms);
    case UNSUBSCRIBE: {
        if (!active->clock.initialized) return RESPONSE_INIT_REQUIRED;
        if (!active->ready_announced) return RESPONSE_CLOCK_NOT_READY;
        uint8_t topic = cmd->properties.unsubscribe.topic_id, index = cmd->properties.unsubscribe.index;
        if (index > telemetry_index_limit(topic)) return RESPONSE_INVALID_ARGUMENT;
        connection_subscription_t *slot = subscription_slot(active, topic, index, false);
        if (slot) memset(slot, 0, sizeof(*slot));
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
    const uint8_t start = c->next_subscription;
    for (unsigned offset = 0; offset < CONNECTION_SUBSCRIPTION_CAPACITY; ++offset) {
        uint8_t source = (start + offset) % CONNECTION_SUBSCRIPTION_CAPACITY;
        connection_subscription_t *sub = &c->subscriptions[source];
        if (!sub->interval_ms) continue;
        bool event = sub->configured && sub->trigger == SUBSCRIPTION_TRIGGER_ON_EVENT;
        uint32_t sequence = event ? c->services.event_sequence(sub->topic, sub->index) : 0;
        bool pending_event = event && sequence != sub->observed_sequence;
        bool fresh_event = event && sequence != sub->sampled_sequence;
        bool refresh = sub->configured && sub->max_silence_ms && sub->last_length &&
            now - sub->last_sent_us >= (uint64_t)sub->max_silence_ms * 1000;
        if (now < sub->next_us && !pending_event && !refresh) continue;
        if (fresh_event || (refresh && !sub->refresh_pending)) sub->next_read_us = 0;
        if (refresh) sub->refresh_pending = true;
        sub->sampled_sequence = sequence;
        uint8_t status = read_sample(c, sub);
        if (check_heartbeat(c)) return 0;
        bool encoder = sub->topic == TOPIC_ENCODER_ODOMETRY;
        uint8_t expected = telemetry_payload_size(sub->topic);
        bool valid = status == RESPONSE_OK && sample_length == expected;
        bool changed = valid && (sub->last_length != expected ||
            memcmp(sub->last_value, sample_payload + 10, expected - 10) != 0);
        bool publish = valid && (changed || refresh ||
            (sub->mode == SUBSCRIPTION_MODE_PERIODIC && (!event || pending_event)));
        if (!valid) { sub->last_length = 0; sub->observed_sequence = sequence; sub->refresh_pending = false; } // Re-announce recovery even if values match.
        if (publish) {
            uint8_t frame[6 + sizeof(telemetry_payload_t)] = {0};
            unsigned prefix = sub->legacy ? (encoder ? 1 : 0) : 2;
            frame[0] = 3 + expected + prefix;
            frame[1] = sub->legacy ? (encoder ? KINISI_MESSAGE_ENCODER_ODOMETRY_EVENT :
                KINISI_MESSAGE_PLATFORM_ODOMETRY_EVENT) : KINISI_MESSAGE_TELEMETRY_EVENT;
            if (sub->legacy && encoder) frame[4] = sub->index;
            if (!sub->legacy) { frame[4] = sub->topic; frame[5] = sub->index; }
            memcpy(frame + 4 + prefix, sample_payload, expected);
            if (!c->send(frame, frame[0] + 1)) return -1;
            // Failed sends must not acknowledge a change or lose the first sample.
            memcpy(sub->last_value, sample_payload + 10, expected - 10);
            sub->last_length = expected;
            sub->last_sent_us = c->now();
            sub->refresh_pending = false;
        }
        if (valid) sub->observed_sequence = sequence;
        now = c->now();
        uint64_t interval_us = (uint64_t)sub->interval_ms * 1000;
        sub->next_us = now + interval_us;
        // A sensor read consumes time. Do not deliver the same cached sample at
        // the next deadline just because its read-completion cadence is later.
        if (sub->next_read_us > sub->next_us) sub->next_us = sub->next_read_us;
        c->next_subscription = (source + 1) % CONNECTION_SUBSCRIPTION_CAPACITY;
        if (publish) return 1;
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
            if (!c->ready_announced || c->lost) return;
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
