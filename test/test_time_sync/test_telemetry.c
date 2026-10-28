#include "connection.h"
#include "initialization.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static connection_t a, b;
static uint64_t ticks;
static uint32_t fault_events[4];
static uint32_t event_sequence(uint8_t topic, uint8_t index) { return topic == TOPIC_MOTOR_FAULTS && index < 4 ? fault_events[index] : 0; }
static uint8_t wire[255], wire_size, sensor_error;
static unsigned sends, reads;
static uint64_t read_duration_us;
static bool busy, running = true, slow_read;
static motor_fault_status diagnostic = {.status_valid = true};
static uint64_t now(void) { return ticks; }
static uint32_t period(void) { return 20; }
static void stop(void) { connection_invalidate(&a); connection_invalidate(&b); }
static bool send_frame(uint8_t *data, uint8_t size) {
    assert(size >= 4);
    if (busy) return false;
    memcpy(wire, data, size); wire_size = size; ++sends; return true;
}
static uint8_t execute(controller_command_t *cmd, protocol_send_fn reply) {
    if (cmd->commandType == INIT) return initialization_validate(cmd);
    if (cmd->commandType == GET_ENCODER_ODOMETRY || cmd->commandType == GET_PLATFORM_ODOMETRY) {
        if (!running) return RESPONSE_ODOMETRY_NOT_INITIALIZED;
        platform_odometry_sample sample = {12345, 0, 1, 1.25, -2.5, 3.0};
        reply((uint8_t *)&sample, cmd->commandType == GET_ENCODER_ODOMETRY ? sizeof(encoder_odometry_sample) : sizeof(sample));
    } else if (cmd->commandType == GET_MOTOR_FAULTS) {
        if (sensor_error) return sensor_error;
        ++reads;
        motor_fault_status status = diagnostic;
        status.overcurrent |= cmd->properties.get_motor_faults.motor_index == 2;
        reply((uint8_t *)&status, sizeof(status));
    } else if (cmd->commandType == GET_MOTOR_CURRENT) {
        ++reads;
        if (sensor_error) return sensor_error;
        double amps = cmd->properties.get_motor_current.motor_index + 0.5;
        reply((uint8_t *)&amps, sizeof(amps));
    } else if (cmd->commandType == GET_TEMPERATURE || cmd->commandType == GET_IMU) {
        ++reads;
        ticks += read_duration_us;
        if (slow_read) ticks += 500000;
        if (cmd->properties.get_temperature.sensor_index == 255) return RESPONSE_SENSOR_NOT_AVAILABLE;
        if (sensor_error) return sensor_error;
        imu_sample value = {-12.5, 2, 3, 4, 5, 6};
        reply((uint8_t *)&value, cmd->commandType == GET_TEMPERATURE ? sizeof(double) : sizeof(value));
    }
    return RESPONSE_OK;
}
static void request(connection_t *c, controller_command_t cmd) {
    cmd.message_id = 9;
    connection_receive(c, (uint8_t *)&cmd, command_request_size(cmd.commandType));
    connection_poll(c);
}
static void init(connection_t *c) {
    connection_init(c, execute, send_frame, now);
    c->services.event_sequence = event_sequence; c->services.stop = stop; c->services.calculation_period_ms = period;
    controller_command_t cmd = {.commandType = INIT};
    cmd.properties.init.protocol_major = 2;
    request(c, cmd); connection_poll(c);
}
static void heartbeat(connection_t *c, bool enabled) {
    controller_command_t cmd = {.commandType = SET_HEARTBEAT_CONFIG};
    cmd.properties.set_heartbeat_config.enabled = enabled;
    cmd.properties.set_heartbeat_config.timeout_ms = 60000;
    request(c, cmd);
}
static void subscribe(connection_t *c, uint8_t topic, uint8_t index, uint32_t interval) {
    controller_command_t cmd = {.commandType = SUBSCRIBE};
    cmd.properties.subscribe.topic_id = topic; cmd.properties.subscribe.index = index;
    cmd.properties.subscribe.interval_ms = interval; request(c, cmd);
}
static void unsubscribe(connection_t *c, uint8_t topic, uint8_t index) {
    controller_command_t cmd = {.commandType = UNSUBSCRIBE};
    cmd.properties.unsubscribe.topic_id = topic; cmd.properties.unsubscribe.index = index; request(c, cmd);
}
static void error(uint8_t code) { assert(wire[1] == KINISI_MESSAGE_ERROR && wire[5] == code); }
static void event(uint8_t topic, uint8_t index) {
    assert(wire[1] == KINISI_MESSAGE_TELEMETRY_EVENT && !wire[2] && !wire[3]);
    assert(wire[4] == topic && wire[5] == index && wire_size == 6 + telemetry_payload_size(topic));
}

static void subscribe_mode(connection_t *c, uint8_t topic, uint8_t index, uint32_t interval, uint8_t mode) {
    controller_command_t cmd = {.commandType = SUBSCRIBE_WITH_MODE};
    cmd.properties.subscribe_with_mode.topic_id = topic;
    cmd.properties.subscribe_with_mode.index = index;
    cmd.properties.subscribe_with_mode.interval_ms = interval;
    cmd.properties.subscribe_with_mode.mode = mode;
    request(c, cmd);
}
static void configured(connection_t *c, uint8_t topic, uint8_t trigger, uint8_t delivery, uint32_t poll, uint32_t silence) {
    controller_command_t cmd = {.commandType=SUBSCRIBE_CONFIGURED};
    cmd.properties.subscribe_configured.topic_id = topic;
    cmd.properties.subscribe_configured.trigger = trigger;
    cmd.properties.subscribe_configured.delivery = delivery;
    cmd.properties.subscribe_configured.poll_interval_ms = poll;
    cmd.properties.subscribe_configured.max_silence_ms = silence;
    request(c, cmd);
}
static void configured_tests(void) {
    init(&a); init(&b); heartbeat(&a, true); heartbeat(&b, true);
    diagnostic = (motor_fault_status){.status_valid=true};
    configured(&a, TOPIC_TEMPERATURE, 1, 1, 0, 0); error(RESPONSE_INVALID_ARGUMENT);
    configured(&a, TOPIC_MOTOR_FAULTS, 1, 1, 100, 0); error(RESPONSE_INVALID_ARGUMENT);
    configured(&a, TOPIC_MOTOR_FAULTS, 0, 1, 0, 0); error(RESPONSE_INVALID_ARGUMENT);
    configured(&a, TOPIC_MOTOR_FAULTS, 1, 0, 0, 500); error(RESPONSE_INVALID_ARGUMENT);
    configured(&a, TOPIC_MOTOR_FAULTS, 1, 1, 0, 99); error(RESPONSE_INVALID_ARGUMENT);
    configured(&a, TOPIC_MOTOR_FAULTS, 1, 1, 0, 500); assert(wire[1] == SUBSCRIBE_CONFIGURED);
    connection_poll(&a); event(TOPIC_MOTOR_FAULTS, 0);
    configured(&b, TOPIC_MOTOR_FAULTS, 1, 0, 0, 0); connection_poll(&b);
    unsigned count = sends;
    ticks += 1000; ++fault_events[0]; diagnostic.overtemperature_warning = true;
    connection_poll(&a); assert(sends == ++count && wire[19] == 1);
    connection_poll(&b); assert(sends == ++count && wire[19] == 1); // Independent consumption.
    ++fault_events[0]; ticks += 1000;
    connection_poll(&a); assert(sends == count); // ON_CHANGE filters repeated observed events.
    connection_poll(&b); assert(sends == ++count); // ALL includes an unchanged observed event.
    for (unsigned i=0; i<4; ++i) { ticks += 100000; connection_poll(&a); connection_poll(&b); }
    assert(sends == count); // Reconciliation isn't a source event.
    ticks += 100000; connection_poll(&a); assert(sends == ++count);
    uint64_t stamp; memcpy(&stamp, wire + 6, 8); assert(stamp == ticks); // Fresh silence refresh.
    // Detect a second condition with no pin transition.
    diagnostic.overtemperature_shutdown = true; ticks += 100000;
    connection_poll(&a); assert(sends == ++count && wire[23] == 1);
    busy = true; ++fault_events[0]; diagnostic.undervoltage = true; ticks += 1000;
    connection_poll(&a); unsigned read_count = reads;
    for (unsigned i=0; i<10; ++i) { ticks += 1000; connection_poll(&a); }
    assert(reads == read_count); // Backpressure cannot cause repeated SPI reads.
    busy = false; connection_poll(&a); assert(sends == ++count && wire[20] == 1);
    // Silence can be shorter than polling and must bypass the cached reading.
    configured(&a, TOPIC_TEMPERATURE, 0, 1, 5000, 250); assert(wire[1] == SUBSCRIBE_CONFIGURED);
    unsubscribe(&a, TOPIC_MOTOR_FAULTS, 0); connection_poll(&a); event(TOPIC_TEMPERATURE, 0);
    read_count = reads; ticks += 250000; busy = true; connection_poll(&a);
    assert(reads == read_count + 1);
    for (unsigned i=0; i<10; ++i) { ticks += 1000; connection_poll(&a); }
    assert(reads == read_count + 1);
    busy = false; connection_poll(&a); event(TOPIC_TEMPERATURE, 0);
    memcpy(&stamp, wire + 6, 8); assert(stamp == ticks - 10000);
    heartbeat(&a, false); count = sends; ticks += 500000; connection_poll(&a); assert(sends == count);
}

static void on_change_tests(void) {
    init(&a); init(&b); heartbeat(&a, true); heartbeat(&b, true);
    subscribe_mode(&a, TOPIC_MOTOR_FAULTS, 0, 100, 2); error(RESPONSE_INVALID_ARGUMENT);
    subscribe_mode(&a, TOPIC_MOTOR_FAULTS, 0, 99, SUBSCRIPTION_MODE_ON_CHANGE); error(RESPONSE_INVALID_ARGUMENT);
    subscribe_mode(&a, TOPIC_MOTOR_FAULTS, 0, 100, SUBSCRIPTION_MODE_ON_CHANGE);
    assert(wire[1] == SUBSCRIBE_WITH_MODE);
    connection_poll(&a); event(TOPIC_MOTOR_FAULTS, 0); // First snapshot needs no interval wait.
    unsigned count = sends;
    for (int i = 0; i < 4; ++i) { ticks += 100000; connection_poll(&a); }
    assert(sends == count); // New timestamps alone never produce a change event.
    diagnostic.overcurrent = true;
    ticks += 50000; connection_poll(&a); assert(sends == count);
    ticks += 50000; connection_poll(&a); event(TOPIC_MOTOR_FAULTS, 0); assert(sends == count + 1);
    assert(wire[22] == 1);
    diagnostic.overcurrent = false;
    ticks += 100000; connection_poll(&a); assert(wire[22] == 0 && sends == count + 2);
    diagnostic.status_valid = false;
    ticks += 100000; connection_poll(&a); assert(wire[17] == 0 && sends == count + 3);
    diagnostic.status_valid = true;
    ticks += 100000; connection_poll(&a); assert(wire[17] == 1 && sends == count + 4);
    count = sends;
    sensor_error = RESPONSE_INTERNAL_ERROR;
    ticks += 100000; connection_poll(&a); assert(sends == count);
    sensor_error = 0;
    ticks += 100000; connection_poll(&a); assert(sends == count + 1); // Recovery snapshot even if same value.
    diagnostic.undervoltage = true;
    busy = true; ticks += 100000; connection_poll(&a); count = sends;
    unsigned reads_before = reads;
    for (unsigned i = 0; i < 4; ++i) { ticks += 1000; connection_poll(&a); }
    assert(reads == reads_before && sends == count);
    busy = false; connection_poll(&a); assert(sends == count + 1 && wire[20]);
    // A slow consumer receives the latest state, not an event backlog.
    diagnostic.undervoltage = false; busy = true;
    ticks += 100000; connection_poll(&a); count = sends;
    diagnostic.undervoltage = true; ticks += 100000; connection_poll(&a);
    busy = false; connection_poll(&a); assert(sends == count);
    // Per-connection baselines: another client receives its own initial state.
    subscribe_mode(&b, TOPIC_MOTOR_FAULTS, 0, 100, SUBSCRIPTION_MODE_ON_CHANGE);
    busy = true; count = sends; connection_poll(&b); assert(sends == count);
    busy = false; connection_poll(&b); assert(sends == count + 1);
    // Unchanged sources cannot starve due periodic sources in the same poll.
    subscribe(&a, TOPIC_MOTOR_FAULTS, 1, 100);
    ticks += 100000; connection_poll(&a); event(TOPIC_MOTOR_FAULTS, 1);
    unsubscribe(&a, TOPIC_MOTOR_FAULTS, 1);
    subscribe_mode(&a, TOPIC_MOTOR_FAULTS, 0, 100, SUBSCRIPTION_MODE_ON_CHANGE);
    connection_poll(&a); event(TOPIC_MOTOR_FAULTS, 0); // Replacement resets its baseline.
    subscribe(&a, TOPIC_MOTOR_FAULTS, 0, 100);
    count = sends; connection_poll(&a); assert(sends == count);
    ticks += 100000; connection_poll(&a); assert(sends == count + 1); // Original wire format still periodic.
    unsubscribe(&a, TOPIC_MOTOR_FAULTS, 0);
    subscribe_mode(&a, TOPIC_TEMPERATURE, 0, 250, SUBSCRIPTION_MODE_ON_CHANGE);
    connection_poll(&a); event(TOPIC_TEMPERATURE, 0);
    count = sends; ticks += 250000; connection_poll(&a); assert(sends == count);
    unsubscribe(&a, TOPIC_TEMPERATURE, 0);
    subscribe_mode(&a, TOPIC_ENCODER_ODOMETRY, 0, 100, SUBSCRIPTION_MODE_ON_CHANGE);
    connection_poll(&a); event(TOPIC_ENCODER_ODOMETRY, 0);
    count = sends; ticks += 100000; connection_poll(&a); assert(sends == count);
    heartbeat(&a, false); heartbeat(&b, false);
    count = sends; ticks += 100000; connection_poll(&a); connection_poll(&b); assert(sends == count);
    for (unsigned i = 0; i < CONNECTION_SUBSCRIPTION_CAPACITY; ++i) assert(!a.subscriptions[i].interval_ms);
}

int main(void) {
    init(&a); init(&b);
    subscribe(&a, TOPIC_IMU, 0, 10); error(RESPONSE_INVALID_ARGUMENT);
    heartbeat(&a, true); heartbeat(&b, true);
    subscribe(&a, 0, 0, 100); error(RESPONSE_INVALID_ARGUMENT);
    subscribe(&a, 255, 0, 100); error(RESPONSE_INVALID_ARGUMENT);
    subscribe(&a, TOPIC_ENCODER_ODOMETRY, 4, 100); error(RESPONSE_INVALID_ARGUMENT);
    subscribe(&a, TOPIC_PLATFORM_ODOMETRY, 1, 100); error(RESPONSE_INVALID_ARGUMENT);
    subscribe(&a, TOPIC_TEMPERATURE, 0, 249); error(RESPONSE_INVALID_ARGUMENT);
    subscribe(&a, TOPIC_IMU, 0, 9); error(RESPONSE_INVALID_ARGUMENT);
    subscribe(&a, TOPIC_IMU, 255, 10); error(RESPONSE_SENSOR_NOT_AVAILABLE);
    running = false;
    subscribe(&a, TOPIC_ENCODER_ODOMETRY, 0, 100); error(RESPONSE_ODOMETRY_NOT_INITIALIZED);
    running = true;
    subscribe(&a, TOPIC_ENCODER_ODOMETRY, 0, 39); error(RESPONSE_INVALID_ARGUMENT);
    subscribe(&a, TOPIC_ENCODER_ODOMETRY, 0, 50); assert(wire[1] == SUBSCRIBE);
    assert(!connection_period_allowed(&a, 26));
    ticks += 50000; connection_poll(&a); event(TOPIC_ENCODER_ODOMETRY, 0);
    encoder_odometry_sample enc; memcpy(&enc, wire + 6, sizeof(enc));
    assert(enc.timestamp_us == 12345 && enc.angle == 1.25);
    controller_command_t old = {.commandType = SUBSCRIBE_ODOMETRY};
    old.properties.subscribe_odometry.source = 0; old.properties.subscribe_odometry.interval_ms = 50;
    request(&b, old);
    ticks += 50000; connection_poll(&b);
    assert(wire[1] == KINISI_MESSAGE_ENCODER_ODOMETRY_EVENT && wire_size == 23);
    connection_poll(&a); event(TOPIC_ENCODER_ODOMETRY, 0); // Per-session event format.
    request(&a, old);
    ticks += 50000; connection_poll(&a);
    assert(wire[1] == KINISI_MESSAGE_ENCODER_ODOMETRY_EVENT);
    subscribe(&a, TOPIC_ENCODER_ODOMETRY, 0, 50);
    ticks += 50000; connection_poll(&a); event(TOPIC_ENCODER_ODOMETRY, 0);
    unsubscribe(&a, TOPIC_ENCODER_ODOMETRY, 0);
    unsubscribe(&a, TOPIC_ENCODER_ODOMETRY, 0); assert(wire[1] == UNSUBSCRIBE);
    subscribe(&a, TOPIC_PLATFORM_ODOMETRY, 0, 50);
    ticks += 50000; connection_poll(&a); event(TOPIC_PLATFORM_ODOMETRY, 0);
    unsubscribe(&a, TOPIC_PLATFORM_ODOMETRY, 0);
    sensor_error = RESPONSE_SAMPLE_NOT_AVAILABLE;
    subscribe(&a, TOPIC_TEMPERATURE, 0, 250); assert(wire[1] == SUBSCRIBE);
    ticks += 250000; unsigned count = sends; connection_poll(&a); assert(sends == count);
    sensor_error = 0; ticks += 250000;
    request(&a, (controller_command_t){.commandType = POLL_TELEMETRY});
    event(TOPIC_TEMPERATURE, 0);
    temperature_sample temp; memcpy(&temp, wire + 6, sizeof(temp));
    assert(temp.celsius == -12.5 && temp.timestamp_us == ticks && temp.clock_quality == 1);
    connection_poll(&a); assert(wire[1] == POLL_TELEMETRY && wire_size == 4);
    unsubscribe(&a, TOPIC_TEMPERATURE, 0);
    subscribe(&a, TOPIC_IMU, 0, 10);
    busy = true; ticks += 10000; connection_poll(&a); count = reads;
    for (unsigned i = 0; i < 8; ++i) { ticks += 1000; connection_poll(&a); }
    assert(reads == count); // Busy transport cannot spin on SPI.
    busy = false; connection_poll(&a); event(TOPIC_IMU, 0);
    imu_telemetry_sample imu; memcpy(&imu, wire + 6, sizeof(imu));
    assert(imu.angular_velocity_z == 6 && imu.timestamp_us == ticks - 8000);
    count = sends; connection_poll(&a); assert(sends == count);
    sensor_error = RESPONSE_INTERNAL_ERROR; ticks += 10000;
    connection_poll(&a); assert(sends == count); // Never resend stale data after failed reads.
    sensor_error = 0; ticks += 10000; connection_poll(&a); event(TOPIC_IMU, 0);
    read_duration_us = 1000;
    ticks += 10000; connection_poll(&a); event(TOPIC_IMU, 0);
    uint64_t previous_sample;
    memcpy(&previous_sample, wire + 6, sizeof(previous_sample));
    ticks += 9000; count = sends; connection_poll(&a); assert(sends == count);
    ticks += 1000; connection_poll(&a); event(TOPIC_IMU, 0);
    memcpy(&imu, wire + 6, sizeof(imu));
    assert(imu.timestamp_us > previous_sample); // Read time must not halve the fresh-sample rate.
    read_duration_us = 0;
    heartbeat(&a, false);
    for (unsigned i = 0; i < CONNECTION_SUBSCRIPTION_CAPACITY; ++i) assert(!a.subscriptions[i].interval_ms);
    heartbeat(&a, true);
    for (unsigned i = 0; i < 11; ++i) { subscribe(&a, TOPIC_IMU, i, 1000); assert(wire[1] == SUBSCRIBE); }
    subscribe(&a, TOPIC_IMU, 11, 1000); error(RESPONSE_SUBSCRIPTION_LIMIT);
    subscribe(&a, TOPIC_IMU, 0, 2000); assert(wire[1] == SUBSCRIBE); // Replace at capacity.
    unsubscribe(&a, TOPIC_IMU, 5); subscribe(&a, TOPIC_IMU, 11, 1000); assert(wire[1] == SUBSCRIBE);
    init(&a); heartbeat(&a, true);
    for (unsigned i = 0; i < CONNECTION_SUBSCRIPTION_CAPACITY; ++i) assert(!a.subscriptions[i].interval_ms);
    subscribe(&a, TOPIC_TEMPERATURE, 0, 250);
    a.heartbeat.timeout_ms = 500;
    ticks += 250000; slow_read = true;
    count = sends;
    request(&a, (controller_command_t){.commandType = POLL_TELEMETRY});
    assert(a.lost && b.lost && sends == count); // Expiry during I/O sends neither sample nor empty frame.
    slow_read = false; init(&a); heartbeat(&a, true);
    subscribe(&a, TOPIC_MOTOR_CURRENT, 4, 100); error(RESPONSE_INVALID_ARGUMENT);
    subscribe(&a, TOPIC_MOTOR_CURRENT, 0, 19); error(RESPONSE_INVALID_ARGUMENT);
    sensor_error = RESPONSE_SAMPLE_OUT_OF_RANGE;
    subscribe(&a, TOPIC_MOTOR_CURRENT, 0, 20); error(RESPONSE_SAMPLE_OUT_OF_RANGE);
    sensor_error = 0;
    for (uint8_t i = 0; i < 4; ++i) {
        subscribe(&a, TOPIC_MOTOR_CURRENT, i, 20); assert(wire[1] == SUBSCRIBE);
    }
    ticks += 20000;
    unsigned seen = 0;
    for (unsigned i = 0; i < 4; ++i) {
        connection_poll(&a); event(TOPIC_MOTOR_CURRENT, wire[5]);
        motor_current_sample sample; memcpy(&sample, wire + 6, sizeof(sample));
        assert(wire[5] < 4 && sample.amps == wire[5] + 0.5 && sample.timestamp_us == ticks);
        seen |= 1U << wire[5];
    }
    assert(seen == 15); // All four sources coexist and keep independent indices.
    sensor_error = RESPONSE_SAMPLE_OUT_OF_RANGE;
    ticks += 20000; count = sends;
    for (unsigned i = 0; i < 4; ++i) connection_poll(&a);
    assert(sends == count); // Do not repeat the old current when the ADC saturates.
    sensor_error = 0; ticks += 20000; connection_poll(&a); event(TOPIC_MOTOR_CURRENT, wire[5]);
    for (uint8_t i = 0; i < 4; ++i) unsubscribe(&a, TOPIC_MOTOR_CURRENT, i);
    count = sends; ticks += 20000; connection_poll(&a); assert(sends == count);
    subscribe(&a, TOPIC_MOTOR_FAULTS, 4, 100); error(RESPONSE_INVALID_ARGUMENT);
    subscribe(&a, TOPIC_MOTOR_FAULTS, 0, 99); error(RESPONSE_INVALID_ARGUMENT);
    for (uint8_t i = 0; i < 4; ++i) { subscribe(&a, TOPIC_MOTOR_FAULTS, i, 100); assert(wire[1] == SUBSCRIBE); }
    ticks += 100000; seen = 0;
    for (unsigned i = 0; i < 4; ++i) {
        connection_poll(&a); event(TOPIC_MOTOR_FAULTS, wire[5]);
        motor_fault_sample sample; memcpy(&sample, wire + 6, sizeof(sample));
        assert(wire_size == 26 && sample.status_valid && sample.overcurrent == (wire[5] == 2));
        assert(sample.timestamp_us == ticks); seen |= 1U << wire[5];
    }
    assert(seen == 15);
    for (uint8_t i = 0; i < 4; ++i) unsubscribe(&a, TOPIC_MOTOR_FAULTS, i);
    count = sends; ticks += 100000; connection_poll(&a); assert(sends == count);
    on_change_tests();
    configured_tests();
    puts("Generic topics: framing, current sensing, bounds, cadence, backpressure, recovery, capacity and expiry passed");
}
