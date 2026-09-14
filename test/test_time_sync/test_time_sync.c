//------------------------------------------------------------
// File name: test_time_sync.c
// Description: Verify clock synchronization and connection lifecycle behavior.
//------------------------------------------------------------
#include "connection.h"
#include "initialization.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

static uint64_t now_us;
static uint8_t wire[255], wire_length;
static unsigned sends, handled;
static bool busy;
/**
 * @brief Return the fixture's controllable monotonic timestamp.
 */
static uint64_t now(void) { return now_us; }
/**
 * @brief Capture a transmitted frame, or simulate a busy transport without accepting it.
 */
static bool send_frame(uint8_t *bytes, uint8_t size)
{
    if (busy) return false;
    memcpy(wire, bytes, size); wire_length = size; ++sends;
    return true;
}
/**
 * @brief Validate the fixture command and record accepted executions for assertions.
 */
static uint8_t handler(controller_command_t *cmd, protocol_send_fn reply)
{
    ++handled;
    assert(connection_current_clock());
    if (cmd->commandType == INIT) {
        uint8_t error = initialization_validate(cmd);
        if (error) return error;
        init_response response = initialization_response();
        reply((uint8_t *)&response, sizeof(response));
    }
    return RESPONSE_OK;
}
/**
 * @brief Send a fixture INIT and verify identity precedes READY.
 */
static void init(connection_t *c, uint8_t capabilities)
{
    uint8_t request[] = {INIT, 42, 0, 1, 1, 0, 0, 2, 0, 0, capabilities};
    connection_receive(c, request, sizeof(request));
    connection_poll(c);
    assert(wire_length == 19 && wire[1] == INIT && wire[2] == 42);
    assert(!c->ready_announced);
}
/**
 * @brief Dispatch a fixture command and poll its queued reply.
 */
static void command(connection_t *c, uint8_t cmd)
{
    uint8_t request[] = {cmd, 99, 0};
    connection_receive(c, request, sizeof(request));
    connection_poll(c);
}
/**
 * @brief Verify the captured ERROR frame identifies the expected command and code.
 */
static void expect_error(uint8_t cmd, uint8_t error)
{
    assert(wire_length == 6 && wire[1] == KINISI_MESSAGE_ERROR);
    assert(wire[4] == cmd && wire[5] == error);
}
/**
 * @brief Inject host receive/send timestamps using a selected controller message ID.
 */
static void respond(connection_t *c, uint16_t id, uint64_t h2, uint64_t h3)
{
    controller_command_t response = {0};
    response.commandType = TIME_SYNC_RESPONSE;
    response.message_id = id;
    response.properties.time_sync_response.host_receive_us = h2;
    response.properties.time_sync_response.host_send_us = h3;
    connection_receive(c, (uint8_t *)&response, command_request_size(TIME_SYNC_RESPONSE));
}
/**
 * @brief Complete a three-sample burst with a known host-clock offset.
 */
static void complete_burst(connection_t *c, uint64_t offset)
{
    for (unsigned i = 0; i < TIME_SYNC_BURST_SIZE; ++i) {
        connection_poll(c);
        assert(wire_length == 4 && wire[1] == KINISI_MESSAGE_TIME_SYNC_REQUEST);
        uint64_t t1 = c->clock.sent_us;
        now_us += 1200;
        respond(c, c->clock.pending_id, t1 + offset + 500, t1 + offset + 700);
        assert(!c->output_length && !c->clock.pending);
    }
    connection_poll(c); // Commit lowest-delay sample after the full burst.
    assert(c->clock.ready && c->clock.offset_us == (int64_t)offset);
}
/**
 * @brief Exercise clock arithmetic, sample rejection, retries, freshness, and conversion bounds.
 */
static void engine_tests(void)
{
    time_sync_t c = {0}; uint64_t converted;
    assert(!time_sync_due(&c, 0));
    assert(!time_sync_convert(&c, 1, &converted));
    time_sync_init(&c, false, 100);
    assert(c.ready && !time_sync_due(&c, UINT64_MAX));
    assert(time_sync_convert(&c, 12345, &converted) && converted == 12345);
    assert(time_sync_quality(&c, UINT64_MAX) == CLOCK_QUALITY_VALID);
    time_sync_init(&c, true, 100);
    assert(!c.ready && time_sync_due(&c, 100));
    // Symmetric 10 us legs, 20 us host processing: offset 1,000,000 us.
    time_sync_sent(&c, 1, 100);
    assert(!time_sync_receive(&c, 2, 1000110, 1000130, 140) && c.pending);
    assert(time_sync_receive(&c, 1, 1000110, 1000130, 140));
    assert(!time_sync_receive(&c, 1, 1000110, 1000130, 140));
    assert(c.best_offset_us == 1000000 && c.best_delay_us == 20);
    assert(!time_sync_finish(&c, 140));
    assert(time_sync_due(&c, 200));
    time_sync_sent(&c, 2, 200);
    // More asymmetric transport must not replace the faster sample.
    assert(time_sync_receive(&c, 2, 1000260, 1000280, 300));
    assert(c.best_offset_us == 1000000);
    assert(time_sync_due(&c, 300));
    time_sync_sent(&c, 3, 300);
    assert(!time_sync_receive(&c, 3, 1000200, 1000199, 400));
    assert(time_sync_finish(&c, 400) == 1);
    assert(time_sync_convert(&c, 800, &converted) && converted == 1000800);
    assert(!time_sync_due(&c, 400 + TIME_SYNC_DEFAULT_INTERVAL_US - 1));
    assert(time_sync_due(&c, 400 + TIME_SYNC_DEFAULT_INTERVAL_US));
    assert(time_sync_quality(&c, 400 + TIME_SYNC_DEFAULT_INTERVAL_US + 3000001) == CLOCK_QUALITY_STALE);
    // Three lost replies retain the previous mapping, with stale quality.
    uint64_t t = c.next_us;
    for (unsigned i = 0; i < 3; ++i) {
        assert(time_sync_due(&c, t));
        time_sync_sent(&c, 4 + i, t);
        assert(!time_sync_due(&c, t + TIME_SYNC_TIMEOUT_US - 1));
        t += TIME_SYNC_TIMEOUT_US;
    }
    assert(!time_sync_due(&c, t));
    assert(time_sync_finish(&c, t) == -1 && c.ready && c.offset_us == 1000000);
    time_sync_init(&c, true, 0);
    assert(c.sequence == 6 && !c.ready);
    assert(time_sync_due(&c, 0));
    time_sync_sent(&c, 7, 100);
    assert(!time_sync_receive(&c, 7, 100, 300, 200)); // processing > RTT
    time_sync_sent(&c, 8, 100);
    assert(!time_sync_receive(&c, 8, 100, 100, 100 + TIME_SYNC_MAX_DELAY_US + 1));
    time_sync_sent(&c, 9, 100);
    assert(!time_sync_receive(&c, 9, 100, UINT64_MAX, 200));
    assert(time_sync_finish(&c, 200) == -1 && !c.ready);
    c.ready = true; c.offset_us = -100;
    assert(!time_sync_convert(&c, 99, &converted));
    assert(time_sync_convert(&c, 100, &converted) && converted == 0);
    c.offset_us = 100;
    assert(!time_sync_convert(&c, INT64_MAX, &converted));
    assert(!time_sync_convert(&c, UINT64_MAX, &converted));
}
/**
 * @brief Exercise INIT/READY ordering, periodic sync, connection isolation, and reset behavior.
 */
static void connection_tests(void)
{
    connection_t c, other;
    connection_init(&c, handler, send_frame, now);
    connection_init(&other, handler, send_frame, now);
    command(&c, GET_PLATFORM_ODOMETRY);
    expect_error(GET_PLATFORM_ODOMETRY, RESPONSE_INIT_REQUIRED);
    assert(handled == 0);
    command(&c, SET_TIME_SYNC_INTERVAL); // Missing payload is a framing error first.
    expect_error(SET_TIME_SYNC_INTERVAL, RESPONSE_INVALID_LENGTH);
    {
        controller_command_t setup = {0};
        setup.commandType = SET_TIME_SYNC_INTERVAL; setup.message_id = 15;
        setup.properties.set_time_sync_interval.interval_ms = 10000;
        connection_receive(&c, (uint8_t *)&setup, command_request_size(SET_TIME_SYNC_INTERVAL));
        connection_poll(&c);
        expect_error(SET_TIME_SYNC_INTERVAL, RESPONSE_INIT_REQUIRED);
        setup.commandType = SET_ODOMETRY_FREQUENCY;
        setup.properties.set_odometry_frequency.frequency = 50;
        connection_receive(&c, (uint8_t *)&setup, command_request_size(SET_ODOMETRY_FREQUENCY));
        connection_poll(&c);
        assert(wire[1] == SET_ODOMETRY_FREQUENCY && wire_length == 4);
    }
    command(&c, STOP_PLATFORM_ODOMETRY);
    assert(wire[1] == STOP_PLATFORM_ODOMETRY && wire_length == 4);
    command(&c, RESET_PLATFORM_ODOMETRY);
    assert(wire[1] == RESET_PLATFORM_ODOMETRY && wire_length == 4);
    command(&c, GET_ODOMETRY_FREQUENCY);
    assert(wire[1] == GET_ODOMETRY_FREQUENCY && wire_length == 4);
    init(&c, CLIENT_CAP_WALL_CLOCK);
    assert(!connection_can_receive(&c)); // Scheduled sync cannot be starved by GETs.
    busy = true; unsigned before = sends;
    connection_poll(&c);
    assert(sends == before && !c.clock.pending && c.clock.attempts == 0);
    busy = false;
    command(&c, GET_PLATFORM_ODOMETRY);
    expect_error(GET_PLATFORM_ODOMETRY, RESPONSE_CLOCK_NOT_READY);
    complete_burst(&c, 1700000000000000ULL);
    assert(!c.ready_announced);
    // READY itself must have been handed to the transport before GET is accepted.
    busy = true; connection_poll(&c); assert(!c.ready_announced);
    busy = false; connection_poll(&c);
    assert(connection_can_receive(&c));
    assert(wire_length == 5 && wire[1] == KINISI_MESSAGE_READY && wire[2] == 42 && wire[4] == 1);
    command(&c, GET_PLATFORM_ODOMETRY);
    assert(wire_length == 4 && wire[1] == GET_PLATFORM_ODOMETRY && wire[2] == 99);
    command(&other, GET_PLATFORM_ODOMETRY);
    expect_error(GET_PLATFORM_ODOMETRY, RESPONSE_INIT_REQUIRED);
    init(&other, 0); connection_poll(&other);
    assert(other.ready_announced && wire[1] == KINISI_MESSAGE_READY && wire[4] == 0);
    assert(c.clock.mode == CLOCK_MODE_WALL);
    // Periodic burst runs without GET or subscriptions and does not repeat READY.
    now_us = c.clock.next_us;
    assert(!connection_can_receive(&c));
    complete_burst(&c, 1700000000000100ULL);
    before = sends; connection_poll(&c); assert(sends == before);
    controller_command_t interval = {0};
    interval.commandType = SET_TIME_SYNC_INTERVAL; interval.message_id = 50;
    interval.properties.set_time_sync_interval.interval_ms = 60000;
    connection_receive(&c, (uint8_t *)&interval, command_request_size(SET_TIME_SYNC_INTERVAL));
    connection_poll(&c);
    assert(c.clock.interval_us == 60000000 && c.clock.next_us == now_us + 60000000);
    interval.properties.set_time_sync_interval.interval_ms = 0;
    connection_receive(&c, (uint8_t *)&interval, command_request_size(SET_TIME_SYNC_INTERVAL));
    connection_poll(&c); expect_error(SET_TIME_SYNC_INTERVAL, RESPONSE_INVALID_ARGUMENT);
    command(&c, GET_TIME_STATUS);
    assert(wire_length == 4 + sizeof(time_status));
    time_status status; memcpy(&status, wire + 4, sizeof(status));
    assert(status.clock_mode == 1 && status.clock_quality == 1 && status.interval_ms == 60000);
    uint16_t old_id = c.clock.sequence;
    init(&c, 1); assert(c.clock.sequence == old_id && !c.clock.ready);
    connection_poll(&c); uint16_t pending_id = c.clock.pending_id;
    respond(&c, old_id, now_us, now_us);
    connection_poll(&c); expect_error(TIME_SYNC_RESPONSE, RESPONSE_INVALID_ARGUMENT);
    assert(c.clock.pending && c.clock.pending_id == pending_id);
    uint8_t malformed[] = {TIME_SYNC_RESPONSE, 55, 0};
    connection_receive(&c, malformed, sizeof(malformed)); connection_poll(&c);
    expect_error(TIME_SYNC_RESPONSE, RESPONSE_INVALID_LENGTH);
    assert(c.clock.pending);
    // Reset discards pending sync, readiness, and any buffered reply.
    connection_reset(&c);
    assert(!c.clock.initialized && !c.clock.pending && !c.ready_announced);
    command(&c, GET_PLATFORM_ODOMETRY); expect_error(GET_PLATFORM_ODOMETRY, RESPONSE_INIT_REQUIRED);
    c.clock.sequence = 65535;
    init(&c, 1); connection_poll(&c);
    assert(c.clock.pending_id == 1);
    // Initial timeout emits ERROR for INIT and never releases the odometry gate.
    for (unsigned i = 0; i < 3; ++i) {
        now_us += TIME_SYNC_TIMEOUT_US;
        connection_poll(&c);
    }
    connection_poll(&c);
    expect_error(INIT, RESPONSE_TIME_SYNC_FAILED);
    assert(wire[2] == 42 && !c.clock.ready && !c.ready_announced);
    now_us = c.clock.next_us;
    complete_burst(&c, 1700000000000000ULL);
    connection_poll(&c); assert(c.ready_announced);
    assert(!connection_current_clock());
}
/**
 * @brief Run this file's assertions and return zero when all checks pass.
 */
int main(void)
{
    _Static_assert(sizeof(encoder_odometry_sample) == 18, "encoder wire layout");
    _Static_assert(sizeof(platform_odometry_sample) == 34, "platform wire layout");
    _Static_assert(sizeof(time_status) == 14, "time status wire layout");
    engine_tests(); connection_tests();
    puts("Time sync arithmetic, retries, readiness, framing, isolation, intervals and reset passed");
    return 0;
}
