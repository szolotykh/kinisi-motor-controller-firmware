//------------------------------------------------------------
// File name: test_session_features.c
// Description: Exercise watchdog and telemetry timing against production session code.
//------------------------------------------------------------
#include "connection.h"
#include "initialization.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint64_t ticks;
static unsigned stops, motions, sent;
static bool busy, calculating;
static uint8_t frame[255], frame_size;
static uint32_t calculation_ms = 20;
static connection_t usb, i2c;

/** @brief Return controlled board uptime, unrelated to wall-clock synchronization. */
static uint64_t now(void) { return ticks; }
/** @brief Capture a complete frame or model nonblocking transport backpressure. */
static bool transmit(uint8_t *data, uint8_t size)
{
    if (busy) return false;
    memcpy(frame, data, size); frame_size = size; ++sent;
    return true;
}
/** @brief Model the production global loss policy for shared motor resources. */
static void stop(void)
{
    ++stops;
    connection_invalidate(&usb);
    connection_invalidate(&i2c);
}
/** @brief Return the actual calculation period used by the sample producer. */
static uint32_t period(void) { return calculation_ms; }
/** @brief Apply subscription constraints across both transport sessions. */
static bool allowed(uint32_t ms)
{
    return connection_period_allowed(&usb, ms) && connection_period_allowed(&i2c, ms);
}
/** @brief Supply samples whose measurement time precedes the independent send deadline. */
static uint8_t execute(controller_command_t *request, protocol_send_fn reply)
{
    if (request->commandType == INIT) return initialization_validate(request);
    if (request->commandType == SET_MOTOR_SPEED) ++motions;
    if (request->commandType == SET_ODOMETRY_FREQUENCY)
        calculation_ms = 1000 / request->properties.set_odometry_frequency.frequency;
    if (request->commandType == GET_ENCODER_ODOMETRY || request->commandType == GET_PLATFORM_ODOMETRY) {
        if (!calculating) return RESPONSE_ODOMETRY_NOT_INITIALIZED;
        platform_odometry_sample sample = {0};
        sample.timestamp_us = ticks / (calculation_ms * 1000ULL) * (calculation_ms * 1000ULL);
        sample.clock_quality = 1;
        reply((uint8_t *)&sample, request->commandType == GET_ENCODER_ODOMETRY ? sizeof(encoder_odometry_sample) : sizeof(sample));
    }
    return RESPONSE_OK;
}
/** @brief Send one validated wire request and drain its ACK or ERROR. */
static void request(connection_t *c, controller_command_t cmd)
{
    cmd.message_id = 12;
    connection_receive(c, (uint8_t *)&cmd, command_request_size(cmd.commandType));
    connection_poll(c);
}
/** @brief Establish uptime mode, preserving the normal INIT/READY ordering. */
static void init(connection_t *c)
{
    controller_command_t cmd = {0};
    cmd.commandType = INIT;
    cmd.properties.init.protocol_major = 2;
    request(c, cmd);
    connection_poll(c);
    assert(c->ready_announced && !c->lost);
}
/** @brief Configure monitoring through the real protocol rather than editing session state. */
static void heartbeat(connection_t *c, bool enabled)
{
    controller_command_t cmd = {0};
    cmd.commandType = SET_HEARTBEAT_CONFIG;
    cmd.properties.set_heartbeat_config.enabled = enabled;
    cmd.properties.set_heartbeat_config.timeout_ms = 500;
    request(c, cmd);
    assert(frame[1] == SET_HEARTBEAT_CONFIG && frame_size == 4);
}
/** @brief Subscribe one source and leave the captured result available for assertions. */
static void subscribe(connection_t *c, uint8_t source, uint32_t ms)
{
    controller_command_t cmd = {0};
    cmd.commandType = SUBSCRIBE_ODOMETRY;
    cmd.properties.subscribe_odometry.source = source;
    cmd.properties.subscribe_odometry.interval_ms = ms;
    request(c, cmd);
}
/** @brief Check the error response includes the original command and message ID. */
static void error(uint8_t command, uint8_t code)
{
    assert(frame_size == 6 && frame[1] == KINISI_MESSAGE_ERROR && frame[2] == 12);
    assert(frame[4] == command && frame[5] == code);
}

/** @brief Verify loss, rate validation, sample selection and transport congestion behavior. */
int main(void)
{
    connection_init(&usb, execute, transmit, now);
    connection_init(&i2c, execute, transmit, now);
    usb.services = i2c.services = (connection_services_t){stop, period, allowed};
    init(&usb); init(&i2c);
    ticks = 10000000;
    connection_poll(&usb);
    assert(!stops && usb.heartbeat.timeout_ms == 500); // Old clients remain opt-in.
    heartbeat(&usb, true);
    ticks += 400000;
    request(&usb, (controller_command_t){.commandType = GET_TIME_STATUS});
    ticks += 400000;
    connection_poll(&usb);
    assert(!stops); // Ordinary traffic substitutes for PING.
    uint8_t malformed[] = {PING, 12, 0, 99};
    connection_receive(&usb, malformed, sizeof(malformed));
    busy = true;
    ticks += 100000;
    connection_poll(&usb);
    assert(stops == 1 && usb.lost && i2c.lost && !usb.output_length);
    busy = false;
    controller_command_t motion = {.commandType = SET_MOTOR_SPEED};
    motion.properties.set_motor_speed.pwm = 25;
    request(&usb, motion);
    error(SET_MOTOR_SPEED, RESPONSE_INIT_REQUIRED);
    assert(!motions);
    request(&usb, (controller_command_t){.commandType = STOP_MOTOR});
    assert(frame[1] == STOP_MOTOR);
    connection_disconnect(&usb);
    assert(stops == 1); // Loss handling is idempotent.
    init(&usb); heartbeat(&usb, true);
    request(&usb, motion);
    assert(motions == 1);

    subscribe(&usb, 0, 50);
    error(SUBSCRIBE_ODOMETRY, RESPONSE_ODOMETRY_NOT_INITIALIZED);
    calculating = true;
    subscribe(&usb, 0, 39);
    error(SUBSCRIBE_ODOMETRY, RESPONSE_INVALID_ARGUMENT);
    subscribe(&usb, 0, 50); // Deliberately not a multiple of 20 ms.
    assert(frame[1] == SUBSCRIBE_ODOMETRY);
    uint64_t first = usb.subscriptions[0].next_us;
    ticks = first - 1;
    unsigned previous = sent;
    connection_poll(&usb);
    assert(sent == previous);
    ticks = first;
    connection_poll(&usb);
    assert(frame[1] == KINISI_MESSAGE_ENCODER_ODOMETRY_EVENT && frame_size == 23);
    assert(frame[2] == 0 && frame[3] == 0 && frame[4] == 0);
    uint64_t measured;
    memcpy(&measured, frame + 5, 8);
    assert(measured <= first && first - measured < 20000);
    assert(usb.subscriptions[0].next_us == first + 50000);
    busy = true;
    ticks = first + 150000;
    connection_poll(&usb);
    busy = false;
    previous = sent;
    connection_poll(&usb);
    assert(sent == previous + 1);
    memcpy(&measured, frame + 5, 8);
    assert(ticks - measured < 20000); // No stale frame accumulated during backpressure.
    assert(usb.subscriptions[0].next_us == first + 200000);
    connection_poll(&usb);
    assert(sent == previous + 1); // No catch-up burst.

    init(&i2c);
    controller_command_t frequency = {.commandType = SET_ODOMETRY_FREQUENCY};
    frequency.properties.set_odometry_frequency.frequency = 25;
    request(&i2c, frequency);
    error(SET_ODOMETRY_FREQUENCY, RESPONSE_INVALID_ARGUMENT);
    assert(calculation_ms == 20);
    heartbeat(&usb, false);
    assert(!usb.subscriptions[0].interval_ms);
    request(&i2c, frequency);
    assert(frame[1] == SET_ODOMETRY_FREQUENCY && calculation_ms == 40);
    heartbeat(&usb, true);
    subscribe(&usb, 4, 100);
    ticks += 100000;
    connection_poll(&usb);
    assert(frame[1] == KINISI_MESSAGE_PLATFORM_ODOMETRY_EVENT && frame_size == 38);
    ticks += 100000;
    request(&usb, (controller_command_t){.commandType = POLL_TELEMETRY});
    assert(frame[1] == KINISI_MESSAGE_PLATFORM_ODOMETRY_EVENT && usb.output_length == 4);
    connection_poll(&usb);
    assert(frame[1] == POLL_TELEMETRY && frame_size == 4 && !usb.output_length);
    calculating = false;
    ticks += 100000;
    request(&usb, (controller_command_t){.commandType = POLL_TELEMETRY});
    assert(frame[1] == POLL_TELEMETRY && frame_size == 4); // No sample still produces an ACK.
    connection_disconnect(&usb);
    assert(stops == 2 && !usb.subscriptions[4].interval_ms);
    puts("Watchdog deadlines, valid traffic, recovery latch, subscriptions, timestamps, rate changes and backpressure passed");
    return 0;
}
