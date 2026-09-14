//------------------------------------------------------------
// File name: test_odometry_timestamp.c
// Description: Verify production odometry timestamps and sample lifecycle.
//------------------------------------------------------------
#include "odometry_manager.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "hw_encoder.h"
#include "protocol.h"
#include <assert.h>
#include <math.h>
#include <setjmp.h>
#include <stdio.h>

static void (*task_fn)(void *);
static void *task_arg;
static jmp_buf task_exit;
static unsigned delays, locked;
static uint16_t counter;
static uint64_t clock_us;
static uint8_t platform_enabled = 1;
/**
 * @brief Report whether the mocked task handle exists.
 */
osThreadState_t osThreadGetState(osThreadId_t thread) { return thread ? 0 : osThreadError; }
/**
 * @brief Capture the production task entry point and argument without launching a thread.
 */
osThreadId_t osThreadNew(void (*fn)(void *), void *arg, const osThreadAttr_t *attr)
{
    (void)attr; task_fn = fn; task_arg = arg; return arg;
}
/**
 * @brief Provide the fixture's initial RTOS tick value.
 */
TickType_t xTaskGetTickCount(void) { return 0; }
/**
 * @brief Stop the fixture after one complete production odometry-task iteration.
 */
void vTaskDelayUntil(TickType_t *last, TickType_t period)
{
    (void)last; (void)period;
    assert(!locked);
    if (delays++) longjmp(task_exit, 1); // Execute exactly one production task iteration.
}
/**
 * @brief Return the fixture mutex token.
 */
SemaphoreHandle_t xSemaphoreCreateMutex(void) { return &locked; }
/**
 * @brief Assert exclusive access and mark the fixture mutex locked.
 */
int xSemaphoreTake(SemaphoreHandle_t mutex, uint32_t timeout)
{
    (void)timeout; assert(mutex && !locked); locked = 1; return 1;
}
/**
 * @brief Assert mutex ownership and release the fixture lock.
 */
void xSemaphoreGive(SemaphoreHandle_t mutex) { assert(mutex && locked); locked = 0; }
/**
 * @brief Return the fixture acquisition time while asserting the odometry lock is held.
 */
uint64_t hw_clock_microseconds(void) { assert(locked); return clock_us; }
/**
 * @brief Return the fixture encoder count while asserting the odometry lock is held.
 */
static uint16_t get_value(uint8_t index) { (void)index; assert(locked); return counter; }
/**
 * @brief Report the mocked encoder as initialized.
 */
static uint8_t initialized(uint8_t index) { (void)index; return 1; }
/**
 * @brief Return the mocked encoder resolution in ticks per revolution.
 */
static double resolution(uint8_t index) { (void)index; return 100; }
/**
 * @brief Provide fixed encoder callbacks for the production odometry task fixture.
 */
const hw_encoder_interface_t *get_encoder_interface(void)
{
    static const hw_encoder_interface_t encoder = {
        .get_value = get_value, .is_initialized = initialized, .get_resolution = resolution
    };
    return &encoder;
}
/**
 * @brief Return the fixture platform integration flag.
 */
uint8_t platform_is_odometry_enabled(void) { return platform_enabled; }
/**
 * @brief Produce a known body-frame increment under the odometry lock.
 */
platform_odometry_t platform_update_odometry(uint8_t *indexes, double *deltas, uint8_t count)
{
    (void)indexes; (void)deltas; (void)count; assert(locked);
    platform_odometry_t delta = {.x = 1}; return delta;
}
/**
 * @brief Run exactly one production odometry-task iteration using the fixture scheduler.
 */
static void step(void)
{
    delays = 0;
    if (!setjmp(task_exit)) task_fn(task_arg);
}
/**
 * @brief Run this file's assertions and return zero when all checks pass.
 */
int main(void)
{
    uint64_t sampled;
    double angle;
    platform_odometry_t pose;
    assert(encoder_get_odometry_sample(0, &angle, &sampled) == RESPONSE_ODOMETRY_NOT_INITIALIZED);
    assert(odometry_manager_get_platform_sample(&pose, &sampled) == RESPONSE_ODOMETRY_NOT_INITIALIZED);
    encoder_start_odometry(0);
    assert(encoder_get_odometry_sample(0, &angle, &sampled) == RESPONSE_SAMPLE_NOT_AVAILABLE);
    assert(encoder_get_odometry_sample(1, &angle, &sampled) == RESPONSE_ODOMETRY_NOT_INITIALIZED);
    counter = 25; clock_us = 20000; step();
    clock_us = 90000; // GET occurs much later than acquisition.
    assert(encoder_get_odometry_sample(0, &angle, &sampled) == RESPONSE_OK);
    assert(sampled == 20000 && fabs(angle - 1.5707963267948966) < 1e-10);
    assert(odometry_manager_get_platform_sample(&pose, &sampled) == RESPONSE_OK);
    assert(sampled == 20000 && pose.x == 1);
    encoder_reset_odometry(0);
    odometry_manager_reset_platform_odometry();
    assert(encoder_get_odometry_sample(0, &angle, &sampled) == RESPONSE_SAMPLE_NOT_AVAILABLE);
    assert(odometry_manager_get_platform_sample(&pose, &sampled) == RESPONSE_SAMPLE_NOT_AVAILABLE);
    counter = 50; clock_us = 100000; step();
    assert(encoder_get_odometry_sample(0, &angle, &sampled) == RESPONSE_OK && sampled == 100000);
    encoder_stop_odometry(0);
    assert(encoder_get_odometry_sample(0, &angle, &sampled) == RESPONSE_ODOMETRY_NOT_INITIALIZED);
    assert(encoder_get_odometry_sample(4, &angle, &sampled) == RESPONSE_INVALID_ARGUMENT);
    platform_enabled = 0;
    assert(odometry_manager_get_platform_sample(&pose, &sampled) == RESPONSE_ODOMETRY_NOT_INITIALIZED);
    platform_enabled = 1;
    odometry_manager_invalidate_platform_sample();
    assert(odometry_manager_get_platform_sample(&pose, &sampled) == RESPONSE_SAMPLE_NOT_AVAILABLE);
    assert(pose.x == 1); // Restart invalidates the timestamp, not the accumulated pose.
    assert(!locked);
    puts("Odometry acquisition timestamps, cached GET, atomic snapshots and reset invalidation passed");
    return 0;
}
