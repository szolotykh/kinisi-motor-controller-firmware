#pragma once
// Included once by the command dispatcher, also compiled by the native board.
#include "imu_tracking.h"
#include "imu_only_estimator.h"
#include "hw_clock.h"
#include "hw_imu.h"
#include "commands.h"
#include <cmsis_os.h>
#include <semphr.h>

#define IMU_TRACKING_WINDOW_US 1000000U
#define IMU_TRACKING_PUBLIC_LATENCY_US 50000U
static imu_only_state_t tracker;
static SemaphoreHandle_t tracking_mutex;
static osThreadId_t tracking_worker;
static bool (*tracking_start_producer)(void);
static void (*tracking_begin)(uint32_t);
static imu_tracking_reader_t tracking_reader;
static uint32_t tracking_generation, acquisition_epoch;
static uint64_t tracking_origin, published_sample;
static uint32_t tracking_elapsed;

void imu_tracking_install_producer(bool (*start)(void), void (*begin)(uint32_t), imu_tracking_reader_t read)
{
    tracking_start_producer = start;
    tracking_begin = begin;
    tracking_reader = read;
}
uint32_t imu_tracking_epoch(void)
{
    return __atomic_load_n(&acquisition_epoch, __ATOMIC_ACQUIRE);
}

static void tracking_publish_active(void)
{
    __atomic_store_n(&acquisition_epoch, imu_only_running(&tracker) ? tracking_generation : 0,
                     __ATOMIC_RELEASE);
}

static void tracking_deadline(uint64_t now)
{
    if (tracker.health == IMU_ONLY_TRACKING && now >= tracking_origin) {
        uint64_t elapsed = now - tracking_origin;
        tracking_elapsed = elapsed < IMU_TRACKING_WINDOW_US ? (uint32_t)elapsed : IMU_TRACKING_WINDOW_US;
    }
    imu_only_expire(&tracker, now);
    if (tracker.health == IMU_ONLY_TRACKING && now >= tracking_origin &&
        now - tracking_origin >= IMU_TRACKING_WINDOW_US + IMU_TRACKING_PUBLIC_LATENCY_US)
        imu_only_fail(&tracker, IMU_ONLY_WINDOW_EXPIRED);
    tracking_publish_active();
}

static void tracking_update(void)
{
    if (!imu_only_running(&tracker)) return;
    // The physical API reads the existing producer cache; it never waits for SPI.
    hw_imu_sample_t raw;
    uint64_t a = 0, g = 0;
    hw_imu_status_t result = tracking_reader ? tracking_reader(hw_clock_microseconds(), &raw, &a, &g)
                                             : hw_imu_read_observation(0, &raw, &a, &g);
    uint64_t now = hw_clock_microseconds();
    tracking_deadline(now);
    if (result == HW_IMU_OK) {
        imu_only_sample_t sample = {a,
                                    g,
                                    raw.acceleration_x,
                                    raw.acceleration_y,
                                    raw.acceleration_z,
                                    raw.angular_velocity_x,
                                    raw.angular_velocity_y,
                                    raw.angular_velocity_z};
        bool calibrating = tracker.health == IMU_ONLY_CALIBRATING;
        imu_only_step(&tracker, now, &sample);
        if (tracker.health == IMU_ONLY_TRACKING) {
            if (calibrating) tracking_origin = tracker.sample_us;
            if (tracker.sample_us - tracking_origin >= IMU_TRACKING_WINDOW_US) {
                imu_only_fail(&tracker, IMU_ONLY_WINDOW_EXPIRED);
            } else {
                published_sample = tracker.sample_us;
            }
        }
    } else if (result != HW_IMU_NOT_READY) {
        imu_only_fail(&tracker, result == HW_IMU_STALE ? IMU_ONLY_STALE : IMU_ONLY_UNAVAILABLE);
    }
    tracking_publish_active();
}

static void tracking_poll(void *unused)
{
    (void)unused;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        // START can arm the cursor between wakes while fusion keeps acquiring
        // 100 Hz observations. Bound the first service delay on every restart.
        vTaskDelayUntil(&last, pdMS_TO_TICKS(2));
        if (!imu_tracking_epoch() || !xSemaphoreTake(tracking_mutex, portMAX_DELAY))
            continue;
        tracking_update();
        xSemaphoreGive(tracking_mutex);
    }
}

void imu_tracking_inhibit(void)
{
    if (tracking_mutex && xSemaphoreTake(tracking_mutex, portMAX_DELAY)) {
        tracking_deadline(hw_clock_microseconds());
        imu_only_fail(&tracker, IMU_ONLY_INHIBITED);
        tracking_publish_active();
        xSemaphoreGive(tracking_mutex);
    }
}

static uint8_t tracking_configure(uint8_t action, uint8_t confirmed, double yaw, uint8_t power_error)
{
    bool stopping = action == IMU_TRACKING_ACTION_STOP;
    if (action > IMU_TRACKING_ACTION_RECALIBRATE || !isfinite(yaw) || fabs(yaw) > 3.14159265358979323846 ||
        (stopping ? (confirmed != 0 || yaw != 0) : confirmed != 1))
        return RESPONSE_INVALID_ARGUMENT;
    if (!stopping && power_error)
        return power_error;
    if (!tracking_mutex)
        tracking_mutex = xSemaphoreCreateMutex();
    if (!tracking_mutex || !xSemaphoreTake(tracking_mutex, portMAX_DELAY))
        return RESPONSE_INTERNAL_ERROR;
    uint8_t result = RESPONSE_OK;
    if (stopping) {
        imu_only_stop(&tracker);
        tracking_origin = published_sample = 0;
        tracking_elapsed = 0;
    } else if ((tracking_start_producer && !tracking_start_producer()) ||
               (!tracking_worker &&
                !(tracking_worker = osThreadNew(tracking_poll, NULL,
                                                &(osThreadAttr_t){.name = "InertialTrack",
                                                                  .priority = osPriorityNormal,
                                                                  .stack_size = 1536})))) {
        result = RESPONSE_INTERNAL_ERROR;
    } else {
        tracking_generation = (tracking_generation + 1u) | UINT32_C(0x80000000);
        if (tracking_begin)
            tracking_begin(tracking_generation);
        imu_only_start(&tracker, hw_clock_microseconds(), yaw, true);
        tracking_origin = published_sample = 0;
        tracking_elapsed = 0;
    }
    tracking_publish_active();
    xSemaphoreGive(tracking_mutex);
    return result;
}

static uint32_t tracking_age(uint64_t now, uint64_t stamp)
{
    return !stamp || stamp > now || now - stamp >= UINT32_MAX ? UINT32_MAX : (uint32_t)(now - stamp);
}

static uint8_t tracking_read(uint8_t command, uint8_t kind, void (*reply)(uint8_t *, uint8_t))
{
    // Only the serialized dispatcher creates the worker/mutex. Boot reads need no allocation.
    if (tracking_mutex && !xSemaphoreTake(tracking_mutex, portMAX_DELAY))
        return RESPONSE_INTERNAL_ERROR;
    // A delayed worker cannot expose a healthy cached vector after the physical
    // producer recorded a discontinuity. The reader touches only the IRQ-safe cache.
    if (tracking_reader) tracking_update();
    uint64_t now = hw_clock_microseconds();
    tracking_deadline(now);
    uint8_t result = RESPONSE_OK;
    if (command == GET_IMU_TRACKING_CONFIGURATION) {
        imu_tracking_configuration wire = {tracker.mounting_yaw, IMU_TRACKING_WINDOW_US};
        reply((uint8_t *)&wire, sizeof(wire));
    } else if (command == GET_IMU_TRACKING_STATUS) {
        uint8_t flags = (tracker.health == IMU_ONLY_TRACKING
                             ? IMU_TRACKING_FLAG_ESTIMATE_VALID | IMU_TRACKING_FLAG_BIAS_READY
                             : 0) |
                        (tracker.health == IMU_ONLY_FAULT || tracker.health == IMU_ONLY_EXPIRED
                             ? IMU_TRACKING_FLAG_RESTART_REQUIRED
                             : 0) |
                        (tracker.health != IMU_ONLY_STOPPED ? IMU_TRACKING_FLAG_MOUNT_CONFIRMED : 0);
        imu_tracking_status wire = {tracker.health,
                                    tracker.fault,
                                    flags,
                                    0,
                                    tracker.sequence,
                                    published_sample,
                                    tracking_elapsed,
                                    tracking_age(now, tracker.accel_us),
                                    tracking_age(now, tracker.gyro_us)};
        reply((uint8_t *)&wire, sizeof(wire));
    } else if (kind > IMU_TRACKING_VECTOR_BIAS)
        result = RESPONSE_INVALID_ARGUMENT;
    else if (tracker.health != IMU_ONLY_TRACKING)
        result = RESPONSE_SAMPLE_NOT_AVAILABLE;
    else {
        imu_tracking_vector wire = {tracker.sequence, tracker.x, tracker.y, tracker.yaw};
        if (kind == IMU_TRACKING_VECTOR_TWIST) {
            double c = cos(tracker.yaw), sn = sin(tracker.yaw);
            wire.x = c * tracker.vx + sn * tracker.vy;
            wire.y = -sn * tracker.vx + c * tracker.vy;
            wire.z = tracker.wz;
        } else if (kind == IMU_TRACKING_VECTOR_BIAS) {
            wire.x = tracker.bias_ax;
            wire.y = tracker.bias_ay;
            wire.z = tracker.bias_gz;
        }
        reply((uint8_t *)&wire, sizeof(wire));
    }
    if (tracking_mutex)
        xSemaphoreGive(tracking_mutex);
    return result;
}
