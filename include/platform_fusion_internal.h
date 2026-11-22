#pragma once
// Included by platform_position.c: the estimator, both chassis-target paths,
// and position cancellation share one RTOS mutex. SPI runs outside this mutex.
#include "hw_drv8873.h"
#include "hw_imu.h"
#include "fusion_estimator.h"
#include "commands.h"
#include <limits.h>

static struct {
    uint8_t mode, health, fault;
    bool encoder_valid, target_live, awaiting_rest, anchor_valid, rebase_pose;
    platform_odometry_t anchor_encoder, anchor_fused;
    uint32_t sequence, generation, event[4];
    uint64_t configured_us, encoder_us, accel_us, gyro_us;
    double yaw, cosine, sine;
    platform_odometry_t pose, twist;
    platform_velocity_t direct;
    fusion_estimator_t estimator;
} experiment;
static platform_velocity_t integral, prior_demand;
static uint64_t correction_time;
static void reset_correction(void)
{
    integral = (platform_velocity_t){0};
    prior_demand = (platform_velocity_t){0};
    correction_time = 0;
}
static uint32_t published_epoch, imu_epoch;
static osThreadId_t supervisor;
static platform_imu_reader_t physical_reader;
static bool (*start_physical_reader)(void);
static void supervise_task(void *unused);

void platform_fusion_install_imu(platform_imu_reader_t reader, bool (*start)(void))
{
    physical_reader = reader;
    start_physical_reader = start;
}

uint32_t platform_fusion_imu_epoch(void) { return __atomic_load_n(&imu_epoch, __ATOMIC_ACQUIRE); }

static void (*ingress_fence)(uint32_t *);
static uint32_t feedback_generation, dequeued_generation;

void platform_fusion_install_ingress_fence(void (*fence)(uint32_t *)) { ingress_fence = fence; }

void platform_fusion_dequeued(void)
{
    dequeued_generation = __atomic_load_n(&feedback_generation, __ATOMIC_ACQUIRE);
}

static void fence_targets(void)
{
    if (!ingress_fence) return;
    ingress_fence(&feedback_generation);
}

static bool fresh_request(void)
{
    return !ingress_fence || dequeued_generation == __atomic_load_n(&feedback_generation, __ATOMIC_ACQUIRE);
}

uint32_t platform_fusion_epoch(void)
{
    return __atomic_load_n(&published_epoch, __ATOMIC_ACQUIRE);
}

static uint32_t age(uint64_t now, uint64_t sample)
{
    return !sample || sample > now || now - sample >= UINT32_MAX ? UINT32_MAX : (uint32_t)(now - sample);
}

static void clear_target(void)
{
    reset_correction();
    active = 0;
    experiment.target_live = false;
    experiment.direct = (platform_velocity_t){0};
    reset_pid();
    controllers_manager_feedback_hold(platform.motor_mask, true);
}

static void restart_estimator(void)
{
    fusion_estimator_init(&experiment.estimator);
    experiment.accel_us = experiment.gyro_us = 0;
}

static void restart_batch(bool reset_origin)
{
    clear_target();
    fence_targets();
    ++experiment.sequence;
    experiment.awaiting_rest = true;
    experiment.rebase_pose = true;
    if (reset_origin) experiment.anchor_valid = false;
    restart_estimator();
    experiment.encoder_us = 0;
    experiment.encoder_valid = false;
    experiment.configured_us = hw_clock_microseconds();
    if (++experiment.generation == 0) ++experiment.generation;
    __atomic_store_n(&imu_epoch, experiment.mode == FUSION_MODE_FUSED ? experiment.generation : 0,
                     __ATOMIC_RELEASE);
    __atomic_store_n(&published_epoch, experiment.mode ? experiment.generation : 0, __ATOMIC_RELEASE);
}

static void invalidate_episode(bool reset_origin, uint8_t health, uint8_t fault)
{
    if (!platform_fusion_epoch() || !xSemaphoreTake(mutex, portMAX_DELAY)) return;
    restart_batch(reset_origin);
    experiment.health = health;
    experiment.fault = fault;
    xSemaphoreGive(mutex);
}

void platform_fusion_discontinuity(bool reset_origin)
{
    invalidate_episode(reset_origin, FUSION_HEALTH_FAULT, FUSION_FAULT_DISCONTINUITY);
}

void platform_fusion_driver_transition(bool sleeping)
{
    // The encoder world frame continues during driver inhibit. Each power edge
    // starts a new source epoch, even when both occur between two odometry ticks.
    invalidate_episode(false, sleeping ? FUSION_HEALTH_FAULT : FUSION_HEALTH_WAITING,
                       sleeping ? FUSION_FAULT_INHIBITED : FUSION_FAULT_NONE);
}

uint8_t platform_fusion_configure(uint8_t mode, double yaw)
{
    if (mode > FUSION_MODE_FUSED || !isfinite(yaw) || fabs(yaw) > 3.14159265358979323846) return RESPONSE_INVALID_ARGUMENT;
    if (mode && !platform.is_initialized) return RESPONSE_PLATFORM_NOT_INITIALIZED;
    if (mode && !geometry_valid()) return RESPONSE_INVALID_ARGUMENT;
    if (mode && hw_motor_drivers_sleeping()) return RESPONSE_DRIVERS_ASLEEP;
    if (!mutex) mutex = xSemaphoreCreateMutex();
    if (!mutex || !xSemaphoreTake(mutex, portMAX_DELAY)) return RESPONSE_INTERNAL_ERROR;
    uint8_t result = RESPONSE_OK;
    if (active || experiment.target_live || odometry_manager_recent_motion(hw_clock_microseconds()) ||
        !controllers_manager_hold_if_stopped(platform.motor_mask))
        result = RESPONSE_INVALID_ARGUMENT;
    else if (mode && !supervisor && !(supervisor = osThreadNew(supervise_task, NULL, &(osThreadAttr_t){
        .name = "FeedbackWatch", .priority = osPriorityNormal, .stack_size = 1024})))
        result = RESPONSE_INTERNAL_ERROR;
    else if (mode == FUSION_MODE_FUSED && start_physical_reader && !start_physical_reader())
        result = RESPONSE_INTERNAL_ERROR;
    else {
        experiment.mode = mode;
        experiment.yaw = yaw;
        experiment.cosine = cos(yaw);
        experiment.sine = sin(yaw);
        restart_batch(true);
        experiment.health = mode ? FUSION_HEALTH_WAITING : FUSION_HEALTH_DISABLED;
        experiment.fault = FUSION_FAULT_NONE;
        for (uint8_t i = 0; i < 4; ++i)
            if (platform_owns_motor(i)) experiment.event[i] = hw_motor_fault_event_sequence(i);
        if (mode && !platform_is_odometry_enabled()) platform_start_odometry();
        if (!mode) controllers_manager_feedback_hold(platform.motor_mask, false);
    }
    if (result == RESPONSE_INTERNAL_ERROR && !experiment.mode)
        controllers_manager_feedback_hold(platform.motor_mask, false);
    xSemaphoreGive(mutex);
    return result;
}

static bool inhibited(void)
{
    if (hw_motor_drivers_sleeping()) return true;
    bool inhibit = false;
    for (uint8_t i = 0; i < 4; ++i) {
        if (!platform_owns_motor(i)) continue;
        uint32_t event = hw_motor_fault_event_sequence(i);
        if (hw_drv8873_fault_active(i) || event != experiment.event[i]) inhibit = true;
        experiment.event[i] = event;
    }
    return inhibit;
}

static void reject_feedback(uint8_t reason)
{
    experiment.rebase_pose = true;
    experiment.health = FUSION_HEALTH_FAULT;
    experiment.fault = reason;
    clear_target();
}

// Allow one normal 20 ms odometry scheduling phase on top of the estimator's
// 35 ms acquisition-age bound when the independent control task consumes it.
static bool sources_expired(uint64_t now)
{
    return age(now, experiment.encoder_us) > 60000 ||
           (experiment.mode == FUSION_MODE_FUSED &&
            (age(now, experiment.accel_us) > 55000 || age(now, experiment.gyro_us) > 55000));
}

static void expire_locked(uint64_t now)
{
    if (experiment.health == FUSION_HEALTH_HEALTHY && sources_expired(now)) {
        ++experiment.sequence;
        reject_feedback(FUSION_FAULT_STALE);
    }
}

static void supervise_task(void *unused)
{
    (void)unused;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        // 1 ms board tick while enabled. An inactive experiment owns no target.
        vTaskDelayUntil(&last, pdMS_TO_TICKS(platform_fusion_epoch() ? 1 : 20));
        if (!platform_fusion_epoch() || !xSemaphoreTake(mutex, portMAX_DELAY)) continue;
        expire_locked(hw_clock_microseconds());
        xSemaphoreGive(mutex);
    }
}

static void qualify_pose(fusion_estimator_t *e, platform_odometry_t pose)
{
    // Apply the last fused-from-encoder world transform to the current raw pose.
    // This includes all coasting translation and rotation over the invalid interval.
    if (experiment.rebase_pose) {
        double a = experiment.anchor_valid ? experiment.anchor_fused.t - experiment.anchor_encoder.t : 0;
        double dx = experiment.anchor_valid ? pose.x - experiment.anchor_encoder.x : pose.x;
        double dy = experiment.anchor_valid ? pose.y - experiment.anchor_encoder.y : pose.y;
        e->x = (experiment.anchor_valid ? experiment.anchor_fused.x : 0) + cos(a)*dx - sin(a)*dy;
        e->y = (experiment.anchor_valid ? experiment.anchor_fused.y : 0) + sin(a)*dx + cos(a)*dy;
        e->yaw = pose.t + a; // The relative odometry frame retains complete turns.
        experiment.rebase_pose = false;
    }
    experiment.anchor_fused = (platform_odometry_t){e->x,e->y,e->yaw};
    experiment.anchor_encoder = pose;
    experiment.anchor_valid = true;
}

static uint8_t imu_problem(hw_imu_status_t status, const hw_imu_sample_t *raw,
                           uint64_t at, uint64_t accel, uint64_t gyro)
{
    if (status == HW_IMU_STALE) return FUSION_FAULT_STALE;
    if (status != HW_IMU_OK || !accel || !gyro) return FUSION_FAULT_IMU_UNAVAILABLE;
    if (accel > at || gyro > at || accel < experiment.accel_us || gyro < experiment.gyro_us)
        return FUSION_FAULT_TIMESTAMP;
    if (age(at, accel) > 35000 || age(at, gyro) > 35000 ||
        (accel > gyro ? accel - gyro : gyro - accel) > 15000) return FUSION_FAULT_STALE;
    if (!isfinite(raw->acceleration_x) || !isfinite(raw->acceleration_y) ||
        !isfinite(raw->acceleration_z) || !isfinite(raw->angular_velocity_x) ||
        !isfinite(raw->angular_velocity_y) || !isfinite(raw->angular_velocity_z)) return FUSION_FAULT_IMU_UNAVAILABLE;
    if (fabs(raw->acceleration_x) >= 55 || fabs(raw->acceleration_y) >= 55 ||
        fabs(raw->acceleration_z) >= 55 || fabs(raw->angular_velocity_x) >= 8 ||
        fabs(raw->angular_velocity_y) >= 8 || fabs(raw->angular_velocity_z) >= 8) return FUSION_FAULT_SATURATION;
    // The experiment supports a centered, level mount. Off-plane rotation or a
    // grossly tilted gravity vector invalidates planar acceleration feedback.
    if (fabs(raw->angular_velocity_x) > 0.35 || fabs(raw->angular_velocity_y) > 0.35 ||
        fabs(raw->acceleration_z - 9.80665) > 1.0) return FUSION_FAULT_MOUNT_INVALID;
    return FUSION_FAULT_NONE;
}

void platform_fusion_observe(uint32_t epoch, uint64_t previous, uint64_t at,
                             platform_odometry_t delta, platform_odometry_t pose, bool valid, bool stationary, bool moved)
{
    if (!epoch || !xSemaphoreTake(mutex, portMAX_DELAY)) return;
    expire_locked(hw_clock_microseconds());
    bool need_imu = epoch == platform_fusion_epoch() && experiment.mode == FUSION_MODE_FUSED;
    xSemaphoreGive(mutex);
    hw_imu_sample_t raw = {0};
    uint64_t accel = 0, gyro = 0;
    hw_imu_status_t status = HW_IMU_NOT_READY;
    if (need_imu && !hw_motor_drivers_sleeping())
        status = physical_reader ? physical_reader(epoch, at, &raw, &accel, &gyro) :
                                   hw_imu_read_observation(0, &raw, &accel, &gyro);
    if (!xSemaphoreTake(mutex, portMAX_DELAY)) return;
    if (epoch != platform_fusion_epoch() || at <= experiment.configured_us) goto done;
    expire_locked(hw_clock_microseconds()); // A late producer cannot erase an already expired deadline.
    ++experiment.sequence;
    experiment.encoder_valid = false;
    if (inhibited()) { reject_feedback(FUSION_FAULT_INHIBITED); goto reset; }
    if (!valid || !geometry_valid()) { reject_feedback(FUSION_FAULT_ENCODER_UNAVAILABLE); goto reset; }
    if (previous < experiment.configured_us) goto done; // First complete interval after configure/reset.
    if (!previous || previous >= at || at - previous > 60000 ||
        (experiment.encoder_us && previous != experiment.encoder_us)) {
        reject_feedback(FUSION_FAULT_DISCONTINUITY);
        experiment.encoder_us = at;
        goto reset;
    }
    const double inverse_dt = 1e6 / (double)(at - previous);
    double wheel[3] = {delta.x * inverse_dt, delta.y * inverse_dt, delta.t * inverse_dt};
    if (!isfinite(pose.x) || !isfinite(pose.y) || !isfinite(pose.t) ||
        !isfinite(wheel[0]) || !isfinite(wheel[1]) || !isfinite(wheel[2])) {
        reject_feedback(FUSION_FAULT_ENCODER_UNAVAILABLE); goto reset;
    }
    experiment.encoder_us = at;
    experiment.pose = pose;
    experiment.twist = (platform_odometry_t){wheel[0], wheel[1], wheel[2]};
    experiment.encoder_valid = true;
    if (need_imu) {
        uint8_t problem = imu_problem(status, &raw, at, accel, gyro);
        if (problem) {
            reject_feedback(problem);
            goto reset;
        }
        // A cached sample may be otherwise recent yet belong to the previous
        // episode. Wait for independent post-transition acquisitions.
        if (status == HW_IMU_OK && accel && gyro && experiment.awaiting_rest &&
            (accel <= experiment.configured_us || gyro <= experiment.configured_us)) goto done;
        experiment.accel_us = accel;
        experiment.gyro_us = gyro;
        fusion_estimator_t *e = &experiment.estimator;
        fusion_imu_t imu = {
            .accel_us = accel, .gyro_us = gyro, .valid = true,
            .ax = -experiment.cosine * raw.acceleration_x + experiment.sine * raw.acceleration_y,
            .ay = -experiment.sine * raw.acceleration_x - experiment.cosine * raw.acceleration_y,
            .az = raw.acceleration_z, .gz = raw.angular_velocity_z
        };
        if (!e->calibrated && moved) {
            fusion_estimator_init(e);
            experiment.health = FUSION_HEALTH_WAITING;
            experiment.fault = FUSION_FAULT_BIAS_NOT_READY;
            clear_target();
            goto done;
        }
        if (!fusion_estimator_step(e, at, wheel, &imu)) {
            if (e->status == FUSION_EST_CALIBRATING || e->status == FUSION_EST_MOTION) {
                experiment.health = FUSION_HEALTH_WAITING;
                experiment.fault = FUSION_FAULT_BIAS_NOT_READY;
                clear_target();
                goto done;
            }
            reject_feedback(e->status == FUSION_EST_TIME ? FUSION_FAULT_TIMESTAMP : FUSION_FAULT_IMU_UNAVAILABLE);
            goto reset;
        }
    }
    if (experiment.awaiting_rest && (!stationary || at - experiment.configured_us < 100000)) {
        experiment.health = FUSION_HEALTH_WAITING;
        experiment.fault = FUSION_FAULT_NONE;
        clear_target();
        goto done;
    }
    experiment.awaiting_rest = false;
    if (need_imu) qualify_pose(&experiment.estimator, pose);
    if (experiment.health != FUSION_HEALTH_HEALTHY) fence_targets();
    experiment.health = FUSION_HEALTH_HEALTHY;
    experiment.fault = FUSION_FAULT_NONE;
    goto done;
reset:
    // Restart bias/freshness after a gap. Preserve the source times for diagnostics.
    fusion_estimator_init(&experiment.estimator);
done:
    expire_locked(hw_clock_microseconds());
    xSemaphoreGive(mutex);
}

static bool usable(void)
{
    if (inhibited()) reject_feedback(FUSION_FAULT_INHIBITED);
    expire_locked(hw_clock_microseconds());
    return experiment.health == FUSION_HEALTH_HEALTHY;
}

static double correction(double target, double sensed, double prior, double *sum,
                         double cap, double dt)
{
    double error = target - sensed;
    if (!target || (target * prior < 0)) *sum = 0;
    else {
        double candidate = fmax(-0.5*cap, fmin(0.5*cap, *sum + dt * error));
        double action = 0.5 * error + candidate;
        if (fabs(action) <= cap || action * error < 0) *sum = candidate;
    }
    return target + fmax(-cap, fmin(cap, 0.5 * error + *sum));
}

static void apply_correction(platform_velocity_t v)
{
    platform_odometry_t t = experiment.twist;
    if (experiment.mode == FUSION_MODE_FUSED) {
        fusion_estimator_t *s = &experiment.estimator;
        t = (platform_odometry_t){s->vx, s->vy, s->wz};
    }
    uint64_t now = hw_clock_microseconds();
    double dt = correction_time && now >= correction_time && now - correction_time <= 150000 ?
        (now - correction_time) * 1e-6 : 0;
    correction_time = now;
    platform_velocity_t corrected = {
        correction(v.x, t.x, prior_demand.x, &integral.x, 0.25, dt),
        platform.type == PLATFORM_DIFFERENTIAL ? 0 : correction(v.y, t.y, prior_demand.y, &integral.y, 0.25, dt),
        correction(v.t, t.t, prior_demand.t, &integral.t, 0.75, dt)
    };
    prior_demand = v;
    platform.set_platform_target_velocity(corrected);
}

uint8_t platform_fusion_velocity(platform_velocity_t v)
{
    if (!isfinite(v.x) || !isfinite(v.y) || !isfinite(v.t)) return RESPONSE_INVALID_ARGUMENT;
    if (!xSemaphoreTake(mutex, portMAX_DELAY)) return RESPONSE_INTERNAL_ERROR;
    if (!fresh_request()) {
        xSemaphoreGive(mutex);
        return RESPONSE_SAMPLE_NOT_AVAILABLE;
    }
    if ((v.x || v.y || v.t) && !usable()) {
        xSemaphoreGive(mutex);
        return RESPONSE_SAMPLE_NOT_AVAILABLE;
    }
    // An accepted direct velocity replaces the previously selected position target.
    if (active) reset_correction();
    active = initialized = 0;
    reset_pid();
    uint8_t result = RESPONSE_OK;
    if (!v.x && !v.y && !v.t) clear_target();
    else {
        experiment.direct = v;
        experiment.target_live = true;
        controllers_manager_feedback_hold(platform.motor_mask, false);
        apply_correction(v);
    }
    xSemaphoreGive(mutex);
    return result;
}

uint8_t platform_fusion_read(uint8_t command, uint8_t kind, void (*reply)(uint8_t *, uint8_t))
{
    if (mutex && !xSemaphoreTake(mutex, portMAX_DELAY)) return RESPONSE_INTERNAL_ERROR;
    uint64_t now = hw_clock_microseconds();
    uint8_t result = RESPONSE_OK;
    expire_locked(now);
    bool valid = experiment.health == FUSION_HEALTH_HEALTHY;
    if (command == GET_FUSION_CONFIGURATION) {
        fusion_configuration config = {experiment.mode, experiment.yaw};
        reply((uint8_t *)&config, sizeof(config));
    } else if (command == GET_FUSION_STATUS) {
        uint8_t flags = (valid ? FUSION_FLAG_ESTIMATE_VALID : 0) |
            (experiment.estimator.calibrated ? FUSION_FLAG_BIAS_READY : 0) |
            (experiment.mode && !experiment.target_live ? FUSION_FLAG_FRESH_TARGET_REQUIRED : 0) |
            (valid && experiment.target_live ? FUSION_FLAG_CORRECTION_ACTIVE : 0);
        fusion_status status = {experiment.mode, experiment.health,
            experiment.fault, flags, experiment.sequence,
            experiment.encoder_us, age(now, experiment.accel_us), age(now, experiment.gyro_us),
            age(now, experiment.encoder_us)};
        reply((uint8_t *)&status, sizeof(status));
    } else {
        platform_odometry_t vector = {0};
        fusion_estimator_t *e = &experiment.estimator;
        if (kind > FUSION_VECTOR_BIAS) result = RESPONSE_INVALID_ARGUMENT;
        else if (!experiment.encoder_valid || age(now, experiment.encoder_us) > 60000 ||
            (kind >= FUSION_VECTOR_FUSED_POSE && (!valid || experiment.mode != FUSION_MODE_FUSED)))
            result = RESPONSE_SAMPLE_NOT_AVAILABLE;
        else {
            switch (kind) {
                case FUSION_VECTOR_ENCODER_POSE: vector = experiment.pose; break;
                case FUSION_VECTOR_ENCODER_TWIST: vector = experiment.twist; break;
                case FUSION_VECTOR_FUSED_POSE: vector = (platform_odometry_t){e->x,e->y,e->yaw}; break;
                case FUSION_VECTOR_FUSED_TWIST: vector = (platform_odometry_t){e->vx,e->vy,e->wz}; break;
                case FUSION_VECTOR_BIAS: vector = (platform_odometry_t){e->bias[0],e->bias[1],e->bias[2]}; break;
            }
            fusion_vector wire = {experiment.sequence,vector.x,vector.y,vector.t};
            reply((uint8_t *)&wire, sizeof(wire));
        }
    }
    if (mutex) xSemaphoreGive(mutex);
    return result;
}
