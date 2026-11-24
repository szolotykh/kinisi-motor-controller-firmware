#pragma once
// Passive observer in the position translation unit to share the release
// estimator, physical reader and mounting transform without linking a second
// acquisition driver. Its mutex, state and lifetime are independent of control.
#include "feedback_source.h"
#include "imu_tracking.h"

static struct {
    SemaphoreHandle_t lock;
    feedback_fusion_mount mount;
    double c, s;
    fusion_estimator_t estimator;
    feedback_frame_t frame;
    uint32_t epoch, generation, sequence;
    uint64_t start, encoder, accel, gyro;
    uint8_t reason, fault;
} passive;

uint32_t passive_fusion_epoch(void)
{
    return __atomic_load_n(&passive.epoch,__ATOMIC_ACQUIRE);
}

uint32_t passive_fusion_acquisition_epoch(void)
{
    // A passive observer cannot replace either preexisting physical producer.
    uint32_t existing = imu_acquisition_epoch(platform_fusion_imu_epoch());
    return existing ? existing : passive_fusion_epoch();
}

static void passive_restart(bool enabled)
{
    fusion_estimator_init(&passive.estimator);
    passive.start = hw_clock_microseconds();
    passive.encoder = passive.accel = passive.gyro = 0;
    passive.reason = FEEDBACK_REASON_CALIBRATING;
    passive.fault = FUSION_FAULT_BIAS_NOT_READY;
    ++passive.generation;
    uint32_t token = 0x40000000U | (passive.generation & 0x3fffffffU);
    __atomic_store_n(&passive.epoch, enabled ? token : 0, __ATOMIC_RELEASE);
    feedback_invalidate(FEEDBACK_SOURCE_FUSED);
}

void passive_fusion_reset(void)
{
    if (passive_fusion_epoch() && xSemaphoreTake(passive.lock,portMAX_DELAY)) {
        passive_restart(passive_fusion_epoch() != 0);
        xSemaphoreGive(passive.lock);
    }
}

static uint8_t passive_configure(bool enable, const double *yaw)
{
    if (!passive.lock) passive.lock = xSemaphoreCreateMutex();
    if (!passive.lock || !xSemaphoreTake(passive.lock,portMAX_DELAY)) return RESPONSE_INTERNAL_ERROR;
    bool run = enable && (yaw || passive.mount.confirmed);
    uint8_t result = RESPONSE_OK;
    if (run && start_physical_reader && !start_physical_reader()) result = RESPONSE_INTERNAL_ERROR;
    else {
        if (yaw) {
            passive.mount = (feedback_fusion_mount){1,*yaw};
            passive.c = cos(*yaw);
            passive.s = sin(*yaw);
        }
        passive_restart(run);
    }
    xSemaphoreGive(passive.lock);
    return result;
}

uint8_t passive_fusion_select(bool enable)
{
    if (!passive.lock && !enable) return RESPONSE_OK;
    return passive_configure(enable,NULL);
}

uint8_t passive_fusion_mount(double yaw)
{
    if (!isfinite(yaw) || fabs(yaw) > 3.14159265358979323846) return RESPONSE_INVALID_ARGUMENT;
    return passive_configure(feedback_requested() == FEEDBACK_SOURCE_FUSED,&yaw);
}

feedback_fusion_mount passive_fusion_get_mount(void) { return passive.mount; } // Serialized command dispatcher.

static void passive_reject(uint8_t reason, uint8_t fault)
{
    if (passive.reason == FEEDBACK_REASON_OK) feedback_invalidate(FEEDBACK_SOURCE_FUSED);
    fusion_estimator_init(&passive.estimator);
    passive.reason=reason;
    passive.fault=fault;
    passive.accel=passive.gyro=0;
}

static bool passive_late(uint64_t at)
{
    return age(at,passive.encoder)>60000 || age(at,passive.accel)>55000 || age(at,passive.gyro)>55000;
}

void passive_fusion_observe(uint32_t epoch, uint64_t previous, uint64_t at, platform_odometry_t delta,
                            bool valid, bool stationary, bool moved)
{
    hw_imu_sample_t raw = {0};
    uint64_t a=0,g=0;
    bool asleep=hw_motor_drivers_sleeping();
    hw_imu_status_t status=HW_IMU_NOT_READY;
    if (valid && !asleep)
        status=physical_reader ? physical_reader(passive_fusion_acquisition_epoch(),at,&raw,&a,&g) :
                                 hw_imu_read_observation(0,&raw,&a,&g);
    if (!xSemaphoreTake(passive.lock,portMAX_DELAY)) return;
    if (epoch != passive_fusion_epoch() || at <= passive.start) goto done;
    if (passive.reason == FEEDBACK_REASON_OK && passive_late(at))
        passive_reject(FEEDBACK_REASON_STALE,FUSION_FAULT_STALE);
    if (asleep || hw_motor_drivers_sleeping()) {
        passive_reject(FEEDBACK_REASON_INHIBITED,FUSION_FAULT_INHIBITED);
        goto done;
    }
    if (!valid || !geometry_valid()) {
        passive_reject(FEEDBACK_REASON_UNAVAILABLE,FUSION_FAULT_ENCODER_UNAVAILABLE);
        goto done;
    }
    if (!previous || previous < passive.start) goto done;
    if (at <= previous || at-previous>60000 || (passive.encoder && previous != passive.encoder)) {
        passive_reject(FEEDBACK_REASON_STALE,FUSION_FAULT_DISCONTINUITY);
        passive.encoder=at;
        goto done;
    }
    passive.encoder=at;
    uint8_t fault=imu_problem(status,&raw,at,a,g,passive.accel,passive.gyro);
    if (fault) {
        uint8_t reason=fault==FUSION_FAULT_STALE ? FEEDBACK_REASON_STALE :
            fault==FUSION_FAULT_IMU_UNAVAILABLE ? FEEDBACK_REASON_UNAVAILABLE : FEEDBACK_REASON_FAULT;
        passive_reject(reason,fault);
        goto done;
    }
    if (a <= passive.start || g <= passive.start) goto done;
    double hz=1e6/(double)(at-previous);
    double wheel[3]={delta.x*hz,delta.y*hz,delta.t*hz};
    if (!isfinite(wheel[0]) || !isfinite(wheel[1]) || !isfinite(wheel[2])) {
        passive_reject(FEEDBACK_REASON_UNAVAILABLE,FUSION_FAULT_ENCODER_UNAVAILABLE);
        goto done;
    }
    if (!passive.estimator.calibrated && (!stationary || moved)) {
        passive_reject(FEEDBACK_REASON_CALIBRATING,FUSION_FAULT_BIAS_NOT_READY);
        goto done;
    }
    passive.accel=a; passive.gyro=g;
    fusion_imu_t imu=body_imu(&raw,a,g,passive.c,passive.s);
    fusion_estimator_t *e=&passive.estimator;
    if (fusion_estimator_step(e,at,wheel,&imu)) {
        passive.reason=FEEDBACK_REASON_OK;
        passive.fault=0;
        ++passive.sequence;
        feedback_anchor(&passive.frame,FEEDBACK_SOURCE_FUSED,e->sample_us,(platform_odometry_t){e->x,e->y,e->yaw});
    } else if (e->status == FUSION_EST_CALIBRATING || e->status == FUSION_EST_MOTION) {
        passive.reason=FEEDBACK_REASON_CALIBRATING;
        passive.fault=FUSION_FAULT_BIAS_NOT_READY;
    } else passive_reject(FEEDBACK_REASON_FAULT,
                           e->status == FUSION_EST_TIME ? FUSION_FAULT_TIMESTAMP : FUSION_FAULT_IMU_UNAVAILABLE);
done:
    xSemaphoreGive(passive.lock);
}

void passive_fusion_sample(feedback_snapshot_t *out)
{
    if (!passive.mount.confirmed) { out->reason=FEEDBACK_REASON_MOUNT_REQUIRED; return; }
    if (!xSemaphoreTake(passive.lock,portMAX_DELAY)) { out->reason=FEEDBACK_REASON_UNAVAILABLE; return; }
    if (hw_motor_drivers_sleeping()) { out->reason=FEEDBACK_REASON_INHIBITED; out->detail=FUSION_FAULT_INHIBITED; }
    else if (!platform_is_odometry_enabled()) { out->reason=FEEDBACK_REASON_NOT_STARTED; out->detail=FUSION_FAULT_ENCODER_UNAVAILABLE; }
    else {
        if (passive.reason == FEEDBACK_REASON_OK && passive_late(hw_clock_microseconds()))
            passive_reject(FEEDBACK_REASON_STALE,FUSION_FAULT_STALE);
        out->reason=passive.reason;
        out->detail=passive.fault;
        fusion_estimator_t *e=&passive.estimator;
        if (!out->reason)
            feedback_snapshot(out,&passive.frame,FEEDBACK_SOURCE_FUSED,passive.sequence,e->sample_us,
                              (platform_odometry_t){e->x,e->y,e->yaw},(platform_odometry_t){e->vx,e->vy,e->wz});
    }
    xSemaphoreGive(passive.lock);
}
