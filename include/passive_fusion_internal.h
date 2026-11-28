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
    imu_mount_frame_t mounting;
    imu_mount_history_t history;
    fusion_estimator_t estimator;
    feedback_frame_t frame;
    uint32_t epoch, generation, sequence;
    uint64_t start, encoder, accel, gyro, last_good;
    struct {
        uint64_t start, end;
        double twist[3];
        bool stationary;
    } previous_wheels;
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
    passive.history=(imu_mount_history_t){0};
    passive.previous_wheels.end=0;
    fusion_estimator_init(&passive.estimator);
    passive.last_good = 0;
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
    bool run = enable && (yaw || passive.mount.confirmed || passive.mounting.enabled);
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

// Command/boot thread is the only configuration writer. Prepare all resources
// before publishing either transform. Observers hold their own episode mutex.
uint8_t platform_fusion_set_mount(const imu_mount_frame_t *frame)
{
    bool enabled=feedback_requested()==FEEDBACK_SOURCE_FUSED && (frame->enabled || passive.mount.confirmed);
    if (!passive.lock) passive.lock=xSemaphoreCreateMutex();
    if (!mutex) mutex=xSemaphoreCreateMutex();
    if (!passive.lock || !mutex || (enabled && start_physical_reader && !start_physical_reader()))
        return RESPONSE_INTERNAL_ERROR;
    if (!xSemaphoreTake(mutex,portMAX_DELAY)) return RESPONSE_INTERNAL_ERROR;
    if (!xSemaphoreTake(passive.lock,portMAX_DELAY)) {
        xSemaphoreGive(mutex);
        return RESPONSE_INTERNAL_ERROR;
    }
    experiment_mount=*frame;
    if (experiment.mode==FUSION_MODE_FUSED) {
        restart_batch(true);
        experiment.health=FUSION_HEALTH_WAITING;
        experiment.fault=FUSION_FAULT_BIAS_NOT_READY;
    }
    passive.mounting=*frame;
    passive_restart(enabled);
    xSemaphoreGive(passive.lock);
    xSemaphoreGive(mutex);
    return RESPONSE_OK;
}

static void passive_reject(uint8_t reason, uint8_t fault)
{
    passive.history=(imu_mount_history_t){0};
    passive.previous_wheels.end=0;
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
    uint8_t fault=imu_problem(status,&raw,at,a,g,passive.accel,passive.gyro,!passive.mounting.enabled);
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
    bool at_rest=stationary && !moved;
    uint64_t watermark=at;
    double wheel_age=0.;
    if (passive.mounting.enabled) {
        // Physical IMU delivery lags the newest wheel boundary. Retain the
        // current interval for the next update; consuming it now would pull
        // a later shaft observation into an earlier inertial estimate.
        uint64_t begin=passive.previous_wheels.start, end=passive.previous_wheels.end;
        bool previous_rest=passive.previous_wheels.stationary;
        for (unsigned i=0;i<3;++i) {
            double current=wheel[i];
            wheel[i]=passive.previous_wheels.twist[i];
            passive.previous_wheels.twist[i]=current;
        }
        passive.previous_wheels.start=previous;
        passive.previous_wheels.end=at;
        passive.previous_wheels.stationary=at_rest;
        if (end!=previous || end<=begin) goto done;
        watermark=a>g?a:g;
        if (watermark<end) watermark=end;
        uint64_t midpoint=begin+(end-begin)/2;
        wheel_age=(watermark-midpoint)*1e-6;
        at_rest=previous_rest;
    }
    if (!passive.estimator.calibrated && !at_rest) {
        passive_reject(FEEDBACK_REASON_CALIBRATING,FUSION_FAULT_BIAS_NOT_READY);
        goto done;
    }
    passive.accel=a; passive.gyro=g;
    fusion_estimator_t *e=&passive.estimator;
    fusion_imu_t imu;
    fault=origin_imu(&passive.mounting,&passive.history,&raw,a,g,passive.c,passive.s,e->bias[2],&imu);
    if (fault==FUSION_FAULT_BIAS_NOT_READY) goto done;
    if (fault) {
        passive_reject(fault==FUSION_FAULT_STALE?FEEDBACK_REASON_STALE:FEEDBACK_REASON_FAULT,fault);
        goto done;
    }
    // An encoder tick can catch up to the die timestamps that we already
    // published. Retain its interval, but never re-integrate that watermark.
    if (passive.mounting.enabled && watermark==e->previous_us) goto done;
    if (fusion_estimator_interval(e,watermark,wheel,&imu,passive.mounting.enabled,wheel_age)) {
        passive.reason=FEEDBACK_REASON_OK;
        passive.fault=0;
        ++passive.sequence;
        passive.last_good=e->sample_us;
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
    if (!passive.lock) { out->reason=FEEDBACK_REASON_MOUNT_REQUIRED; return; }
    if (!xSemaphoreTake(passive.lock,portMAX_DELAY)) { out->reason=FEEDBACK_REASON_UNAVAILABLE; return; }
    if (!passive.mount.confirmed && !passive.mounting.enabled) out->reason=FEEDBACK_REASON_MOUNT_REQUIRED;
    else if (hw_motor_drivers_sleeping()) { out->reason=FEEDBACK_REASON_INHIBITED; out->detail=FUSION_FAULT_INHIBITED; }
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

#if BOARD_HAS_FEEDBACK_SOURCE_HEALTH
void passive_health_counter_edge(bool owned)
{
    if (!passive_fusion_epoch() || !xSemaphoreTake(passive.lock,portMAX_DELAY)) return;
    if (passive_fusion_epoch()) {
        if (owned) passive_reject(FEEDBACK_REASON_UNAVAILABLE,FUSION_FAULT_ENCODER_UNAVAILABLE);
        else if (!passive.estimator.calibrated)
            passive_reject(FEEDBACK_REASON_CALIBRATING,FUSION_FAULT_BIAS_NOT_READY);
    }
    xSemaphoreGive(passive.lock);
}

void passive_health_fault(uint8_t reason,uint8_t detail)
{
    if (!passive_fusion_epoch() || !xSemaphoreTake(passive.lock,portMAX_DELAY)) return;
    if (passive_fusion_epoch() && passive.reason==FEEDBACK_REASON_OK)
        passive_reject(reason,detail);
    xSemaphoreGive(passive.lock);
}

void passive_health_read(feedback_source_health *out)
{
    out->reason=FEEDBACK_REASON_NOT_STARTED;
    out->max_age_us=60000U;
    if (!passive.lock || !xSemaphoreTake(passive.lock,portMAX_DELAY)) return;
    out->timestamp_us=passive.last_good;
    out->sequence=passive.last_good?passive.sequence:0;
    if (feedback_requested()==FEEDBACK_SOURCE_FUSED) {
        if (!passive.mount.confirmed && !passive.mounting.enabled) out->reason=FEEDBACK_REASON_MOUNT_REQUIRED;
        else if (hw_motor_drivers_sleeping()) {out->reason=FEEDBACK_REASON_INHIBITED;out->detail=FUSION_FAULT_INHIBITED;}
        else if (platform_is_odometry_enabled()) {
            if (passive.reason==FEEDBACK_REASON_OK && passive_late(hw_clock_microseconds()))
                passive_reject(FEEDBACK_REASON_STALE,FUSION_FAULT_STALE);
            out->reason=passive.reason; out->detail=passive.fault;
        } else out->detail=FUSION_FAULT_ENCODER_UNAVAILABLE;
    }
    xSemaphoreGive(passive.lock);
}
#endif
