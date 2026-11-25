#pragma once
// Included by command_handler.c. Producer observations never select a source or
// change a control target. The health lock has no nested estimator/controller locks.
static SemaphoreHandle_t health_lock;
static feedback_imu_reader_t health_reader;
static bool (*health_encoder_timer)(uint8_t);
static struct {
    uint64_t a,g,good_a,good_g,barrier_a,barrier_g,power_boundary;
    uint32_t sequence;
    uint8_t reason,detail;
    bool recovering;
    hw_imu_sample_t raw;
} physical_imu={.reason=FEEDBACK_REASON_WAITING_SAMPLE};

typedef struct { double x,y,w; } motion3_t;
static struct {
    bool mount,calibrated,have_window,conflict;
    imu_mount_frame_t mounting;
    imu_mount_history_t history;
    uint64_t mounting_boundary;
    double c,s;
    uint32_t calibration_count, agreements, disagreements;
    uint64_t calibration_start,previous_a,previous_g,window,compared,last_encoder;
    motion3_t bias,sum,initial,previous,impulse;
    double wheel_yaw;
} comparison;

static void health_init(void)
{
    // Only board startup or the serialized command dispatcher creates the mutex.
    if (!health_lock) health_lock=xSemaphoreCreateMutex();
}

void feedback_health_install(feedback_imu_reader_t reader,bool (*timer)(uint8_t))
{
    health_init();
    health_reader=reader;
    health_encoder_timer=timer;
}

bool feedback_encoder_sampling(uint8_t i)
{
    return !health_encoder_timer || health_encoder_timer(i);
}

static uint32_t raw_epoch = 0x20000000U;

uint32_t feedback_health_raw_epoch(void)
{
    return __atomic_load_n(&raw_epoch,__ATOMIC_ACQUIRE);
}

void feedback_health_power_transition(void)
{
    // Invalidate even across a complete sleep/wake between producer polls.
    __atomic_add_fetch(&raw_epoch,1,__ATOMIC_RELEASE);
    if (health_lock && xSemaphoreTake(health_lock,portMAX_DELAY)) {
        physical_imu.power_boundary=hw_clock_microseconds();
        xSemaphoreGive(health_lock);
    }
    feedback_health_reset_comparison();
    feedback_health_poll();
}

static void health_forget_window(void)
{
    comparison.have_window=false;
    comparison.compared=comparison.previous_a=comparison.previous_g=comparison.last_encoder=0;
    comparison.disagreements=comparison.agreements=0;
    comparison.conflict=false;
}

static void reset_comparison_locked(void)
{
    comparison.history=(imu_mount_history_t){0};
    health_forget_window();
    comparison.calibrated=false;
    comparison.calibration_start=comparison.calibration_count=0;
    comparison.sum=(motion3_t){0};
}

void feedback_health_reset_comparison(void)
{
    if (!health_lock || !xSemaphoreTake(health_lock,portMAX_DELAY)) return;
    reset_comparison_locked();
    xSemaphoreGive(health_lock);
}

void feedback_health_counter_edge(bool owned)
{
    if (!health_lock || !xSemaphoreTake(health_lock,portMAX_DELAY)) return;
    if (owned || !comparison.calibrated) reset_comparison_locked();
    xSemaphoreGive(health_lock);
}

void feedback_health_mount(double yaw)
{
    if (!health_lock || !xSemaphoreTake(health_lock,portMAX_DELAY)) return;
    reset_comparison_locked();
    comparison.mount=true; comparison.c=cos(yaw); comparison.s=sin(yaw);
    xSemaphoreGive(health_lock);
}

#if BOARD_HAS_IMU_MOUNTING
uint8_t platform_fusion_set_mount(const imu_mount_frame_t *frame);
static imu_mount_config_t installation;

imu_mount_config_t imu_mount_get(void) { return installation; } // Serialized command/restore thread.

static uint8_t apply_mount(const imu_mount_config_t *mount)
{
    health_init();
    if (!tracking_mutex) tracking_mutex=xSemaphoreCreateMutex();
    if (!tracking_mutex || !health_lock) return RESPONSE_INTERNAL_ERROR;
    imu_mount_frame_t frame=imu_mount_frame(mount);
    // Reserve every mutable observer before changing the first. Normal polling
    // releases the health lock before calling fusion, and tracking never takes
    // the health or fusion locks. No source estimate spans installations.
    if (!xSemaphoreTake(tracking_mutex,portMAX_DELAY)) return RESPONSE_INTERNAL_ERROR;
    if (!xSemaphoreTake(health_lock,portMAX_DELAY)) {
        xSemaphoreGive(tracking_mutex);
        return RESPONSE_INTERNAL_ERROR;
    }
    uint8_t result=platform_fusion_set_mount(&frame);
    if (result!=RESPONSE_OK) {
        xSemaphoreGive(health_lock);
        xSemaphoreGive(tracking_mutex);
        return result;
    }
    imu_only_stop(&tracker);
    tracking_mount=frame;
    tracking_history=(imu_mount_history_t){0};
    tracking_origin=published_sample=tracking_elapsed=0;
    tracking_publish_active();
    feedback_invalidate(FEEDBACK_SOURCE_IMU_ONLY);
    comparison.mounting=frame;
    comparison.mounting_boundary=hw_clock_microseconds();
    reset_comparison_locked();
    installation=*mount;
    xSemaphoreGive(health_lock);
    xSemaphoreGive(tracking_mutex);
    return RESPONSE_OK;
}

uint8_t imu_mount_set(const imu_mount_config_t *mount)
{
    if (!imu_mount_valid(mount)) return RESPONSE_INVALID_ARGUMENT;
    if (!feedback_switch_safe()) return RESPONSE_SOURCE_SWITCH_UNSAFE;
    if (mount->configured==installation.configured && mount->roll_cdeg==installation.roll_cdeg &&
        mount->pitch_cdeg==installation.pitch_cdeg && mount->yaw_cdeg==installation.yaw_cdeg &&
        mount->x_um==installation.x_um && mount->y_um==installation.y_um && mount->z_um==installation.z_um)
        return RESPONSE_OK;
    return apply_mount(mount);
}

bool imu_mount_restore(const imu_mount_config_t *mount)
{
    return imu_mount_valid(mount) && apply_mount(mount)==RESPONSE_OK;
}
#endif

static void health_imu_failed(uint8_t reason,uint8_t detail)
{
    comparison.history=(imu_mount_history_t){0};
    physical_imu.reason=reason; physical_imu.detail=detail;
    physical_imu.recovering=physical_imu.good_a!=0;
    physical_imu.barrier_a=physical_imu.a;
    physical_imu.barrier_g=physical_imu.g;
    if (!comparison.calibrated) reset_comparison_locked();
    else health_forget_window();
}

static void health_poll_locked(void)
{
    if (hw_motor_drivers_sleeping()) {
        health_imu_failed(FEEDBACK_REASON_INHIBITED,FUSION_FAULT_INHIBITED);
        return;
    }
    hw_imu_sample_t raw={0};
    uint64_t a=0,g=0;
    hw_imu_status_t status=(health_reader?health_reader:hw_imu_read_observation)(0,&raw,&a,&g);
    uint64_t now=hw_clock_microseconds();
    // Unusable channels retain their previous true acquisition times. Future
    // and reversed stamps never become public provenance.
    bool invalid_time=(a && (a>now || a<physical_imu.a)) || (g && (g>now || g<physical_imu.g));
    if (a && a<=now && a>physical_imu.a) physical_imu.a=a;
    if (g && g<=now && g>physical_imu.g) physical_imu.g=g;
    if (invalid_time) {health_imu_failed(FEEDBACK_REASON_FAULT,FUSION_FAULT_TIMESTAMP);return;}
    if (status!=HW_IMU_OK || !a || !g) {
        bool stale=status==HW_IMU_STALE || (status==HW_IMU_NOT_READY && physical_imu.good_a &&
            (tracking_age(now,physical_imu.a)>35000 || tracking_age(now,physical_imu.g)>35000));
        health_imu_failed(stale?FEEDBACK_REASON_STALE:status==HW_IMU_NOT_READY?FEEDBACK_REASON_WAITING_SAMPLE:FEEDBACK_REASON_UNAVAILABLE,
                          stale?FUSION_FAULT_STALE:FUSION_FAULT_IMU_UNAVAILABLE);
        return;
    }
    if (now-a>35000 || now-g>35000 || (a>g?a-g:g-a)>15000) {
        health_imu_failed(FEEDBACK_REASON_STALE,FUSION_FAULT_STALE);return;
    }
    if (!isfinite(raw.acceleration_x) || !isfinite(raw.acceleration_y) || !isfinite(raw.acceleration_z) ||
        !isfinite(raw.angular_velocity_x) || !isfinite(raw.angular_velocity_y) || !isfinite(raw.angular_velocity_z)) {
        health_imu_failed(FEEDBACK_REASON_FAULT,FUSION_FAULT_IMU_UNAVAILABLE);return;
    }
    if (fabs(raw.acceleration_x)>=55 || fabs(raw.acceleration_y)>=55 || fabs(raw.acceleration_z)>=55 ||
        fabs(raw.angular_velocity_x)>=8 || fabs(raw.angular_velocity_y)>=8 || fabs(raw.angular_velocity_z)>=8) {
        health_imu_failed(FEEDBACK_REASON_FAULT,FUSION_FAULT_SATURATION);return;
    }
    if (a<=physical_imu.power_boundary || g<=physical_imu.power_boundary ||
        (physical_imu.recovering && (a<=physical_imu.barrier_a || g<=physical_imu.barrier_g))) {
        physical_imu.reason=FEEDBACK_REASON_WAITING_SAMPLE;return;
    }
    if (a!=physical_imu.good_a || g!=physical_imu.good_g) ++physical_imu.sequence;
    physical_imu.good_a=a; physical_imu.good_g=g;physical_imu.raw=raw;
    physical_imu.reason=FEEDBACK_REASON_OK;physical_imu.detail=FUSION_FAULT_NONE;
    physical_imu.recovering=false;
}

void feedback_health_poll(void)
{
    if (!health_lock || !xSemaphoreTake(health_lock,portMAX_DELAY)) return;
    health_poll_locked();
    uint8_t reason=physical_imu.reason, detail=physical_imu.detail;
    xSemaphoreGive(health_lock);
    if (reason!=FEEDBACK_REASON_OK) {
        passive_health_fault(reason,detail);
        platform_fusion_observed_imu_loss(detail?detail:FUSION_FAULT_IMU_UNAVAILABLE);
    }
}

static void health_compare(uint64_t at,uint64_t previous,platform_odometry_t delta,bool valid,bool stationary)
{
    uint64_t a=physical_imu.good_a,g=physical_imu.good_g;
    if ((!comparison.mount && !comparison.mounting.enabled) || !valid || physical_imu.reason!=FEEDBACK_REASON_OK ||
        !previous || at<=previous || at-previous>60000 || a>at || g>at ||
        tracking_age(at,a)>35000 || tracking_age(at,g)>35000) {
        health_forget_window();
        comparison.calibration_count=0;comparison.calibration_start=0;comparison.sum=(motion3_t){0};
        return;
    }
    if (comparison.last_encoder && comparison.last_encoder!=previous) health_forget_window();
    comparison.last_encoder=at;
    hw_imu_sample_t r=physical_imu.raw;
    if (comparison.mounting.enabled) {
        if (a<=comparison.mounting_boundary || g<=comparison.mounting_boundary) return;
        imu_mount_result_t mr=imu_mount_observe(&comparison.mounting,&comparison.history,&r,a,g,comparison.calibrated?comparison.bias.w:0,60000,&r);
        if (mr==MOUNT_NEED_HISTORY) return;
        if (mr!=MOUNT_SAMPLE_OK) {
            reset_comparison_locked();
            return;
        }
    }
    if (fabs(r.acceleration_z-9.80665)>1.0 || fabs(r.angular_velocity_x)>.35 || fabs(r.angular_velocity_y)>.35) {
        if (!comparison.calibrated) reset_comparison_locked();
        else health_forget_window();
        return;
    }
    motion3_t f={-comparison.c*r.acceleration_x+comparison.s*r.acceleration_y,
                 -comparison.s*r.acceleration_x-comparison.c*r.acceleration_y,r.angular_velocity_z};
    if (comparison.mounting.enabled) f=(motion3_t){r.acceleration_x,r.acceleration_y,r.angular_velocity_z};
    if (!comparison.calibrated) {
        if (!stationary || hypot(f.x,f.y)>.8 || fabs(f.w)>.08) {
            comparison.calibration_count=0;comparison.calibration_start=0;comparison.sum=(motion3_t){0};
        } else if (a!=comparison.previous_a && g!=comparison.previous_g) {
            if (!comparison.calibration_count) comparison.calibration_start=at;
            comparison.sum.x+=f.x;comparison.sum.y+=f.y;comparison.sum.w+=f.w;
            unsigned n=++comparison.calibration_count;
            if (n>=20 && at-comparison.calibration_start>=480000) {
                comparison.bias=(motion3_t){comparison.sum.x/n,comparison.sum.y/n,comparison.sum.w/n};
                comparison.calibrated=true;
            }
        }
        comparison.previous_a=a;comparison.previous_g=g;
        return;
    }
    double seconds=(at-previous)*1e-6;
    motion3_t wheel={delta.x/seconds,delta.y/seconds,delta.t/seconds};
    if (!isfinite(wheel.x) || !isfinite(wheel.y) || !isfinite(wheel.w)) {health_forget_window();return;}
    // Repeated die stamps contribute zero new inertial impulse. Wheel yaw
    // still accrues on every genuinely new wheel interval.
    f.x-=comparison.bias.x;f.y-=comparison.bias.y;f.w-=comparison.bias.w;
    // Body-coordinate derivative of translational velocity on a level mount.
    f.x+=f.w*wheel.y; f.y-=f.w*wheel.x;
    if (!comparison.have_window || a<comparison.previous_a || g<comparison.previous_g ||
        a-comparison.previous_a>60000 || g-comparison.previous_g>60000) {
        health_forget_window();
        comparison.have_window=true; comparison.window=at; comparison.initial=wheel;
        comparison.impulse=(motion3_t){0};comparison.wheel_yaw=0;
    } else {
        double da=(a-comparison.previous_a)*1e-6,dg=(g-comparison.previous_g)*1e-6;
        comparison.impulse.x+=.5*(comparison.previous.x+f.x)*da;
        comparison.impulse.y+=.5*(comparison.previous.y+f.y)*da;
        comparison.impulse.w+=.5*(comparison.previous.w+f.w)*dg;
        comparison.wheel_yaw+=delta.t;
        if (at-comparison.window>=200000) {
            // Two disjoint 200 ms windows distinguish persistent disagreement from
            // a single phase-shifted transient. Recovery requires two agreeing windows.
            double dv=hypot(wheel.x-comparison.initial.x-comparison.impulse.x,wheel.y-comparison.initial.y-comparison.impulse.y);
            double dw=fabs(comparison.wheel_yaw-comparison.impulse.w);
            double duration=(at-comparison.window)*1e-6;
            bool mismatch=dv>.12+.25*hypot(comparison.impulse.x,comparison.impulse.y) || dw>.32*duration+.015;
            comparison.disagreements=mismatch?(comparison.disagreements<2?comparison.disagreements+1:2):0;
            comparison.agreements=mismatch?0:(comparison.agreements<2?comparison.agreements+1:2);
            if (comparison.disagreements>=2) comparison.conflict=true;
            if (comparison.agreements>=2) comparison.conflict=false;
            comparison.compared=at;
            comparison.initial=wheel;comparison.window=at;comparison.impulse=(motion3_t){0};comparison.wheel_yaw=0;
        }
    }
    comparison.last_encoder=at;
    comparison.previous=f;comparison.previous_a=a;comparison.previous_g=g;
}

void feedback_health_observe(uint64_t at,uint64_t previous,platform_odometry_t delta,bool valid,bool stationary)
{
    if (!health_lock || !xSemaphoreTake(health_lock,portMAX_DELAY)) return;
    bool sampled=comparison.mount || comparison.mounting.enabled;
    if (sampled) health_poll_locked();
    health_compare(at,previous,delta,valid,stationary);
    uint8_t reason=physical_imu.reason,detail=physical_imu.detail;
    xSemaphoreGive(health_lock);
    if (sampled && reason!=FEEDBACK_REASON_OK) {
        passive_health_fault(reason,detail);
        platform_fusion_observed_imu_loss(detail?detail:FUSION_FAULT_IMU_UNAVAILABLE);
    }
}

static uint8_t feedback_health_read(uint8_t command,uint8_t source,void (*reply)(uint8_t *,uint8_t))
{
    if (source>FEEDBACK_SOURCE_FUSED) return RESPONSE_INVALID_ARGUMENT;
    if (!health_lock) return RESPONSE_INTERNAL_ERROR;
    feedback_health_poll();
    feedback_source_health out={.source=source};
    feedback_source_health wheel={0};
    if (command==GET_FEEDBACK_SOURCE_HEALTH) {
        odometry_health_read(&wheel);
        if (source==FEEDBACK_SOURCE_ENCODER) {out=wheel;out.source=source;}
        if (wheel.detail==FUSION_FAULT_ENCODER_UNAVAILABLE) passive_health_fault(wheel.reason,wheel.detail);
    }
    if (source==FEEDBACK_SOURCE_FUSED) passive_health_read(&out);
    if (!xSemaphoreTake(health_lock,portMAX_DELAY)) return RESPONSE_INTERNAL_ERROR;
    uint64_t now=hw_clock_microseconds();
    if (command==GET_FEEDBACK_IMU_CHANNELS) {
        feedback_imu_channels dies={physical_imu.a,physical_imu.g,tracking_age(now,physical_imu.a),tracking_age(now,physical_imu.g)};
        xSemaphoreGive(health_lock);reply((uint8_t *)&dies,sizeof(dies));return RESPONSE_OK;
    }
    if (source==FEEDBACK_SOURCE_IMU_ONLY) {
        out.max_age_us=35000;out.reason=physical_imu.reason;out.detail=physical_imu.detail;
        out.sequence=physical_imu.sequence;
        out.timestamp_us=physical_imu.good_a<physical_imu.good_g?physical_imu.good_a:physical_imu.good_g;
        if (physical_imu.recovering && out.reason==FEEDBACK_REASON_WAITING_SAMPLE) out.flags|=FEEDBACK_HEALTH_FLAG_RECOVERING;
    }
    if (source==FEEDBACK_SOURCE_FUSED && out.reason==FEEDBACK_REASON_OK && physical_imu.reason!=FEEDBACK_REASON_OK) {
        out.reason=physical_imu.reason;out.detail=physical_imu.detail;
    }
    out.age_us=tracking_age(now,out.timestamp_us);
    if (out.timestamp_us) out.flags|=FEEDBACK_HEALTH_FLAG_PROVENANCE;
    if (out.timestamp_us && out.age_us<=out.max_age_us) out.flags|=FEEDBACK_HEALTH_FLAG_FRESH;
    else if (out.reason==FEEDBACK_REASON_OK) {out.reason=FEEDBACK_REASON_STALE;out.detail=FUSION_FAULT_STALE;}
    if (out.reason==FEEDBACK_REASON_CALIBRATING) out.flags|=FEEDBACK_HEALTH_FLAG_RECOVERING;
    if (out.detail==FUSION_FAULT_SATURATION) out.flags|=FEEDBACK_HEALTH_FLAG_SATURATION;
    if (out.detail==FUSION_FAULT_TIMESTAMP) out.flags|=FEEDBACK_HEALTH_FLAG_TIMESTAMP_FAULT;
    out.health=out.reason==FEEDBACK_REASON_OK?FEEDBACK_HEALTH_HEALTHY:
        out.detail==FUSION_FAULT_SATURATION?FEEDBACK_HEALTH_SUSPECT:FEEDBACK_HEALTH_UNAVAILABLE;
    if (wheel.reason==FEEDBACK_REASON_OK && tracking_age(now,wheel.timestamp_us)<=wheel.max_age_us &&
        comparison.compared && comparison.last_encoder && tracking_age(now,comparison.last_encoder)<=60000 &&
        tracking_age(now,comparison.compared)<=260000 && physical_imu.reason==FEEDBACK_REASON_OK) {
        out.flags|=FEEDBACK_HEALTH_FLAG_COMPARISON_AVAILABLE;
        if (comparison.conflict) {
            if (out.reason==FEEDBACK_REASON_OK) out.health=FEEDBACK_HEALTH_INDETERMINATE;
            out.flags|=FEEDBACK_HEALTH_FLAG_DISAGREEMENT;
        }
    }
    xSemaphoreGive(health_lock);
    reply((uint8_t *)&out,sizeof(out));return RESPONSE_OK;
}
