#include "imu_mount.h"

// The mock delivers physical BMI088 die values; use independent basis vectors:
// board roll +90 maps sensor (sx,sy,sz) to chassis (-sx,-sz,-sy).
static hw_imu_sample_t right_wall(double center_x, double yaw_rate, double yaw_acceleration)
{
    // At y=-0.1 m the chip has +0.1*alpha forward and +0.1*w^2 left.
    return (hw_imu_sample_t){-(center_x+.1*yaw_acceleration),-9.80665,-.1*yaw_rate*yaw_rate,
                             0,-yaw_rate,0};
}

static void mounted_imu_case(void)
{
    controller_command_t mount={.commandType=SET_IMU_MOUNT,
        .properties.set_imu_mount={9000,0,0,0,-100000,40000}};
    request(mount,RESPONSE_OK);
    const uint64_t started=now;
    imu_action(IMU_TRACKING_ACTION_START,1,1.4,RESPONSE_OK); // Legacy yaw must not rotate a second time.
    injected_status=HW_IMU_OK;
    for (int i=0;i<65;++i) {
        now+=10000; injected_accel=injected_gyro=now-1000;
        injected=right_wall(0,0,0);
        run_task(tracking_task);
    }
    assert(imu_status().state==IMU_TRACKING_STATE_TRACKING);
    imu_mount_config_t changed=imu_mount_get();
    changed.x_um=90000;
    for (unsigned lock=1;lock<=4;++lock) {
        // Interrupt each reservation in the shared install. The old transform,
        // estimator and public snapshot must still describe the same episode.
        uint32_t sequence=imu_status().sequence;
        fail_mutex_take=lock;
        assert(!imu_mount_restore(&changed));
        assert(!fail_mutex_take && imu_mount_get().x_um==0);
        assert(imu_status().state==IMU_TRACKING_STATE_TRACKING && imu_status().sequence==sequence);
    }
    for (int i=1;i<=40;++i) {
        now+=10000; injected_accel=injected_gyro=now-1000;
        double w=i<=20 ? i*.05 : 1.0;
        injected=right_wall(0,w,i<=20 ? 5. : 0.);
        run_task(tracking_task);
    }
    imu_tracking_vector p=imu_vector(0),v=imu_vector(1);
    assert(hypot(p.x,p.y)<1e-8 && hypot(v.x,v.y)<1e-8 && p.z>.28 && p.z<.32 && v.z>.99);
    // Origin translation survives the same rotation and lever correction.
    for (int i=0;i<12;++i) {
        now+=10000; injected_accel=injected_gyro=now-1000;
        injected=right_wall(1,1,0);
        run_task(tracking_task);
    }
    v=imu_vector(1);
    assert(v.x>.10 && fabs(v.y)<.03);
    // A source read older than START is allowed only until this epoch has
    // actually consumed data. A later rewind to that stamp is a timestamp fault.
    now+=1000; injected_accel=started; injected_gyro=now-100;
    run_task(tracking_task);
    assert(imu_status().fault==IMU_TRACKING_FAULT_TIMESTAMP);
    imu_action(IMU_TRACKING_ACTION_RESET,1,-.8,RESPONSE_OK);
    for (int i=0;i<65;++i) {
        now+=10000; injected_accel=injected_gyro=now-1000;
        injected=right_wall(0,0,0); run_task(tracking_task);
    }
    assert(imu_status().state==IMU_TRACKING_STATE_TRACKING && fabs(imu_vector(0).z)<1e-12);
    now+=40000;
    assert(imu_status().fault==IMU_TRACKING_FAULT_STALE);
    // A successful geometry change disarms the one-second diagnostic.
    mount.properties.set_imu_mount.x_um=1000;
    request(mount,RESPONSE_OK);
    assert(imu_status().state==IMU_TRACKING_STATE_STOPPED && !motion_writes);
}

static void mounted_fusion_case(void)
{
    feedback_health_install(hw_imu_read_observation,sampling_enabled);
    request((controller_command_t){.commandType=SET_IMU_MOUNT,
        .properties.set_imu_mount={9000,0,0,0,-100000,40000}},RESPONSE_OK);
    configure(PLATFORM_DIFFERENTIAL);
    injected_status=HW_IMU_OK;
    injected=right_wall(0,0,0);
    request((controller_command_t){.commandType=START_PLATFORM_ODOMETRY},RESPONSE_OK);
    sample_ticks(45,true);
    assert(health_status(0).flags&FEEDBACK_HEALTH_FLAG_COMPARISON_AVAILABLE);
    select_source(FEEDBACK_SOURCE_FUSED,RESPONSE_OK); // No independent legacy mount declaration.
    sample_ticks(40,true);
    assert(selected_status().valid && health_status(2).health==FEEDBACK_HEALTH_HEALTHY);
    request((controller_command_t){.commandType=SET_FUSION_EXPERIMENT,
        .properties.set_fusion_experiment={FUSION_MODE_FUSED,-1.3}},RESPONSE_OK);
    sample_ticks(45,true);
    assert(diagnostic().health==FUSION_HEALTH_HEALTHY);
    fusion_vector active=vector(FUSION_VECTOR_FUSED_TWIST);
    feedback_source_vector passive=selected_vector(1);
    assert(hypot(active.x,active.y)<1e-7 && hypot(passive.x,passive.y)<1e-7);
    double peak_active=0,peak_passive=0;
    for (unsigned i=1;i<=36;++i) {
        // a is 1 ms earlier than g. The first gyro revealing either change
        // of slope is unavailable at that accel time. Check both public
        // velocities throughout the maneuver against the original 0.02 m/s gate.
        double w=i<=20 ? .04*i : .8;
        double alpha=i<=20 ? 2. : 0.;
        injected=right_wall(0,w-alpha*.001,alpha);
        injected.angular_velocity_y=-w;
        sample_ticks(1,true);
        active=vector(FUSION_VECTOR_FUSED_TWIST);
        passive=selected_vector(1);
        peak_active=fmax(peak_active,hypot(active.x,active.y));
        peak_passive=fmax(peak_passive,hypot(passive.x,passive.y));
    }
    printf("Causal mount transition: peak active %.9f, passive %.9f m/s; final active %.9f, passive %.9f m/s\n",
           peak_active,peak_passive,hypot(active.x,active.y),hypot(passive.x,passive.y));
    assert(peak_active<=.02 && peak_passive<=.02 && active.z>.79);
    assert(health_status(1).flags&FEEDBACK_HEALTH_FLAG_DISAGREEMENT); // Encoders did not turn.
    injected.acceleration_x=56; sample_ticks(1,true);
    assert(diagnostic().fault==FUSION_FAULT_SATURATION);
    assert(health_status(1).flags&FEEDBACK_HEALTH_FLAG_SATURATION);
    assert(!selected_status().valid && !motion_writes);
}

static void mounted_fast_case(void)
{
    request((controller_command_t){.commandType=SET_IMU_MOUNT},RESPONSE_OK);
    configure(PLATFORM_DIFFERENTIAL);
    request((controller_command_t){.commandType=SET_ODOMETRY_FREQUENCY,
        .properties.set_odometry_frequency={200}},RESPONSE_OK);
    request((controller_command_t){.commandType=START_PLATFORM_ODOMETRY},RESPONSE_OK);
    injected_status=HW_IMU_OK; injected=(hw_imu_sample_t){0,0,9.80665,0,0,0};
    for (int i=0;i<40;++i) {
        now+=5000; injected_accel=injected_gyro=now;
        run_task(odom_task);
    }
    select_source(FEEDBACK_SOURCE_FUSED,RESPONSE_OK);
    uint64_t last_stamp=0;
    uint32_t sequence=0;
    for (int i=0;i<430;++i) {
        now+=5000;
        if (i%2==0) injected_accel=injected_gyro=now;
        run_task(odom_task);
        if (i<150) continue;
        feedback_source_status s=selected_status();
        assert(s.valid && s.timestamp_us==injected_gyro);
        if (last_stamp==s.timestamp_us) assert(s.sequence==sequence);
        else if (last_stamp) assert(s.sequence==sequence+1);
        sequence=s.sequence; last_stamp=s.timestamp_us;
    }
}

static void mounted_slow_case(bool displaced)
{
    feedback_health_install(hw_imu_read_observation,sampling_enabled);
    request((controller_command_t){.commandType=SET_IMU_MOUNT,
        .properties.set_imu_mount={9000,0,0,0,displaced?-100000:0,0}}, RESPONSE_OK);
    configure(PLATFORM_DIFFERENTIAL);
    injected_status=HW_IMU_OK; injected=right_wall(0,0,0);
    request((controller_command_t){.commandType=SET_ODOMETRY_FREQUENCY,
        .properties.set_odometry_frequency={25}},RESPONSE_OK);
    request((controller_command_t){.commandType=START_PLATFORM_ODOMETRY},RESPONSE_OK);
    sample_ticks(10,true);
    select_source(FEEDBACK_SOURCE_FUSED,RESPONSE_OK);
    request((controller_command_t){.commandType=SET_FUSION_EXPERIMENT,
        .properties.set_fusion_experiment={FUSION_MODE_FUSED,0}},RESPONSE_OK);
    // The real odometry task wakes every 20 ms for shaft rest sampling; two
    // wakes publish one complete 40 ms platform batch at this setting.
    for (int i=0;i<96;++i) {
        now+=20000; injected_accel=now-2000; injected_gyro=now-1000;
        run_task(odom_task);
    }
    fusion_status active=diagnostic();
    feedback_source_status passive=selected_status();
    if (active.health!=FUSION_HEALTH_HEALTHY || !passive.valid)
        fprintf(stderr,"25 Hz displaced=%d active=%d/%d passive=%d/%d age=%u/%u/%u\n",displaced,
                active.health,active.fault,passive.reason,passive.source_detail,
                (unsigned)active.encoder_age_us,(unsigned)active.accel_age_us,(unsigned)active.gyro_age_us);
    assert(active.health==FUSION_HEALTH_HEALTHY && passive.valid);
    assert(health_status(1).flags&FEEDBACK_HEALTH_FLAG_COMPARISON_AVAILABLE);
    for (int i=0;i<2;++i) { now+=20000; run_task(odom_task); } // Held pair now exceeds 35 ms.
    assert(!selected_status().valid && diagnostic().fault==FUSION_FAULT_STALE);
}
