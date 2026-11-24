// Real framed requests, acquisition tasks, board clock, encoders and retained journal.
static void select_source(uint8_t source,uint8_t expected)
{
    request((controller_command_t){.commandType=SET_FEEDBACK_SOURCE,.properties.set_feedback_source={source}},expected);
}
static feedback_source_status selected_status(void)
{
    request((controller_command_t){.commandType=GET_FEEDBACK_SOURCE_STATUS},RESPONSE_OK);
    feedback_source_status s;
    assert(sizeof(s)==27 && reply_len==31);
    memcpy(&s,reply+4,sizeof(s));
    if (!s.valid) assert(s.active==FEEDBACK_SOURCE_NONE && !s.sequence && !s.timestamp_us && s.age_us==UINT32_MAX);
    return s;
}
static feedback_source_vector captured_vector(uint8_t kind, feedback_source_status key)
{
    request((controller_command_t){.commandType=GET_FEEDBACK_SOURCE_VECTOR,.properties.get_feedback_source_vector={kind,key.origin_id,key.sequence}},RESPONSE_OK);
    feedback_source_vector v;
    assert(sizeof(v)==28 && reply_len==32);
    memcpy(&v,reply+4,sizeof(v));
    return v;
}
static feedback_source_vector selected_vector(uint8_t kind)
{
    return captured_vector(kind,selected_status());
}
static void stale_capture(feedback_source_status key)
{
    request((controller_command_t){.commandType=GET_FEEDBACK_SOURCE_VECTOR,
        .properties.get_feedback_source_vector={0,key.origin_id,key.sequence}},RESPONSE_SAMPLE_NOT_AVAILABLE);
}
static void set_mount(double yaw,uint8_t expected)
{
    request((controller_command_t){.commandType=SET_FEEDBACK_FUSION_MOUNT,.properties.set_feedback_fusion_mount={yaw}},expected);
}
static void invalid_vector(void)
{
    request((controller_command_t){.commandType=GET_FEEDBACK_SOURCE_VECTOR},RESPONSE_SAMPLE_NOT_AVAILABLE);
}

static void selector_imu(void)
{
    select_source(FEEDBACK_SOURCE_IMU_ONLY,RESPONSE_OK);
    assert(selected_status().reason==FEEDBACK_REASON_NOT_STARTED && !tracking_task.fn);
    imu_action(0,1,0,RESPONSE_OK);
    assert(selected_status().reason==FEEDBACK_REASON_CALIBRATING);
    tracking_ticks(55,0,0);
    feedback_source_status s=selected_status();
    imu_tracking_status native=imu_status();
    assert(s.valid && s.active==FEEDBACK_SOURCE_IMU_ONLY && s.sequence==native.sequence &&
           s.timestamp_us==native.timestamp_us && s.age_us==now-native.timestamp_us);
    uint32_t origin=s.origin_id;
    tracking_ticks(12,1,.4);
    feedback_source_vector pose=selected_vector(0),twist=selected_vector(1);
    imu_tracking_vector p=imu_vector(0),t=imu_vector(1);
    assert(pose.sequence==p.sequence && pose.x==p.x && pose.z==p.z && pose.x>0 && pose.z>0);
    assert(twist.sequence==t.sequence && twist.x==t.x && twist.z==t.z && twist.x>0);
    // Realistic UART service spacing exceeds one publication interval. A second
    // client can capture the newer sample without changing this pair.
    feedback_source_status held=selected_status();
    assert(held.max_age_ms==50);
    tracking_ticks(1,1,.4);
    feedback_source_status newer=selected_status();
    assert(newer.sequence!=held.sequence && newer.origin_id==held.origin_id);
    assert(captured_vector(0,held).x==pose.x);
    tracking_ticks(1,1,.4);
    assert(captured_vector(1,held).x==twist.x);
    assert(captured_vector(1,newer).sequence==newer.sequence);
    select_source(FEEDBACK_SOURCE_ENCODER,RESPONSE_SOURCE_SWITCH_UNSAFE);
    connect();
    assert(selected_status().origin_id==origin && selected_status().sequence>p.sequence);
    tracking_ticks(4,0,0); stale_capture(held);
    tracking_ticks(95,0,0);
    s=selected_status();
    assert(s.requested==FEEDBACK_SOURCE_IMU_ONLY && s.reason==FEEDBACK_REASON_EXPIRED &&
           s.source_detail==IMU_TRACKING_FAULT_WINDOW_EXPIRED);
    invalid_vector(); stale_capture(held);
    imu_action(2,1,0,RESPONSE_OK);
    assert(selected_status().origin_id!=origin);
    tracking_ticks(54,0,0);
    assert(selected_status().valid && selected_vector(0).x==0);
    // A gyro watermark captured just before the switch may be delivered
    // afterward. Only a genuinely post-switch source acquisition anchors.
    select_source(FEEDBACK_SOURCE_ENCODER,RESPONSE_OK);
    now+=10000;
    select_source(FEEDBACK_SOURCE_IMU_ONLY,RESPONSE_OK);
    now+=1000; injected_accel=injected_gyro=now-2000; run_task(tracking_task);
    assert(imu_status().state==IMU_TRACKING_STATE_TRACKING);
    assert(selected_status().reason==FEEDBACK_REASON_WAITING_SAMPLE); invalid_vector();
    tracking_ticks(1,0,0); assert(selected_status().valid);
    now+=35001;
    assert(selected_status().reason==FEEDBACK_REASON_STALE);
    invalid_vector();
    assert(!motion_writes && !platform.is_initialized);
}

static void selector_platform(void)
{
    assert(selected_status().requested==FEEDBACK_SOURCE_ENCODER && !selected_status().valid);
    connection_init(&session,command_handler,send_frame,hw_clock_microseconds);
    request((controller_command_t){.commandType=GET_FEEDBACK_SOURCE_STATUS},RESPONSE_INIT_REQUIRED);
    request((controller_command_t){.commandType=INIT,
        .properties.init={.protocol_major=2,.protocol_minor=21,.client_capabilities=CLIENT_CAP_WALL_CLOCK}},RESPONSE_OK);
    request((controller_command_t){.commandType=SET_FEEDBACK_SOURCE},RESPONSE_CLOCK_NOT_READY);
    connect();
    configure(PLATFORM_DIFFERENTIAL);
    request((controller_command_t){.commandType=START_PLATFORM_ODOMETRY},RESPONSE_OK);
    select_source(1,RESPONSE_SOURCE_SWITCH_UNSAFE);
    sample_ticks(7,true);
    feedback_source_status enc=selected_status();
    assert(enc.active==FEEDBACK_SOURCE_ENCODER && enc.timestamp_us==now && !enc.age_us);
    wheel_count[0]+=50; wheel_count[1]+=95;
    sample_ticks(1,true);
    feedback_source_vector p=selected_vector(0), t=selected_vector(1);
    platform_odometry_t native; uint64_t native_at;
    assert(odometry_manager_get_platform_sample(&native,&native_at)==RESPONSE_OK);
    assert(p.x==native.x && p.z==native.t && p.x>0 && p.z!=0 && t.x>0 && native_at==now);
    select_source(2,RESPONSE_SOURCE_SWITCH_UNSAFE);
    sample_ticks(7,true);
    request((controller_command_t){.commandType=START_PLATFORM_CONTROLLER,
        .properties.start_platform_controller={.kp=1,.integral_limit=10}},RESPONSE_OK);
    select_source(2,RESPONSE_SOURCE_SWITCH_UNSAFE);
    request((controller_command_t){.commandType=STOP_PLATFORM_CONTROLLER},RESPONSE_OK);
    // Verify independent encoder movement since the latest task sample immediately blocks mutation.
    ++wheel_count[3];
    select_source(2,RESPONSE_SOURCE_SWITCH_UNSAFE);
    sample_ticks(8,true);
    select_source(2,RESPONSE_OK);
    assert(selected_status().reason==FEEDBACK_REASON_MOUNT_REQUIRED);
    invalid_vector();
    set_mount(INFINITY,RESPONSE_INVALID_ARGUMENT);
    set_mount(0,RESPONSE_OK);
    assert(selected_status().reason==FEEDBACK_REASON_CALIBRATING && !platform_is_controller_running());
    request((controller_command_t){.commandType=GET_FEEDBACK_FUSION_MOUNT},RESPONSE_OK);
    assert(reply_len==13 && reply[4]==1);
    injected_status=HW_IMU_OK;
    injected=(hw_imu_sample_t){0,0,9.80665,0,0,0};
    sample_ticks(32,true);
    feedback_source_status fused=selected_status();
    assert(fused.active==FEEDBACK_SOURCE_FUSED && fused.timestamp_us==now && !fused.age_us);
    assert(selected_vector(0).x==0 && !platform_fusion_epoch() && diagnostic().mode==FUSION_MODE_LEGACY_ENCODER);
    // Wheels stay still; gyro yaw and linear acceleration distinguish fused telemetry from raw encoder odometry.
    injected.acceleration_x=-1; injected.angular_velocity_z=.5;
    sample_ticks(9,true);
    p=selected_vector(0); t=selected_vector(1);
    assert(p.x>0 && p.z>0 && t.z>.49 && t.x>0);
    assert(selected_status().sequence==p.sequence && selected_status().origin_id==fused.origin_id);
    connect();
    assert(selected_status().sequence==p.sequence && selected_status().origin_id==fused.origin_id);
    sample_ticks(4,false);
    assert(selected_status().reason==FEEDBACK_REASON_STALE);
    invalid_vector();
    injected=(hw_imu_sample_t){0,0,9.80665,0,0,0}; sample_ticks(30,true);
    assert(selected_status().valid && selected_status().origin_id!=fused.origin_id);
    assert(selected_vector(0).x==0);
    fused=selected_status();
    request((controller_command_t){.commandType=SET_DRIVER_SLEEP,.properties.set_driver_sleep={1}},RESPONSE_OK);
    assert(selected_status().reason==FEEDBACK_REASON_INHIBITED); stale_capture(fused);
    request((controller_command_t){.commandType=SET_DRIVER_SLEEP,.properties.set_driver_sleep={0}},RESPONSE_OK);
    assert(!selected_status().valid); stale_capture(fused);
    sample_ticks(32,true); assert(selected_status().valid);
    wheel_count[1]+=180; sample_ticks(8,true);
    select_source(0,RESPONSE_OK);
    assert(!selected_status().valid); // Wait for an actual post-switch producer sample.
    sample_ticks(1,true);
    assert(selected_vector(0).x==0 && selected_vector(0).z==0);
    // Raw world frame is already rotated; selector axes follow the new heading.
    platform_odometry_t anchor;
    assert(odometry_manager_get_platform_sample(&anchor,&native_at)==RESPONSE_OK && fabs(anchor.t)>.1);
    wheel_count[0]+=30; wheel_count[1]+=30; sample_ticks(1,true);
    assert(odometry_manager_get_platform_sample(&native,&native_at)==RESPONSE_OK);
    p=selected_vector(0);
    double dx=native.x-anchor.x, dy=native.y-anchor.y;
    assert(p.x>0 && fabs(p.x - (cos(anchor.t)*dx+sin(anchor.t)*dy))<1e-12);
    assert(fabs(p.y - (-sin(anchor.t)*dx+cos(anchor.t)*dy))<1e-12 && p.z==native.t-anchor.t);
    enc=selected_status();
    assert(enc.max_age_ms==60);
    request((controller_command_t){.commandType=BRAKE_PLATFORM},RESPONSE_OK);
    select_source(1,RESPONSE_SOURCE_SWITCH_UNSAFE);
    request((controller_command_t){.commandType=COAST_PLATFORM},RESPONSE_OK);
    now+=61000; assert(selected_status().reason==FEEDBACK_REASON_STALE); invalid_vector();
    sample_ticks(2,true); assert(selected_status().origin_id!=enc.origin_id);
    sample_ticks(7,true);
    enc=selected_status();
    select_source(FEEDBACK_SOURCE_IMU_ONLY,RESPONSE_OK); stale_capture(enc);
    select_source(FEEDBACK_SOURCE_ENCODER,RESPONSE_OK); sample_ticks(1,true);
    assert(selected_status().origin_id!=enc.origin_id); stale_capture(enc);
    assert(!platform_is_controller_running() && !motion_writes && !braked_mask);
}

static void selector_legacy(bool factory)
{
    assert(selected_status().requested==0 && !selected_status().valid);
    assert(!platform_is_controller_running() && !platform_is_odometry_enabled() && !motion_writes);
    if (factory) assert(field(0,0)==0);
    else {
        assert(field(0,0)==PLATFORM_MECANUM && field(1,0)==.12 && field(2,0)==.48 && field(3,0)==.31);
        for (unsigned i=0;i<4;++i) {
            assert(field(4,i)==(i ? 2000 : 4096));
            assert(field(5,i)==(i!=3) && field(6,i)==(i%2==0));
        }
        check_frequency(25000);
    }
}
