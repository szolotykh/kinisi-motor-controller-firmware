#include "feedback_source.h"

static int stopped_counter=-1;
static bool sampling_enabled(uint8_t i) { return i!=stopped_counter; }

static feedback_source_health health_status(uint8_t source)
{
    request((controller_command_t){.commandType=GET_FEEDBACK_SOURCE_HEALTH,
        .properties.get_feedback_source_health={source}},RESPONSE_OK);
    feedback_source_health out;
    assert(sizeof(out)==28 && reply_len==32);
    memcpy(&out,reply+4,sizeof(out));
    assert(out.source==source && out.age_us==(out.timestamp_us?now-out.timestamp_us:UINT32_MAX));
    return out;
}

static feedback_imu_channels die_times(void)
{
    request((controller_command_t){.commandType=GET_FEEDBACK_IMU_CHANNELS},RESPONSE_OK);
    feedback_imu_channels out;
    assert(sizeof(out)==24 && reply_len==28);
    memcpy(&out,reply+4,sizeof(out));
    return out;
}

static void source_health_scenario(unsigned preset)
{
    feedback_health_install(hw_imu_read_observation,sampling_enabled);
    assert(health_status(0).reason==FEEDBACK_REASON_NOT_STARTED);
    assert(health_status(1).health==FEEDBACK_HEALTH_UNAVAILABLE && die_times().gyro_age_us==UINT32_MAX);
    assert(health_status(2).reason==FEEDBACK_REASON_NOT_STARTED);
    request((controller_command_t){.commandType=GET_FEEDBACK_SOURCE_HEALTH,
        .properties.get_feedback_source_health={3}},RESPONSE_INVALID_ARGUMENT);
    configure(preset);
    request((controller_command_t){.commandType=START_PLATFORM_ODOMETRY},RESPONSE_OK);
    injected_status=HW_IMU_OK;injected=(hw_imu_sample_t){0,0,9.80665,0,0,0};
    sample_ticks(8,true);
    set_mount(0,RESPONSE_OK);
    sample_ticks(53,true);
    feedback_source_health e=health_status(0),i=health_status(1),f=health_status(2);
    assert(e.health==FEEDBACK_HEALTH_HEALTHY && e.flags&FEEDBACK_HEALTH_FLAG_COMPARISON_AVAILABLE);
    assert(i.health==FEEDBACK_HEALTH_HEALTHY && i.timestamp_us==injected_accel && i.sequence==health_status(1).sequence);
    assert(f.reason==FEEDBACK_REASON_NOT_STARTED && f.flags&FEEDBACK_HEALTH_FLAG_COMPARISON_AVAILABLE);
    assert(!tracking_task.fn && selected_status().requested==FEEDBACK_SOURCE_ENCODER);
    // Matching straight travel is deliberately indistinguishable from constant
    // steady slip. Stagger the shaft counts using each preset's real kinematics.
    for (unsigned k=0;k<40;k++) {
        wheel_count[0]+=4;
        wheel_count[1]+=preset==PLATFORM_OMNI?-4:4;
        if (preset==PLATFORM_MECANUM) {wheel_count[2]+=4;wheel_count[3]+=4;}
        sample_ticks(1,true);
    }
    assert(health_status(0).health==FEEDBACK_HEALTH_HEALTHY && health_status(1).flags&FEEDBACK_HEALTH_FLAG_COMPARISON_AVAILABLE);
    sample_ticks(28,true);
    injected.angular_velocity_z=1.2; // Plausible blocked counts or external rotation; neither is directly attributable.
    sample_ticks(35,true);
    assert(health_status(0).health==FEEDBACK_HEALTH_INDETERMINATE && health_status(1).health==FEEDBACK_HEALTH_INDETERMINATE);
    f=health_status(2);
    assert(f.health==FEEDBACK_HEALTH_UNAVAILABLE && f.flags&FEEDBACK_HEALTH_FLAG_DISAGREEMENT);
    injected.angular_velocity_z=0; sample_ticks(33,true); assert(health_status(0).health==FEEDBACK_HEALTH_HEALTHY);
    injected.acceleration_x=-1.8;sample_ticks(33,true);
    assert(health_status(0).flags&FEEDBACK_HEALTH_FLAG_DISAGREEMENT);
    injected.acceleration_x=0;sample_ticks(33,true);
    select_source(2,RESPONSE_OK);sample_ticks(35,true);
    assert(health_status(2).health==FEEDBACK_HEALTH_HEALTHY && selected_status().valid);
    if (preset!=PLATFORM_MECANUM) {
        feedback_source_status before=selected_status();
        stopped_counter=3;sample_ticks(1,true);
        assert(health_status(0).health==FEEDBACK_HEALTH_HEALTHY && health_status(2).health==FEEDBACK_HEALTH_HEALTHY);
        assert(selected_status().valid && selected_status().origin_id==before.origin_id);
        set_mount(0,RESPONSE_SOURCE_SWITCH_UNSAFE);
        stopped_counter=-1;sample_ticks(1,true);
        assert(selected_status().origin_id==before.origin_id && health_status(2).health==FEEDBACK_HEALTH_HEALTHY);
    }
    feedback_source_status frame=selected_status();
    i=health_status(1);
    uint64_t a=injected_accel;
    now+=40000; injected_gyro=now;
    feedback_imu_channels dies=die_times();
    assert(dies.accel_timestamp_us==a && dies.gyro_timestamp_us==now && dies.accel_age_us==50000 && !dies.gyro_age_us);
    feedback_source_health loss=health_status(1);
    assert(loss.reason==FEEDBACK_REASON_STALE && loss.sequence==i.sequence && !(loss.flags&FEEDBACK_HEALTH_FLAG_FRESH));
    assert(!selected_status().valid && health_status(2).timestamp_us==frame.timestamp_us);
    stale_capture(frame);
    injected_accel=now; assert(health_status(1).reason==FEEDBACK_REASON_WAITING_SAMPLE);
    now+=1000;injected_gyro=now;
    assert(health_status(1).health==FEEDBACK_HEALTH_HEALTHY && !selected_status().valid);
    sample_ticks(36,true);assert(selected_status().valid && selected_status().origin_id!=frame.origin_id);
    i=health_status(1);injected.acceleration_y=56;
    loss=health_status(1);
    assert(loss.health==FEEDBACK_HEALTH_SUSPECT && loss.flags&FEEDBACK_HEALTH_FLAG_SATURATION && loss.sequence==i.sequence);
    injected.acceleration_y=0;
    assert(health_status(1).flags&FEEDBACK_HEALTH_FLAG_RECOVERING);
    now+=1000;injected_accel=now+1;assert(health_status(1).flags&FEEDBACK_HEALTH_FLAG_TIMESTAMP_FAULT);
    now+=1000;injected_accel=injected_gyro=now; assert(health_status(1).health==FEEDBACK_HEALTH_HEALTHY);
    connect();assert(!platform_is_controller_running() && !motion_writes);
    select_source(0,RESPONSE_OK); sample_ticks(2,true);
    controller_command_t start={.commandType=START_PLATFORM_CONTROLLER,.properties.start_platform_controller={8,0,0,100}};
    controller_command_t drive={.commandType=SET_PLATFORM_TARGET_VELOCITY,.properties.set_platform_target_velocity={.x=.2}};
    request(start,RESPONSE_OK); request(drive,RESPONSE_OK);run_task(controller_task);assert(motion_writes);
    injected_status=HW_IMU_IO_ERROR;sample_ticks(2,true);
    assert(health_status(1).health==FEEDBACK_HEALTH_UNAVAILABLE && health_status(0).health==FEEDBACK_HEALTH_HEALTHY);
    run_task(controller_task); assert(platform_is_controller_running() && !hw_motor_outputs_stopped());
    e=health_status(0);stopped_counter=0;
    wheel_count[0]+=12000;unsigned writes=motion_writes;
    run_task(controller_task); assert_coasting();
    assert(!platform_is_controller_running() && motion_writes==writes);
    loss=health_status(0);assert(loss.health==FEEDBACK_HEALTH_UNAVAILABLE && loss.detail==FUSION_FAULT_ENCODER_UNAVAILABLE);
    assert(loss.timestamp_us==e.timestamp_us && !selected_status().valid);
    request(start,RESPONSE_ENCODER_NOT_INITIALIZED);request(drive,RESPONSE_CONTROLLER_NOT_INITIALIZED);
    request((controller_command_t){.commandType=GET_PLATFORM_ODOMETRY},RESPONSE_SAMPLE_NOT_AVAILABLE);
    sample_ticks(2,true);stopped_counter=-1;
    request((controller_command_t){.commandType=GET_PLATFORM_ODOMETRY},RESPONSE_SAMPLE_NOT_AVAILABLE);
    sample_ticks(1,true);request((controller_command_t){.commandType=GET_PLATFORM_ODOMETRY},RESPONSE_SAMPLE_NOT_AVAILABLE);
    sample_ticks(2,true);request((controller_command_t){.commandType=GET_PLATFORM_ODOMETRY},RESPONSE_OK);
    run_task(controller_task);
    assert(health_status(0).health==FEEDBACK_HEALTH_HEALTHY && motion_writes==writes);
    request(drive,RESPONSE_CONTROLLER_NOT_INITIALIZED);request(start,RESPONSE_OK);run_task(controller_task);
    assert_coasting();request(drive,RESPONSE_OK);run_task(controller_task);assert(motion_writes>writes);
    request((controller_command_t){.commandType=STOP_PLATFORM_CONTROLLER},RESPONSE_OK);
    assert_coasting();
    imu_action(IMU_TRACKING_ACTION_START,1,0,RESPONSE_OK);
    tracking_ticks(165,0,0);
    assert(imu_status().state==IMU_TRACKING_STATE_EXPIRED && health_status(1).health==FEEDBACK_HEALTH_HEALTHY);
    assert(!platform_is_controller_running());
}

static void source_health_edges(unsigned index)
{
    feedback_health_install(hw_imu_read_observation,sampling_enabled);
    injected_status=HW_IMU_OK;injected=(hw_imu_sample_t){0,0,9.80665,0,0,0};
    now=1000000;injected_accel=injected_gyro=now-2000;
    feedback_source_health good=health_status(1);
    now+=3000;injected_accel=injected_gyro=now-2000;
    request((controller_command_t){.commandType=SET_DRIVER_SLEEP,.properties.set_driver_sleep={1}},RESPONSE_OK);
    now+=2000;request((controller_command_t){.commandType=SET_DRIVER_SLEEP,.properties.set_driver_sleep={0}},RESPONSE_OK);
    feedback_source_health held=health_status(1);
    assert(held.health==FEEDBACK_HEALTH_UNAVAILABLE && held.timestamp_us==good.timestamp_us);
    now+=2000;injected_accel=now;assert(health_status(1).health==FEEDBACK_HEALTH_UNAVAILABLE);
    injected_gyro=now;assert(health_status(1).health==FEEDBACK_HEALTH_HEALTHY);
    configure(PLATFORM_DIFFERENTIAL);
    request((controller_command_t){.commandType=SET_ODOMETRY_FREQUENCY,.properties.set_odometry_frequency={1}},RESPONSE_OK);
    request((controller_command_t){.commandType=START_PLATFORM_ODOMETRY},RESPONSE_OK);
    sample_ticks(10,true);assert(odometry_feedback_at_rest(now));
    stopped_counter=index;
    assert(!odometry_feedback_at_rest(now));sample_ticks(2,true);
    stopped_counter=-1;
    select_source(FEEDBACK_SOURCE_IMU_ONLY,RESPONSE_SOURCE_SWITCH_UNSAFE);
    set_mount(0,RESPONSE_SOURCE_SWITCH_UNSAFE);
    sample_ticks(5,true);select_source(FEEDBACK_SOURCE_IMU_ONLY,RESPONSE_SOURCE_SWITCH_UNSAFE);
    sample_ticks(2,true);select_source(FEEDBACK_SOURCE_IMU_ONLY,RESPONSE_OK);
    assert(!motion_writes);
}

void health_queue_setup(void)
{
    feedback_health_install(hw_imu_read_observation,sampling_enabled);
    configure(PLATFORM_DIFFERENTIAL);
    request((controller_command_t){.commandType=START_PLATFORM_CONTROLLER,.properties.start_platform_controller={8,0,0,100}},RESPONSE_OK);
    request((controller_command_t){.commandType=SET_PLATFORM_TARGET_VELOCITY,.properties.set_platform_target_velocity={.x=.2}},RESPONSE_OK);
}
void health_queue_shaft(bool failed) { stopped_counter=failed?0:-1;feedback_guard_platform_control(); }
void health_queue_output(bool running)
{
    run_task(controller_task);
    assert((platform_is_controller_running()!=0)==running && hw_motor_outputs_stopped()!=running);
}
void run_counter_queues(unsigned bus);

static void live_fused_outage(bool position)
{
    configure(PLATFORM_DIFFERENTIAL);
    request((controller_command_t){.commandType=START_PLATFORM_ODOMETRY},RESPONSE_OK);
    request((controller_command_t){.commandType=START_PLATFORM_CONTROLLER,.properties.start_platform_controller={8,0,0,100}},RESPONSE_OK);
    injected_status=HW_IMU_OK;injected=(hw_imu_sample_t){0,0,9.80665,0,0,0};
    request((controller_command_t){.commandType=SET_FUSION_EXPERIMENT,.properties.set_fusion_experiment={FUSION_MODE_FUSED,0}},RESPONSE_OK);
    sample_ticks(45,true);
    controller_command_t target={.commandType=SET_PLATFORM_TARGET_VELOCITY,.properties.set_platform_target_velocity={.x=.2}};
    if (position) {
        request((controller_command_t){.commandType=INITIALIZE_PLATFORM_POSITION_CONTROLLER,
            .properties.initialize_platform_position_controller={.linear_kp=1,.angular_kp=1,.position_tolerance=.01,
                .heading_tolerance=.01,.max_linear_speed=.3,.max_angular_speed=.6}},RESPONSE_OK);
        target=(controller_command_t){.commandType=SET_PLATFORM_POSITION,.properties.set_platform_position={.x=.2}};
    }
    request(target,RESPONSE_OK);run_task(controller_task);assert(!hw_motor_outputs_stopped());
    now+=1000;injected_status=HW_IMU_IO_ERROR;feedback_health_poll();
    assert_coasting();
    assert(diagnostic().health==FUSION_HEALTH_FAULT && diagnostic().flags==FUSION_FLAG_FRESH_TARGET_REQUIRED);
    now+=2000;injected_status=HW_IMU_OK;injected_accel=injected_gyro=now;
    feedback_health_poll();request(target,RESPONSE_SAMPLE_NOT_AVAILABLE);
    sample_ticks(34,true);run_task(controller_task);assert_coasting();
    assert(diagnostic().health==FUSION_HEALTH_HEALTHY);
    request(target,RESPONSE_OK);run_task(controller_task);assert(!hw_motor_outputs_stopped());
}

static uint16_t captured_endpoints;
static void capture_endpoint_fence(uint16_t edges) { captured_endpoints|=edges; }
static uint16_t no_inflight_changes(void) { return 0; }
static void buffered_request(controller_command_t cmd,uint16_t barriers,uint8_t expected)
{
    cmd.message_id=++next_id;reply_len=0;
    connection_receive_queued(&session,(uint8_t *)&cmd,command_request_size(cmd.commandType),barriers<<ENCODER_BARRIER_SHIFT);
    connection_poll(&session);
    if ((expected && reply[5]!=expected) || reply[1]!=(expected?KINISI_MESSAGE_ERROR:cmd.commandType)) fprintf(stderr,"buffered cmd=%u barrier=%x reply=%u error=%u expected=%u\n",cmd.commandType,barriers,reply[1],reply[5],expected);
    assert(reply_len>=4 && (expected ? (reply[1]==KINISI_MESSAGE_ERROR && reply[5]==expected) : reply[1]==cmd.commandType));
}

static void binding_recovery(bool chassis)
{
    feedback_health_install(hw_imu_read_observation,sampling_enabled);
    feedback_install_encoder_fence(capture_endpoint_fence,no_inflight_changes);
    configure(chassis?PLATFORM_MECANUM:PLATFORM_DIFFERENTIAL);
    controller_command_t setup={.commandType=INITIALIZE_MOTOR_CONTROLLER,
        .properties.initialize_motor_controller={.motor_index=3,.encoder_index=2,.encoder_resolution=800,.kp=8,.integral_limit=100}};
    controller_command_t speed={.commandType=SET_MOTOR_TARGET_SPEED,.properties.set_motor_target_speed={.motor_index=3,.speed=2}};
    controller_command_t init_position={.commandType=INITIALIZE_MOTOR_POSITION_CONTROLLER,
        .properties.initialize_motor_position_controller={.motor_index=3,.kp=1,.max_speed=.3,.tolerance=.01}};
    controller_command_t position={.commandType=SET_MOTOR_POSITION,.properties.set_motor_position={.motor_index=3,.position=.2}};
    if (chassis) {
        setup=(controller_command_t){.commandType=START_PLATFORM_CONTROLLER,.properties.start_platform_controller={8,0,0,100}};
        speed=(controller_command_t){.commandType=SET_PLATFORM_TARGET_VELOCITY,.properties.set_platform_target_velocity={.x=.2}};
        init_position=(controller_command_t){.commandType=INITIALIZE_PLATFORM_POSITION_CONTROLLER,
            .properties.initialize_platform_position_controller={.linear_kp=1,.angular_kp=1,.position_tolerance=.01,.heading_tolerance=.01,.max_linear_speed=.3,.max_angular_speed=.6}};
        position=(controller_command_t){.commandType=SET_PLATFORM_POSITION,.properties.set_platform_position={.x=.2}};
    } else {
        controller_command_t other=setup;
        other.properties.initialize_motor_controller.motor_index=2;other.properties.initialize_motor_controller.encoder_index=1;
        request(other,RESPONSE_OK);
    }
    request(setup,RESPONSE_OK);request(speed,RESPONSE_OK);run_task(controller_task);
    stopped_counter=chassis?3:2;feedback_guard_platform_control();assert_coasting();
    captured_endpoints=0;stopped_counter=-1;feedback_guard_platform_control();
    const uint16_t old=captured_endpoints;
    if (chassis) {
        configure(PLATFORM_DIFFERENTIAL);
        request((controller_command_t){.commandType=START_PLATFORM_ODOMETRY},RESPONSE_OK);
        sample_ticks(3,true);
    } else setup.properties.initialize_motor_controller.encoder_index=3;
    request(setup,RESPONSE_OK);
    request(init_position,RESPONSE_OK);
    buffered_request(speed,old,RESPONSE_SAMPLE_NOT_AVAILABLE);
    buffered_request(position,old,RESPONSE_SAMPLE_NOT_AVAILABLE);
    run_task(controller_task);assert_coasting();
    if (!chassis) {
        controller_command_t unrelated=speed;unrelated.properties.set_motor_target_speed.motor_index=2;
        buffered_request(unrelated,old,RESPONSE_OK);
    }
    request(speed,RESPONSE_OK);run_task(controller_task);assert(!hw_motor_outputs_stopped());
}
