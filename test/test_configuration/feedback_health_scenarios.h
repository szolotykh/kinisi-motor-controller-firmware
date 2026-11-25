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
