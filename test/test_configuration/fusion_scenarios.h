// Exercises wire requests, the production RTOS loop bodies and actual wheel PWM.
static void simulated_fence(uint32_t *generation) { ++*generation; }
static fusion_status diagnostic(void)
{
    request((controller_command_t){.commandType=GET_FUSION_STATUS}, RESPONSE_OK);
    fusion_status status;
    assert(reply_len == sizeof(status) + 4);
    memcpy(&status, reply + 4, sizeof(status));
    return status;
}

static fusion_vector vector(uint8_t kind)
{
    request((controller_command_t){.commandType=GET_FUSION_VECTOR, .properties.get_fusion_vector={kind}}, RESPONSE_OK);
    fusion_vector v;
    memcpy(&v, reply + 4, sizeof(v));
    return v;
}

static void sample_ticks(unsigned ticks, bool fresh)
{
    for (unsigned i = 0; i < ticks; ++i) {
        now += 20000;
        if (fresh) {
            injected_accel = now - 10000;
            injected_gyro = now - 9000;
        }
        run_task(odom_task);
    }
}

static void assert_coasting(void)
{
    for (unsigned i = 0; i < 4; ++i) assert(!output[i]);
}

static void check_deadline(controller_command_t target)
{
    for (unsigned observer = 0; observer < 3; ++observer) {
        now += 20000;
        injected_accel = injected_gyro = now - 20000;
        run_task(odom_task);
        request(target, RESPONSE_OK); run_task(controller_task);
        bool driven = false;
        for (unsigned i = 0; i < 4; ++i) driven |= output[i] != 0;
        assert(driven);
        now += 36000; // Past the 55 ms IMU deadline, before the 60 ms encoder deadline.
        if (observer < 2) {
            if (observer == 1) { run_task(supervisor_task); assert_coasting(); }
            fusion_status stale = diagnostic();
            assert(stale.fault == FUSION_FAULT_STALE && stale.health == FUSION_HEALTH_FAULT &&
                   (stale.flags & FUSION_FLAG_FRESH_TARGET_REQUIRED));
            assert_coasting();
        }
        now += 4000;
        injected_accel = injected_gyro = now - 10000;
        run_task(odom_task); // Third case: first observer is the late producer, no GET or wheel tick.
        fusion_status recovered = diagnostic();
        assert(recovered.health == FUSION_HEALTH_HEALTHY &&
               (recovered.flags & FUSION_FLAG_FRESH_TARGET_REQUIRED) &&
               !(recovered.flags & FUSION_FLAG_CORRECTION_ACTIVE));
        run_task(controller_task); assert_coasting();
    }
}

static void check_rapid_inhibit(controller_command_t target, bool inertial)
{
    request(target, RESPONSE_OK); run_task(controller_task);
    fusion_status running = diagnostic();
    assert(running.flags & FUSION_FLAG_CORRECTION_ACTIVE);
    request((controller_command_t){.commandType=SET_DRIVER_SLEEP}, RESPONSE_OK); // Already awake.
    assert(diagnostic().sequence == running.sequence && (diagnostic().flags & FUSION_FLAG_CORRECTION_ACTIVE));
    fail_driver_transition = inertial;
    request((controller_command_t){.commandType=SET_DRIVER_SLEEP, .properties.set_driver_sleep={1}},
            inertial ? RESPONSE_INTERNAL_ERROR : RESPONSE_OK);
    fusion_status sleeping = diagnostic();
    assert_coasting();
    assert(sleeping.health == FUSION_HEALTH_FAULT && sleeping.fault == FUSION_FAULT_INHIBITED &&
           sleeping.flags == FUSION_FLAG_FRESH_TARGET_REQUIRED && sleeping.encoder_age_us == UINT32_MAX);
    if (inertial) {
        request((controller_command_t){.commandType=SET_DRIVER_SLEEP}, RESPONSE_INTERNAL_ERROR);
        assert(diagnostic().fault == FUSION_FAULT_INHIBITED && diagnostic().sequence == sleeping.sequence);
    }
    fail_driver_transition = false;
    request((controller_command_t){.commandType=SET_DRIVER_SLEEP}, RESPONSE_OK); // No odometry tick.
    fusion_status waking = diagnostic();
    assert(waking.health == FUSION_HEALTH_WAITING && waking.flags == FUSION_FLAG_FRESH_TARGET_REQUIRED &&
           waking.sequence != sleeping.sequence);
    request((controller_command_t){.commandType=START_PLATFORM_CONTROLLER,
        .properties.start_platform_controller={2,0,0,100}}, RESPONSE_OK);
    request(target, RESPONSE_SAMPLE_NOT_AVAILABLE);
    run_task(controller_task); assert_coasting();
    sample_ticks(4, inertial); // Recent stationary history from before wake cannot complete qualification.
    request(target, RESPONSE_SAMPLE_NOT_AVAILABLE);
    sample_ticks(inertial ? 30 : 3, inertial);
    assert(diagnostic().health == FUSION_HEALTH_HEALTHY &&
           (diagnostic().flags & FUSION_FLAG_FRESH_TARGET_REQUIRED));
    run_task(controller_task); assert_coasting();
    request(target, RESPONSE_OK); run_task(controller_task);
    assert(diagnostic().flags & FUSION_FLAG_CORRECTION_ACTIVE);
    request((controller_command_t){.commandType=SET_PLATFORM_TARGET_VELOCITY}, RESPONSE_OK);
}

static void check_recovery_pose(controller_command_t goal, controller_command_t pos_init)
{
    // Create a real fused-vs-encoder heading offset, then coast while the IMU is missing.
    injected.angular_velocity_z = 2.0;
    for (unsigned i = 0; i < 180; ++i) { wheel_count[0] += 2; sample_ticks(1, true); }
    fusion_vector before_f = vector(FUSION_VECTOR_FUSED_POSE), before_e = vector(FUSION_VECTOR_ENCODER_POSE);
    double angle = before_f.z - before_e.z;
    assert(before_f.z > 2 * M_PI && fabs(angle) > 0.015);
    injected.angular_velocity_z = 0.004;
    injected_status = HW_IMU_IO_ERROR;
    for (unsigned i = 0; i < 6; ++i) {
        wheel_count[0] += 15; wheel_count[1] += 45;
        sample_ticks(1, true);
    }
    injected_status = HW_IMU_OK;
    for (unsigned i = 0; i < 6; ++i) {
        wheel_count[0] += 15; wheel_count[1] += 45;
        sample_ticks(1, true);
    }
    sample_ticks(32, true);
    fusion_vector after_f = vector(FUSION_VECTOR_FUSED_POSE), after_e = vector(FUSION_VECTOR_ENCODER_POSE);
    double dx = after_e.x - before_e.x, dy = after_e.y - before_e.y;
    assert(hypot(dx, dy) > 0.01 && fabs(after_e.z - before_e.z) > 0.02);
    assert(fabs(after_f.x - (before_f.x + cos(angle)*dx - sin(angle)*dy)) < 0.0001);
    assert(fabs(after_f.y - (before_f.y + sin(angle)*dx + cos(angle)*dy)) < 0.0001);
    assert(after_f.z > 2 * M_PI && fabs(after_f.z - after_e.z - angle) < 0.0001);
    injected_status = HW_IMU_IO_ERROR; sample_ticks(1, true);
    injected_status = HW_IMU_OK; sample_ticks(32, true);
    assert(fabs(vector(FUSION_VECTOR_FUSED_POSE).z - after_f.z) < 0.0001); // Stationary outage, raw angle.

    assert_coasting();
    request(pos_init, RESPONSE_OK);
    goal.properties.set_platform_position.x = after_f.x;
    goal.properties.set_platform_position.y = after_f.y;
    goal.properties.set_platform_position.t = after_f.z;
    request(goal, RESPONSE_OK); run_task(controller_task); assert_coasting();
    request((controller_command_t){.commandType=RESET_PLATFORM_POSITION}, RESPONSE_OK);
    sample_ticks(32, true);
    fusion_vector reset = vector(FUSION_VECTOR_FUSED_POSE);
    assert(fabs(reset.x) < 0.0001 && fabs(reset.y) < 0.0001 && fabs(reset.z) < 0.0001);
}

static void fusion_scenario(int type)
{
    const controller_command_t target = {.commandType=SET_PLATFORM_TARGET_VELOCITY,
        .properties.set_platform_target_velocity={0.12,0.08,0.14}};
    const controller_command_t zero = {.commandType=SET_PLATFORM_TARGET_VELOCITY};
    const controller_command_t position_init = {.commandType=INITIALIZE_PLATFORM_POSITION_CONTROLLER,
        .properties.initialize_platform_position_controller={.linear_kp=1, .angular_kp=1,
            .position_tolerance=0.01, .heading_tolerance=0.01, .max_linear_speed=0.3, .max_angular_speed=0.6}};
    const controller_command_t goal = {.commandType=SET_PLATFORM_POSITION,
        .properties.set_platform_position={0.1,0.05,0.02}};
    controller_command_t select = {.commandType=SET_FUSION_EXPERIMENT,
        .properties.set_fusion_experiment={FUSION_MODE_ENCODER_CORRECTION,1.5707963267948966}};
    fusion_status s = diagnostic();
    assert(s.mode == 0 && s.health == FUSION_HEALTH_DISABLED && s.accel_age_us == UINT32_MAX);
    request((controller_command_t){.commandType=GET_FUSION_VECTOR}, RESPONSE_SAMPLE_NOT_AVAILABLE);
    request(select, RESPONSE_PLATFORM_NOT_INITIALIZED);
    configure(type);
    if (type != PLATFORM_MECANUM) {
        request((controller_command_t){.commandType=SET_MOTOR_SPEED, .properties.set_motor_speed={3,20}}, RESPONSE_OK);
        request(select, RESPONSE_INVALID_ARGUMENT);
        request((controller_command_t){.commandType=STOP_MOTOR, .properties.stop_motor={3}}, RESPONSE_OK);
    }
    request((controller_command_t){.commandType=SET_PLATFORM_VELOCITY,
        .properties.set_platform_velocity={10,0,0}}, RESPONSE_OK);
    request(select, RESPONSE_INVALID_ARGUMENT);
    request((controller_command_t){.commandType=COAST_PLATFORM}, RESPONSE_OK);
    request((controller_command_t){.commandType=START_PLATFORM_CONTROLLER,
        .properties.start_platform_controller={2,0,0,100}}, RESPONSE_OK);
    request(position_init, RESPONSE_OK);
    request(select, RESPONSE_OK);
    request(goal, RESPONSE_SAMPLE_NOT_AVAILABLE);
    // A new pre-motion configuration can ACK; actual wheel motion, even on a
    // standalone channel, keeps it WAITING until a full quiet window is observed.
    for (unsigned i = 0; i < 8; ++i) {
        wheel_count[3] += 1;
        sample_ticks(1, false);
        assert(diagnostic().health == FUSION_HEALTH_WAITING);
        request(target, RESPONSE_SAMPLE_NOT_AVAILABLE);
        run_task(controller_task); assert_coasting();
    }
    request(select, RESPONSE_INVALID_ARGUMENT);
    sample_ticks(6, false);
    s = diagnostic();
    assert(s.health == FUSION_HEALTH_HEALTHY && !observation_calls && s.encoder_age_us == 0);
    request(target, RESPONSE_OK);
    run_task(controller_task);
    assert(diagnostic().flags & FUSION_FLAG_CORRECTION_ACTIVE);
    double comparator[4]; memcpy(comparator, output, sizeof(comparator));
    assert(motion_writes);
    double first[4], capped[4];
    for (unsigned i = 0; i < 4; ++i) first[i] = controllers_manager_get_motor_controller_state(i).target_speed;
    for (unsigned tick = 0; tick < 40; ++tick) { sample_ticks(5, false); run_task(controller_task); }
    bool integrated = false;
    for (unsigned i = 0; i < 4; ++i) {
        capped[i] = controllers_manager_get_motor_controller_state(i).target_speed;
        integrated |= fabs(capped[i] - first[i]) > 0.01;
    }
    assert(integrated); // Persistent stationary-wheel error accumulates only to the PI output cap.
    for (unsigned tick = 0; tick < 3; ++tick) { sample_ticks(5, false); run_task(controller_task); }
    for (unsigned i = 0; i < 4; ++i) assert(fabs(controllers_manager_get_motor_controller_state(i).target_speed - capped[i]) < 1e-9);
    request(zero, RESPONSE_OK); request(target, RESPONSE_OK); run_task(controller_task);
    for (unsigned i = 0; i < 4; ++i) assert(fabs(controllers_manager_get_motor_controller_state(i).target_speed - first[i]) < 1e-9);
    request(select, RESPONSE_INVALID_ARGUMENT); // Active direct target, even between wheel ticks.
    request(zero, RESPONSE_OK);
    for (unsigned i = 0; i < 4; ++i) assert(!output[i]);
    check_rapid_inhibit(target, false);
    select.properties.set_fusion_experiment.mode = FUSION_MODE_FUSED;
    request(select, RESPONSE_OK);
    request(target, RESPONSE_SAMPLE_NOT_AVAILABLE);
    request(zero, RESPONSE_OK);
    injected = (hw_imu_sample_t){0.02,0.03,9.80665,0,0,0.004};
    injected_status = HW_IMU_OK;
    sample_ticks(30, true);
    s = diagnostic();
    assert(s.health == FUSION_HEALTH_HEALTHY && s.flags == 7 &&
           s.accel_age_us == 10000 && s.gyro_age_us == 9000 && s.encoder_age_us == 0);
    fusion_vector bias = vector(FUSION_VECTOR_BIAS);
    assert(bias.sequence == s.sequence && fabs(bias.x - 0.03) < 1e-9 &&
           fabs(bias.y + 0.02) < 1e-9 && fabs(bias.z - 0.004) < 1e-9);
    request(target, RESPONSE_OK); run_task(controller_task);
    for (unsigned i = 0; i < 4; ++i) assert(fabs(comparator[i] - output[i]) < 1e-6);
    check_rapid_inhibit(target, true);
    check_deadline(target);
    request(target, RESPONSE_OK); run_task(controller_task);
    sample_ticks(2, false); // Re-reading identical source events does not create new source time.
    s = diagnostic();
    assert(s.health == FUSION_HEALTH_FAULT && s.fault == FUSION_FAULT_STALE &&
           s.flags == FUSION_FLAG_FRESH_TARGET_REQUIRED);
    for (unsigned i = 0; i < 4; ++i) { assert(!output[i]); wheel_count[i] += 42; }
    run_task(controller_task); // External motion cannot restart a latched wheel PID.
    for (unsigned i = 0; i < 4; ++i) assert(!output[i]);
    request(target, RESPONSE_SAMPLE_NOT_AVAILABLE);
    request((controller_command_t){.commandType=GET_FUSION_VECTOR,
        .properties.get_fusion_vector={FUSION_VECTOR_FUSED_POSE}}, RESPONSE_SAMPLE_NOT_AVAILABLE);
    platform_fusion_install_ingress_fence(simulated_fence);
    platform_fusion_dequeued(); // Request was in flight before healthy source recovery.
    sample_ticks(32, true);
    request(target, RESPONSE_SAMPLE_NOT_AVAILABLE);
    platform_fusion_dequeued();
    platform_fusion_install_ingress_fence(NULL);
    s = diagnostic(); assert(s.health == FUSION_HEALTH_HEALTHY && !(s.flags & FUSION_FLAG_CORRECTION_ACTIVE));
    run_task(controller_task);
    for (unsigned i = 0; i < 4; ++i) assert(!output[i]);
    check_recovery_pose(goal, position_init);
    request(position_init, RESPONSE_OK); request(goal, RESPONSE_OK); run_task(controller_task);
    assert(diagnostic().flags & FUSION_FLAG_CORRECTION_ACTIVE);
    motor_fault = true; sample_ticks(1, true);
    s = diagnostic(); assert(s.health == FUSION_HEALTH_FAULT && s.fault == FUSION_FAULT_INHIBITED);
    for (unsigned i = 0; i < 4; ++i) assert(!output[i]);
    motor_fault = false; sample_ticks(33, true);
    request(goal, RESPONSE_OK); run_task(controller_task);
    injected.acceleration_x = 58.0; sample_ticks(1, true);
    assert(diagnostic().fault == FUSION_FAULT_SATURATION);
    injected.acceleration_x = 0.02; injected.acceleration_z = -9.8;
    sample_ticks(1, true); assert(diagnostic().fault == FUSION_FAULT_MOUNT_INVALID);
    injected.acceleration_z = 9.80665;
    injected_status = HW_IMU_IO_ERROR; sample_ticks(1, true);
    assert(diagnostic().fault == FUSION_FAULT_IMU_UNAVAILABLE);
    injected_status = HW_IMU_OK;
    injected_gyro = now + 20001; sample_ticks(1, false);
    assert(diagnostic().fault == FUSION_FAULT_TIMESTAMP);
    sample_ticks(32, true); assert(diagnostic().health == FUSION_HEALTH_HEALTHY);
    request(target, RESPONSE_OK); run_task(controller_task);
    now += 100000; run_task(controller_task); // Stalled odometry never holds the previous target.
    assert(diagnostic().fault == FUSION_FAULT_STALE);
    for (unsigned i = 0; i < 4; ++i) assert(!output[i]);
    request((controller_command_t){.commandType=SET_BOARD_IDLE, .properties.set_board_idle={1}}, RESPONSE_OK);
    unsigned reads = observation_calls; sample_ticks(2, true); assert(observation_calls == reads);
    assert(diagnostic().fault == FUSION_FAULT_INHIBITED);
    request(select, RESPONSE_BOARD_IDLE);
    request((controller_command_t){.commandType=SET_BOARD_IDLE}, RESPONSE_OK);
    sample_ticks(34, true);
    assert(diagnostic().flags & FUSION_FLAG_FRESH_TARGET_REQUIRED);
    assert(!platform_is_controller_running());
}
