// Real framed session and scheduler task, behind independently timestamped raw samples.
static imu_tracking_status imu_status(void)
{
    request((controller_command_t){.commandType = GET_IMU_TRACKING_STATUS}, RESPONSE_OK);
    imu_tracking_status value;
    assert(reply_len == 4 + sizeof(value));
    memcpy(&value, reply + 4, sizeof(value));
    return value;
}

static void imu_action(unsigned action, unsigned confirmed, double yaw, unsigned error)
{
    request((controller_command_t){.commandType = SET_IMU_TRACKING,
                                   .properties.set_imu_tracking = {action, confirmed, yaw}},
            error);
}

static imu_tracking_vector imu_vector(unsigned kind)
{
    request((controller_command_t){.commandType = GET_IMU_TRACKING_VECTOR,
                                   .properties.get_imu_tracking_vector = {kind}},
            RESPONSE_OK);
    imu_tracking_vector value;
    assert(reply_len == 4 + sizeof(value));
    memcpy(&value, reply + 4, sizeof(value));
    return value;
}

static void tracking_ticks(unsigned count, double accel, double gyro)
{
    injected_status = HW_IMU_OK;
    for (unsigned i = 0; i < count; ++i) {
        now += 10000;
        injected_accel = now - 1000;
        injected_gyro = now - 2000;
        injected = (hw_imu_sample_t){accel, 0, 9.80665, 0, 0, gyro};
        run_task(tracking_task);
    }
}

static void verify_source_window(uint64_t lag, bool freeze_source)
{
    imu_action(IMU_TRACKING_ACTION_RESET, 1, 0, RESPONSE_OK);
    injected_status = HW_IMU_OK;
    injected = (hw_imu_sample_t){0, 0, 9.80665, 0, 0, 0};
    imu_tracking_status status = imu_status();
    for (unsigned i = 0; i < 60 && status.state == IMU_TRACKING_STATE_CALIBRATING; ++i) {
        now += 10000;
        injected_accel = injected_gyro = now-lag;
        run_task(tracking_task);
        status = imu_status();
    }
    uint64_t origin = status.timestamp_us;
    for (unsigned i = 0; i < 100; ++i) {
        if (i) {
            now += 10000;
            injected_accel = injected_gyro = now-lag;
            run_task(tracking_task);
            status = imu_status();
        }
        assert(status.state == IMU_TRACKING_STATE_TRACKING && status.timestamp_us == origin+i*10000u);
        assert(imu_vector(0).sequence == status.sequence && now-status.timestamp_us == lag);
    }
    assert(now >= origin+1000000); // Last eligible estimate arrives at/after the wall-time boundary.
    if (freeze_source) {
        now = origin+1050000;
        status = imu_status();
        assert(status.state==IMU_TRACKING_STATE_FAULT && status.fault==IMU_TRACKING_FAULT_STALE);
        assert(status.timestamp_us==origin+990000);
        return;
    }
    now += 10000; injected_accel = injected_gyro = now-lag;
    run_task(tracking_task);
    status=imu_status();
    assert(status.state==IMU_TRACKING_STATE_EXPIRED && status.timestamp_us==origin+990000);
    assert(status.elapsed_us==1000000 && now-origin<=1050000);
    request((controller_command_t){.commandType=GET_IMU_TRACKING_VECTOR}, RESPONSE_SAMPLE_NOT_AVAILABLE);
    now=origin+1050000;
    assert(imu_status().state==IMU_TRACKING_STATE_EXPIRED);
}

static void imu_tracking_scenario(void)
{
    assert(imu_status().state == 0 && imu_status().accel_age_us == UINT32_MAX);
    assert(!platform.is_initialized && !tasks_started);
    imu_action(0, 0, 0, RESPONSE_INVALID_ARGUMENT);
    imu_action(0, 2, 0, RESPONSE_INVALID_ARGUMENT);
    imu_action(3, 1, INFINITY, RESPONSE_INVALID_ARGUMENT);
    imu_action(1, 1, 0, RESPONSE_INVALID_ARGUMENT);
    imu_action(1, 0, 1, RESPONSE_INVALID_ARGUMENT);
    imu_action(4, 1, 0, RESPONSE_INVALID_ARGUMENT);
    request((controller_command_t){.commandType = GET_IMU_TRACKING_VECTOR}, RESPONSE_SAMPLE_NOT_AVAILABLE);
    request((controller_command_t){.commandType = GET_IMU_TRACKING_VECTOR,
                                   .properties.get_imu_tracking_vector = {3}},
            RESPONSE_INVALID_ARGUMENT);
    imu_action(0, 1, acos(-1), RESPONSE_OK);
    uint32_t first_epoch = imu_acquisition_epoch(0);
    assert((first_epoch & UINT32_C(0x80000000)) && imu_acquisition_epoch(37) == 37);
    injected_status = HW_IMU_NOT_READY;
    now += 50000;
    run_task(tracking_task);
    assert(imu_status().state == 1 && !imu_status().timestamp_us);
    tracking_ticks(52, .01, .01);
    assert(imu_status().state == 2);
    imu_tracking_vector bias = imu_vector(2);
    assert(fabs(bias.x + .01) < 1e-12 && fabs(bias.z - .01) < 1e-12);
    tracking_ticks(20, 1.01, .31);
    imu_tracking_status status = imu_status();
    assert(status.state == 2 && status.flags == 11 && !status.reserved && status.timestamp_us == now - 2000);
    assert(status.accel_age_us == 1000 && status.gyro_age_us == 2000);
    imu_tracking_vector p = imu_vector(0), v = imu_vector(1);
    assert(p.sequence == status.sequence && v.sequence == p.sequence && p.x < 0 && p.z > 0 && v.x < 0);
    assert(!controller_task.fn && !odom_task.fn && !platform.is_initialized && !motion_writes);
    now += 35001;
    assert(imu_status().fault == 3);
    tracking_ticks(1, .01, .01);
    assert(imu_status().state == 3);
    request((controller_command_t){.commandType = GET_IMU_TRACKING_VECTOR}, RESPONSE_SAMPLE_NOT_AVAILABLE);
    imu_action(2, 1, 0, RESPONSE_OK);
    assert(!imu_status().timestamp_us);
    assert(imu_acquisition_epoch(0) != first_epoch && imu_acquisition_epoch(37) == 37);
    tracking_ticks(1, 0, 1);
    assert(imu_status().fault == 1);
    imu_action(3, 1, 0, RESPONSE_OK);
    tracking_ticks(52, 0, 0);
    request((controller_command_t){.commandType = GET_IMU_TRACKING_CONFIGURATION}, RESPONSE_OK);
    imu_tracking_configuration config;
    memcpy(&config, reply + 4, sizeof(config));
    assert(config.window_us == 1000000 && config.mounting_yaw_rad == 0);
    tracking_ticks(config.window_us / 10000 + 2, 0, 0);
    assert(imu_status().state == 4 && imu_status().fault == 7);
    assert(imu_status().elapsed_us == config.window_us);
    now += 100000;
    assert(imu_status().elapsed_us == config.window_us);
    verify_source_window(10000, false);
    verify_source_window(15000, false);
    verify_source_window(15000, true);
    imu_action(2, 1, 0, RESPONSE_OK);
    tracking_ticks(52, 0, 0);
    --now;
    assert(imu_status().fault == IMU_TRACKING_FAULT_TIMESTAMP);
    ++now;
    imu_action(2, 1, 0, RESPONSE_OK);
    request((controller_command_t){.commandType = SET_BOARD_IDLE, .properties.set_board_idle = {1}},
            RESPONSE_OK);
    assert(imu_status().fault == 8);
    imu_action(2, 1, 0, RESPONSE_BOARD_IDLE);
    imu_action(1, 0, 0, RESPONSE_OK);
    request((controller_command_t){.commandType = SET_BOARD_IDLE}, RESPONSE_OK);
    assert(imu_status().state == 0);
    // Raw diagnostics and independent tracking operations never change an existing motor command.
    request((controller_command_t){.commandType = INITIALIZE_MOTOR, .properties.initialize_motor = {0, 0}},
            RESPONSE_OK);
    request((controller_command_t){.commandType = SET_MOTOR_SPEED, .properties.set_motor_speed = {0, 40}},
            RESPONSE_OK);
    unsigned writes = motion_writes;
    imu_action(0, 1, 0, RESPONSE_OK);
    tracking_ticks(52, 0, 0);
    assert(imu_status().state == 2);
    injected_status = HW_IMU_IO_ERROR;
    now += 1000;
    run_task(tracking_task);
    assert(imu_status().fault == 2);
    imu_action(1, 0, 0, RESPONSE_OK);
    assert(output[0] == 40 && motion_writes == writes && !fram_writes);
    puts("PASS IMU-only wire lifecycle, no platform, source stamps, power inhibit, expiry and independent live "
         "motor");
}
