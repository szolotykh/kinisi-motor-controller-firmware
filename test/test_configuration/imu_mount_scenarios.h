// The fixture drives complete protocol frames. Separate processes represent cold boots.
static imu_mount_configuration read_mount(void)
{
    request((controller_command_t){.commandType=GET_IMU_MOUNT}, RESPONSE_OK);
    imu_mount_configuration m;
    assert(reply_len == sizeof(m) + 4);
    memcpy(&m, reply + 4, sizeof(m));
    return m;
}

static void mount_storage_case(const char *mode, int preset)
{
    const imu_mount_configuration example = {9000, -2000, 3500, 80000, -100000, 40000, 1};
    imu_mount_configuration m = read_mount();
    if (!strcmp(mode, "mount-restore")) {
        assert(memcmp(&m, &example, sizeof(m)) == 0);
        assert(!tasks_started && !motion_writes && !platform_is_odometry_enabled());
        assert(!platform_is_controller_running() && imu_status().state == IMU_TRACKING_STATE_STOPPED);
        if (preset) verify(preset);
        else assert(field(0, 0) == PLATFORM_NONE);
        return;
    }
    assert(!m.configured);
    controller_command_t set = {.commandType=SET_IMU_MOUNT,
        .properties.set_imu_mount={9000, -2000, 3500, 80000, -100000, 40000}};
    if (platform.is_initialized) {
        // A migrated installation must observe quiet counters before reconfiguration.
        request(set, RESPONSE_SOURCE_SWITCH_UNSAFE);
        sample_ticks(8, true);
    }
    request(set, RESPONSE_OK);
    set.properties.set_imu_mount.pitch_cdeg = -9001;
    request(set, RESPONSE_INVALID_ARGUMENT);
    connect();
    m = read_mount();
    assert(memcmp(&m, &example, sizeof(m)) == 0);
    if (preset && !platform.is_initialized) configure(preset);
    if (!strcmp(mode, "mount-cut")) write_budget = 32;
    request((controller_command_t){.commandType=SAVE_CONFIGURATION},
            write_budget == 32 ? RESPONSE_INTERNAL_ERROR : RESPONSE_OK);
    assert(!motion_writes);
}
