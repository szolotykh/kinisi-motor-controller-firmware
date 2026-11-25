#include "config_store.h"

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
static board_mount_configuration read_board_mount(void)
{
    request((controller_command_t){.commandType=GET_BOARD_MOUNT}, RESPONSE_OK);
    board_mount_configuration mount;
    assert(reply_len==sizeof(mount)+4 && sizeof(mount)==19);
    memcpy(&mount,reply+4,sizeof(mount));
    return mount;
}

static void check_mount_pair(board_mount_configuration board, imu_mount_configuration chip)
{
    board_mount_configuration actual_board=read_board_mount();
    imu_mount_configuration actual_chip=read_mount();
    assert(memcmp(&actual_board,&board,sizeof(board))==0);
    assert(memcmp(&actual_chip,&chip,sizeof(chip))==0);
}

// Expectations derived independently from the Mini outline/U601, with
// extrinsic rotations applied to the three separate basis vectors.
static const struct {
    board_mount_configuration board;
    imu_mount_configuration chip;
    bool legacy_input;
} installations[]={
    {{0,0,0,0,0,0,1},{0,0,0,-15373,26953,2075,1},false},
    {{9000,0,0,0,-100000,0,1},{9000,0,0,-15373,-102075,26953,1},false},
    {{3700,-2300,11100,23000,-79000,11000,1},{3700,-2300,11100,11645,-105999,21450,1},false},
    {{0,0,0,-10000000,10000000,10000000,1},{0,0,0,-10015373,10026953,10002075,1},false},
    {{0,0,0,10015373,-10026953,-10002075,1},{0,0,0,10000000,-10000000,-10000000,1},true}
};

static void board_reference_case(const char *mode,unsigned index)
{
    assert(index<sizeof(installations)/sizeof(*installations));
    board_mount_configuration b=installations[index].board;
    imu_mount_configuration c=installations[index].chip;
    if (!strcmp(mode,"board-ref-restore")) {
        check_mount_pair(b,c);
        assert(!motion_writes && !tasks_started && !platform_is_controller_running());
        return;
    }
    if (!strcmp(mode,"board-ref-cut"))
        check_mount_pair(installations[4].board,installations[4].chip);
    else
        check_mount_pair((board_mount_configuration){0},(imu_mount_configuration){0});
    controller_command_t set;
    if (installations[index].legacy_input) {
        set=(controller_command_t){.commandType=SET_IMU_MOUNT,
            .properties.set_imu_mount={c.roll_cdeg,c.pitch_cdeg,c.yaw_cdeg,c.x_um,c.y_um,c.z_um}};
    } else {
        set=(controller_command_t){.commandType=SET_BOARD_MOUNT,
            .properties.set_board_mount={b.roll_cdeg,b.pitch_cdeg,b.yaw_cdeg,b.x_um,b.y_um,b.z_um}};
    }
    request(set,RESPONSE_OK);
    if (!installations[index].legacy_input) {
        set.properties.set_board_mount.x_um=10000001;
        request(set,RESPONSE_INVALID_ARGUMENT);
    }
    check_mount_pair(b,c);
    connect();
    check_mount_pair(b,c);
    if (!strcmp(mode,"board-ref-cut")) write_budget=32;
    request((controller_command_t){.commandType=SAVE_CONFIGURATION},
            write_budget==32?RESPONSE_INTERNAL_ERROR:RESPONSE_OK);
    if (strcmp(mode,"board-ref-cut")) {
        // Verify the versioned durable representation as well as cold readback:
        // a pair of reversible getters alone cannot prove which reference was saved.
        uint8_t record[CONFIG_STORE_MAX_PAYLOAD];
        assert(config_store_load_latest(record,sizeof(record))==91);
        assert(record[66]==(installations[index].legacy_input?3:4));
        const void *entered=installations[index].legacy_input?(const void *)&c:(const void *)&b;
        assert(memcmp(record+72,entered,19)==0);
    }
    assert(!motion_writes);
}

static void board_reference_guard(void)
{
    controller_command_t set={.commandType=SET_BOARD_MOUNT};
    request(set,RESPONSE_OK);
    configure(PLATFORM_DIFFERENTIAL);
    request((controller_command_t){.commandType=START_PLATFORM_CONTROLLER,
        .properties.start_platform_controller={3,0,0,100}},RESPONSE_OK);
    set.properties.set_board_mount.x_um=456;
    request(set,RESPONSE_SOURCE_SWITCH_UNSAFE);
    check_mount_pair(installations[0].board,installations[0].chip);
    assert(!motion_writes);
}

static void centered_board_turn(void)
{
    request((controller_command_t){.commandType=SET_BOARD_MOUNT},RESPONSE_OK);
    imu_action(IMU_TRACKING_ACTION_START,1,0,RESPONSE_OK);
    injected_status=HW_IMU_OK;
    for (int n=0;n<100;++n) {
        now+=10000; injected_accel=injected_gyro=now-1000;
        double w=n<60?0:(n<80?(n-59)*.05:1);
        double alpha=n>=60 && n<80?5:0;
        const double x=-.015373,y=.026953;
        // Flat board's BMI088 +X and +Y oppose chassis +X and +Y.
        injected=(hw_imu_sample_t){alpha*y+w*w*x,-alpha*x+w*w*y,9.80665,0,0,w};
        run_task(tracking_task);
    }
    imu_tracking_vector pose=imu_vector(0), velocity=imu_vector(1);
    assert(pose.z>.28 && pose.z<.32 && hypot(pose.x,pose.y)<1e-9);
    assert(velocity.z>.99 && hypot(velocity.x,velocity.y)<1e-9);
    assert(!motion_writes);
}
