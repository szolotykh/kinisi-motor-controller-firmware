#pragma once
#include "commands.h"
#include "imu_mount.h"

static inline imu_mount_configuration imu_mount_pack(imu_mount_config_t m)
{
    return (imu_mount_configuration){m.roll_cdeg, m.pitch_cdeg, m.yaw_cdeg,
                                    m.x_um, m.y_um, m.z_um, m.configured};
}

static inline imu_mount_config_t imu_mount_unpack(imu_mount_configuration m)
{
    return (imu_mount_config_t){.roll_cdeg=m.roll_cdeg, .pitch_cdeg=m.pitch_cdeg,
        .yaw_cdeg=m.yaw_cdeg, .x_um=m.x_um, .y_um=m.y_um, .z_um=m.z_um,
        .configured=m.configured != 0};
}
