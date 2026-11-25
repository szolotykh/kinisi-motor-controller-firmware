#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t roll_cdeg, pitch_cdeg, yaw_cdeg;
    int32_t x_um, y_um, z_um;
    bool configured;
} imu_mount_config_t;

static inline bool imu_mount_valid(const imu_mount_config_t *m)
{
    if (!m) return false;
    if (!m->configured)
        return !(m->roll_cdeg || m->pitch_cdeg || m->yaw_cdeg || m->x_um || m->y_um || m->z_um);
    return m->roll_cdeg >= -18000 && m->roll_cdeg <= 18000 &&
           m->pitch_cdeg >= -9000 && m->pitch_cdeg <= 9000 &&
           m->yaw_cdeg >= -18000 && m->yaw_cdeg <= 18000 &&
           m->x_um >= -10000000 && m->x_um <= 10000000 &&
           m->y_um >= -10000000 && m->y_um <= 10000000 &&
           m->z_um >= -10000000 && m->z_um <= 10000000;
}

// Serialized configuration command and boot restore entry points.
imu_mount_config_t imu_mount_get(void);
uint8_t imu_mount_set(const imu_mount_config_t *mount);
bool imu_mount_restore(const imu_mount_config_t *mount);
