#pragma once
#include "imu_mount.h"
#include "hw_imu.h"
#include <math.h>

typedef struct { double x, y, z; } mount_xyz_t;
typedef struct {
    double rotation[3][3];
    mount_xyz_t offset;
    bool enabled, displaced;
} imu_mount_frame_t;
typedef struct { uint64_t stamp; double yaw_rate; } mount_gyro_sample_t;
typedef struct {
    uint64_t g, a, corrected_a;
    mount_xyz_t raw_gyro, raw_accel, origin_accel;
    // Two causal rates plus up to two later 100 Hz observations inside 15 ms skew.
    mount_gyro_sample_t gyros[4]; // newest first
} imu_mount_history_t;
typedef enum {
    MOUNT_SAMPLE_OK, MOUNT_NEED_HISTORY, MOUNT_BAD_TIME, MOUNT_GAP, MOUNT_BAD_DATA,
    MOUNT_SATURATED, MOUNT_OFF_PLANE
} imu_mount_result_t;

static inline imu_mount_frame_t imu_mount_frame(const imu_mount_config_t *m)
{
    imu_mount_frame_t f = {.enabled=m->configured};
    if (!f.enabled) return f;
    const double unit = 3.14159265358979323846 / 18000.;
    const double cr=cos(m->roll_cdeg*unit), sr=sin(m->roll_cdeg*unit);
    const double cp=cos(m->pitch_cdeg*unit), sp=sin(m->pitch_cdeg*unit);
    const double cy=cos(m->yaw_cdeg*unit), sy=sin(m->yaw_cdeg*unit);
    // Rz(yaw) Ry(pitch) Rx(roll) times BMI088-to-board diag(-1,-1,1).
    f.rotation[0][0]=-cy*cp; f.rotation[0][1]=sy*cr-cy*sp*sr; f.rotation[0][2]=cy*sp*cr+sy*sr;
    f.rotation[1][0]=-sy*cp; f.rotation[1][1]=-cy*cr-sy*sp*sr; f.rotation[1][2]=sy*sp*cr-cy*sr;
    f.rotation[2][0]=sp;     f.rotation[2][1]=-cp*sr;          f.rotation[2][2]=cp*cr;
    f.offset=(mount_xyz_t){m->x_um*1e-6,m->y_um*1e-6,m->z_um*1e-6};
    f.displaced=m->x_um || m->y_um;
    return f;
}

static inline mount_xyz_t imu_mount_rotate(const imu_mount_frame_t *f, mount_xyz_t v)
{
    const double (*r)[3]=f->rotation;
    return (mount_xyz_t){r[0][0]*v.x+r[0][1]*v.y+r[0][2]*v.z,
                         r[1][0]*v.x+r[1][1]*v.y+r[1][2]*v.z,
                         r[2][0]*v.x+r[2][1]*v.y+r[2][2]*v.z};
}

static inline bool mount_same(mount_xyz_t a,mount_xyz_t b)
{
    return a.x==b.x && a.y==b.y && a.z==b.z;
}

// Return a sample in chassis axes at the tracking origin, retaining gravity (+Z)
// for the estimator's existing gravity/rest checks. Neither axis is retimestamped.
static inline imu_mount_result_t imu_mount_observe(const imu_mount_frame_t *f, imu_mount_history_t *h,
        const hw_imu_sample_t *raw, uint64_t a, uint64_t g, double yaw_bias, uint32_t max_interval_us, hw_imu_sample_t *out)
{
    mount_xyz_t acc={raw->acceleration_x,raw->acceleration_y,raw->acceleration_z};
    mount_xyz_t w={raw->angular_velocity_x,raw->angular_velocity_y,raw->angular_velocity_z};
    if (!a || !g || a<h->a || g<h->g ||
        (g==h->g && !mount_same(w,h->raw_gyro)) || (a==h->a && !mount_same(acc,h->raw_accel)))
        return MOUNT_BAD_TIME;
    if (!isfinite(acc.x)||!isfinite(acc.y)||!isfinite(acc.z)||!isfinite(w.x)||!isfinite(w.y)||!isfinite(w.z))
        return MOUNT_BAD_DATA;
    if (fabs(acc.x)>=55 || fabs(acc.y)>=55 || fabs(acc.z)>=55 || fabs(w.x)>=8 || fabs(w.y)>=8 || fabs(w.z)>=8)
        return MOUNT_SATURATED;
    if ((a>g?a-g:g-a)>15000) return MOUNT_GAP;
    if (f->displaced && ((h->g && g-h->g>max_interval_us) || (h->a && a-h->a>max_interval_us)))
        return MOUNT_GAP;
    const mount_xyz_t raw_gyro=w;
    w=imu_mount_rotate(f,w);
    if (fabs(w.x)>.35 || fabs(w.y)>.35) return MOUNT_OFF_PLANE;
    if (g!=h->g) {
        if (f->displaced) {
            for (unsigned i=3;i>0;--i) h->gyros[i]=h->gyros[i-1];
            h->gyros[0]=(mount_gyro_sample_t){g,w.z};
        }
        h->g=g;
        h->raw_gyro=raw_gyro;
    }
    // Record the raw stamp even during warmup so a duplicate or rewind cannot
    // replace an earlier observation while the lever correction awaits history.
    h->a=a;
    h->raw_accel=acc;
    if (a!=h->corrected_a) {
        mount_xyz_t origin=imu_mount_rotate(f,acc);
        if (f->displaced) {
            unsigned i=0;
            while (i<3 && h->gyros[i].stamp>a) ++i;
            if (i==3 || !h->gyros[i+1].stamp)
                return h->corrected_a || h->gyros[3].stamp ? MOUNT_GAP : MOUNT_NEED_HISTORY;
            const mount_gyro_sample_t latest=h->gyros[i], prior=h->gyros[i+1];
            if (a-latest.stamp>max_interval_us) return MOUNT_GAP;
            // Both rates were acquired by a. Project their backward slope
            // forward only to a, even when this read also delivered a newer gyro.
            const double alpha=(latest.yaw_rate-prior.yaw_rate)*1e6/(double)(latest.stamp-prior.stamp);
            const double yaw_rate=latest.yaw_rate+alpha*((a-latest.stamp)*1e-6)-yaw_bias;
            // Planar motion rotates about chassis Z. A vertical lever is
            // collinear with that axis; roll/pitch bias cannot generate XY travel.
            origin.x+=alpha*f->offset.y+yaw_rate*yaw_rate*f->offset.x;
            origin.y-=alpha*f->offset.x-yaw_rate*yaw_rate*f->offset.y;
        }
        h->corrected_a=a;
        h->origin_accel=origin;
    }
    *out=(hw_imu_sample_t){h->origin_accel.x,h->origin_accel.y,h->origin_accel.z,w.x,w.y,w.z};
    return fabs(out->acceleration_z-9.80665)>1.0 ? MOUNT_OFF_PLANE : MOUNT_SAMPLE_OK;
}
