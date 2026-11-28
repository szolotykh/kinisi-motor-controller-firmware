#include "imu_mount_math.h"
#include <assert.h>
#include <stdio.h>

// Independently undo extrinsic roll, pitch and yaw by elementary inverse
// rotations in reverse order. Generate chip-specific force in chassis axes.
static mount_xyz_t chip_axes(imu_mount_config_t m, mount_xyz_t v)
{
    const double radians = acos(-1.0) / 18000.0;
    const double y=m.yaw_cdeg*radians, p=m.pitch_cdeg*radians, r=m.roll_cdeg*radians;
    double x1=cos(y)*v.x+sin(y)*v.y, y1=-sin(y)*v.x+cos(y)*v.y;
    double x2=cos(p)*x1-sin(p)*v.z, z2=sin(p)*x1+cos(p)*v.z;
    return (mount_xyz_t){-x2,-(cos(r)*y1+sin(r)*z2),-sin(r)*y1+cos(r)*z2};
}

static double test_pose(imu_mount_config_t m)
{
    const double x=m.x_um*1e-6,y=m.y_um*1e-6;
    imu_mount_frame_t f=imu_mount_frame(&m);
    imu_mount_history_t history={0};
    double max_error=0;
    for (unsigned i=0;i<90;++i) {
        const uint64_t g=100000+i*10000, a=i%2?g-1800:g+2200;
        const double gt=(g-100000.)*1e-6,at=(a-100000.)*1e-6;
        const double alpha=1.7, rate=0.6+alpha*at;
        const double ax=.7+.05*at, ay=-.2+.1*at;
        mount_xyz_t chip=chip_axes(m,(mount_xyz_t){ax-alpha*y-rate*rate*x,
                                        ay+alpha*x-rate*rate*y,9.80665});
        // A stationary roll/pitch gyro bias must not couple chip height
        // into the inferred translation of a planar chassis.
        mount_xyz_t w=chip_axes(m,(mount_xyz_t){.06,-.017,.6+alpha*gt});
        hw_imu_sample_t raw={chip.x,chip.y,chip.z,w.x,w.y,w.z}, out;
        imu_mount_result_t status=imu_mount_observe(&f,&history,&raw,a,g,0,35000,&out);
        // At i=1 the gyro at 110 ms is newer than the accel at 108.2 ms.
        // Only one causal rate exists until the third observation.
        if (f.displaced && i<2) {assert(status==MOUNT_NEED_HISTORY);continue;}
        assert(status==MOUNT_SAMPLE_OK);
        max_error=fmax(max_error,fabs(out.acceleration_x-ax));
        max_error=fmax(max_error,fabs(out.acceleration_y-ay));
        max_error=fmax(max_error,fabs(out.acceleration_z-9.80665));
        max_error=fmax(max_error,fabs(out.angular_velocity_z-(.6+alpha*gt)));
        // A duplicate read has the identical corrected payload, including when
        // later bias estimates differ or one gyro update arrives before accel.
        hw_imu_sample_t held;
        assert(imu_mount_observe(&f,&history,&raw,a,g,.02,35000,&held)==MOUNT_SAMPLE_OK);
        assert(held.acceleration_x==out.acceleration_x && held.acceleration_y==out.acceleration_y);
    }
    assert(max_error<1e-10);
    return max_error;
}

static void causal_offset_test(void)
{
    const imu_mount_config_t mount={.y_um=-100000,.configured=true};
    const imu_mount_frame_t f=imu_mount_frame(&mount);
    imu_mount_history_t h={0};
    hw_imu_sample_t raw={.acceleration_z=9.80665},out;
    // Ramp at 2 rad/s² up to 620 ms; the accel at 630 ms must use that history.
    for (unsigned i=0;i<3;i++) {
        raw.angular_velocity_z=.4+i*.02;
        uint64_t t=600000+i*10000;
        assert(imu_mount_observe(&f,&h,&raw,t,t,0,35000,&out)==(i ? MOUNT_SAMPLE_OK : MOUNT_NEED_HISTORY));
    }
    raw.acceleration_x=-.2;
    raw.acceleration_y=-.1*.46*.46;
    for (int sign=-1;sign<=1;sign+=2) {
        imu_mount_history_t independent=h;
        raw.angular_velocity_z=sign*2.;
        assert(imu_mount_observe(&f,&independent,&raw,630000,640000,0,35000,&out)==MOUNT_SAMPLE_OK);
        assert(fabs(out.acceleration_x)<1e-12 && fabs(out.acceleration_y)<1e-12);
        assert(out.angular_velocity_z==sign*2.); // Gyro keeps its own 640 ms stamp.
    }

    // Two gyro samples can lead the accelerometer within its 15 ms skew
    // allowance. Neither may enter a correction at the older accel stamp.
    raw.acceleration_x=0;
    raw.acceleration_y=0;
    raw.angular_velocity_z=1.;
    assert(imu_mount_observe(&f,&h,&raw,620000,630000,0,35000,&out)==MOUNT_SAMPLE_OK);
    raw.acceleration_x=-.2;
    raw.acceleration_y=-.1*.45*.45;
    raw.angular_velocity_z=-1.;
    assert(imu_mount_observe(&f,&h,&raw,625000,640000,0,35000,&out)==MOUNT_SAMPLE_OK);
    assert(fabs(out.acceleration_x)<1e-12 && fabs(out.acceleration_y)<1e-12);
    hw_imu_sample_t held=out;
    assert(imu_mount_observe(&f,&h,&raw,625000,640000,.07,35000,&out)==MOUNT_SAMPLE_OK);
    assert(out.acceleration_x==held.acceleration_x && out.acceleration_y==held.acceleration_y);
    assert(imu_mount_observe(&f,&h,&raw,625001,641000,0,35000,&out)==MOUNT_GAP);

    // During warmup the timestamp and the raw vector are already known.
    // A changed duplicate or rewound sample must remain observable as a fault.
    imu_mount_history_t cold={0};
    raw=(hw_imu_sample_t){.acceleration_z=9.80665};
    assert(imu_mount_observe(&f,&cold,&raw,100000,101000,0,35000,&out)==MOUNT_NEED_HISTORY);
    assert(imu_mount_observe(&f,&cold,&raw,99999,101000,0,35000,&out)==MOUNT_BAD_TIME);
    raw.acceleration_x=1;
    assert(imu_mount_observe(&f,&cold,&raw,100000,102000,0,35000,&out)==MOUNT_BAD_TIME);
    raw.acceleration_x=0;
    raw.angular_velocity_x=1;
    assert(imu_mount_observe(&f,&cold,&raw,100001,101000,0,35000,&out)==MOUNT_BAD_TIME);
}

int main(void)
{
    causal_offset_test();
    const imu_mount_config_t cases[]={
        {0,0,0,0,0,0,true}, {0,0,0,0,-100000,0,true}, {0,0,0,0,100000,0,true},
        {9000,0,0,0,-100000,40000,true},{-9000,0,0,0,100000,40000,true},
        {0,9000,0,120000,0,0,true},{0,-9000,0,-120000,0,0,true},
        {18000,0,0,0,0,250000,true},{3500,-2700,12300,80000,-100000,40000,true}
    };
    double error=0;
    for (unsigned i=0;i<sizeof(cases)/sizeof(*cases);++i) error=fmax(error,test_pose(cases[i]));
    imu_mount_config_t c={18000,-9000,-18000,10000000,-10000000,10000000,true};
    assert(imu_mount_valid(&c)); c.z_um++; assert(!imu_mount_valid(&c)); c=(imu_mount_config_t){0};
    assert(imu_mount_valid(&c)); c.x_um=1; assert(!imu_mount_valid(&c)); assert(!imu_mount_valid(NULL));
    imu_mount_frame_t f=imu_mount_frame(&cases[1]); imu_mount_history_t h={0}; hw_imu_sample_t out;
    hw_imu_sample_t r={0,0,9.80665,0,0,0};
    assert(imu_mount_observe(&f,&h,&r,10000,10000,0,35000,&out)==MOUNT_NEED_HISTORY);
    assert(imu_mount_observe(&f,&h,&r,20000,20000,0,35000,&out)==MOUNT_SAMPLE_OK);
    r.acceleration_y=.1;assert(imu_mount_observe(&f,&h,&r,20000,20000,0,35000,&out)==MOUNT_BAD_TIME);r.acceleration_y=0;
    assert(imu_mount_observe(&f,&h,&r,19000,21000,0,35000,&out)==MOUNT_BAD_TIME);
    assert(imu_mount_observe(&f,&h,&r,60000,60000,0,35000,&out)==MOUNT_GAP);
    r.acceleration_y=56;assert(imu_mount_observe(&f,&h,&r,30000,30000,0,35000,&out)==MOUNT_SATURATED);
    r.acceleration_y=0;r.angular_velocity_x=1;
    assert(imu_mount_observe(&f,&h,&r,30000,30000,0,35000,&out)==MOUNT_OFF_PLANE);
    printf("PASS 9 mounts, staggered source clocks, chassis acceleration/gyro max error %.3g SI, history and physical-fault guards\n",error);
    return 0;
}
