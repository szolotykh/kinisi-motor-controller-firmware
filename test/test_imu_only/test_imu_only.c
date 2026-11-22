#include "imu_only_estimator.h"
#include <assert.h>
#include <stdio.h>

static void feed(imu_only_state_t *s, uint64_t now, uint64_t a, uint64_t g, double ax, double ay, double gz)
{
    imu_only_sample_t m = {a, g, ax, ay, 9.80665, 0, 0, gz};
    imu_only_step(s, now, &m);
}

static void zero_origin(imu_only_state_t *s, double yaw)
{
    assert(imu_only_start(s, 0, yaw, true));
    for (uint64_t t = 10000; t <= 510000; t += 10000)
        feed(s, t, t, t, .01, -.02, .005);
    assert(s->health == IMU_ONLY_TRACKING && s->sample_us == 510000);
    assert(s->x == 0 && s->y == 0 && s->yaw == 0 && s->vx == 0 && s->vy == 0);
}

static void kinematics(void)
{
    const double pi = acos(-1);
    imu_only_state_t s = {0};
    zero_origin(&s, pi / 2);
    assert(fabs(s.bias_ax - .02) < 1e-12 && fabs(s.bias_ay - .01) < 1e-12 && fabs(s.bias_gz - .005) < 1e-12);
    // 1 m/s² for exactly one second in sensor +X, then one second of coasting.
    for (uint64_t t = 520000; t <= 1520000; t += 10000)
        feed(&s, t, t, t, (t < 1520000 ? 1. : 0.) + .01, -.02, .005);
    assert(fabs(s.vx) < 1e-12 && fabs(s.vy - 1) < 1e-12 && fabs(s.y - .5) < 1e-12);
    for (uint64_t t = 1530000; t <= 2520000; t += 10000)
        feed(&s, t, t, t, .01, -.02, .005);
    assert(fabs(s.y - 1.5) < 1e-12 && fabs(s.vy - 1) < 1e-12); // never motion-aided zeroing
    // More than one complete rotation; no wrapping of public yaw.
    for (uint64_t t = 2530000; t <= 3530000; t += 10000)
        feed(&s, t, t, t, .01, -.02, 7.005);
    assert(fabs(s.yaw - 7) < 1e-11 && s.wz == 7 && s.health == IMU_ONLY_TRACKING);
}

static void asynchronous(void)
{
    imu_only_state_t s = {0};
    assert(imu_only_start(&s, 0, 0, true));
    for (uint64_t t = 10000; t <= 510000; t += 10000)
        feed(&s, t, t, t - 2000, 0, 0, 0);
    assert(s.health == IMU_ONLY_CALIBRATING);
    feed(&s, 520000, 520000, 518000, 0, 0, 0);
    assert(s.health == IMU_ONLY_TRACKING && s.sample_us == 518000 && s.origin_us == 510000);
    zero_origin(&s, 0);
    feed(&s, 520000, 520000, 510000, 1.01, -.02, .005);
    assert(s.sample_us == 510000 && s.x == 0);
    uint32_t seq = s.sequence;
    feed(&s, 522000, 520000, 510000, 1.01, -.02, .005);
    assert(s.sequence == seq);
    feed(&s, 525000, 520000, 525000, 1.01, -.02, .005);
    assert(s.sample_us == 520000 && s.vx == 0);
    feed(&s, 530000, 530000, 525000, 2.01, -.02, .005);
    assert(s.sample_us == 525000 && fabs(s.vx - .005) < 1e-12);
    feed(&s, 535000, 530000, 535000, 2.01, -.02, .005);
    assert(s.sample_us == 530000 && fabs(s.vx - .010) < 1e-12);
    feed(&s, 545000, 545000, 545000, 2.01, -.02, .005);
    assert(fabs(s.vx - .040) < 1e-12);
}

static void lifecycle(void)
{
    imu_only_state_t s = {0};
    assert(!imu_only_start(&s, 0, NAN, true) && !imu_only_start(&s, 0, 0, false));
    assert(imu_only_start(&s, 0, 0, true));
    imu_only_expire(&s, 100000);
    assert(s.health == IMU_ONLY_CALIBRATING);
    imu_only_expire(&s, 100001);
    assert(s.fault == IMU_ONLY_UNAVAILABLE);
    zero_origin(&s, 0);
    imu_only_expire(&s, s.accel_us + 35000);
    assert(s.health == IMU_ONLY_TRACKING);
    feed(&s, 550000, 550000, 550000, .01, -.02, .005); // late pair cannot resurrect pose
    assert(s.fault == IMU_ONLY_STALE);
    assert(imu_only_start(&s, 600000, 0, true));
    feed(&s, 605000, 599000, 599000, 1, 0, 0);
    assert(!s.accel_count && s.health == IMU_ONLY_CALIBRATING);
    feed(&s, 610000, 610000, 610000, 1, 0, 0);
    assert(s.fault == IMU_ONLY_NOT_STATIONARY);
    zero_origin(&s, 0);
    feed(&s, 520000, 520001, 520000, 0, 0, 0);
    assert(s.fault == IMU_ONLY_TIMESTAMP);
    zero_origin(&s, 0);
    feed(&s, 520000, 509999, 520000, 0, 0, 0);
    assert(s.fault == IMU_ONLY_TIMESTAMP);
    zero_origin(&s, 0);
    feed(&s, 514000, 510000, 514000, 0.2, -.02, .005);
    assert(s.fault == IMU_ONLY_TIMESTAMP); // changed payload at a held acceleration timestamp
    zero_origin(&s, 0);
    feed(&s, 530000, 530000, 510000, 0, 0, 0);
    assert(s.fault == IMU_ONLY_STALE);
    zero_origin(&s, 0);
    feed(&s, 520000, 520000, 520000, 55, 0, 0);
    assert(s.fault == IMU_ONLY_SATURATION);
    zero_origin(&s, 0);
    feed(&s, 520000, 520000, 520000, NAN, 0, 0);
    assert(s.fault == IMU_ONLY_UNAVAILABLE);
    zero_origin(&s, 0);
    imu_only_sample_t tilted = {520000, 520000, 0, 0, 8., 0, 0, 0};
    imu_only_step(&s, 520000, &tilted);
    assert(s.fault == IMU_ONLY_MOUNT_INVALID);
    imu_only_stop(&s);
    assert(s.health == IMU_ONLY_STOPPED && !s.sample_us && !s.bias_ax);
}

int main(void)
{
    kinematics();
    asynchronous();
    lifecycle();
    puts("PASS independent source clocks, calibrated axes, integration, coast, unwrapped yaw and fail-closed "
         "lifecycle");
}
