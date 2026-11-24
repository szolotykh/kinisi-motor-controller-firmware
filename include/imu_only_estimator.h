#pragma once
// C99 planar inertial diagnostic. Sensor input uses SI units in a level,
// centered sensor frame. No wheel, command, clock service or actuator dependencies.
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

enum { IMU_ONLY_STOPPED, IMU_ONLY_CALIBRATING, IMU_ONLY_TRACKING, IMU_ONLY_FAULT, IMU_ONLY_EXPIRED };
enum {
    IMU_ONLY_NONE,
    IMU_ONLY_NOT_STATIONARY,
    IMU_ONLY_UNAVAILABLE,
    IMU_ONLY_STALE,
    IMU_ONLY_TIMESTAMP,
    IMU_ONLY_SATURATION,
    IMU_ONLY_BIAS_NOT_READY,
    IMU_ONLY_WINDOW_EXPIRED,
    IMU_ONLY_INHIBITED,
    IMU_ONLY_MOUNT_INVALID
};
enum { IMU_ONLY_START_GRACE_US = 100000, IMU_ONLY_MAX_AGE_US = 35000, IMU_ONLY_MAX_SKEW_US = 15000 };

typedef struct {
    uint64_t accel_us, gyro_us;
    double ax, ay, az, gx, gy, gz;
} imu_only_sample_t;

// At most one event per distinct time remains ahead of the joint watermark.
// Refuse a stream exceeding the queue bound instead of dropping inertial motion.
typedef struct {
    uint64_t time;
    double x, y;
} imu_only_event_t;
typedef struct {
    imu_only_event_t pending[12];
    unsigned head, count;
    double x, y;
    uint64_t applied_us;
} imu_only_axis_t;
typedef struct {
    double x, y, yaw, vx, vy, wz, bias_ax, bias_ay, bias_gz; // velocity in origin/world axes
    uint64_t sample_us, accel_us, gyro_us, origin_us, started_us, last_now, first_accel, first_gyro;
    uint32_t sequence, accel_count, gyro_count;
    uint8_t health, fault;
    double mounting_yaw, cosine, sine;
    imu_only_axis_t accel, gyro;
    imu_only_sample_t previous;
} imu_only_state_t;

static inline bool imu_only_running(const imu_only_state_t *s)
{
    return s->health == IMU_ONLY_CALIBRATING || s->health == IMU_ONLY_TRACKING;
}

static inline void imu_only_fail(imu_only_state_t *s, uint8_t reason)
{
    if (!imu_only_running(s))
        return;
    s->health = reason == IMU_ONLY_WINDOW_EXPIRED ? IMU_ONLY_EXPIRED : IMU_ONLY_FAULT;
    s->fault = reason;
    ++s->sequence;
}

static inline void imu_only_stop(imu_only_state_t *s)
{
    uint32_t sequence = s->sequence + 1;
    *s = (imu_only_state_t){.sequence = sequence};
}

static inline bool imu_only_start(imu_only_state_t *s, uint64_t now_us, double mounting_yaw_rad,
                                  bool stationary_confirmed)
{
    if (!stationary_confirmed || !isfinite(mounting_yaw_rad) || fabs(mounting_yaw_rad) > 3.14159265358979323846)
        return false;
    imu_only_stop(s);
    s->health = IMU_ONLY_CALIBRATING;
    s->started_us = s->last_now = now_us;
    s->mounting_yaw = mounting_yaw_rad;
    s->cosine = cos(mounting_yaw_rad);
    s->sine = sin(mounting_yaw_rad);
    return true;
}

static inline void imu_only_expire(imu_only_state_t *s, uint64_t now_us)
{
    if (!imu_only_running(s))
        return;
    if (now_us < s->last_now) {
        imu_only_fail(s, IMU_ONLY_TIMESTAMP);
        return;
    }
    s->last_now = now_us;
    if (!s->accel_us || !s->gyro_us) {
        if (now_us - s->started_us > IMU_ONLY_START_GRACE_US)
            imu_only_fail(s, IMU_ONLY_UNAVAILABLE);
        return;
    }
    if (now_us - s->accel_us > IMU_ONLY_MAX_AGE_US || now_us - s->gyro_us > IMU_ONLY_MAX_AGE_US)
        imu_only_fail(s, IMU_ONLY_STALE);
    else if (s->health == IMU_ONLY_CALIBRATING && now_us - s->started_us > 2000000)
        imu_only_fail(s, IMU_ONLY_BIAS_NOT_READY);
}

static inline bool imu_only_append(imu_only_axis_t *axis, uint64_t at, double x, double y)
{
    if (axis->count == 12)
        return false;
    axis->pending[(axis->head + axis->count) % 12] = (imu_only_event_t){at, x, y};
    ++axis->count;
    return true;
}

static inline void imu_only_advance_axis(imu_only_axis_t *axis, uint64_t time)
{
    if (axis->count && axis->pending[axis->head].time == time) {
        axis->applied_us = time;
        axis->x = axis->pending[axis->head].x;
        axis->y = axis->pending[axis->head].y;
        axis->head = (axis->head + 1) % 12;
        --axis->count;
    }
}

static inline void imu_only_integrate(imu_only_state_t *s, uint64_t watermark, bool bmi088_registers)
{
    while (s->sample_us < watermark) {
        uint64_t end = watermark;
        if (s->accel.count && s->accel.pending[s->accel.head].time < end)
            end = s->accel.pending[s->accel.head].time;
        if (s->gyro.count && s->gyro.pending[s->gyro.head].time < end)
            end = s->gyro.pending[s->gyro.head].time;
        double dt = (double)(end - s->sample_us) * 1e-6;
        double turn = s->gyro.x * dt;
        double c = cos(s->yaw + turn * 0.5), sn = sin(s->yaw + turn * 0.5);
        double ax = c * s->accel.x - sn * s->accel.y, ay = sn * s->accel.x + c * s->accel.y;
        s->x += s->vx * dt + 0.5 * ax * dt * dt;
        s->y += s->vy * dt + 0.5 * ay * dt * dt;
        s->vx += ax * dt;
        s->vy += ay * dt;
        s->yaw += turn;
        s->sample_us = end;
        // The generic step interface accepts ideal held SI inputs. Register
        // reconstruction is specific to the configured v0.4 BMI088 producer.
        if (!bmi088_registers) {
            imu_only_advance_axis(&s->accel, end);
            imu_only_advance_axis(&s->gyro, end);
            continue;
        }
        if (s->accel.count && s->accel.pending[s->accel.head].time == end) {
            double old_x = s->accel.x, old_y = s->accel.y;
            // The older die can still hold a calibration value from before
            // the origin. Only time inside this integration window has area.
            uint64_t begin = s->accel.applied_us < s->origin_us ? s->origin_us : s->accel.applied_us;
            double interval = (double)(end - begin) * 1e-6;
            imu_only_advance_axis(&s->accel, end);
            // Complete the preceding measured interval by trapezoidal quadrature.
            // For a first-order 40 Hz accel pole, integral(input) also contains
            // tau * (filtered_end - filtered_start). Only closed intervals qualify.
            double weight = 0.5 * interval + 1.0 / (6.283185307179586 * 40.0);
            double dx = weight * (s->accel.x - old_x), dy = weight * (s->accel.y - old_y);
            double ce = cos(s->yaw), se = sin(s->yaw);
            s->vx += ce * dx - se * dy;
            s->vy += se * dx + ce * dy;
        }
        if (s->gyro.count && s->gyro.pending[s->gyro.head].time == end) {
            double previous = s->gyro.x;
            double interval = (double)(end - s->gyro.applied_us) * 1e-6;
            imu_only_advance_axis(&s->gyro, end);
            // Estimate the instantaneous rate at this acquisition. Suppress
            // differences within three modeled standard deviations at 100 Hz.
            double delta = s->gyro.x - previous;
            double excess = fmax(0.0, fabs(delta) - 0.00684005);
            double gain = fmin(1.0, 1.0 / (6.283185307179586 * 32.0 * interval));
            s->wz = s->gyro.x + gain * copysign(excess, delta);
        }
    }
    if (!bmi088_registers)
        s->wz = s->gyro.x;
}

// Advance to one actual gyro acquisition for which acceleration coverage is
// available. A later gyro stays queued even if it arrived in the same read.
static inline void imu_only_complete_gyro(imu_only_state_t *s)
{
    if (!s->origin_us)
        return;
    uint64_t time;
    bool first = s->health == IMU_ONLY_CALIBRATING;
    if (first && s->gyro.applied_us == s->origin_us) {
        time = s->origin_us;
    } else if (s->gyro.count) {
        time = s->gyro.pending[s->gyro.head].time;
    } else {
        return;
    }
    if (time > s->accel_us || time > s->gyro_us)
        return;
    imu_only_integrate(s, time, true);
    if (first) {
        // The public origin is the first fully covered gyro acquisition. Retain
        // integrated velocity in that frame if calibration ended between ticks.
        double c = cos(s->yaw), sn = sin(s->yaw);
        double vx = c * s->vx + sn * s->vy;
        s->vy = -sn * s->vx + c * s->vy;
        s->vx = vx;
        s->x = s->y = s->yaw = 0.;
        s->health = IMU_ONLY_TRACKING;
    }
}

static inline void imu_only_process(imu_only_state_t *s, uint64_t now_us, const imu_only_sample_t *m,
                                    bool publish_on_gyro, bool advance)
{
    imu_only_expire(s, now_us);
    if (!imu_only_running(s) || !m)
        return;
    uint64_t a = m->accel_us, g = m->gyro_us;
    if (a > now_us || g > now_us || a < s->accel_us || g < s->gyro_us) {
        imu_only_fail(s, IMU_ONLY_TIMESTAMP);
        return;
    }
    if (a <= s->started_us || g <= s->started_us)
        return; // discard pre-anchor cache
    if (now_us - a > IMU_ONLY_MAX_AGE_US || now_us - g > IMU_ONLY_MAX_AGE_US ||
        (a > g ? a - g : g - a) > IMU_ONLY_MAX_SKEW_US) {
        imu_only_fail(s, IMU_ONLY_STALE);
        return;
    }
    if (!isfinite(m->ax) || !isfinite(m->ay) || !isfinite(m->az) || !isfinite(m->gx) || !isfinite(m->gy) ||
        !isfinite(m->gz)) {
        imu_only_fail(s, IMU_ONLY_UNAVAILABLE);
        return;
    }
    if (fabs(m->ax) >= 55 || fabs(m->ay) >= 55 || fabs(m->az) >= 55 || fabs(m->gx) >= 8 || fabs(m->gy) >= 8 ||
        fabs(m->gz) >= 8) {
        imu_only_fail(s, IMU_ONLY_SATURATION);
        return;
    }
    if (fabs(m->az - 9.80665) > 1 || fabs(m->gx) > .35 || fabs(m->gy) > .35) {
        imu_only_fail(s, IMU_ONLY_MOUNT_INVALID);
        return;
    }
    bool new_a = a != s->accel_us, new_g = g != s->gyro_us;
    if ((!new_a && (m->ax != s->previous.ax || m->ay != s->previous.ay || m->az != s->previous.az)) ||
        (!new_g && (m->gx != s->previous.gx || m->gy != s->previous.gy || m->gz != s->previous.gz))) {
        imu_only_fail(s, IMU_ONLY_TIMESTAMP);
        return;
    }
    if (!new_a && !new_g) {
        if (publish_on_gyro && advance)
            imu_only_complete_gyro(s);
        return;
    }
    double ax = s->cosine * m->ax - s->sine * m->ay, ay = s->sine * m->ax + s->cosine * m->ay;
    s->accel_us = a;
    s->gyro_us = g;
    s->previous = *m;
    ++s->sequence;
    if (s->health == IMU_ONLY_CALIBRATING && !s->origin_us) {
        if (hypot(ax, ay) > .40 || fabs(m->gz) > .10 || hypot(m->gx, m->gy) > .10 ||
            (s->accel_count >= 5 && hypot(ax - s->bias_ax, ay - s->bias_ay) > .18) ||
            (s->gyro_count >= 5 && fabs(m->gz - s->bias_gz) > .035)) {
            imu_only_fail(s, IMU_ONLY_NOT_STATIONARY);
            return;
        }
        if (new_a) {
            if (!s->accel_count)
                s->first_accel = a;
            ++s->accel_count;
            s->bias_ax += (ax - s->bias_ax) / s->accel_count;
            s->bias_ay += (ay - s->bias_ay) / s->accel_count;
        }
        if (new_g) {
            if (!s->gyro_count)
                s->first_gyro = g;
            ++s->gyro_count;
            s->bias_gz += (m->gz - s->bias_gz) / s->gyro_count;
        }
        if (s->accel_count >= 25 && s->gyro_count >= 25 && a - s->first_accel >= 500000 &&
            g - s->first_gyro >= 500000) {
            // The second calibration event is available at this instant. Do not publish
            // the origin until the slower source also covers it.
            s->origin_us = s->sample_us = a > g ? a : g;
            if (a == g)
                s->health = IMU_ONLY_TRACKING;
            s->accel.x = ax - s->bias_ax;
            s->accel.y = ay - s->bias_ay;
            s->accel.applied_us = a;
            s->gyro.applied_us = g;
            s->gyro.x = s->wz = m->gz - s->bias_gz;
        }
        return;
    }
    // A slower channel may catch up to the zero-origin instant. Its observation
    // is then the causal held value for the first forward integration interval.
    if (new_a && a <= s->sample_us) {
        s->accel.applied_us = a;
        s->accel.x = ax - s->bias_ax;
        s->accel.y = ay - s->bias_ay;
    }
    if (new_g && g <= s->sample_us) {
        s->gyro.applied_us = g;
        s->gyro.x = m->gz - s->bias_gz;
        if (publish_on_gyro)
            s->wz = s->gyro.x;
    }
    if ((new_a && a > s->sample_us && !imu_only_append(&s->accel, a, ax - s->bias_ax, ay - s->bias_ay)) ||
        (new_g && g > s->sample_us && !imu_only_append(&s->gyro, g, m->gz - s->bias_gz, 0))) {
        imu_only_fail(s, IMU_ONLY_TIMESTAMP);
        return;
    }
    if (publish_on_gyro) {
        if (advance)
            imu_only_complete_gyro(s);
        return;
    }
    uint64_t watermark = a < g ? a : g;
    if (watermark >= s->origin_us) {
        s->health = IMU_ONLY_TRACKING;
        imu_only_integrate(s, watermark, false);
    }
}

static inline void imu_only_step(imu_only_state_t *s, uint64_t now_us, const imu_only_sample_t *m)
{
    imu_only_process(s, now_us, m, false, true);
}
