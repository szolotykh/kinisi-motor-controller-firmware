// Pure body-frame estimator arithmetic shared by firmware and sensor-only replay.
// No command, reference, plant, traction or simulator state enters this interface.
#ifndef KINISI_FUSION_ESTIMATOR_H
#define KINISI_FUSION_ESTIMATOR_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

enum { FUSION_EST_CALIBRATING = 1, FUSION_EST_OK = 0,
       FUSION_EST_TIME = 2, FUSION_EST_IMU = 3, FUSION_EST_MOTION = 4 };
typedef struct {
    uint64_t accel_us, gyro_us;
    double ax, ay, az, gz; // Chassis SI at the tracking origin, including gravity along +Z.
    bool valid;
} fusion_imu_t;
typedef struct {
    double x, y, yaw, vx, vy, wz;
    uint64_t sample_us;
    unsigned status;
    bool calibrated;
    // Internal state. Family 0 complementary; family 1 bias-aware linear KF.
    unsigned family, count;
    uint64_t previous_us, last_accel_us, last_gyro_us, calibration_start_us;
    double bias[3], sum[3], square[3], covariance[4][4];
    double last_ax, last_ay, last_gz;
    double correction_tau, wheel_variance, accel_noise, bias_walk;
    double wheel_tau, wheel_filtered[2];
    double wheel_yaw_filtered;
    bool innovation_gate;
} fusion_estimator_t;

static inline void fusion_estimator_init(fusion_estimator_t *s)
{
    memset(s, 0, sizeof(*s));
    s->status = FUSION_EST_CALIBRATING;
    s->family = 0;
    // Development-frozen complementary parameters; see release comparison.
    // The covariance alternative is exercised only by the development runner.
    s->correction_tau = 0.10;
    s->wheel_variance = 0.01;
    s->accel_noise = 0.03;
    s->bias_walk = 0.0001;
    s->wheel_tau = 0.08;
    s->innovation_gate = true;
    for (unsigned i = 0; i < 4; ++i)
        s->covariance[i][i] = i < 2 ? 0.0001 : 0.000025;
}

// Encoder twist covers (previous_us, now]. IMU timestamps are acquisition times,
// not SPI completion times. A held sample is usable for at most 35 ms; calibration
// counts only samples for which BOTH channels advanced. No auto reset on failure.
static inline bool fusion_estimator_interval(fusion_estimator_t *s, uint64_t now,
        const double wheel[3], const fusion_imu_t *imu, bool source_aligned, double wheel_age)
{
    s->status = FUSION_EST_TIME;
    if (!now || (s->previous_us && (now <= s->previous_us || now - s->previous_us > 60000))) return false;
    double dt = s->previous_us ? (now - s->previous_us) * 1e-6 : 0;
    s->previous_us = now;
    s->status = FUSION_EST_IMU;
    if (!imu || !imu->valid || !imu->accel_us || !imu->gyro_us ||
        imu->accel_us > now || imu->gyro_us > now ||
        now - imu->accel_us > 35000 || now - imu->gyro_us > 35000 ||
        (imu->accel_us > imu->gyro_us ? imu->accel_us - imu->gyro_us : imu->gyro_us - imu->accel_us) > 15000 ||
        imu->accel_us < s->last_accel_us || imu->gyro_us < s->last_gyro_us ||
        !isfinite(imu->ax) || !isfinite(imu->ay) || !isfinite(imu->az) || !isfinite(imu->gz) ||
        fabs(imu->ax) > 55 || fabs(imu->ay) > 55 || fabs(imu->az) > 55 || fabs(imu->gz) > 8.0 ||
        !isfinite(wheel[0]) || !isfinite(wheel[1]) || !isfinite(wheel[2])) return false;
    bool fresh = imu->accel_us > s->last_accel_us && imu->gyro_us > s->last_gyro_us;
    double ax=imu->ax, ay=imu->ay, gz=imu->gz;
    // Align delayed observations to the encoder interval midpoint. Only a
    // bounded, adjacent-sample slope is extrapolated; startup/repeated samples
    // use zero-order hold. This is timestamp arithmetic, not a plant model.
    if (s->calibrated && fresh && s->last_accel_us && s->last_gyro_us) {
        double accel_dt=(imu->accel_us-s->last_accel_us)*1e-6;
        double gyro_dt=(imu->gyro_us-s->last_gyro_us)*1e-6;
        double accel_ahead=fmax(0.,(now-imu->accel_us)*1e-6-dt*.5);
        double gyro_ahead=fmax(0.,(now-imu->gyro_us)*1e-6-(source_aligned ? 0. : dt*.5));
        if (accel_dt >= .005 && accel_dt <= .035) { ax+=(imu->ax-s->last_ax)*accel_ahead/accel_dt; ay+=(imu->ay-s->last_ay)*accel_ahead/accel_dt; }
        if (gyro_dt >= .005 && gyro_dt <= .035) gz+=(imu->gz-s->last_gz)*gyro_ahead/gyro_dt;
    }
    s->last_ax=imu->ax; s->last_ay=imu->ay; s->last_gz=imu->gz;
    s->last_accel_us = imu->accel_us;
    s->last_gyro_us = imu->gyro_us;
    if (!s->calibrated) {
        s->status = FUSION_EST_CALIBRATING;
        bool stationary = hypot(wheel[0], wheel[1]) < 0.008 && fabs(wheel[2]) < 0.02 &&
            hypot(imu->ax, imu->ay) < 0.3 && fabs(imu->gz) < 0.05 && fabs(imu->az - 9.80665) < 0.3;
        if (!stationary) {
            s->count = 0; s->calibration_start_us = 0;
            memset(s->sum, 0, sizeof(s->sum)); memset(s->square, 0, sizeof(s->square));
            s->status = FUSION_EST_MOTION;
            return false;
        }
        if (fresh) {
            double values[3] = {imu->ax, imu->ay, imu->gz};
            if (!s->count) s->calibration_start_us = imu->accel_us;
            ++s->count;
            for (unsigned i = 0; i < 3; ++i) { s->sum[i] += values[i]; s->square[i] += values[i]*values[i]; }
            if (s->count >= 25 && imu->accel_us - s->calibration_start_us >= 480000) {
                bool stable = true;
                for (unsigned i = 0; i < 3; ++i) {
                    s->bias[i] = s->sum[i] / s->count;
                    double variance = s->square[i] / s->count - s->bias[i]*s->bias[i];
                    if (variance > (i == 2 ? 0.0001 : 0.01)) stable = false;
                }
                s->calibrated = stable;
            }
        }
        if (!s->calibrated) return false;
    }
    if (!dt) return false;
    double measured[2];
    for (unsigned i=0;i<2;++i) {
        s->wheel_filtered[i] += dt/(s->wheel_tau+dt)*(wheel[i]-s->wheel_filtered[i]);
        measured[i]=s->wheel_filtered[i];
    }
    s->wheel_yaw_filtered += dt/(s->wheel_tau+dt)*(wheel[2]-s->wheel_yaw_filtered);
    double old_vx = s->vx, old_vy = s->vy;
    double previous_wz = s->wz;
    s->wz = gz - s->bias[2];
    double interval_wz = source_aligned ? .5*(previous_wz+s->wz) : s->wz;
    double angle = interval_wz * dt, c = cos(angle), sn = sin(angle);
    if (source_aligned) {
        // The shaft observation is an interval mean. Carry its midpoint
        // estimate to the source watermark before forming an innovation.
        // The complementary innovation below already rejects wheel noise.
        double wc=cos(s->wz*wheel_age), ws=sin(s->wz*wheel_age);
        measured[0]=wc*wheel[0]+ws*wheel[1]+(ax-s->bias[0])*wheel_age;
        measured[1]=-ws*wheel[0]+wc*wheel[1]+(ay-s->bias[1])*wheel_age;
    }
    // d(v_body)/dt = a_body - omega cross v_body. Rotate old body velocity
    // into the new body axes, then integrate the measured specific force.
    s->vx = c*old_vx + sn*old_vy + (ax - s->bias[0])*dt;
    s->vy = -sn*old_vx + c*old_vy + (ay - s->bias[1])*dt;
    if (s->family == 0) {
        double gain = dt / (s->correction_tau + dt);
        if (s->innovation_gate) {
            // Continuous disagreement weighting, frozen on development replay.
            // No motion reference, slip labels or preset-specific parameters.
            double dv=hypot(measured[0]-s->vx,measured[1]-s->vy)/0.03;
            double dw=(s->wheel_yaw_filtered-s->wz)/0.10;
            gain *= fmax(0.05,1.0/(1.0+dv*dv+dw*dw));
        }
        s->vx += gain*(measured[0] - s->vx);
        s->vy += gain*(measured[1] - s->vy);
    } else {
        // State [vx, vy, bax, bay]. Linear time-varying Kalman prediction;
        // gyro-calibrated rotation is an input, wheel twist the observation.
        double f[4][4] = {{c,sn,-dt,0},{-sn,c,0,-dt},{0,0,1,0},{0,0,0,1}};
        double fp[4][4] = {{0}}, p[4][4] = {{0}};
        for (unsigned i=0;i<4;++i) for(unsigned j=0;j<4;++j) for(unsigned k=0;k<4;++k)
            fp[i][j] += f[i][k]*s->covariance[k][j];
        for (unsigned i=0;i<4;++i) for(unsigned j=0;j<4;++j) for(unsigned k=0;k<4;++k)
            p[i][j] += fp[i][k]*f[j][k];
        for (unsigned i=0;i<4;++i) p[i][i] += i<2 ? s->accel_noise*s->accel_noise*dt*dt : s->bias_walk*s->bias_walk*dt;
        double state[4] = {s->vx,s->vy,s->bias[0],s->bias[1]};
        for (unsigned axis=0;axis<2;++axis) {
            double innovation = measured[axis]-state[axis];
            double denom = p[axis][axis]+s->wheel_variance, gain[4], row[4];
            for (unsigned i=0;i<4;++i) { gain[i]=p[i][axis]/denom; row[i]=p[axis][i]; }
            for (unsigned i=0;i<4;++i) {
                state[i] += gain[i]*innovation;
                for (unsigned j=0;j<4;++j) p[i][j] -= gain[i]*row[j];
            }
        }
        s->vx=state[0]; s->vy=state[1]; s->bias[0]=state[2]; s->bias[1]=state[3];
        memcpy(s->covariance,p,sizeof(p));
    }
    if (source_aligned) {
        // Integrate adjacent endpoint velocities in the same world frame.
        double c0=cos(s->yaw), s0=sin(s->yaw);
        double c1=c0*c-s0*sn, s1=s0*c+c0*sn;
        s->x+=.5*dt*(old_vx*c0-old_vy*s0+s->vx*c1-s->vy*s1);
        s->y+=.5*dt*(old_vx*s0+old_vy*c0+s->vx*s1+s->vy*c1);
    } else {
        double heading = s->yaw + angle*0.5;
        s->x += (s->vx*cos(heading)-s->vy*sin(heading))*dt;
        s->y += (s->vx*sin(heading)+s->vy*cos(heading))*dt;
    }
    s->yaw += angle;
    if (!isfinite(s->x) || !isfinite(s->y) || !isfinite(s->yaw) ||
        !isfinite(s->vx) || !isfinite(s->vy) || !isfinite(s->wz)) {
        s->status = FUSION_EST_IMU;
        return false;
    }
    s->sample_us = now;
    s->status = FUSION_EST_OK;
    return true;
}

static inline bool fusion_estimator_step(fusion_estimator_t *s, uint64_t now,
                                         const double wheel[3], const fusion_imu_t *imu)
{
    return fusion_estimator_interval(s,now,wheel,imu,false,0.);
}
#endif
