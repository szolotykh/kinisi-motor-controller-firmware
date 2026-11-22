#include "fusion_estimator.h"
#include <assert.h>
#include <stdio.h>

static void calibrate(fusion_estimator_t *s, unsigned family)
{
    fusion_estimator_init(s); s->family=family; s->wheel_tau=0; // Isolate exact propagation math.
    double wheel[3]={0};
    for(uint64_t t=1000000;t<=1600000;t+=20000) {
        fusion_imu_t imu={t-10000,t-10000,.02,-.01,9.80665,.003,true};
        (void)fusion_estimator_step(s,t,wheel,&imu);
    }
    assert(s->calibrated && s->status==FUSION_EST_OK);
    assert(fabs(s->bias[0]-.02)<1e-12 && fabs(s->bias[1]+.01)<1e-12);
    assert(fabs(s->bias[2]-.003)<1e-12 && hypot(s->x,s->y)<1e-12);
}
int main(void)
{
    for(unsigned family=0;family<2;++family) {
        fusion_estimator_t s; calibrate(&s,family);
        double wheel[3]={0};
        fusion_imu_t imu={1610000,1610000,.02,-.01,9.80665,.003,true};
        assert(fusion_estimator_step(&s,1620000,wheel,&imu));
        assert(!fusion_estimator_step(&s,1620000,wheel,&imu) && s.status==FUSION_EST_TIME);
        assert(!fusion_estimator_step(&s,1610000,wheel,&imu));
        assert(fusion_estimator_step(&s,1640000,wheel,&imu)); // held but not expired
        assert(!fusion_estimator_step(&s,1660000,wheel,&imu) && s.status==FUSION_EST_IMU);
        imu.accel_us=imu.gyro_us=1680001;
        assert(!fusion_estimator_step(&s,1680000,wheel,&imu)); // future
        imu.accel_us=imu.gyro_us=1690000; imu.gz=NAN;
        assert(!fusion_estimator_step(&s,1700000,wheel,&imu));
        imu.gz=.003; imu.accel_us=1730000; imu.gyro_us=1700000;
        assert(!fusion_estimator_step(&s,1740000,wheel,&imu)); // skew
        assert(s.sample_us==1640000); // a failure cannot refresh the published timestamp
        calibrate(&s,family);
        // Straight constant acceleration, no slip: actual measured acceleration
        // and wheel velocity agree. There is no command/reference API.
        for(uint64_t t=1620000;t<=2600000;t+=20000) {
            wheel[0]=(t-1600000)*1e-6;
            imu=(fusion_imu_t){t-10000,t-10000,1.02,-.01,9.80665,.003,true};
            assert(fusion_estimator_step(&s,t,wheel,&imu));
        }
        assert(fabs(s.vx-1)<1e-9 && fabs(s.x-.51)<1e-9 && fabs(s.y)<1e-9);
        // Rotation at fixed body-forward speed: +ay=omega*vx cancels rotating axes.
        for(uint64_t t=2620000;t<=3600000;t+=20000) {
            wheel[0]=1; wheel[2]=1;
            imu=(fusion_imu_t){t-10000,t-10000,.02,.99,9.80665,1.003,true};
            assert(fusion_estimator_step(&s,t,wheel,&imu));
        }
        assert(fabs(s.yaw-1)<1e-9 && fabs(s.vx-1)<.02 && fabs(s.vy)<.02);
        fusion_estimator_init(&s);
        assert(!s.calibrated && !s.sample_us && s.x==0 && s.yaw==0);
        wheel[0]=.1;
        for(uint64_t t=1000000;t<=2000000;t+=20000) {
            imu=(fusion_imu_t){t-10000,t-10000,0,0,9.80665,0,true};
            assert(!fusion_estimator_step(&s,t,wheel,&imu));
        }
        assert(!s.calibrated && s.status==FUSION_EST_MOTION);
    }
    fusion_estimator_t gated, baseline;
    calibrate(&gated,0); baseline=gated; baseline.innovation_gate=false;
    double wheel[3]={.1,0,0};
    for(uint64_t t=1620000;t<=2000000;t+=20000) {
        fusion_imu_t imu={t-10000,t-10000,.02,-.01,9.80665,.303,true};
        assert(fusion_estimator_step(&gated,t,wheel,&imu));
        assert(fusion_estimator_step(&baseline,t,wheel,&imu));
    }
    assert(hypot(gated.vx,gated.vy)<hypot(baseline.vx,baseline.vy));
    wheel[0]=0;
    for(uint64_t t=2020000;t<=6000000;t+=20000) {
        fusion_imu_t imu={t-10000,t-10000,.02,-.01,9.80665,.003,true};
        assert(fusion_estimator_step(&gated,t,wheel,&imu));
    }
    assert(hypot(gated.vx,gated.vy)<.0001); // Consistent samples restore correction.
    fusion_estimator_init(&gated);
    fusion_imu_t held={990000,990000,.02,-.01,9.80665,.003,true};
    assert(!fusion_estimator_step(&gated,1000000,wheel,&held));
    assert(!fusion_estimator_step(&gated,1020000,wheel,&held));
    assert(gated.count==1 && !gated.calibrated); // Repeated values are not new samples.
    assert(!fusion_estimator_step(&gated,1100000,wheel,&held) && gated.status==FUSION_EST_TIME);
    printf("Estimator state size: %zu bytes\n",sizeof(fusion_estimator_t));
    puts("PASS estimator calibration, body-frame propagation, timestamps, stale/future/skew, nonfinite and reset");
    return 0;
}
