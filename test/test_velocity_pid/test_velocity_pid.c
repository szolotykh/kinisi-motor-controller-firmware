#include "pid_controller.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void near(double a, double b) { assert(fabs(a-b) < 1e-9); }

static void step_response(double dt)
{
    // A simple first-order simulated motor, not hardware tuning proof.
    pid_controller_t c;
    pid_controller_init(&c,dt,8,30,0.1,60);
    double speed=0;
    const double targets[]={4,30,4,-3};
    for (unsigned phase=0;phase<4;phase++) {
        for (int i=0;i<(int)(10/dt);i++) {
            double pwm=pid_controller_update(&c,speed,targets[phase]);
            assert(isfinite(pwm) && fabs(pwm)<=100 && fabs(c.integrator)<=60);
            speed += dt*(0.12*pwm-speed)/0.2;
        }
        if (phase!=1) assert(fabs(speed-targets[phase])<0.03);
    }
}

static double friction_response(double dt, double kp, double ki, double limit)
{
    // A motor requiring 12% PWM to break away and 8% to overcome running
    // friction. This exercises startup, which the frictionless model misses.
    pid_controller_t c;
    pid_controller_init(&c, dt, kp, ki, 0, limit);
    double speed = 0;
    for (unsigned phase = 0; phase < 2; ++phase) {
        double target = phase ? -2.5 : 2.5;
        for (int i = 0; i < (int)(90 / dt); ++i) {
            double pwm = pid_controller_update(&c, speed, target);
            assert(fabs(pwm) <= 100 && fabs(c.integrator) <= limit);
            double drive = fabs(pwm) > 8 ? copysign(0.12 * (fabs(pwm) - 8), pwm) : 0;
            if (fabs(speed) < 1e-6 && fabs(pwm) < 12) drive = 0;
            speed += dt * (drive - speed) / 0.2;
        }
        if (ki > 0 && limit >= 30) assert(fabs(speed - target) < 0.01);
        else near(speed, 0);
    }
    return speed;
}

int main(void)
{
    step_response(0.01); step_response(0.02); step_response(0.05);
    near(friction_response(0.1, 0.1, 0, 30), 0); // Old UI defaults stall.
    near(friction_response(0.1, 1, 0, 30), 0); // Reported Kp=1, Ki=0 stalls too.
    near(friction_response(0.1, 1, 1, 7), 0); // Too small an I limit still stalls.
    friction_response(0.01, 1, 1, 100);
    friction_response(0.1, 1, 1, 100); // New UI starting gains, in both directions.
    pid_controller_t c;
    // P is an absolute output, not an increment added on every tick.
    pid_controller_init(&c, 0.1, 2, 0, 0, 30);
    for (int i=0; i<1000; ++i) near(pid_controller_update(&c, 1, 4), 6);
    near(pid_controller_update(&c, 4, 4), 0);
    near(pid_controller_update(&c, 5, 4), -2);

    // I integrates elapsed time and is bounded in output units.
    pid_controller_init(&c, 0.1, 0, 2, 0, 0.5);
    near(pid_controller_update(&c, 0, 1), 0.2);
    near(pid_controller_update(&c, 0, 1), 0.4);
    near(pid_controller_update(&c, 0, 1), 0.5);
    for (int i=0; i<20; ++i) near(pid_controller_update(&c, 0, 1), 0.5);
    pid_controller_init(&c, 0.1, 0, 2, 0, 0);
    for (int i=0; i<20; ++i) near(pid_controller_update(&c, 0, 1), 0);

    // Positive and negative actuator saturation cannot wind up the integral.
    for (int sign=-1; sign<=1; sign+=2) {
        pid_controller_init(&c, 0.1, 20, 1, 0, 100);
        for (int i=0; i<100; ++i) near(pid_controller_update(&c, 0, sign*10), sign*100);
        near(c.integrator, 0);
    }
    // A stored integral must be allowed to unwind, even during saturation.
    pid_controller_init(&c, 0.1, 0, 10, 0, 100);
    c.integrator=100; c.differentiator=100; c.kd=1;
    c.previousSpeed=2; c.previousError=-1; c.has_previous=true;
    near(pid_controller_update(&c, 2, 1), 100);
    near(c.integrator,99);

    // Derivative on measurement: no initialization or target-step kick.
    pid_controller_init(&c, 0.01, 0, 0, 1, 30);
    near(pid_controller_update(&c, 10, 12), 0);
    near(pid_controller_update(&c, 10, 15), 0);
    near(pid_controller_update(&c, 11, 15), -50);
    near(pid_controller_update(&c, 11, 15), -25); // Decays with the same sign.
    near(pid_controller_update(&c, 11, 15), -12.5);
    c.T=0.1;
    near(pid_controller_update(&c, 11, 15), -12.5/11);
    near(pid_controller_update(&c, 11, 0), 0);
    assert(!c.has_previous && c.integrator == 0 && c.differentiator == 0);
    near(pid_controller_update(&c, 20, 25), 0); // Fresh derivative baseline.

    // Equal elapsed time at different update rates produces equal I output.
    pid_controller_t a,b;
    pid_controller_init(&a,0.01,0,1,0,100);
    pid_controller_init(&b,0.1,0,1,0,100);
    for (int i=0;i<100;i++) pid_controller_update(&a,0,1);
    for (int i=0;i<10;i++) pid_controller_update(&b,0,1);
    near(a.integrator,b.integrator); near(a.integrator,1);
    assert(!pid_controller_settings_valid(1,-1,0,30));
    assert(!pid_controller_settings_valid(1,0,NAN,30));
    assert(!pid_controller_settings_valid(1,0,0,101));
    assert(!pid_controller_settings_valid(1,0,0,-1));
    near(pid_controller_update(&a,NAN,1),0);
    a.T=0; near(pid_controller_update(&a,0,1),0);
    pid_controller_reset(&b);
    near(b.ki,1); near(b.T,0.1); near(b.max_integral,100);
    assert(!b.has_previous && b.motorPWM==0 && b.integrator==0);
    puts("Velocity PID: absolute P, timed I, contribution limits, saturation/unwind, filtered D, zero/reset and invalid inputs passed");
}
