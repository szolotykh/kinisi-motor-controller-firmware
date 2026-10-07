#include "platform.h"
#include "platform_common.h"
#include "platform_position.h"
#include "encoder_odometry.h"
#include "protocol.h"
#include "semphr.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static unsigned locked, pwm_writes, reset_count;
static uint8_t running;
static uint64_t now = 100000, sample = 100000;
static uint8_t sample_error;
static platform_odometry_t pose;
static double wheel_speed[4];

static position_settings_t linear = {2, 0.5, 0.01, 0, 0, 0}, angular = {3, 1, 0.02, 0, 0, 0};

SemaphoreHandle_t xSemaphoreCreateMutex(void) { return &locked; }
int xSemaphoreTake(SemaphoreHandle_t m, uint32_t timeout)
{ (void)timeout; assert(m == &locked && !locked); locked = 1; return 1; }
void xSemaphoreGive(SemaphoreHandle_t m) { assert(m == &locked && locked); locked = 0; }
uint64_t hw_clock_microseconds(void) { return now; }
uint32_t odometry_manager_get_period_ms(void) { return 20; }
void odometry_manager_initialize(void) {}
void odometry_manager_invalidate_platform_sample(void) { sample_error = RESPONSE_SAMPLE_NOT_AVAILABLE; }
uint8_t odometry_manager_get_platform_sample(platform_odometry_t *p, uint64_t *s)
{ *p = pose; *s = sample; return sample_error; }
platform_odometry_t odometry_manager_get_platform_odometry(void) { return pose; }
void odometry_manager_reset_platform_odometry(void)
{ pose = (platform_odometry_t){0}; reset_count++; sample_error = RESPONSE_SAMPLE_NOT_AVAILABLE; }
uint8_t controllers_manager_is_running(uint8_t i) { return (running >> i) & 1; }
void controllers_manager_initialize_controller_multiple(uint8_t mask, double kp, double ki, double kd, double limit)
{ (void)kp; (void)ki; (void)kd; (void)limit; running = mask; }
void controllers_manager_stop_controller_multiple(uint8_t mask) { running &= ~mask; }
void controllers_manager_brake_multiple(uint8_t mask) { running &= ~mask; }
void controllers_manager_set_target_speed_multiple(uint8_t *indexes, double *speeds, uint8_t count)
{ for (uint8_t i = 0; i < count; ++i) wheel_speed[indexes[i]] = speeds[i]; }
static void init_motor(motorIndex i, bool r) { (void)i; (void)r; }
static void pwm(motorIndex i, double v) { (void)i; (void)v; pwm_writes++; }
static void init_encoder(encoder_index_t i, double r, uint8_t reverse) { (void)i; (void)r; (void)reverse; }
static uint8_t encoder_ready(encoder_index_t i) { (void)i; return 1; }
static double encoder_resolution(encoder_index_t i) { (void)i; return 100; }
static void start_encoder(uint8_t i) { (void)i; }
const hw_motor_interface_t *get_motor_interface(void)
{ static const hw_motor_interface_t h = {.initialize = init_motor, .set_speed = pwm}; return &h; }
const hw_encoder_interface_t *get_encoder_interface(void)
{ static const hw_encoder_interface_t h = {.initialize = init_encoder, .is_initialized = encoder_ready, .get_resolution = encoder_resolution}; return &h; }
const encoder_odometry_interface_t *get_encoder_odometry_interface(void)
{ static const encoder_odometry_interface_t h = {.start = start_encoder}; return &h; }

static void fresh(void) { sample_error = RESPONSE_OK; sample = now; }
static void near(double actual, double expected) { assert(fabs(actual - expected) < 1e-8); }

static void convergence(bool differential)
{
    platform_odometry_t current = {0.4, -0.2, 2.8}, goal = {-0.5, 0.8, -2.9};
    position_platform_state_t state = {0};
    position_settings_t l = linear, a = angular;
    l.ki = 0.2; l.kd = 0.1; l.integral_limit = 0.2;
    a.ki = 0.3; a.kd = 0.1; a.integral_limit = 0.4;
    for (unsigned i = 0; i < 4000; ++i) {
        platform_velocity_t v = position_platform_pid_velocity(l, a, &state, current, goal, differential, 0.02);
        assert(hypot(v.x, v.y) <= linear.max_speed + 1e-10);
        assert(fabs(v.t) <= angular.max_speed);
        if (differential) near(v.y, 0);
        current.x += 0.02 * (cos(current.t) * v.x - sin(current.t) * v.y);
        current.y += 0.02 * (sin(current.t) * v.x + cos(current.t) * v.y);
        current.t += 0.02 * v.t;
    }
    assert(hypot(goal.x - current.x, goal.y - current.y) <= linear.tolerance);
    assert(fabs(remainder(goal.t - current.t, 2 * M_PI)) <= angular.tolerance);
}

static void pid_math(void)
{
    position_settings_t s = {.kp=1, .max_speed=10, .ki=0.5, .integral_limit=0.2};
    position_pid_state_t state = {0};
    near(position_pid_velocity(s, &state, 1, 0.1, false), 1.05);
    for (int i=0; i<20; i++) position_pid_velocity(s, &state, 1, 0.1, false);
    near(state.integral, 0.2);
    near(position_pid_velocity(s, &state, 1, 0.1, false), 1.2);
    position_pid_reset(&state);
    s.max_speed=1;
    for (int i=0; i<100; i++) near(position_pid_velocity(s, &state, 10, 0.1, false), 1);
    near(state.integral, 0); // No windup while speed-limited.
    s.max_speed=100; s.kp=2; s.ki=0; s.kd=0.5;
    position_pid_reset(&state);
    near(position_pid_velocity(s, &state, 1, 0.1, false), 2); // No first-sample derivative kick.
    near(position_pid_velocity(s, &state, 0.8, 0.1, false), 1.6 - 0.5/0.6);
    position_pid_reset(&state);
    position_pid_velocity(s, &state, M_PI-0.01, 0.1, true);
    position_pid_velocity(s, &state, -M_PI+0.01, 0.1, true);
    near(state.derivative, 0.02/0.12); // Wrapped angular delta, not a 2-pi impulse.
    s.tolerance=0.02;
    near(position_pid_velocity(s, &state, 0.01, 0.1, false), 0);
    assert(!state.has_previous && state.integral == 0 && state.derivative == 0);
    near(position_pid_velocity(s, &state, NAN, 0.1, false), 0);
    near(position_pid_velocity(s, &state, 1, 0, false), 2);
    assert(!state.has_previous);
    s.ki=-1; assert(!position_settings_valid(s));
    s.ki=0; s.kd=NAN; assert(!position_settings_valid(s));
    s.kd=0; s.integral_limit=-1; assert(!position_settings_valid(s));
    // Independent X/Y integral histories, vector speed cap and anti-windup.
    position_platform_state_t ps = {0};
    position_settings_t l = {.kp=1,.max_speed=0.5,.ki=1,.integral_limit=0.2};
    platform_velocity_t v = position_platform_pid_velocity(l, angular, &ps,
        (platform_odometry_t){0}, (platform_odometry_t){10, 1, 0}, false, 0.1);
    near(hypot(v.x,v.y), 0.5);
    near(ps.x.integral,0); near(ps.y.integral,0);
    ps = (position_platform_state_t){0}; l.max_speed=10;
    v = position_platform_pid_velocity(l, angular, &ps,
        (platform_odometry_t){0}, (platform_odometry_t){0.1, -0.2, 0}, false, 0.1);
    near(ps.x.integral,0.01); near(ps.y.integral,-0.02);
    near(v.x,0.11); near(v.y,-0.22);
}

int main(void)
{
    pid_math();
    convergence(false);
    convergence(true);
    assert(!position_settings_valid((position_settings_t){0, 1, 0, 0, 0, 0}));
    assert(!position_settings_valid((position_settings_t){1, INFINITY, 0, 0, 0, 0}));
    assert(!position_settings_valid((position_settings_t){1, 1, -1, 0, 0, 0}));
    near(position_velocity(linear, 0.005), 0);
    near(position_velocity(linear, -100), -0.5);
    platform_velocity_t v = position_platform_velocity(linear, angular,
        (platform_odometry_t){0, 0, M_PI/2}, (platform_odometry_t){1, 0, M_PI/2}, false);
    near(v.x, 0); near(v.y, -0.5); near(v.t, 0);
    v = position_platform_velocity(linear, angular,
        (platform_odometry_t){0, 0, M_PI-0.05}, (platform_odometry_t){0, 0, -M_PI+0.05}, false);
    near(v.t, 0.3);
    v = position_platform_velocity(linear, angular, (platform_odometry_t){0},
        (platform_odometry_t){1, 1, 0}, false);
    near(hypot(v.x, v.y), 0.5);
    v = position_platform_velocity(linear, angular, (platform_odometry_t){0},
        (platform_odometry_t){-1, 0, 0}, true);
    near(v.x, 0); near(v.y, 0); assert(fabs(v.t) == 1);

    assert(platform_position_initialize(linear, angular) == RESPONSE_PLATFORM_NOT_INITIALIZED);
    initialize_differential_platform(0, 0, 0, 0, 0.1, 0.3, 100);
    assert(platform_position_initialize(linear, angular) == RESPONSE_CONTROLLER_NOT_INITIALIZED);
    platform_start_velocity_controller((plaform_controller_settings_t){1, 0, 0, 10});
    assert(platform_position_initialize(linear, angular) == RESPONSE_OK);
    assert(platform_position_set((platform_odometry_t){1, 0, 0}) == RESPONSE_SAMPLE_NOT_AVAILABLE);
    fresh();
    assert(platform_position_set((platform_odometry_t){1, 0, 0}) == RESPONSE_OK);
    platform_position_update();
    // 0.5 m/s divided by 0.05 m radius: wheel commands are rad/s, never raw PWM.
    near(wheel_speed[0], 10); near(wheel_speed[1], 10); assert(!pwm_writes);
    pose.x = 1;
    platform_position_update(); near(wheel_speed[0], 0); near(wheel_speed[1], 0);
    assert(platform_position_set((platform_odometry_t){1, 0, 0.2}) == RESPONSE_OK);
    platform_position_update(); near(wheel_speed[0], -1.8); near(wheel_speed[1], 1.8);
    // Stale feedback cancels motion; fresh feedback alone cannot restart it.
    now += 60001;
    platform_position_update(); near(wheel_speed[0], 0);
    fresh(); platform_position_update(); near(wheel_speed[0], 0);
    assert(platform_position_set((platform_odometry_t){2, 0, 0}) == RESPONSE_OK);
    platform_position_update(); near(wheel_speed[0], 10);
    assert(platform_position_reset() == RESPONSE_OK && reset_count == 1);
    near(wheel_speed[0], 0); near(pose.x, 0);
    assert(platform_position_set((platform_odometry_t){1, 0, 0}) == RESPONSE_SAMPLE_NOT_AVAILABLE);
    fresh();
    assert(platform_position_set((platform_odometry_t){1, 0, 0}) == RESPONSE_OK);
    platform_brake(); platform_position_update(); assert(!running);
    assert(platform_position_set((platform_odometry_t){1, 0, 0}) == RESPONSE_CONTROLLER_NOT_INITIALIZED);
    platform_start_velocity_controller((plaform_controller_settings_t){1, 0, 0, 10});
    assert(platform_position_initialize(linear, angular) == RESPONSE_OK);
    fresh();
    assert(platform_position_set((platform_odometry_t){1, 0, 0}) == RESPONSE_OK);
    platform_set_target_velocity((platform_velocity_t){0.1, 0, 0});
    platform_position_update(); near(wheel_speed[0], 2);
    assert(platform_position_set((platform_odometry_t){1, 0, 0}) == RESPONSE_CONTROLLER_NOT_INITIALIZED);

    // Holonomic bases use rotated world-frame error and preserve odometry on init.
    initialize_omni_platform(0, 0, 0, 0, 0, 0, 0.1, 0.2, 100);
    platform_start_velocity_controller((plaform_controller_settings_t){1, 0, 0, 10});
    pose = (platform_odometry_t){2, 3, M_PI/2};
    assert(platform_position_initialize(linear, angular) == RESPONSE_OK);
    near(pose.x, 2); near(pose.y, 3);
    fresh();
    assert(platform_position_set((platform_odometry_t){3, 3, M_PI/2}) == RESPONSE_OK);
    platform_position_update(); near(wheel_speed[0], -5); near(wheel_speed[1], -5); near(wheel_speed[2], 10);
    sample_error = RESPONSE_ODOMETRY_NOT_INITIALIZED;
    platform_position_update(); near(wheel_speed[0], 0); near(wheel_speed[1], 0); near(wheel_speed[2], 0);
    fresh(); platform_position_update(); near(wheel_speed[2], 0);
    platform_stop_odometry();
    assert(platform_position_set((platform_odometry_t){0}) == RESPONSE_CONTROLLER_NOT_INITIALIZED);
    initialize_mecanum_platform(0, 0, 0, 0, 0, 0, 0, 0, 0.4, 0.3, 0.1, 100);
    platform_start_velocity_controller((plaform_controller_settings_t){1, 0, 0, 10});
    assert(platform_position_initialize(linear, angular) == RESPONSE_OK);
    fresh(); pose = (platform_odometry_t){0};
    assert(platform_position_set((platform_odometry_t){1, 0, 0}) == RESPONSE_OK);
    platform_position_update();
    for (unsigned i = 0; i < 4; ++i) near(wheel_speed[i], 10);
    platform_coast(); platform_position_update(); assert(!running);
    assert(platform_position_set((platform_odometry_t){0}) == RESPONSE_CONTROLLER_NOT_INITIALIZED);
    // Real platform dispatch accumulates I using elapsed time and clears it on retarget/reset.
    initialize_differential_platform(0, 0, 0, 0, 0.1, 0.3, 100);
    platform_start_velocity_controller((plaform_controller_settings_t){1, 0, 0, 10});
    position_settings_t full = {.kp=1,.max_speed=10,.tolerance=0.001,.ki=1,.kd=0.1,.integral_limit=1};
    pose = (platform_odometry_t){0}; fresh();
    assert(platform_position_initialize(full, full) == RESPONSE_OK);
    assert(platform_position_set((platform_odometry_t){0.1, 0, 0}) == RESPONSE_OK);
    platform_position_update(); near(wheel_speed[0], 2);
    now += 20000; fresh(); platform_position_update(); near(wheel_speed[0], 2.04);
    assert(platform_position_set((platform_odometry_t){0.2, 0, 0}) == RESPONSE_OK);
    platform_position_update(); near(wheel_speed[0], 4);
    assert(platform_position_reset() == RESPONSE_OK);
    near(wheel_speed[0], 0); fresh();
    assert(platform_position_set((platform_odometry_t){0.2, 0, 0}) == RESPONSE_OK);
    platform_position_update(); near(wheel_speed[0], 4);
    puts("Platform position frames, limits, differential kinematics, prerequisites, stale feedback, reset and overrides passed");
}
