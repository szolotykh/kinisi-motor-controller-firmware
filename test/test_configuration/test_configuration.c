// Production session, command, platform and storage paths; simulated devices and scheduler.
#include "commands_handler.h"
#include "connection.h"
#include "initialization.h"
#include "wake_barriers.h"
#include "controller_configuration.h"
#include "config_store.h"
#include "hw_drv8873.h"
#include "hw_fram.h"
#include "hw_gpio.h"
#include "hw_imu.h"
#include "hw_temperature.h"
#include "hw_current_sense.h"
#include "platform.h"
#include "platform_common.h"
#include "cmsis_os.h"
#include "semphr.h"
#include <assert.h>
#include <setjmp.h>
#include <math.h>
#include "platform_fusion.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t fram[8192], motor_enabled[4], reversed_motor[4], reversed_encoder[4];
static double encoder_scale[4], output[4];
static unsigned motion_writes, tasks_started, fram_writes, fram_reads, braked_mask;
static int write_budget = -1, failed_motor = -1, failed_encoder = -1;
static uint32_t frequency = 100000;
static bool driver_sleep, fail_driver_transition;
bool hw_motor_outputs_stopped(void) {
    return !braked_mask && !output[0] && !output[1] && !output[2] && !output[3];
}
bool hw_motor_drivers_sleeping(void) { return driver_sleep; }
bool hw_motor_set_driver_sleep(bool sleeping) {
    if (sleeping && !driver_sleep) {
        for (unsigned i = 0; i < 4; ++i) assert(output[i] == 0 && !controllers_manager_is_running(i));
        assert(!platform_is_controller_running());
    }
    driver_sleep = sleeping || fail_driver_transition;
    return !fail_driver_transition;
}
void run_usb_idle_lifecycle(void);
void run_sleep_queues(unsigned blocked_bus, bool board_idle, uint8_t stale_command);
void check_idle_hook(bool expected);
void check_idle_preemption(void);

bool hw_fram_read(uint16_t address, void *data, uint16_t length)
{
    assert((size_t)address + length <= sizeof(fram));
    memcpy(data, fram + address, length);
    fram_reads++;
    return true;
}

bool hw_fram_write(uint16_t address, const void *data, uint16_t length)
{
    assert((size_t)address + length <= sizeof(fram));
    size_t n = write_budget >= 0 && write_budget < length ? (size_t)write_budget : length;
    memcpy(fram + address, data, n);
    fram_writes++;
    if (write_budget >= 0) write_budget -= n;
    return n == length;
}

static void motor_init(uint8_t i, bool reverse)
{
    assert(i < 4);
    output[i] = 0;
    motor_enabled[i] = i != failed_motor;
    reversed_motor[i] = reverse;
}
static uint8_t motor_ready(uint8_t i) { return motor_enabled[i]; }
static uint8_t motor_reverse(uint8_t i) { return reversed_motor[i]; }
static void pwm(uint8_t i, double value) { output[i] = value; motion_writes += value != 0; }
static void coast(uint8_t i) { output[i] = 0; braked_mask &= ~(1u << i); }
static void brake(uint8_t i) { if (motor_enabled[i]) braked_mask |= 1u << i; }
static void encoder_init(uint8_t i, double res, uint8_t reverse) { encoder_scale[i] = res; reversed_encoder[i] = reverse; }
static uint8_t encoder_ready(uint8_t i) { return encoder_scale[i] != 0; }
static double scale(uint8_t i) { return encoder_scale[i]; }
static uint16_t wheel_count[4];
static uint16_t count(uint8_t i) { return wheel_count[i]; }
uint8_t hw_encoder_is_reversed(uint8_t i) { return reversed_encoder[i]; }
bool hw_encoder_try_initialize(uint8_t i, double res, uint8_t reverse)
{
    if (i == failed_encoder) return false;
    encoder_init(i, res, reverse);
    return true;
}
void hw_encoder_discard_restored_setup(void)
{
    memset(encoder_scale, 0, sizeof(encoder_scale));
    memset(reversed_encoder, 0, sizeof(reversed_encoder));
}
void hw_motor_discard_restored_setup(void)
{
    memset(output, 0, sizeof(output));
    memset(motor_enabled, 0, sizeof(motor_enabled));
    memset(reversed_motor, 0, sizeof(reversed_motor));
    frequency = 100000;
}
const hw_motor_interface_t *get_motor_interface(void)
{
    static const hw_motor_interface_t iface = {motor_init, motor_reverse, motor_ready, pwm, coast, brake};
    return &iface;
}
const hw_encoder_interface_t *get_encoder_interface(void)
{
    static const hw_encoder_interface_t iface = {.initialize=encoder_init, .get_value=count,
        .is_initialized=encoder_ready, .get_resolution=scale};
    return &iface;
}
bool hw_motor_pwm_frequency_allowed(uint32_t value) { return value == 100000 || value == 25000 || value == 20000; }
bool hw_motor_set_pwm_frequency(uint32_t value)
{
    assert(hw_motor_pwm_frequency_allowed(value));
    memset(output, 0, sizeof(output));
    frequency = value;
    return true;
}
uint32_t hw_motor_get_pwm_frequency(void) { return frequency; }
static uint64_t now = 1000;
static hw_imu_sample_t injected;
static uint64_t injected_accel, injected_gyro;
static hw_imu_status_t injected_status = HW_IMU_UNAVAILABLE;
static unsigned observation_calls;
static bool motor_fault;
uint64_t hw_clock_microseconds(void) { return now; }
const gpio_interface_t *get_gpio_interface(void) { return NULL; }
bool hw_drv8873_fault_active(uint8_t index) { return motor_fault && index == 0; }
// Unexpected motor-protection writes or sensor access must fail this fixture.
bool hw_motor_current_limit_allowed(uint8_t i, const hw_motor_current_limit_t *s) { abort(); }
bool hw_motor_set_current_limit(uint8_t i, const hw_motor_current_limit_t *s) { abort(); }
bool hw_motor_get_current_limit(uint8_t i, hw_motor_current_limit_t *s) { abort(); }
bool hw_motor_read_faults(uint8_t i, hw_motor_faults_t *f) { abort(); }
bool hw_motor_clear_faults(uint8_t i, hw_motor_faults_t *f) { abort(); }
hw_imu_status_t hw_imu_read_observation(uint8_t index, hw_imu_sample_t *sample,
                                        uint64_t *accel_us, uint64_t *gyro_us) {
    ++observation_calls;
    assert(index == 0);
    *sample = injected; *accel_us = injected_accel; *gyro_us = injected_gyro;
    return injected_status;
}
hw_imu_status_t hw_imu_read(uint8_t i, hw_imu_sample_t *s) { abort(); }
hw_temperature_status_t hw_temperature_read(uint8_t i, double *v) { abort(); }
hw_current_sense_status_t hw_current_sense_read(uint8_t i, double *v) { abort(); }
osThreadState_t osThreadGetState(osThreadId_t t) { return t ? 0 : osThreadError; }
typedef struct { void (*fn)(void *); void *arg; } task_t;
static task_t controller_task, odom_task, supervisor_task;
static jmp_buf task_end;
static bool first_delay;
osThreadId_t osThreadNew(void (*fn)(void *), void *arg, const osThreadAttr_t *attr)
{
    task_t *task = !strcmp(attr->name, "ControllerTask") ? &controller_task :
        !strcmp(attr->name, "FeedbackWatch") ? &supervisor_task : &odom_task;
    *task = (task_t){fn, arg};
    ++tasks_started;
    return task;
}
TickType_t xTaskGetTickCount(void) { return 0; }
void vTaskDelayUntil(TickType_t *last, TickType_t period)
{
    if (!first_delay) longjmp(task_end, 1);
    first_delay = false;
}
#if BOARD_HAS_FUSION_EXPERIMENT
static void run_task(task_t task)
{
    assert(task.fn);
    first_delay = true;
    if (!setjmp(task_end)) task.fn(task.arg);
}
#endif
SemaphoreHandle_t xSemaphoreCreateMutex(void) { static unsigned locks[12], n; assert(n < 12); return &locks[n++]; }
int xSemaphoreTake(SemaphoreHandle_t m, uint32_t timeout) { assert(m && !*(unsigned *)m); *(unsigned *)m = 1; return 1; }
void xSemaphoreGive(SemaphoreHandle_t m) { assert(m && *(unsigned *)m); *(unsigned *)m = 0; }

static connection_t session;
static uint8_t reply[255], reply_len;
static uint16_t next_id;
static bool send_frame(uint8_t *frame, uint8_t n) { memcpy(reply, frame, n); reply_len = n; return true; }
static void request(controller_command_t cmd, uint8_t expected_error)
{
    reply_len = 0;
    cmd.message_id = ++next_id;
    connection_receive(&session, (uint8_t *)&cmd, command_request_size(cmd.commandType));
    connection_poll(&session);
    assert(reply_len >= 4 && reply[2] == (next_id & 255) && reply[3] == next_id >> 8);
    if (expected_error) {
        assert(reply_len == 6 && reply[1] == KINISI_MESSAGE_ERROR && reply[4] == cmd.commandType && reply[5] == expected_error);
    } else {
        assert(reply[1] == cmd.commandType);
    }
}
static void connect(void)
{
    connection_init(&session, command_handler, send_frame, hw_clock_microseconds);
    controller_command_t init = {.commandType=INIT, .properties.init={.protocol_major=2, .protocol_minor=14}};
    request(init, RESPONSE_OK);
    connection_poll(&session);
    assert(reply[1] == KINISI_MESSAGE_READY);
}
static double field(uint8_t id, uint8_t index)
{
    request((controller_command_t){.commandType=GET_CONFIGURATION_FIELD, .properties.get_configuration_field={id, index}}, RESPONSE_OK);
    double value;
    assert(reply_len == sizeof(value) + 4);
    memcpy(&value, reply + 4, sizeof(value));
    return value;
}
static void check_frequency(uint32_t hz)
{
    request((controller_command_t){.commandType=GET_PWM_FREQUENCY}, RESPONSE_OK);
    uint32_t value;
    assert(reply_len == sizeof(value) + 4);
    memcpy(&value, reply + 4, sizeof(value));
    assert(value == hz);
}
static void configure(int type)
{
    controller_command_t c = {0};
    if (type == PLATFORM_DIFFERENTIAL) {
        c.commandType = INITIALIZE_DIFFERENTIAL_PLATFORM;
        c.properties.initialize_differential_platform = (typeof(c.properties.initialize_differential_platform)){
            1,0,0,1,0.09,0.25,720};
    } else if (type == PLATFORM_OMNI) {
        c.commandType = INITIALIZE_OMNI_PLATFORM;
        c.properties.initialize_omni_platform = (typeof(c.properties.initialize_omni_platform)){
            1,0,1,0,1,0,0.09,0.155,720};
    } else {
        c.commandType = INITIALIZE_MECANUM_PLATFORM;
        c.properties.initialize_mecanum_platform = (typeof(c.properties.initialize_mecanum_platform)){
            1,0,1,0,0,1,0,1,0.4,0.29,0.11,720};
    }
    request(c, RESPONSE_OK);
    // Exercise the ordinary standalone channel commands and per-wheel overrides.
    if (type != PLATFORM_MECANUM) {
        request((controller_command_t){.commandType=INITIALIZE_MOTOR, .properties.initialize_motor={3, 1}}, RESPONSE_OK);
    }
    if (type == PLATFORM_DIFFERENTIAL) {
        request((controller_command_t){.commandType=INITIALIZE_MOTOR, .properties.initialize_motor={2, 0}}, RESPONSE_OK);
    }
    for (uint8_t i = 0; i < 4; ++i) {
        request((controller_command_t){.commandType=INITIALIZE_ENCODER, .properties.initialize_encoder={
            .encoder_index=i, .encoder_resolution=800.5 + i, .is_reversed=(i % 2)}}, RESPONSE_OK);
    }
    request((controller_command_t){.commandType=SET_PWM_FREQUENCY, .properties.set_pwm_frequency={25000}}, RESPONSE_OK);
}
static void verify(int type)
{
    assert(field(0,0) == type);
    assert(field(1,0) == (type == 3 ? 0.11 : 0.09));
    assert(field(2,0) == (type == 1 ? 0.25 : type == 2 ? 0 : 0.4));
    assert(field(3,0) == (type == 1 ? 0 : type == 2 ? 0.155 : 0.29));
    for (uint8_t i = 0; i < 4; ++i) {
        assert(field(4,i) == 800.5 + i && field(5,i) == i % 2);
        assert(field(6,i) == (i == 0 || (i == 2 && type != 1) || (i == 3 && type != 3)));
    }
    check_frequency(25000);
}

static void sleep_policy(bool board)
{
    controller_command_t set_sleep = {.commandType=SET_DRIVER_SLEEP, .properties.set_driver_sleep={1}};
    controller_command_t get_sleep = {.commandType=GET_DRIVER_SLEEP};
    if (board) {
        set_sleep = (controller_command_t){.commandType=SET_BOARD_IDLE, .properties.set_board_idle={1}};
        get_sleep.commandType = GET_BOARD_IDLE;
    }
    connection_init(&session, command_handler, send_frame, hw_clock_microseconds);
    request(set_sleep, RESPONSE_INIT_REQUIRED);
    request(get_sleep, RESPONSE_INIT_REQUIRED);
    request((controller_command_t){.commandType=INIT,
        .properties.init={.protocol_major=2, .client_capabilities=CLIENT_CAP_WALL_CLOCK}}, RESPONSE_OK);
    request(set_sleep, RESPONSE_CLOCK_NOT_READY);
    request(get_sleep, RESPONSE_CLOCK_NOT_READY);
    assert(!connection_board_idle() && !driver_sleep);
    connect();
    request(get_sleep, RESPONSE_OK);
    assert(reply_len == 5 && reply[4] == 0);
    request((controller_command_t){.commandType=SET_DRIVER_SLEEP, .properties.set_driver_sleep={2}}, RESPONSE_INVALID_ARGUMENT);
    if (board) {
        request((controller_command_t){.commandType=SET_BOARD_IDLE, .properties.set_board_idle={2}}, RESPONSE_INVALID_ARGUMENT);
        check_idle_hook(false);
        check_idle_preemption();
    }
    configure(PLATFORM_DIFFERENTIAL);
    request((controller_command_t){.commandType=SAVE_CONFIGURATION},
        BOARD_HAS_DRV8870_SLEEP ? RESPONSE_SENSOR_NOT_AVAILABLE : RESPONSE_OK);
    unsigned writes = fram_writes;
    request((controller_command_t){.commandType=INITIALIZE_MOTOR_CONTROLLER,
        .properties.initialize_motor_controller={3,1,3,1,803.5,1,0,0,10}}, RESPONSE_OK);
    request((controller_command_t){.commandType=INITIALIZE_MOTOR_POSITION_CONTROLLER,
        .properties.initialize_motor_position_controller={3,1,1,0.01}}, RESPONSE_OK);
    request((controller_command_t){.commandType=SET_MOTOR_POSITION, .properties.set_motor_position={3,2}}, RESPONSE_OK);
    request((controller_command_t){.commandType=START_PLATFORM_CONTROLLER,
        .properties.start_platform_controller={.kp=1, .integral_limit=10}}, RESPONSE_OK);
    request((controller_command_t){.commandType=SET_PLATFORM_TARGET_VELOCITY,
        .properties.set_platform_target_velocity={0.1,0,0}}, RESPONSE_OK);
    assert(controllers_manager_is_running(3) && platform_is_controller_running());
    if (board) {
        // Already active: no-op exit must leave the running controllers untouched.
        request((controller_command_t){.commandType=SET_BOARD_IDLE, .properties.set_board_idle={0}}, RESPONSE_OK);
        assert(controllers_manager_is_running(3) && platform_is_controller_running());
        fail_driver_transition = true;
        request(set_sleep, RESPONSE_INTERNAL_ERROR);
        assert(connection_board_idle() && driver_sleep);
        fail_driver_transition = false;
    }
    request(set_sleep, RESPONSE_OK);
    if (board) {
        request(set_sleep, RESPONSE_OK);
        check_idle_hook(true);
        for (unsigned state = 0; state < 2; ++state)
            request((controller_command_t){.commandType=SET_DRIVER_SLEEP, .properties.set_driver_sleep={state}}, RESPONSE_BOARD_IDLE);
    }
    assert(!platform_is_controller_running() && !controllers_manager_is_running(3) && driver_sleep);
    connect();
    request(get_sleep, RESPONSE_OK);
    assert(reply[4] == 1);
    const uint8_t blocked[] = {
        INITIALIZE_MOTOR, SET_MOTOR_SPEED, BRAKE_MOTOR, INITIALIZE_MOTOR_CONTROLLER, SET_MOTOR_TARGET_SPEED,
        INITIALIZE_MOTOR_POSITION_CONTROLLER, INITIALIZE_MOTOR_POSITION_PID_CONTROLLER, SET_MOTOR_POSITION,
        INITIALIZE_DIFFERENTIAL_PLATFORM, INITIALIZE_OMNI_PLATFORM, INITIALIZE_MECANUM_PLATFORM,
        SET_PLATFORM_VELOCITY, START_PLATFORM_CONTROLLER, BRAKE_PLATFORM, SET_PLATFORM_TARGET_VELOCITY,
        INITIALIZE_PLATFORM_POSITION_CONTROLLER, INITIALIZE_PLATFORM_POSITION_PID_CONTROLLER, SET_PLATFORM_POSITION,
        GET_MOTOR_CURRENT_LIMIT, SET_MOTOR_CURRENT_LIMIT, CLEAR_MOTOR_FAULTS,
    };
    for (unsigned i = 0; i < sizeof(blocked); ++i) {
        controller_command_t cmd = {.commandType=blocked[i]};
        if (cmd.commandType == SET_MOTOR_CURRENT_LIMIT) cmd.properties.set_motor_current_limit.threshold_ma = 4000;
        request(cmd, RESPONSE_DRIVERS_ASLEEP);
    }
    if (BOARD_HAS_DRV8870_SLEEP) check_frequency(25000);
    else verify(PLATFORM_DIFFERENTIAL);
    request((controller_command_t){.commandType=COAST_PLATFORM}, RESPONSE_OK);
    if (board) {
        set_sleep.properties.set_board_idle.idle = 0;
        fail_driver_transition = true;
        request(set_sleep, RESPONSE_INTERNAL_ERROR);
        request(get_sleep, RESPONSE_OK);
        assert(reply[4] == 1 && driver_sleep && connection_board_idle());
        fail_driver_transition = false;
    } else set_sleep.properties.set_driver_sleep.sleeping = 0;
    request(set_sleep, RESPONSE_OK);
    assert(!driver_sleep);
    if (board) check_idle_hook(false);
    if (BOARD_HAS_DRV8870_SLEEP) check_frequency(25000);
    else verify(PLATFORM_DIFFERENTIAL);
    assert(fram_writes == writes);
    request((controller_command_t){.commandType=GET_MOTOR_POSITION, .properties.get_motor_position={3}},
        RESPONSE_CONTROLLER_NOT_INITIALIZED);
    request((controller_command_t){.commandType=SET_PLATFORM_TARGET_VELOCITY}, RESPONSE_CONTROLLER_NOT_INITIALIZED);
    request((controller_command_t){.commandType=SET_MOTOR_TARGET_SPEED, .properties.set_motor_target_speed={3,1}},
        RESPONSE_CONTROLLER_NOT_INITIALIZED);
    if (board) {
        controller_command_t enter = {.commandType=SET_BOARD_IDLE, .properties.set_board_idle={1}};
        request((controller_command_t){.commandType=SET_DRIVER_SLEEP, .properties.set_driver_sleep={1}}, RESPONSE_OK);
        fail_driver_transition = true;
        request(enter, RESPONSE_INTERNAL_ERROR);
        // Independent inhibit must be confirmed before even releasing board idle.
        request(set_sleep, RESPONSE_INTERNAL_ERROR);
        assert(driver_sleep && connection_board_idle());
        fail_driver_transition = false;
        request(set_sleep, RESPONSE_OK);
        assert(driver_sleep && !connection_board_idle());
        request(enter, RESPONSE_OK);
        request(enter, RESPONSE_OK);
        request(set_sleep, RESPONSE_OK);
        assert(driver_sleep && !connection_board_idle());
        request(enter, RESPONSE_OK);
        set_sleep.message_id = ++next_id;
        connection_receive_queued(&session, (const uint8_t *)&set_sleep,
            command_request_size(SET_BOARD_IDLE), BEFORE_BOARD_WAKE);
        connection_poll(&session);
        assert(reply_len == 6 && reply[1] == KINISI_MESSAGE_ERROR &&
               reply[4] == SET_BOARD_IDLE && reply[5] == RESPONSE_BOARD_IDLE);
        assert(driver_sleep && connection_board_idle()); // Restart must clear both volatile inhibits.
    } else {
        set_sleep.properties.set_driver_sleep.sleeping = 1;
        request(set_sleep, RESPONSE_OK);
    }
}

static void fresh_brake_commands(void)
{
    controller_command_t motor_brake = {.commandType=BRAKE_MOTOR, .properties.brake_motor={3}};
    controller_command_t platform_brake = {.commandType=BRAKE_PLATFORM};
    request(motor_brake, RESPONSE_OK);
    request(platform_brake, RESPONSE_OK);
    assert(braked_mask == 0); // Ordinary brake on uninitialized hardware is a no-op.
    configure(PLATFORM_DIFFERENTIAL); // Owns wheels 0 and 1; both standalone motors are also ready.
    request((controller_command_t){.commandType=SET_DRIVER_SLEEP, .properties.set_driver_sleep={1}}, RESPONSE_OK);
    request(motor_brake, RESPONSE_DRIVERS_ASLEEP);
    request(platform_brake, RESPONSE_DRIVERS_ASLEEP);
    request((controller_command_t){.commandType=SET_DRIVER_SLEEP, .properties.set_driver_sleep={0}}, RESPONSE_OK);
    assert(!motion_writes && !braked_mask);
    request((controller_command_t){.commandType=BRAKE_MOTOR, .properties.brake_motor={0}}, RESPONSE_MOTOR_OWNED);
    request(motor_brake, RESPONSE_OK);
    assert(braked_mask == (1u << 3));
    request((controller_command_t){.commandType=STOP_MOTOR, .properties.stop_motor={3}}, RESPONSE_OK);
    assert(braked_mask == 0);
    request(platform_brake, RESPONSE_OK);
    assert(braked_mask == 3);
    request((controller_command_t){.commandType=COAST_PLATFORM}, RESPONSE_OK);
    assert(!braked_mask && !motion_writes);
}

#if BOARD_HAS_FUSION_EXPERIMENT
#include "fusion_scenarios.h"
#endif
int main(int argc, char **argv)
{
    assert(argc == 4);
    const char *mode = argv[1], *path = argv[2];
    int type = atoi(argv[3]);
    if (!strcmp(mode, "usb-idle")) {
        run_usb_idle_lifecycle();
        assert(!motion_writes && !braked_mask);
        return 0;
    }
    if (strstr(mode, "-queues")) {
        bool board = strncmp(mode, "idle-", 5) == 0;
        uint8_t stale = strstr(mode, "motor-brake-queues") ? BRAKE_MOTOR :
                        strstr(mode, "platform-brake-queues") ? BRAKE_PLATFORM : SET_MOTOR_SPEED;
        run_sleep_queues(type, board, stale);
        assert(output[3] == 70 && motion_writes == (board ? 2u : 3u));
        return 0;
    }
    FILE *file = fopen(path, "rb");
    if (file) { assert(fread(fram, 1, sizeof(fram), file) == sizeof(fram)); fclose(file); }
    if (!strcmp(mode, "driver-fault")) failed_motor = 1;
    if (!strcmp(mode, "encoder-fault")) failed_encoder = 2;
    controller_configuration_restore(); // same cold-boot entry point used before transport initialization
    assert(!tasks_started && !motion_writes && !fram_writes);
    for (int i = 0; i < 4; ++i) assert(output[i] == 0);
    connect();
#if BOARD_HAS_FUSION_EXPERIMENT
    if (!strcmp(mode, "fusion")) { fusion_scenario(type); return 0; }
#endif
    if (!strcmp(mode, "idle-reboot")) {
        request((controller_command_t){.commandType=GET_BOARD_IDLE}, RESPONSE_OK);
        assert(!reply[4]);
        request((controller_command_t){.commandType=GET_DRIVER_SLEEP}, RESPONSE_OK);
        assert(!reply[4] && !motion_writes && !tasks_started);
        return 0;
    }
    if (!strcmp(mode, "brake-policy")) { fresh_brake_commands(); return 0; }
    if (!strcmp(mode, "unsupported")) {
        request((controller_command_t){.commandType=GET_FUSION_STATUS}, RESPONSE_SENSOR_NOT_AVAILABLE);
        request((controller_command_t){.commandType=SET_FUSION_EXPERIMENT}, RESPONSE_SENSOR_NOT_AVAILABLE);
        configure(type);
        if (!BOARD_HAS_BOARD_IDLE) {
            request((controller_command_t){.commandType=GET_BOARD_IDLE}, RESPONSE_SENSOR_NOT_AVAILABLE);
            request((controller_command_t){.commandType=SET_BOARD_IDLE, .properties.set_board_idle={1}}, RESPONSE_SENSOR_NOT_AVAILABLE);
        }
        assert(!connection_board_idle() && !driver_sleep);
        if (BOARD_HAS_DRIVER_SLEEP) {
            request((controller_command_t){.commandType=GET_DRIVER_SLEEP}, RESPONSE_OK);
            assert(!reply[4]);
        } else {
            request((controller_command_t){.commandType=GET_DRIVER_SLEEP}, RESPONSE_SENSOR_NOT_AVAILABLE);
            request((controller_command_t){.commandType=SET_DRIVER_SLEEP, .properties.set_driver_sleep={1}}, RESPONSE_SENSOR_NOT_AVAILABLE);
        }
        request((controller_command_t){.commandType=SAVE_CONFIGURATION}, RESPONSE_SENSOR_NOT_AVAILABLE);
        request((controller_command_t){.commandType=GET_CONFIGURATION_FIELD}, RESPONSE_SENSOR_NOT_AVAILABLE);
        assert(platform_is_initialized() && !fram_writes && !fram_reads);
        check_frequency(25000);
        assert(encoder_scale[3] == 803.5 && reversed_motor[3]);
        return 0;
    }
    if (!strcmp(mode, "sleep-policy") || !strcmp(mode, "idle-policy")) {
        sleep_policy(!strcmp(mode, "idle-policy"));
    } else if (!strcmp(mode, "wake-boot")) {
        request((controller_command_t){.commandType=GET_BOARD_IDLE}, RESPONSE_OK);
        assert(reply[4] == 0);
        request((controller_command_t){.commandType=GET_DRIVER_SLEEP}, RESPONSE_OK);
        assert(reply[4] == 0 && !platform_is_controller_running() && !motion_writes);
        verify(PLATFORM_DIFFERENTIAL);
    } else if (!strcmp(mode, "blank") || !strcmp(mode, "driver-fault") || !strcmp(mode, "encoder-fault")) {
        assert(field(0,0) == 0);
        check_frequency(100000);
        request((controller_command_t){.commandType=SAVE_CONFIGURATION}, RESPONSE_PLATFORM_NOT_INITIALIZED);
        request((controller_command_t){.commandType=SET_PLATFORM_VELOCITY}, RESPONSE_PLATFORM_NOT_INITIALIZED);
        for (int i = 0; i < 4; ++i) {
            assert(!motor_enabled[i] && !encoder_ready(i));
        }
    } else if (!strcmp(mode, "verify")) {
        verify(type);
        assert(!platform_is_controller_running() && !platform_is_odometry_enabled());
        request((controller_command_t){.commandType=SET_PLATFORM_TARGET_VELOCITY}, RESPONSE_CONTROLLER_NOT_INITIALIZED);
        request((controller_command_t){.commandType=SET_PLATFORM_VELOCITY, .properties.set_platform_velocity={12,0,0}}, RESPONSE_OK);
        assert(motion_writes > 0);
    } else if (!strcmp(mode, "save") || !strcmp(mode, "interrupt")) {
        configure(type);
        verify(type);
        // Persist while moving to prove the next process cannot recover the nonzero duty.
        request((controller_command_t){.commandType=SET_PLATFORM_VELOCITY, .properties.set_platform_velocity={12,0,0}}, RESPONSE_OK);
        assert(motion_writes > 0);
        bool interrupted = !strcmp(mode, "interrupt");
        write_budget = interrupted ? 33 : -1;
        request((controller_command_t){.commandType=SAVE_CONFIGURATION}, interrupted ? RESPONSE_INTERNAL_ERROR : RESPONSE_OK);
        request((controller_command_t){.commandType=SET_PWM_FREQUENCY, .properties.set_pwm_frequency={20000}}, RESPONSE_OK);
        connect();
        check_frequency(20000);
        assert(field(0,0) == type);
    } else if (!strcmp(mode, "encoder-free-save") || !strcmp(mode, "mixed-save")) {
        request((controller_command_t){.commandType=INITIALIZE_DIFFERENTIAL_PLATFORM,
            .properties.initialize_differential_platform={0,1,1,0,0.11,0.45,0}}, RESPONSE_OK);
        if (!strcmp(mode, "mixed-save"))
            request((controller_command_t){.commandType=INITIALIZE_ENCODER,
                .properties.initialize_encoder={.encoder_index=1, .encoder_resolution=720}}, RESPONSE_OK);
        request((controller_command_t){.commandType=SAVE_CONFIGURATION}, RESPONSE_OK);
    } else if (!strcmp(mode, "encoder-free-verify") || !strcmp(mode, "mixed-verify")) {
        assert(field(0,0) == PLATFORM_DIFFERENTIAL && field(1,0) == 0.11);
        assert(field(4,0) == 0 && field(5,0) == 1 && field(6,1) == 1);
        for (unsigned i = 0; i < 4; ++i) assert(encoder_ready(i) == (!strcmp(mode, "mixed-verify") && i == 1));
        request((controller_command_t){.commandType=START_PLATFORM_CONTROLLER,
            .properties.start_platform_controller={.kp=1, .integral_limit=10}}, RESPONSE_ENCODER_NOT_INITIALIZED);
        request((controller_command_t){.commandType=START_PLATFORM_ODOMETRY}, RESPONSE_ENCODER_NOT_INITIALIZED);
        assert(!tasks_started && !motion_writes && !platform_is_odometry_enabled() && !platform_is_controller_running());
        // Configure only the missing feedback channels. The same commands must
        // then succeed without repeating the saved platform initialization.
        for (uint8_t i = 0; i < 2; ++i) {
            if (!encoder_ready(i))
                request((controller_command_t){.commandType=INITIALIZE_ENCODER,
                    .properties.initialize_encoder={.encoder_index=i, .encoder_resolution=720}}, RESPONSE_OK);
        }
        request((controller_command_t){.commandType=START_PLATFORM_ODOMETRY}, RESPONSE_OK);
        request((controller_command_t){.commandType=START_PLATFORM_CONTROLLER,
            .properties.start_platform_controller={.kp=1, .integral_limit=10}}, RESPONSE_OK);
        assert(platform_is_odometry_enabled() && platform_is_controller_running());
    } else if (!strcmp(mode, "bad-frequency")) {
        // Valid journal and checksum, but the board does not support the recorded frequency.
        uint8_t data[72];
        uint32_t bad = 12345;
        assert(config_store_load(data, sizeof(data)));
        memcpy(data + 56, &bad, sizeof(bad));
        assert(config_store_save(data, sizeof(data)));
    } else { abort(); }
    file = fopen(path, "wb");
    assert(file && fwrite(fram, 1, sizeof(fram), file) == sizeof(fram));
    assert(fclose(file) == 0);
    return 0;
}
