// Production session, command, platform and storage paths; simulated devices and scheduler.
#include "commands_handler.h"
#include "connection.h"
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t fram[8192], motor_enabled[4], reversed_motor[4], reversed_encoder[4];
static double encoder_scale[4], output[4];
static unsigned motion_writes, tasks_started, fram_writes, fram_reads;
static int write_budget = -1, failed_motor = -1, failed_encoder = -1;
static uint32_t frequency = 100000;
static bool driver_sleep;
bool hw_motor_drivers_sleeping(void) { return driver_sleep; }
bool hw_motor_set_driver_sleep(bool sleeping) {
    if (sleeping && !driver_sleep) {
        for (unsigned i = 0; i < 4; ++i) assert(output[i] == 0 && !controllers_manager_is_running(i));
        assert(!platform_is_controller_running());
    }
    driver_sleep = sleeping;
    return true;
}
void run_sleep_queues(unsigned blocked_bus);

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
static void coast(uint8_t i) { output[i] = 0; }
static void encoder_init(uint8_t i, double res, uint8_t reverse) { encoder_scale[i] = res; reversed_encoder[i] = reverse; }
static uint8_t encoder_ready(uint8_t i) { return encoder_scale[i] != 0; }
static double scale(uint8_t i) { return encoder_scale[i]; }
static uint16_t count(uint8_t i) { return 0; }
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
    static const hw_motor_interface_t iface = {motor_init, motor_reverse, motor_ready, pwm, coast, coast};
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
uint64_t hw_clock_microseconds(void) { return 1000; }
const gpio_interface_t *get_gpio_interface(void) { return NULL; }
bool hw_drv8873_fault_active(uint8_t index) { return false; }
// Unexpected motor-protection writes or sensor access must fail this fixture.
bool hw_motor_current_limit_allowed(uint8_t i, const hw_motor_current_limit_t *s) { abort(); }
bool hw_motor_set_current_limit(uint8_t i, const hw_motor_current_limit_t *s) { abort(); }
bool hw_motor_get_current_limit(uint8_t i, hw_motor_current_limit_t *s) { abort(); }
bool hw_motor_read_faults(uint8_t i, hw_motor_faults_t *f) { abort(); }
bool hw_motor_clear_faults(uint8_t i, hw_motor_faults_t *f) { abort(); }
hw_imu_status_t hw_imu_read(uint8_t i, hw_imu_sample_t *s) { abort(); }
hw_temperature_status_t hw_temperature_read(uint8_t i, double *v) { abort(); }
hw_current_sense_status_t hw_current_sense_read(uint8_t i, double *v) { abort(); }
osThreadState_t osThreadGetState(osThreadId_t t) { return t ? 0 : osThreadError; }
osThreadId_t osThreadNew(void (*fn)(void *), void *arg, const osThreadAttr_t *attr)
{ static int thread; tasks_started++; return &thread; }
TickType_t xTaskGetTickCount(void) { return 0; }
void vTaskDelayUntil(TickType_t *last, TickType_t period) { abort(); }
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

static void sleep_policy(void)
{
    controller_command_t set_sleep = {.commandType=SET_DRIVER_SLEEP, .properties.set_driver_sleep={1}};
    controller_command_t get_sleep = {.commandType=GET_DRIVER_SLEEP};
    connection_init(&session, command_handler, send_frame, hw_clock_microseconds);
    request(set_sleep, RESPONSE_INIT_REQUIRED);
    request(get_sleep, RESPONSE_INIT_REQUIRED);
    connect();
    request(get_sleep, RESPONSE_OK);
    assert(reply_len == 5 && reply[4] == 0);
    request((controller_command_t){.commandType=SET_DRIVER_SLEEP, .properties.set_driver_sleep={2}}, RESPONSE_INVALID_ARGUMENT);
    configure(PLATFORM_DIFFERENTIAL);
    request((controller_command_t){.commandType=SAVE_CONFIGURATION}, RESPONSE_OK);
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
    request(set_sleep, RESPONSE_OK);
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
    verify(PLATFORM_DIFFERENTIAL);
    request((controller_command_t){.commandType=COAST_PLATFORM}, RESPONSE_OK);
    set_sleep.properties.set_driver_sleep.sleeping = 0;
    request(set_sleep, RESPONSE_OK);
    verify(PLATFORM_DIFFERENTIAL);
    assert(fram_writes == writes);
    request((controller_command_t){.commandType=GET_MOTOR_POSITION, .properties.get_motor_position={3}},
        RESPONSE_CONTROLLER_NOT_INITIALIZED);
    request((controller_command_t){.commandType=SET_PLATFORM_TARGET_VELOCITY}, RESPONSE_CONTROLLER_NOT_INITIALIZED);
    request((controller_command_t){.commandType=SET_MOTOR_TARGET_SPEED, .properties.set_motor_target_speed={3,1}},
        RESPONSE_CONTROLLER_NOT_INITIALIZED);
    set_sleep.properties.set_driver_sleep.sleeping = 1;
    request(set_sleep, RESPONSE_OK);
}

int main(int argc, char **argv)
{
    assert(argc == 4);
    const char *mode = argv[1], *path = argv[2];
    int type = atoi(argv[3]);
    if (!strcmp(mode, "sleep-queues")) {
        run_sleep_queues(type);
        assert(output[3] == 70 && motion_writes == 2);
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
    if (!strcmp(mode, "unsupported")) {
        configure(type);
        request((controller_command_t){.commandType=GET_DRIVER_SLEEP}, RESPONSE_SENSOR_NOT_AVAILABLE);
        request((controller_command_t){.commandType=SET_DRIVER_SLEEP, .properties.set_driver_sleep={1}}, RESPONSE_SENSOR_NOT_AVAILABLE);
        request((controller_command_t){.commandType=SAVE_CONFIGURATION}, RESPONSE_SENSOR_NOT_AVAILABLE);
        request((controller_command_t){.commandType=GET_CONFIGURATION_FIELD}, RESPONSE_SENSOR_NOT_AVAILABLE);
        assert(platform_is_initialized() && !fram_writes && !fram_reads);
        check_frequency(25000);
        assert(encoder_scale[3] == 803.5 && reversed_motor[3]);
        return 0;
    }
    if (!strcmp(mode, "sleep-policy")) {
        sleep_policy();
    } else if (!strcmp(mode, "wake-boot")) {
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
