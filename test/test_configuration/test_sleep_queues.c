// Drive the normal command task with independent USB/I2C ingress and delayed
// transmit availability. Stop the task after a fresh post-wake command succeeds.
#include "commands_manager.h"
#include "commands_handler.h"
#include "connection.h"
#include "hw_motor.h"
#include "os_interface.h"
#include "protocol.h"
#include "usb_device.h"
#include <assert.h>
#include <setjmp.h>
#include <stdbool.h>

message_queue_t CommandQueue, I2CCommandQueue;
#ifdef MP_V3_1
extern USBD_HandleTypeDef hUsbDeviceFS;
extern unsigned usb_starts, usb_stops;
extern bool vbus_present;
#else
USBD_HandleTypeDef hUsbDeviceFS = {.dev_state=USBD_STATE_CONFIGURED, .pClassData=&hUsbDeviceFS};
void MX_USB_DEVICE_Init(void) {}
void MX_USB_DEVICE_Process(void) {}
#endif
void initialize_external_i2c(void) {}
void mark_pending_external_i2c_requests(uint16_t barriers) { mark_pending_frames(&I2CCommandQueue, barriers); }
uint32_t hw_motor_fault_event_sequence(uint8_t i) { (void)i; return 0; }
static uint32_t masked;
uint32_t __get_PRIMASK(void) { return masked; }
void __disable_irq(void) { masked = 1; }
void __set_PRIMASK(uint32_t value) { masked = value; }

static unsigned stale_bus, phase, ticks;
static uint8_t stale_command;
static bool blocked, attempted, ready[2], board, usb_lifecycle;
static unsigned ready_count[2];
static uint8_t state_byte[20];
static uint8_t replies[20], errors[20];
static jmp_buf finished;
static char partial[256];
static size_t partial_size;

static bool receive(unsigned bus, const uint8_t *data, uint16_t n) {
    if (blocked && bus == stale_bus) { attempted = true; return false; }
    uint8_t id = data[2];
    assert(n >= 4 && id < sizeof(replies));
    if (data[1] == KINISI_MESSAGE_READY) { ready[bus] = true; ++ready_count[bus]; }
    else {
        replies[id] = data[1];
        if (n == 5) state_byte[id] = data[4];
        if (data[1] == KINISI_MESSAGE_ERROR) { assert(n == 6); errors[id] = data[5]; }
    }
    return true;
}
uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t n) { return receive(0, data, n) ? 0 : 1; }
uint8_t try_send_external_i2c(uint8_t *data, uint16_t n) { return receive(1, data, n); }
static message_queue_t *queue(unsigned bus) { return bus ? &I2CCommandQueue : &CommandQueue; }
static size_t encode(controller_command_t cmd, char *data) {
    size_t n = command_request_size(cmd.commandType);
    data[0] = (char)n;
    memcpy(data + 1, &cmd, n);
    return n + 1;
}
static void issue(unsigned bus, controller_command_t cmd) {
    char frame[256];
    size_t size = encode(cmd, frame);
    if (bus) assert(enqueue(queue(bus), frame));
    else assert(enqueue_multi(queue(bus), frame, size));
}
static controller_command_t speed(unsigned id, double value) {
    return (controller_command_t){.commandType=SET_MOTOR_SPEED, .message_id=id,
        .properties.set_motor_speed={.motor_index=3, .pwm=value}};
}
#ifdef MP_V3_1
// Exercise real VBUS processing and both real sessions. CDC transfers model
// host frames. The idle hook's simulated IRQ returns without changing the mode.
void check_idle_hook(bool expected);
static void usb_idle_turn(void)
{
    switch (phase) {
    case 0:
        assert(usb_starts == 1 && usb_stops == 0);
        issue(0, (controller_command_t){.commandType=INIT, .message_id=1, .properties.init={.protocol_major=2}});
        break;
    case 1:
        if (!ready[0]) return;
        issue(0, (controller_command_t){.commandType=SET_BOARD_IDLE, .message_id=2, .properties.set_board_idle={1}});
        break;
    case 2:
        if (!replies[2]) return;
        assert(replies[2] == SET_BOARD_IDLE && hw_motor_drivers_sleeping());
        check_idle_hook(true);
        issue(0, (controller_command_t){.commandType=PING, .message_id=3});
        issue(0, (controller_command_t){.commandType=GET_BOARD_IDLE, .message_id=4});
        issue(0, (controller_command_t){.commandType=GET_DRIVER_SLEEP, .message_id=5});
        break;
    case 3:
        if (!replies[5]) return;
        assert(replies[3] == PING && replies[4] == GET_BOARD_IDLE && state_byte[4] == 1);
        assert(replies[5] == GET_DRIVER_SLEEP && state_byte[5] == 1 && connection_board_idle());
        issue(0, (controller_command_t){.commandType=SET_BOARD_IDLE, .message_id=6, .properties.set_board_idle={0}});
        break;
    case 4:
        if (!replies[6]) return;
        assert(replies[6] == SET_BOARD_IDLE && !connection_board_idle() && !hw_motor_drivers_sleeping());
        assert(usb_starts == 1 && !usb_stops && ready_count[0] == 1 && hUsbDeviceFS.dev_address == 21);
        issue(0, (controller_command_t){.commandType=SET_DRIVER_SLEEP, .message_id=7, .properties.set_driver_sleep={1}});
        issue(0, (controller_command_t){.commandType=SET_BOARD_IDLE, .message_id=8, .properties.set_board_idle={1}});
        break;
    case 5:
        if (!replies[8]) return;
        assert(replies[7] == SET_DRIVER_SLEEP && replies[8] == SET_BOARD_IDLE);
        vbus_present = false;
        break;
    case 6:
        if (!usb_stops) return;
        assert(connection_board_idle() && hw_motor_drivers_sleeping() && !hUsbDeviceFS.pClassData);
        vbus_present = true;
        break;
    case 7:
        if (usb_starts != 2) return;
        issue(0, (controller_command_t){.commandType=SET_BOARD_IDLE, .message_id=9, .properties.set_board_idle={0}});
        break;
    case 8:
        if (!replies[9]) return;
        assert(errors[9] == RESPONSE_INIT_REQUIRED && connection_board_idle() && hw_motor_drivers_sleeping());
        issue(0, (controller_command_t){.commandType=INIT, .message_id=10, .properties.init={.protocol_major=2}});
        break;
    case 9:
        if (ready_count[0] != 2) return;
        issue(0, (controller_command_t){.commandType=GET_BOARD_IDLE, .message_id=11});
        issue(0, (controller_command_t){.commandType=SET_BOARD_IDLE, .message_id=12, .properties.set_board_idle={0}});
        break;
    case 10:
        if (!replies[12]) return;
        assert(replies[11] == GET_BOARD_IDLE && state_byte[11] == 1 && replies[12] == SET_BOARD_IDLE);
        assert(!connection_board_idle() && hw_motor_drivers_sleeping());
        assert(usb_starts == 2 && usb_stops == 1 && hUsbDeviceFS.dev_address == 22);
        longjmp(finished, 1);
    }
    ++phase;
}
#endif
#if BOARD_HAS_FEEDBACK_SOURCE_HEALTH
static bool counter_outage;
void health_queue_setup(void);
void health_queue_shaft(bool failed);
void health_queue_output(bool running);
static void counter_queue_turn(void)
{
    controller_command_t start={.commandType=START_PLATFORM_CONTROLLER,.message_id=6,
        .properties.start_platform_controller={8,0,0,100}};
    controller_command_t target={.commandType=SET_PLATFORM_TARGET_VELOCITY,.message_id=7,
        .properties.set_platform_target_velocity={.x=.2}};
    switch (phase) {
    case 0:
        for (unsigned i=0;i<2;i++) issue(i,(controller_command_t){.commandType=INIT,.message_id=i+1,.properties.init={.protocol_major=2}});
        break;
    case 1:
        if (!ready[0] || !ready[1]) return;
        health_queue_setup();health_queue_output(true);
        health_queue_shaft(true);health_queue_output(false);
        blocked=true;issue(stale_bus,(controller_command_t){.commandType=PING,.message_id=5});
        break;
    case 2:
        if (!attempted) return;
        issue(stale_bus,start);
        partial_size=encode(target,partial);assert(enqueue_multi(queue(stale_bus),partial,2));
        break;
    case 3:
        health_queue_shaft(false); // No peer has read the delayed command yet.
        break;
    case 4:
        assert(enqueue_multi(queue(stale_bus),partial+2,partial_size-2));blocked=false;
        break;
    case 5:
        if (!replies[7]) return;
        assert(errors[6]==RESPONSE_SAMPLE_NOT_AVAILABLE && errors[7]==RESPONSE_CONTROLLER_NOT_INITIALIZED);
        health_queue_output(false);
        start.message_id=8;target.message_id=9;issue(stale_bus,start);issue(stale_bus,target);
        break;
    case 6:
        if (!replies[9]) return;
        assert(replies[8]==START_PLATFORM_CONTROLLER && replies[9]==SET_PLATFORM_TARGET_VELOCITY);
        health_queue_output(true);longjmp(finished,1);
    }
    ++phase;
}
#endif
static void next_turn(uint32_t ms) {
#if BOARD_HAS_FEEDBACK_SOURCE_HEALTH
    if (counter_outage) {assert(ticks++<100);counter_queue_turn();return;}
#endif
    assert(!masked && ms == 1 && ++ticks < 100);
#ifdef MP_V3_1
    if (usb_lifecycle) { usb_idle_turn(); return; }
#endif
    const unsigned other = 1 - stale_bus;
    controller_command_t transition = {.commandType=board ? SET_BOARD_IDLE : SET_DRIVER_SLEEP};
    switch (phase) {
    case 0:
        for (unsigned i = 0; i < 2; ++i)
            issue(i, (controller_command_t){.commandType=INIT, .message_id=i+1, .properties.init={.protocol_major=2}});
        break;
    case 1:
        if (!ready[0] || !ready[1]) return;
        issue(stale_bus, (controller_command_t){.commandType=INITIALIZE_MOTOR, .message_id=3,
            .properties.initialize_motor={3, 0}});
        break;
    case 2:
        if (replies[3] != INITIALIZE_MOTOR) return;
        issue(stale_bus, speed(4, 40));
        break;
    case 3:
        if (replies[4] != SET_MOTOR_SPEED) return;
        transition.message_id = 5;
        if (board) transition.properties.set_board_idle.idle = 1;
        else transition.properties.set_driver_sleep.sleeping = 1;
        issue(other, transition);
        break;
    case 4: {
        if (replies[5] != transition.commandType) return;
        assert(hw_motor_drivers_sleeping());
        blocked = true;
        issue(stale_bus, (controller_command_t){.commandType=GET_DRIVER_SLEEP, .message_id=6});
        issue(stale_bus, speed(7, 50));
        if (board) {
            issue(stale_bus, (controller_command_t){.commandType=SET_DRIVER_SLEEP, .message_id=11,
                .properties.set_driver_sleep={0}});
            issue(stale_bus, (controller_command_t){.commandType=SUBSCRIBE, .message_id=12,
                .properties.subscribe={TOPIC_TEMPERATURE,0,250}});
        }
        controller_command_t stale = speed(8, 60);
        stale.commandType = stale_command;
        if (!stale_bus) {
            partial_size = encode(stale, partial);
            assert(enqueue_multi(queue(0), partial, 2));
        } else issue(stale_bus, stale);
        break;
    }
    case 5:
        if (!attempted) return;
        transition.message_id = 9;
        issue(other, transition);
        break;
    case 6:
        if (replies[9] != transition.commandType) return;
        assert(!hw_motor_drivers_sleeping());
        blocked = false;
        if (!stale_bus) assert(enqueue_multi(queue(0), partial + 2, partial_size - 2));
        issue(stale_bus, speed(10, 70));
        break;
    case 7:
        if (!replies[10]) return;
        assert(replies[6] == GET_DRIVER_SLEEP);
        assert(errors[7] == RESPONSE_DRIVERS_ASLEEP && errors[8] == RESPONSE_DRIVERS_ASLEEP);
        if (board) assert(errors[11] == RESPONSE_BOARD_IDLE && errors[12] == RESPONSE_BOARD_IDLE);
        assert(replies[10] == SET_MOTOR_SPEED);
        if (board) longjmp(finished, 1);
        issue(other, (controller_command_t){.commandType=SET_DRIVER_SLEEP, .message_id=11,
            .properties.set_driver_sleep={1}});
        break;
    case 8: {
        if (replies[11] != SET_DRIVER_SLEEP) return;
        blocked = true;
        attempted = false;
        issue(stale_bus, (controller_command_t){.commandType=GET_DRIVER_SLEEP, .message_id=12});
        controller_command_t delayed = {.commandType=SET_DRIVER_SLEEP, .message_id=13, .properties.set_driver_sleep={0}};
        if (stale_bus) issue(stale_bus, delayed);
        else {
            partial_size = encode(delayed, partial);
            assert(enqueue_multi(queue(0), partial, 2));
        }
        break;
    }
    case 9:
        if (!attempted) return;
        transition.message_id = 14;
        issue(other, transition); // Old wake still queued on the other transport.
        break;
    case 10:
        if (replies[14] != SET_DRIVER_SLEEP) return;
        assert(!hw_motor_drivers_sleeping());
        transition.message_id = 15;
        transition.properties.set_driver_sleep.sleeping = 1;
        issue(other, transition);
        break;
    case 11:
        if (replies[15] != SET_DRIVER_SLEEP) return;
        assert(hw_motor_drivers_sleeping());
        blocked = false;
        if (!stale_bus) assert(enqueue_multi(queue(0), partial + 2, partial_size - 2));
        break;
    case 12:
        if (!replies[13]) return;
        assert(errors[13] == RESPONSE_DRIVERS_ASLEEP && hw_motor_drivers_sleeping());
        transition.message_id = 16;
        issue(other, transition);
        break;
    case 13:
        if (replies[16] != SET_DRIVER_SLEEP) return;
        issue(stale_bus, speed(17, 70));
        break;
    case 14:
        if (!replies[17]) return;
        assert(replies[17] == SET_MOTOR_SPEED);
        longjmp(finished, 1);
    }
    phase++;
}
const os_interface_t *get_os_interface(void) {
    static const os_interface_t os = {.delay_ms=next_turn};
    return &os;
}
void CommandHandlerTask(void *);
void run_sleep_queues(unsigned blocked_bus, bool board_idle, uint8_t command) {
    stale_command = command;
    board = board_idle;
    stale_bus = blocked_bus;
    if (!setjmp(finished)) CommandHandlerTask(NULL);
}

void run_usb_idle_lifecycle(void) {
    usb_lifecycle = true;
    if (!setjmp(finished)) CommandHandlerTask(NULL);
}

#if BOARD_HAS_FEEDBACK_SOURCE_HEALTH
void run_counter_queues(unsigned bus) {
    stale_bus=bus;counter_outage=true;
    if (!setjmp(finished)) CommandHandlerTask(NULL);
}
#endif
