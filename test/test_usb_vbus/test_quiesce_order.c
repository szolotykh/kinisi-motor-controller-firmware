// Include the real task implementation so safety assertions can inspect its
// private session state. Hardware/RTOS boundaries are host fakes.
#include "../../src/commands_manager.c"

#if BOARD_HAS_FUSION_EXPERIMENT
void platform_fusion_install_ingress_fence(void (*fence)(uint32_t *)) { (void)fence; }
void platform_fusion_dequeued(void) {}
#endif
message_queue_t CommandQueue, I2CCommandQueue;
unsigned losses;
static uint32_t global_mask;
static double output[4];
static bool controller_running[4], platform_running;
static uint8_t i2c_reply[255], i2c_reply_length;

uint32_t hw_motor_fault_event_sequence(uint8_t index) { (void)index; return 0; }
uint8_t CDC_Transmit_FS(uint8_t *data, uint16_t length) { (void)data; (void)length; return 1; }
uint8_t try_send_external_i2c(uint8_t *data, uint16_t length) {
    assert(length <= sizeof(i2c_reply));
    memcpy(i2c_reply, data, length); i2c_reply_length = (uint8_t)length; return 1;
}
bool hw_motor_drivers_sleeping(void) { return false; }
void mark_pending_external_i2c_requests(uint8_t barriers) { mark_pending_frames(&I2CCommandQueue, barriers); }
void controller_configuration_restore(void) {}
void initialize_external_i2c(void) {}
const os_interface_t *get_os_interface(void) { assert(!"task loop not used in this harness"); return NULL; }
uint64_t hw_clock_microseconds(void) { return 1000; }
uint32_t odometry_manager_get_period_ms(void) { return 10; }
uint8_t command_handler(controller_command_t *cmd, void (*reply)(uint8_t *, uint8_t)) {
    if (cmd->commandType == INIT) {
        uint8_t error = initialization_validate(cmd);
        if (error) return error;
        init_response response = initialization_response();
        reply((uint8_t *)&response, sizeof(response)); return RESPONSE_OK;
    }
    assert(cmd->commandType == GET_DRIVER_SLEEP);
    uint8_t sleeping = 0; reply(&sleeping, 1); return RESPONSE_OK;
}

uint32_t __get_PRIMASK(void) { return global_mask; }
void __disable_irq(void) { global_mask = 1; }
void __set_PRIMASK(uint32_t mask) { global_mask = mask; }
void platform_stop_velocity_controller(void) { platform_running = false; ++losses; }
void controllers_manager_stop_controller(uint8_t index) { controller_running[index] = false; }
static uint8_t initialized(uint8_t index) { (void)index; return 1; }
static void coast(uint8_t index) {
    assert(!platform_running && !controller_running[index]);
    output[index] = 0;
}
const hw_motor_interface_t *get_motor_interface(void) {
    static const hw_motor_interface_t motor = {.is_initialized=initialized, .stop=coast};
    return &motor;
}
#if BOARD_HAS_FUSION_EXPERIMENT
static void check_recovery_fence(void)
{
    char frame[] = {3, SET_PLATFORM_POSITION, 1, 0}, output_frame[256];
    unsigned generation = 0;
    uint8_t barriers;
    int length;
    init_queue(&CommandQueue); init_queue(&I2CCommandQueue);
    assert(enqueue_multi(&CommandQueue, frame, 2) && enqueue(&I2CCommandQueue, frame));
    fence_feedback_targets(&generation);
    assert(generation == 1 && enqueue_multi(&CommandQueue, frame + 2, 2));
    assert(take_command(&CommandQueue, output_frame, &length, &barriers) &&
           barriers == BEFORE_FEEDBACK_READY);
    assert(take_command(&I2CCommandQueue, output_frame, &length, &barriers) &&
           barriers == BEFORE_FEEDBACK_READY);
    assert(enqueue(&I2CCommandQueue, frame) &&
           take_command(&I2CCommandQueue, output_frame, &length, &barriers) && barriers == 0);
}
#endif
void test_prepare_running(void) {
#if BOARD_HAS_FUSION_EXPERIMENT
    check_recovery_fence();
#endif
    init_queue(&CommandQueue); init_queue(&I2CCommandQueue);
    char frame[] = {1, 17};
    assert(enqueue(&CommandQueue, frame) && enqueue(&I2CCommandQueue, frame));
    CommandQueue.incomplete_count = I2CCommandQueue.incomplete_count = 1;
    connection_init(&usb_connection, command_handler, try_usb, hw_clock_microseconds);
    connection_init(&i2c_connection, command_handler, try_i2c, hw_clock_microseconds);
    usb_connection.clock.ready = i2c_connection.clock.ready = true;
    usb_connection.output_length = i2c_connection.output_length = 4;
    for (unsigned i = 0; i < 4; ++i) { output[i] = 50; controller_running[i] = true; }
    platform_running = true;
}

static void i2c_request(controller_command_t cmd) {
    char frame[256];
    frame[0] = (char)command_request_size(cmd.commandType);
    memcpy(frame + 1, &cmd, (uint8_t)frame[0]);
    assert(enqueue(&I2CCommandQueue, frame));
    i2c_reply_length = 0;
    process_queue(&i2c_connection, &I2CCommandQueue);
    connection_poll(&i2c_connection);
    assert(i2c_reply_length >= 4);
}
void test_i2c_recover(void) {
    i2c_request((controller_command_t){.commandType=GET_DRIVER_SLEEP, .message_id=1});
    assert(i2c_reply[1] == KINISI_MESSAGE_ERROR && i2c_reply[5] == RESPONSE_INIT_REQUIRED);
    i2c_request((controller_command_t){.commandType=INIT, .message_id=2,
        .properties.init={.protocol_major=2, .protocol_minor=17}});
    assert(i2c_reply[1] == INIT);
    connection_poll(&i2c_connection);
    assert(i2c_reply[1] == KINISI_MESSAGE_READY && i2c_connection.clock.ready && !i2c_connection.lost);
    i2c_request((controller_command_t){.commandType=GET_DRIVER_SLEEP, .message_id=3});
    assert(i2c_reply[1] == GET_DRIVER_SLEEP && i2c_reply_length == 5 && !i2c_reply[4]);
    assert(!platform_running);
    for (unsigned i = 0; i < 4; ++i) assert(!output[i] && !controller_running[i]);
}
void test_assert_running(void) {
    assert(platform_running && usb_connection.clock.ready && i2c_connection.clock.ready);
    for (unsigned i = 0; i < 4; ++i) assert(output[i] == 50 && controller_running[i]);
}
void test_assert_quiesced(void) {
    assert(!global_mask && !platform_running);
    for (unsigned i = 0; i < 4; ++i) assert(!output[i] && !controller_running[i]);
    assert(!CommandQueue.count && !I2CCommandQueue.count);
    assert(!CommandQueue.incomplete_count && !I2CCommandQueue.incomplete_count);
    assert(usb_connection.lost && i2c_connection.lost);
    assert(!usb_connection.clock.ready && !i2c_connection.clock.ready);
    assert(!usb_connection.output_length && !i2c_connection.output_length);
}
