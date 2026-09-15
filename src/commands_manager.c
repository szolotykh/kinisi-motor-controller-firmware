//------------------------------------------------------------
// File name: commands_manager.c
// Description: Run independent USB/I2C sessions and their periodic clock synchronization.
//------------------------------------------------------------
#include "commands_manager.h"
#include "commands_handler.h"
#include "message_queue.h"
#include "hardware_i2c.h"
#include "os_interface.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <assert.h>
#include <stdlib.h>
#include "initialization.h"
#include "connection.h"
#include "hw_clock.h"
#include "controllers_manager.h"
#include "odometry_manager.h"

extern USBD_HandleTypeDef hUsbDeviceFS;



static connection_t usb_connection, i2c_connection;
static volatile uint8_t usb_disconnected;

/** @brief Coast all motors, stop PID updates and discard queued work on both transports. */
static void stop_on_connection_loss(void)
{
    // Motor resources are shared. Neither transport may replay queued motion after loss.
    platform_stop_velocity_controller();
    const hw_motor_interface_t *motor = get_motor_interface();
    for (uint8_t index = 0; index < 4; ++index) {
        controllers_manager_stop_controller(index);
        if (motor->is_initialized(index)) motor->stop(index);
    }
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    init_queue(&CommandQueue);
    init_queue(&I2CCommandQueue);
    __set_PRIMASK(mask);
    connection_invalidate(&usb_connection);
    connection_invalidate(&i2c_connection);
}

/** @brief Validate a global calculation period against both transport subscriptions. */
static bool period_allowed(uint32_t period_ms)
{
    return connection_period_allowed(&usb_connection, period_ms) &&
        connection_period_allowed(&i2c_connection, period_ms);
}
/**
 * @brief Flag a USB disconnect for deferred session cleanup.
 * @note IRQ-safe notification; the command task stops motors and invalidates sessions.
 */
void commands_manager_usb_disconnected(void) { usb_disconnected = 1; }
/**
 * @brief Try to copy a frame to USB while protecting against concurrent deinitialization.
 * @return True only when the USB stack accepts the transmission.
 */
static bool try_usb(uint8_t *data, uint8_t length)
{
    // DeInit runs in the USB IRQ and frees pClassData; protect the check and send.
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    bool sent = !usb_disconnected && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED &&
        hUsbDeviceFS.pClassData && CDC_Transmit_FS(data, length) == USBD_OK;
    __set_PRIMASK(mask);
    return sent;
}
/**
 * @brief Attempt one nonblocking I2C frame transmission.
 * @return True only after a master read allows the transport to accept the frame.
 */
static bool try_i2c(uint8_t *data, uint8_t length)
{
    return try_send_external_i2c(data, length) != 0;
}

/**
 * @brief Copy and remove one queued frame while excluding IRQ producers.
 * @return True when a frame was dequeued; length excludes its prefix.
 */
static bool take_command(message_queue_t *queue, char *buffer, int *length)
{
    // IRQ producers also update queue->count; dequeue must not race their increment.
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    bool available = !is_queue_empty(queue);
    if (available) dequeue(queue, buffer, length);
    __set_PRIMASK(mask);
    return available;
}

typedef struct commands_manager
{
    thread_handle_t threadHandler;
} commands_manager_t;

static commands_manager_t commands_manager = {
    .threadHandler = NULL
};

/**
 * @brief Poll USB/I2C requests and service their independent clock sessions.
 * @param argument Unused RTOS task argument.
 * @note Runs indefinitely and maintains the monotonic clock rollover extension.
 */
void CommandHandlerTask(void *argument);

/**
 * @brief Poll USB/I2C requests and service their independent clock sessions.
 * @param argument Unused RTOS task argument.
 * @note Runs indefinitely and maintains the monotonic clock rollover extension.
 */
void CommandHandlerTask(void *argument)
{
    static_assert(
        sizeof(controller_command_t) + 1 <= MESSAGE_QUEUE_MAX_STR_LENGTH,
        "Size of command queue less them motor command size.");
    // Initialize queues
    init_queue(&CommandQueue);
    init_queue(&I2CCommandQueue);

    // Initialize USB and I2C interfaces
    MX_USB_DEVICE_Init();
    initialize_external_i2c();

    const os_interface_t* os = get_os_interface();
    connection_init(&usb_connection, command_handler, try_usb, hw_clock_microseconds);
    connection_init(&i2c_connection, command_handler, try_i2c, hw_clock_microseconds);
    const connection_services_t services = {
        .stop = stop_on_connection_loss,
        .calculation_period_ms = odometry_manager_get_period_ms,
        .period_allowed = period_allowed
    };
    usb_connection.services = i2c_connection.services = services;

    while(1)
    {
        // Maintain the HAL millisecond wrap extension even if both peers stop reading.
        (void)hw_clock_microseconds();
        if (usb_disconnected) {
            uint32_t mask = __get_PRIMASK();
            __disable_irq();
            init_queue(&CommandQueue);
            usb_disconnected = 0;
            __set_PRIMASK(mask);
            // Every physical loss stops shared outputs, even if USB had not completed
            // another INIT since the previous loss while I2C resumed operation.
            stop_on_connection_loss();
        }
        // Expire watchdogs before accepting queued commands, including stale motion.
        connection_poll(&usb_connection);
        connection_poll(&i2c_connection);
        // Handle commands from USB interface
        if (connection_can_receive(&usb_connection))
        {
            char commandBuffer[MESSAGE_QUEUE_MAX_STR_LENGTH];
            int data_len;
            if (take_command(&CommandQueue, commandBuffer, &data_len))
                connection_receive(&usb_connection, (const uint8_t *)commandBuffer, data_len);
        }

        // Handle commands from I2C interface
        if (connection_can_receive(&i2c_connection))
        {
            char commandBuffer[MESSAGE_QUEUE_MAX_STR_LENGTH];
            int data_len;
            if (take_command(&I2CCommandQueue, commandBuffer, &data_len))
                connection_receive(&i2c_connection, (const uint8_t *)commandBuffer, data_len);
        }
        
        connection_poll(&usb_connection);
        connection_poll(&i2c_connection);
        // Small delay
        os->delay_ms(1);
    }
}

/**
 * @brief Create the command task using the configured operating-system interface.
 */
void commands_manager_start(void)
{
    const os_interface_t* os = get_os_interface();
    commands_manager.threadHandler = os->create_thread(CommandHandlerTask, "CommandsTask", NULL);
}
