//------------------------------------------------------------
// File name: commands_manager.c
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

// Change length of messages that command queue can store.
// Max command + message lenght byte.
#define MESSAGE_QUEUE_MAX_STR_LENGTH sizeof(controller_command_t) + 1

typedef struct commands_manager
{
    thread_handle_t threadHandler;
} commands_manager_t;

static commands_manager_t commands_manager = {
    .threadHandler = NULL
};

void CommandHandlerTask(void *argument);

void CommandHandlerTask(void *argument)
{
    static_assert(
        sizeof(controller_command_t) + 1 == MESSAGE_QUEUE_MAX_STR_LENGTH,
        "Size of command queue less them motor command size.");
    // Initialize queues
    init_queue(&CommandQueue);
    init_queue(&I2CCommandQueue);

    // Initialize USB and I2C interfaces
    MX_USB_DEVICE_Init();
    initialize_external_i2c();

    const os_interface_t* os = get_os_interface();

    while(1)
    {
        // Handle commands from USB interface
        if(!is_queue_empty(&CommandQueue)) 
        {
            char commandBuffer[MESSAGE_QUEUE_MAX_STR_LENGTH];
            int data_len;
            dequeue(&CommandQueue, commandBuffer, &data_len);
            command_handler((controller_command_t*)(&commandBuffer[0]), command_callback_usb);
        }

        // Handle commands from I2C interface
        if(!is_queue_empty(&I2CCommandQueue)) 
        {
            char commandBuffer[MESSAGE_QUEUE_MAX_STR_LENGTH];
            int data_len;
            dequeue(&I2CCommandQueue, commandBuffer, &data_len);
            command_handler((controller_command_t*)(&commandBuffer[0]), command_callback_i2c);
        }
        
        // Small delay
        os->delay_ms(1);
    }
}

void commands_manager_start(void)
{
    const os_interface_t* os = get_os_interface();
    commands_manager.threadHandler = os->create_thread(CommandHandlerTask, "CommandsTask", NULL);
}

// The USB endpoint stays busy while the host is not draining it (or after an
// unplug), so the retry loop is bounded. Without the timeout the single
// CommandHandlerTask would block forever and starve the I2C command path too.
#define USB_TRANSMIT_TIMEOUT_MS 1000U

void command_callback_usb(uint8_t* resonse, uint8_t data_len)
{
    const os_interface_t* os = get_os_interface();
    unsigned int timeout = USB_TRANSMIT_TIMEOUT_MS;
    while (CDC_Transmit_FS(resonse, data_len) == USBD_BUSY)
    {
        if (timeout-- == 0)
        {
            // Host is not reading; drop the response instead of hanging.
            return;
        }
        os->delay_ms(1);
    }
}

void command_callback_i2c(uint8_t* resonse, uint8_t data_len)
{
    // Send response to I2C interface
    send_external_i2c(resonse, data_len);
}