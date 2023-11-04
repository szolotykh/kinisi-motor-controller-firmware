//------------------------------------------------------------
// File name: commands_manager.c
//------------------------------------------------------------
#include "commands_manager.h"
#include "commands_handler.h"
#include "message_queue.h"
#include "hardware_i2c.h"
#include <assert.h>
#include <cmsis_os.h>

// Change length of messages that command queue can store.
// Max command + message lenght byte.
#define MESSAGE_QUEUE_MAX_STR_LENGTH sizeof(controller_command_t) + 1

void CommandHandlerTask(void *argument);

const osThreadAttr_t CommandsTask_attributes = {
  .name = "CommandsTask",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityNormal,
};


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
        osDelay(1);
    }
}

void commands_manager_init(commands_manager_t* commands_manager)
{
    commands_manager->threadHandler = osThreadNew(CommandHandlerTask, NULL, &CommandsTask_attributes);
}

void command_callback_usb(uint8_t* resonse, uint16_t data_len)
{
    CDC_Transmit_FS(resonse, data_len);
}

void command_callback_i2c(uint8_t* resonse, uint16_t data_len)
{
    // Send response to I2C interface
    send_external_i2c(resonse, data_len);
}