#include "stm32f4xx.h"
#include "hw_gpio.h"
#include "hw_motor.h"
#include "usb_device.h"
#include "hw_encoder.h"
#include "commands.h"
#include "main.h"
#include "platform.h"

#include "cmsis_os.h"
#include <FreeRTOS.h>
#include "task.h"
#include <stdio.h>

#define HAL_PCD_MODULE_ENABLED
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

void OTG_FS_IRQHandler(void);

char commandBuffer[64];


osThreadId_t CommandsTaskHandle;
const osThreadAttr_t CommandsTask_attributes = {
  .name = "CommandsTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ControllerTask */
osThreadId_t ControllerTaskHandle;
const osThreadAttr_t ControllerTask_attributes = {
  .name = "ControllerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CommandsQueue01 */
osMessageQueueId_t CommandsQueue01Handle;
const osMessageQueueAttr_t CommandsQueue01_attributes = {
  .name = "CommandsQueue01"
};
/* Definitions for myTimer01 */
osTimerId_t myTimer01Handle;
const osTimerAttr_t myTimer01_attributes = {
  .name = "myTimer01"
};

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void Callback01(void *argument);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    initialize_gpio();

    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

    // Init USB
    // MX_USB_DEVICE_Init();  ???

    osKernelInitialize();
    myTimer01Handle = osTimerNew(Callback01, osTimerPeriodic, NULL, &myTimer01_attributes);
    CommandsQueue01Handle = osMessageQueueNew (16, sizeof(uint16_t), &CommandsQueue01_attributes);
    CommandsTaskHandle = osThreadNew(StartDefaultTask, NULL, &CommandsTask_attributes);
    ControllerTaskHandle = osThreadNew(StartTask02, NULL, &ControllerTask_attributes);

    osKernelStart();
}

void StartDefaultTask(void *argument)
{
    MX_USB_DEVICE_Init();
    while(1)
    {
        if(commandBuffer[0] != '\0')
        {
            switch(commandBuffer[0])
            {
            case INITIALIZE_MOTOR:
                initialize_motor(commandBuffer[1]);
            break;

            case SET_MOTOR_SPEED:
                {
                signed short speed = commandBuffer[3] | commandBuffer[4]<<8;
                signed short direction = commandBuffer[2];
                set_motor_speed(commandBuffer[1], direction, speed);
                }
            break;

            case INITIALIZE_ENCODER:
                initialize_encoder(commandBuffer[1]);
            break;

            case GET_ENCODER_VALUE:
                {
                unsigned int value = get_encoder_value(commandBuffer[1]);
                char buf[4];
                buf[3] = (value >> 24) & 0xFF;
                buf[2] = (value >> 16) & 0xFF;
                buf[1] = (value >> 8) & 0xFF;
                buf[0] = value & 0xFF;
                CDC_Transmit_FS(buf, 4);
                }
            break;

            case STATUS_LED_TOGGLE:
                gpio_toggle_status_led();
            break;

            case PLATFORM_INITIALIZE:
                init_platform();
            break;

            case PLATFORM_SET_VELOCITY_INPUT:
                set_velocity_input(commandBuffer[1], commandBuffer[2], commandBuffer[3]);
            break;
            }

            // Reset command buffer
            memset(commandBuffer, '\0', 64);
        }
    }
}

void StartTask02(void *argument)
{
    for(;;)
    {
        //gpio_toggle_status_led();
        osDelay(500);
    }
}

/* Callback01 function */
void Callback01(void *argument)
{
}

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1) {}
}


void MemManage_Handler(void)
{
    while (1) {}
}

void BusFault_Handler(void)
{
    while (1) {}
}

void UsageFault_Handler(void)
{
    while (1) {}
}

void DebugMon_Handler(void)
{
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
// Interrupt handler for USB
void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}