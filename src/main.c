#include "stm32f4xx.h"
#include "hw_gpio.h"
#include "hw_motor.h"
#include "usb_device.h"
#include "hw_encoder.h"
#include "commands.h"
#include "main.h"
#include "platform.h"
#include "utils.h"

#include "cmsis_os.h"
#include <FreeRTOS.h>
#include "task.h"
#include <stdio.h>

#include <controller.h>

#define HAL_PCD_MODULE_ENABLED
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

void OTG_FS_IRQHandler(void);

char commandBuffer[64];

mecanum_velocity_t CurrentVelocity = { .motor0 = 0, .motor1 = 0, .motor2 = 0, .motor3 = 0};
mecanum_velocity_t TargetVelocity = { .motor0 = 0, .motor1 = 0, .motor2 = 0, .motor3 = 0};


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
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "CommandsTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for CommandsQueue01 */
osMessageQueueId_t CommandsQueue01Handle;
const osMessageQueueAttr_t CommandsQueue01_attributes = {
  .name = "CommandsQueue01"
};

void StartCommandTask(void *argument);
void StartControllerTask(void *argument);
void EncoderTaskFunction(void *argument);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    initialize_gpio();

    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);

    osKernelInitialize();
    CommandsQueue01Handle = osMessageQueueNew (16, sizeof(uint16_t), &CommandsQueue01_attributes);

    CommandsTaskHandle = osThreadNew(StartCommandTask, NULL, &CommandsTask_attributes);
    EncoderTaskHandle = osThreadNew(EncoderTaskFunction, NULL, &EncoderTask_attributes);
    ControllerTaskHandle = osThreadNew(StartControllerTask, NULL, &ControllerTask_attributes);

    osKernelStart();
}

void StartCommandTask(void *argument)
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
                taskENTER_CRITICAL();
                TargetVelocity = get_mecanum_velocities(commandBuffer[1], commandBuffer[2], commandBuffer[3]);
                taskEXIT_CRITICAL();
            break;
            }

            // Reset command buffer
            memset(commandBuffer, '\0', 64);
        }
    }
}

void EncoderTaskFunction(void *argument)
{
    initialize_encoder(ENCODER0);
    initialize_encoder(ENCODER1);
    initialize_encoder(ENCODER2);
    initialize_encoder(ENCODER3);

    const TickType_t xFrequency = pdMS_TO_TICKS(50);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        taskENTER_CRITICAL();
        encoder_update_state(ENCODER0, get_encoder_value(ENCODER0));
        encoder_update_state(ENCODER1, get_encoder_value(ENCODER1));
        encoder_update_state(ENCODER2, get_encoder_value(ENCODER2));
        encoder_update_state(ENCODER3, get_encoder_value(ENCODER3));
        taskEXIT_CRITICAL();
    }
}

void StartControllerTask(void *argument)
{
    unsigned int pwm_limit = 840;

    int targetVelocity = 20; // ticks per 50 ms
    double currentSpeed = 0;
    double kp = 1;
    double ki = 0.05 ;
    double ierror = 0;
    // Max ticks per sec 2,408.4848
    initialize_motor(0);
    const TickType_t xFrequency = pdMS_TO_TICKS(200);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    unsigned int seq = 0;
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        taskENTER_CRITICAL();
        unsigned int value = encoder_get_value(0);
        int velocity = encoder_get_velocity(0);
        taskEXIT_CRITICAL();

        double error = targetVelocity - velocity;
        ierror += error;

        currentSpeed = currentSpeed + error*kp + ierror*ki;

        if(currentSpeed > pwm_limit){
            currentSpeed = pwm_limit;
        }else if(currentSpeed < -pwm_limit){
            currentSpeed = -pwm_limit;
        }

        set_motor_speed(0, currentSpeed > 0, abs(currentSpeed));

        print_controller_state(seq, velocity, currentSpeed);
        HAL_GPIO_TogglePin (GPIOC, GPIO_PIN_12);
        seq++;
    }
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