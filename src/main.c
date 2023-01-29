#include "stm32f4xx.h"
#include "hw_gpio.h"
#include "hw_motor.h"
#include "usb_device.h"
#include "hw_encoder.h"
#include "commands.h"
#include "main.h"
#include "platform.h"
#include "utils.h"
#include "hw_config.h"

#include "cmsis_os.h"
#include <FreeRTOS.h>
#include "task.h"
#include <semphr.h>
#include <stdio.h>

#include <controller.h>



#define ENCDER_EVENT_PER_REV 1557.21f
#define MAX_RPM 92.8f
#define MAX_RPS MAX_RPM/60.0f
#define MAX_TICKS_PER_SEC MAX_RPS*ENCDER_EVENT_PER_REV
// 116*4/5

#define CONTROLLER_UPDATE_INTERVAL 200
#define ENCODER_CAPTURE_INTERVAL 100


#define HAL_PCD_MODULE_ENABLED
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

void OTG_FS_IRQHandler(void);

char commandBuffer[64];

mecanum_velocity_t TargetVelocity = { .motor0 = 0, .motor1 = 0, .motor2 = 0, .motor3 = 0};

static SemaphoreHandle_t mutex;


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

    mutex = xSemaphoreCreateMutex();
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

            case MOTOR_SET_CONTROLLER:
                // SetMotorController 
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

            case PLATFORM_SET_CONTROLLER:
                // SetController
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

    const TickType_t xFrequency = pdMS_TO_TICKS(ENCODER_CAPTURE_INTERVAL);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        xSemaphoreTake(mutex, portMAX_DELAY);
        encoder_update_state(ENCODER0, get_encoder_value(ENCODER0));
        encoder_update_state(ENCODER1, get_encoder_value(ENCODER1));
        encoder_update_state(ENCODER2, get_encoder_value(ENCODER2));
        encoder_update_state(ENCODER3, get_encoder_value(ENCODER3));
        xSemaphoreGive(mutex);
    }
}

void StartControllerTask(void *argument)
{
    /*
    MX_USB_DEVICE_Init();
    int seq1 = 0;
    const TickType_t xFrequency1 = pdMS_TO_TICKS(200);
    TickType_t xLastWakeTime1 = xTaskGetTickCount();
    int motorToMonitor = MOTOR3;

    initialize_motor(motorToMonitor);
    set_motor_speed(motorToMonitor, 1, 200);

    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime1, xFrequency1);
        xSemaphoreTake(mutex, portMAX_DELAY);
        encoder_get_velocity(motorToMonitor),
        xSemaphoreGive(mutex);
        print_controller_state(seq1, encoder_get_velocity(motorToMonitor), 0);
        seq1++;
        //xSemaphoreGive(mutex);
    }

    return;
    */
    // ------------------------------------------------
    TargetVelocity.motor0 = 0;
    TargetVelocity.motor1 = 0;
    TargetVelocity.motor2 = 0;
    TargetVelocity.motor3 = 20;

    double max_encoder_velocity_per_capture = MAX_TICKS_PER_SEC * ENCODER_CAPTURE_INTERVAL / 1000;

    double kp = 0.1f;
    double ki = 0.01f; // 0.05f;
    double kd = 0.1f;
    Controller motorControllers[NUMBER_MOTORS] = {
        init_pid_controller(kp, ki, kd),
        init_pid_controller(kp, ki, kd),
        init_pid_controller(kp, ki, kd),
        init_pid_controller(kp, ki, kd),
    };
    motorControllers[3].motorPWM = 190;

    // Initialize all motors
    initialize_motor_all();

    const TickType_t xFrequency = pdMS_TO_TICKS(CONTROLLER_UPDATE_INTERVAL);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    unsigned int seq = 0;

    //set_motor_speed(3, 240, 1);
    //osDelay(2);
    //set_motor_speed(0, 40, 1);
    

    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Get current velocity from encoder
        xSemaphoreTake(mutex, portMAX_DELAY);
        int currentVelocity[NUMBER_MOTORS] = {
            encoder_get_velocity(ENCODER0),
            encoder_get_velocity(ENCODER1),
            encoder_get_velocity(ENCODER2),
            encoder_get_velocity(ENCODER3),
        };
        xSemaphoreGive(mutex);

        // Convert input velocity from procent to ticks per encoder caputre interval
        double targetEncoderVelocity[NUMBER_MOTORS] = {
            TargetVelocity.motor0 * max_encoder_velocity_per_capture / 100,
            TargetVelocity.motor1 * max_encoder_velocity_per_capture / 100,
            TargetVelocity.motor2 * max_encoder_velocity_per_capture / 100,
            TargetVelocity.motor3 * max_encoder_velocity_per_capture / 100
        };

        int motorPWM[NUMBER_MOTORS];
        // Calculate new velocity for motors
        for(int i = 0; i < NUMBER_MOTORS; i++)
        {
            motorPWM[i] = update_pid_controller(&motorControllers[i], currentVelocity[i], targetEncoderVelocity[i]);
        }

        // Set motors speed and direction
        for(int i = 0; i < NUMBER_MOTORS; i++)
        {
            set_motor_speed(i, motorPWM[i] > 0, abs(motorPWM[i]));
        }

        //set_motor_speed(3, 35, 1);

        int motorToMonitor = MOTOR3;
        print_controller_state(seq, currentVelocity[motorToMonitor], (int)motorPWM[motorToMonitor]);
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