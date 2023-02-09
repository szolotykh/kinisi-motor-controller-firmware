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

#include <cmsis_os.h>
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

double TargetVelocity[4] = { 0 };
Controller* motor_controllers[NUMBER_MOTORS];

static SemaphoreHandle_t mutex;


osThreadId_t CommandsTaskHandle;
const osThreadAttr_t CommandsTask_attributes = {
  .name = "CommandsTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ControllerTask */
osThreadId_t ControllerTaskHandle = NULL;
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

typedef struct controller_info_t
{
    char state;
    motorIndex mIndex;
    encoder_index_t eIndex;
    Controller controller;
} controller_info_t;

controller_info_t ControllerInfo[NUMBER_MOTORS] = { 
    {.state = 0},
    {.state = 0},
    {.state = 0},
    {.state = 0},
 };

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
    //EncoderTaskHandle = osThreadNew(EncoderTaskFunction, NULL, &EncoderTask_attributes);

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
                signed short speed = commandBuffer[3] | commandBuffer[4] << 8;
                signed short direction = commandBuffer[2];
                set_motor_speed(commandBuffer[1], direction, speed);
                }
            break;

            case MOTOR_SET_CONTROLLER:
                {
                char motorIndex = commandBuffer[1];
                if (ControllerTaskHandle == NULL)
                {
                    ControllerTaskHandle = osThreadNew(StartControllerTask, NULL, &ControllerTask_attributes);
                }

                // SetMotorController 
                Controller controller;
                controller.kp = decode_double(commandBuffer + 2);
                controller.ki = decode_double(commandBuffer + 10);
                controller.kd = decode_double(commandBuffer + 18);

                controller.motorPWM = 0;

                controller_info_t controllerInfo;
                controllerInfo.state = 1;
                controllerInfo.controller = controller;
                controllerInfo.mIndex = motorIndex;
                controllerInfo.eIndex = motorIndex;

                initialize_motor(controllerInfo.mIndex);
                initialize_encoder(controllerInfo.eIndex);

                ControllerInfo[motorIndex] = controllerInfo;
                }
            break;

            case MOTOR_DEL_CONTROLLER:
                {
                ControllerInfo[commandBuffer[1]].state = 10; // STOPPING
                }
            break;

            case MOTOR_SET_TARGET_VELOCITY:
                {
                signed short speed = commandBuffer[3] | commandBuffer[4] << 8;
                signed short direction = commandBuffer[2];
                TargetVelocity[commandBuffer[1]] = ((direction * 2) - 1) * speed;
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

            case PLATFORM_SET_CONTROLLER:
                // SetController
            break;

            case PLATFORM_SET_VELOCITY_INPUT:
                {
                mecanum_velocity_t mecanumVelocity = get_mecanum_velocities(commandBuffer[1], commandBuffer[2], commandBuffer[3]);
                TargetVelocity[MOTOR0] = mecanumVelocity.motor0;
                TargetVelocity[MOTOR1] = mecanumVelocity.motor1;
                TargetVelocity[MOTOR2] = mecanumVelocity.motor2;
                TargetVelocity[MOTOR3] = mecanumVelocity.motor3;
                }
            break;
            }
            // Reset command buffer
            memset(commandBuffer, '\0', 64);
        }
        osDelay(1);
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
    TODO: Add option to send initial short signal to a motor to move it it from static position.
    Like this:
    set_motor_speed(3, 240, 1);
    osDelay(2);
    set_motor_speed(0, 40, 1);
    */

    double max_encoder_velocity_per_capture = MAX_TICKS_PER_SEC * CONTROLLER_UPDATE_INTERVAL / 1000;
    const TickType_t xFrequency = pdMS_TO_TICKS(CONTROLLER_UPDATE_INTERVAL);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    unsigned int seq = 0;
    unsigned int previousEncoderValue[NUMBER_MOTORS] = { 0 };
    uint32_t flags;

    for(;;)
    {
        for (int index = 0; index < NUMBER_MOTORS; index++)
        {
            
            if( ControllerInfo[index].state == 1 )
            {
            vTaskDelayUntil(&xLastWakeTime, xFrequency);

            // Get current velocity from encoder
            xSemaphoreTake(mutex, portMAX_DELAY);
            unsigned int currentEncoderValue = get_encoder_value(index);
            xSemaphoreGive(mutex);

            unsigned int currentVelocity = currentEncoderValue - previousEncoderValue[index];
            previousEncoderValue[index] = currentEncoderValue;

            // Convert input velocity from procent to ticks per encoder caputre interval
            double targetEncoderVelocity = TargetVelocity[index] * max_encoder_velocity_per_capture / 100.0;

            // Calculate new velocity for motor
            update_pid_controller(&ControllerInfo[index].controller, currentVelocity, targetEncoderVelocity);
            }
        }

        for (int index = 0; index < NUMBER_MOTORS; index++)
        {
            if( ControllerInfo[index].state == 1 )
            {
            // Set motor speed and direction
            unsigned short direction = ControllerInfo[index].controller.motorPWM > 0;
            unsigned int speed = abs(ControllerInfo[index].controller.motorPWM);
            set_motor_speed(ControllerInfo[index].mIndex, direction, speed);
            }
        }

        // Stopping controllers and motors
        for (int index = 0; index < NUMBER_MOTORS; index++)
        {
            if( ControllerInfo[index].state == 10 )
            {
                // Stop controller
                ControllerInfo[index].state = 0;
                ControllerInfo[index].controller.motorPWM = 0;
                //Stop motor
                set_motor_speed(ControllerInfo[index].mIndex, 0, 0);
            }
        }

        /*
        int motorToMonitor = MOTOR0;
        print_controller_state(seq, currentVelocity, motorPWM);
        
        seq++;
        flags = osThreadFlagsGet();
        if(flags == 0x7){
            break;
        }
        */
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