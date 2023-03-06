//------------------------------------------------------------
// File name: controllers_manager.c
//------------------------------------------------------------
#include "controllers_manager.h"
#include <cmsis_os.h>
#include <hw_config.h>
#include <utils.h>
#include <stdlib.h>

#define CONTROLLER_UPDATE_INTEVRAL 100

#define ENCDER_EVENT_PER_REV 1557.21f
#define MAX_RPM 92.8f
#define MAX_RPS MAX_RPM/60.0f
#define MAX_TICKS_PER_SEC MAX_RPS*ENCDER_EVENT_PER_REV
// 116*4/5

const osThreadAttr_t ControllerTask_attributes = {
  .name = "ControllerTask",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityNormal,
};

void StartControllerTask(void *argument)
{
    /*
    TODO: Add option to send initial short signal to a motor to move it it from static position.
    Like this:
    set_motor_speed(3, 240, 1);
    osDelay(2);
    set_motor_speed(0, 40, 1);
    */
    controllers_manager_input_t* controllers_manager_input = (controllers_manager_input_t*)argument;

    double max_encoder_velocity_per_capture = MAX_TICKS_PER_SEC * CONTROLLER_UPDATE_INTEVRAL / 1000;
    const TickType_t xFrequency = pdMS_TO_TICKS(CONTROLLER_UPDATE_INTEVRAL);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    unsigned int seq = 0;
    unsigned int previousEncoderValue[4] = { 0 };

    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        unsigned int seq_update = 0;
        for (int index = 0; index < NUMBER_MOTORS; index++)
        {
            if( controllers_manager_input->ControllerInfo[index].state == RUN )
            {
                // Get current velocity from encoder
                unsigned int currentEncoderValue = get_encoder_value(index);

                int currentVelocity = currentEncoderValue - previousEncoderValue[index];
                previousEncoderValue[index] = currentEncoderValue;

                // Convert input velocity from procent to ticks per encoder caputre interval
                double targetEncoderVelocity = controllers_manager_input->TargetVelocity[index] * max_encoder_velocity_per_capture / 100.0;

                // Calculate new velocity for motor
                pid_controller_update(&controllers_manager_input->ControllerInfo[index].controller, currentVelocity, targetEncoderVelocity);

                print_controller_state(
                    seq,
                    index, 
                    currentVelocity, 
                    targetEncoderVelocity,
                    controllers_manager_input->ControllerInfo[index].controller.motorPWM
                    );

                seq_update = 1;
            }
        }
        seq += seq_update;

        // Set motor speed
        for (int index = 0; index < NUMBER_MOTORS; index++)
        {
            if( controllers_manager_input->ControllerInfo[index].state == RUN )
            {
            // Set motor speed and direction
            unsigned short direction = controllers_manager_input->ControllerInfo[index].controller.motorPWM > 0;
            unsigned int speed = abs(controllers_manager_input->ControllerInfo[index].controller.motorPWM);
            set_motor_speed(controllers_manager_input->ControllerInfo[index].mIndex, direction, speed);
            }
        }

        // Stopping controllers and motors
        for (int index = 0; index < NUMBER_MOTORS; index++)
        {
            if( controllers_manager_input->ControllerInfo[index].state == STOPPING )
            {
                // Stop controller
                controllers_manager_input->ControllerInfo[index].state = STOP;
                controllers_manager_input->ControllerInfo[index].controller.motorPWM = 0;
                //Stop motor
                set_motor_speed(controllers_manager_input->ControllerInfo[index].mIndex, 0, 0);
            }
        }
    }
}

void controllers_manager_init(controllers_manager_t* controllers_manager, controllers_manager_input_t* controllers_manager_input)
{
    controllers_manager->threadHandler = osThreadNew(StartControllerTask, controllers_manager_input, &ControllerTask_attributes);
}

int controllers_manager_is_not_init(const controllers_manager_t* controllers_manager)
{
    osThreadState_t status = osThreadGetState(controllers_manager->threadHandler);
    return status == osThreadError;
}
