//------------------------------------------------------------
// File name: controllers_manager.c
//------------------------------------------------------------
#include "controllers_manager.h"
#include <cmsis_os.h>
#include <hw_config.h>
#include <utils.h>
#include <stdlib.h>
#include <math.h>

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

    const TickType_t xFrequency = pdMS_TO_TICKS(controllers_manager_input->update_interval_ms);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    unsigned int seq = 0;
    unsigned int previousEncoderValue[4] = { 0 };

    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        unsigned int seq_update = 0;

        // Aquire controller state mutex before updating controller state
        if (xSemaphoreTake(controllers_manager_input->controller_state_mutex, portMAX_DELAY)) {

            // Update controller state for all controllers
            for (int index = 0; index < NUMBER_MOTORS; index++)
            {
                controller_info_t* controller_info = &controllers_manager_input->ControllerInfo[index];

                // Check if controller is running
                if(controller_info->state == RUN)
                {
                    pid_controller_t* controller = &controller_info->controller;

                    // Get current velocity from encoder
                    const unsigned int current_encoder_value = get_encoder_value(index);

                    // Calculate change in encoder ticks and update previous encoder value
                    int last_encoder_change = current_encoder_value - previousEncoderValue[index];
                    previousEncoderValue[index] = current_encoder_value;

                    // Caltulate current motor speed in radians per second from encoder ticks
                    double current_motor_speed = 2.0 * M_PI * ((double)last_encoder_change / controller->encoder_resolution) * (1.0 / controller->T);

                    // Calculate new velocity for motor
                    pid_controller_update(
                        &controllers_manager_input->ControllerInfo[index].controller,
                        current_motor_speed,
                        controllers_manager_input->TargetMotorSpeed[index]);
                }
            }

            // Release controller state mutex
            xSemaphoreGive(controllers_manager_input->controller_state_mutex);
        }
        seq += seq_update;

        // Set motor speed
        for (int index = 0; index < NUMBER_MOTORS; index++)
        {
            if( controllers_manager_input->ControllerInfo[index].state == RUN )
            {
            // Set motor speed
            set_motor_speed(
                controllers_manager_input->ControllerInfo[index].mIndex,
                controllers_manager_input->ControllerInfo[index].controller.motorPWM);
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
                stop_motor(controllers_manager_input->ControllerInfo[index].mIndex);
            }
        }
    }
}

void controllers_manager_init(controllers_manager_t* controllers_manager, controllers_manager_input_t* controllers_manager_input)
{
    controllers_manager_input->controller_state_mutex = xSemaphoreCreateMutex();
    if (controllers_manager_input->controller_state_mutex == NULL) {
        // Handle error: Failed to create the mutex
    }

    controllers_manager->threadHandler = osThreadNew(StartControllerTask, controllers_manager_input, &ControllerTask_attributes);
}

int controllers_manager_is_not_init(const controllers_manager_t* controllers_manager)
{
    osThreadState_t status = osThreadGetState(controllers_manager->threadHandler);
    return status == osThreadError;
}
