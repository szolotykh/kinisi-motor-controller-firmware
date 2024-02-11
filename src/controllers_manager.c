//------------------------------------------------------------
// File name: controllers_manager.c
//------------------------------------------------------------
#include "controllers_manager.h"
#include "commands.h"
#include <cmsis_os.h>
#include <hw_config.h>
#include <utils.h>
#include <stdlib.h>
#include <math.h>
#include <hw_motor.h>

#define PID_CONTROLLER_UPDATE_INTERVAL 100

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
    controllers_manager_state_t* controllers_manager_state = (controllers_manager_state_t*)argument;

    const TickType_t xFrequency = pdMS_TO_TICKS(controllers_manager_state->update_interval_ms);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    unsigned int seq = 0;
    uint16_t previousEncoderValue[4] = { 0 };

    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        unsigned int seq_update = 0;

        // Aquire controller state mutex before updating controller state
        if (xSemaphoreTake(controllers_manager_state->controller_state_mutex, portMAX_DELAY)) {

            // Update controller state for all controllers
            for (int index = 0; index < NUMBER_MOTORS; index++)
            {
                controller_info_t* controller_info = &controllers_manager_state->ControllerInfo[index];

                // Check if controller is running
                if(controller_info->state == RUN)
                {
                    pid_controller_t* controller = &controller_info->controller;

                    // Get current velocity from encoder
                    const uint16_t current_encoder_value = get_encoder_value(index);
                    // Calculate change in encoder value
                    uint16_t raw_change = current_encoder_value - previousEncoderValue[index];

                    // Check for overflow and adjust
                    int last_encoder_change;
                    if (raw_change > 32768) { // Half of UINT16_MAX, detecting large backward movement (underflow)
                        last_encoder_change = -(65536 - raw_change); // Adjust for underflow
                    } else {
                        last_encoder_change = raw_change; // No overflow or underflow
                    }
                    
                    previousEncoderValue[index] = current_encoder_value;

                    // Caltulate current motor speed in radians per second from encoder ticks
                    double current_motor_speed = 2.0 * M_PI * ((double)last_encoder_change / controller->encoder_resolution) * (1.0 / controller->T);

                    // Calculate new velocity for motor
                    pid_controller_update(
                        &controllers_manager_state->ControllerInfo[index].controller,
                        current_motor_speed,
                        controllers_manager_state->TargetMotorSpeed[index]);
                }
            }

            // Set motor speed
            for (int index = 0; index < NUMBER_MOTORS; index++)
            {
                if( controllers_manager_state->ControllerInfo[index].state == RUN )
                {
                // Set motor speed
                set_motor_speed(
                    controllers_manager_state->ControllerInfo[index].mIndex,
                    controllers_manager_state->ControllerInfo[index].controller.motorPWM);
                }
            }

            // Release controller state mutex
            xSemaphoreGive(controllers_manager_state->controller_state_mutex);
        }
    }
}

void controllers_manager_init(controllers_manager_t* controllers_manager)
{
    controllers_manager->state.controller_state_mutex = xSemaphoreCreateMutex();
    if (controllers_manager->state.controller_state_mutex == NULL) {
        // Handle error: Failed to create the mutex
    }

    controllers_manager->threadHandler = osThreadNew(StartControllerTask, &controllers_manager->state, &ControllerTask_attributes);
}

uint8_t controllers_manager_is_not_init(const controllers_manager_t* controllers_manager)
{
    osThreadState_t status = osThreadGetState(controllers_manager->threadHandler);
    return status == osThreadError;
}

void controllers_manager_initialize_controller(controllers_manager_t* controllers_manager, uint8_t motor_index, uint8_t encoder_index, double kp, double ki, double kd, bool is_reversed, double encoder_resolution, double integral_limit)
{
    // Initialize controller manager which starts task for all controllers
    if (controllers_manager_is_not_init(controllers_manager))
    {
        controllers_manager_init(controllers_manager);
    }

    // SetMotorController
    pid_controller_t controller; 
    pid_controller_init(
        &controller,
        ((double)PID_CONTROLLER_UPDATE_INTERVAL)/1000.0, // PID controller update interval in seconds
        kp,
        ki,
        kd,
        is_reversed,
        encoder_resolution,
        integral_limit
    );

    controller_info_t controllerInfo;
    controllerInfo.state = RUN;
    controllerInfo.controller = controller;
    controllerInfo.mIndex = motor_index;
    controllerInfo.eIndex = encoder_index;

    // Initialize motor and encoder if controller is not running
    if (controllers_manager->state.ControllerInfo[motor_index].state == STOP)
    {
        initialize_motor(controllerInfo.mIndex, is_reversed);
        initialize_encoder(controllerInfo.eIndex);
    }

    if (xSemaphoreTake(controllers_manager->state.controller_state_mutex, portMAX_DELAY))
    {
        controllers_manager->state.ControllerInfo[motor_index] = controllerInfo;
        xSemaphoreGive(controllers_manager->state.controller_state_mutex);
    }
}

void controllers_manager_initialize_controller_multiple(controllers_manager_t* controllers_manager, uint8_t motor_selection, double kp, double ki, double kd, double encoder_resolution, double integral_limit)
{
    // Check if all selected motors and encoders are initialized
    for (uint8_t motor_index = 0; motor_index < NUMBER_MOTORS; motor_index++)
    {
        if (motor_selection & (1 << motor_index))
        {
            if (motor_is_initialized(motor_index) == 0 || encoder_is_initialized(motor_index) == 0)
            {
                // TODO: Add return value to indicate error
                return;
            }
        }
    }

    // Initialize controller manager which starts task for all controllers
    if (controllers_manager_is_not_init(controllers_manager))
    {
        controllers_manager_init(controllers_manager);
    }

    if (xSemaphoreTake(controllers_manager->state.controller_state_mutex, portMAX_DELAY))
    {
        for (uint8_t motor_index = 0; motor_index < NUMBER_MOTORS; motor_index++)
        {
            if (motor_selection & (1 << motor_index))
            {
                // Builing PID controller
                pid_controller_t controller; 
                pid_controller_init(
                    &controller,
                    ((double)PID_CONTROLLER_UPDATE_INTERVAL)/1000.0, // PID controller update interval in seconds
                    kp,
                    ki,
                    kd,
                    motor_is_reversed(motor_index),
                    encoder_resolution,
                    integral_limit
                );

                // Building controller info
                controller_info_t controllerInfo;
                controllerInfo.state = RUN;
                controllerInfo.controller = controller;
                controllerInfo.mIndex = motor_index;
                controllerInfo.eIndex = motor_index;

                controllers_manager->state.ControllerInfo[motor_index] = controllerInfo;
            }
        }

    xSemaphoreGive(controllers_manager->state.controller_state_mutex);
    }
}

void controllers_manager_delete_controller(controllers_manager_t* controllers_manager, uint8_t motor_index)
{
    if (xSemaphoreTake(controllers_manager->state.controller_state_mutex, portMAX_DELAY)) {

        controllers_manager->state.ControllerInfo[motor_index].state = STOP;
        controllers_manager->state.ControllerInfo[motor_index].controller = (pid_controller_t){0};
        
        // Stop motor
        stop_motor(controllers_manager->state.ControllerInfo[motor_index].mIndex);

        // Set target speed to zero
        controllers_manager->state.TargetMotorSpeed[motor_index] = 0;

        // Release controller state mutex
        xSemaphoreGive(controllers_manager->state.controller_state_mutex);
    }
}

void controllers_manager_set_target_speed(controllers_manager_t* controllers_manager, uint8_t motor_index, double target_speed)
{
    if (xSemaphoreTake(controllers_manager->state.controller_state_mutex, portMAX_DELAY)) {
        controllers_manager->state.TargetMotorSpeed[motor_index] = target_speed;
        xSemaphoreGive(controllers_manager->state.controller_state_mutex);
    }
}

void controllers_manager_set_target_speed_multiple(controllers_manager_t* controllers_manager, uint8_t* motor_indexes, double* target_speeds, uint8_t motor_count)
{
    if (xSemaphoreTake(controllers_manager->state.controller_state_mutex, portMAX_DELAY)) {
        for (uint8_t i = 0; i < motor_count; i++)
        {
            controllers_manager->state.TargetMotorSpeed[motor_indexes[i]] = target_speeds[i];
        }
        xSemaphoreGive(controllers_manager->state.controller_state_mutex);
    }

}

motor_controller_state controllers_manager_get_motor_controller_state(const controllers_manager_t* controllers_manager, uint8_t motor_index)
{
    const pid_controller_t* controller = &controllers_manager->state.ControllerInfo[motor_index].controller;
    motor_controller_state state = {0};
    if (controllers_manager->state.ControllerInfo[motor_index].state == RUN)
    {
        if (xSemaphoreTake(controllers_manager->state.controller_state_mutex, portMAX_DELAY)) {
            state.motor_index = motor_index;
            state.kp = controller->kp;
            state.ki = controller->ki;
            state.kd = controller->kd;
            state.target_speed = controller->target_speed;
            state.current_speed = controller->previousSpeed; // Get state should always access after controller update
            state.error = controller->previousError;
            state.output = controller->motorPWM;
            xSemaphoreGive(controllers_manager->state.controller_state_mutex);
        }
    }
    return state;
}