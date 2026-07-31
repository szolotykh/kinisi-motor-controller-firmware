//------------------------------------------------------------
// File name: controllers_manager.c
//------------------------------------------------------------
#include "controllers_manager.h"
#include "commands.h"
#include "loop_frequency.h"
#include <cmsis_os.h>
#include <semphr.h>
#include <hw_config.h>
#include <utils.h>
#include <stdlib.h>
#include <math.h>
#include <hw_motor.h>
#include <hw_encoder.h>

// Update interval for PID controller in milliseconds
#define PID_CONTROLLER_UPDATE_INTERVAL 100

typedef struct controller_info_t
{
    enum {
        STOP,
        RUN
    } state;
    motorIndex mIndex;
    encoder_index_t eIndex;
    pid_controller_t controller;
} controller_info_t;

// Motor controller manager state
typedef struct controllers_manager_state
{
    double target_motor_speed[4]; // In radians per second
    controller_info_t Controller_info[4];
    uint32_t update_interval_ms;
    uint16_t previousEncoderValue[4];
    SemaphoreHandle_t controller_state_mutex;
} controllers_manager_state_t;

// Motor controller manager
typedef struct controllers_manager
{
    osThreadId_t thread_handler;
    controllers_manager_state_t state;
} controllers_manager_t;

static controllers_manager_t controllers_manager = {
    .state = {
        .target_motor_speed = {0},
        .Controller_info = {{.state = STOP}, {.state = STOP},{.state = STOP},{.state = STOP}},
        .update_interval_ms = PID_CONTROLLER_UPDATE_INTERVAL,
        .previousEncoderValue = {0},
        .controller_state_mutex = NULL
    }
};

static const hw_motor_interface_t* motor = NULL;
static const hw_encoder_interface_t* encoder = NULL;

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

    TickType_t xLastWakeTime = xTaskGetTickCount();
    unsigned int seq = 0;

    for(;;)
    {
        // Recompute the period each iteration so SET_CONTROLLER_FREQUENCY takes
        // effect at runtime (the interval is a shared state variable).
        const TickType_t xPeriod = pdMS_TO_TICKS(controllers_manager_state->update_interval_ms);
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        unsigned int seq_update = 0;

        // Aquire controller state mutex before updating controller state
        if (xSemaphoreTake(controllers_manager_state->controller_state_mutex, portMAX_DELAY)) {

            // Update controller state for all controllers
            for (int index = 0; index < NUMBER_MOTORS; index++)
            {
                controller_info_t* controller_info = &controllers_manager_state->Controller_info[index];

                // Check if controller is running
                if(controller_info->state == RUN)
                {
                    pid_controller_t* controller = &controller_info->controller;

                    // Get current velocity from encoder
                    const uint16_t current_encoder_value = encoder->get_value(index);
                    // Calculate change in encoder value
                    uint16_t raw_change = current_encoder_value - controllers_manager_state->previousEncoderValue[index];

                    // Check for overflow and adjust
                    int last_encoder_change;
                    if (raw_change > 32768) { // Half of UINT16_MAX, detecting large backward movement (underflow)
                        last_encoder_change = -(65536 - raw_change); // Adjust for underflow
                    } else {
                        last_encoder_change = raw_change; // No overflow or underflow
                    }
                    
                    controllers_manager_state->previousEncoderValue[index] = current_encoder_value;

                    // Caltulate current motor speed in radians per second from encoder ticks
                    double current_motor_speed = 2.0 * M_PI * ((double)last_encoder_change / encoder->get_resolution(index)) * (1.0 / controller->T);

                    // Calculate new velocity for motor
                    pid_controller_update(
                        &controllers_manager_state->Controller_info[index].controller,
                        current_motor_speed,
                        controllers_manager_state->target_motor_speed[index]);
                }
            }

            // Set motor speed
            for (int index = 0; index < NUMBER_MOTORS; index++)
            {
                if( controllers_manager_state->Controller_info[index].state == RUN )
                {
                // Set motor speed
                motor->set_speed(
                    controllers_manager_state->Controller_info[index].mIndex,
                    controllers_manager_state->Controller_info[index].controller.motorPWM);
                }
            }

            // Release controller state mutex
            xSemaphoreGive(controllers_manager_state->controller_state_mutex);
        }
    }
}

void controllers_manager_init()
{
    motor = get_motor_interface();
    encoder = get_encoder_interface();
    
    controllers_manager.state.controller_state_mutex = xSemaphoreCreateMutex();
    if (controllers_manager.state.controller_state_mutex == NULL) {
        // Handle error: Failed to create the mutex
    }

    const osThreadAttr_t ControllerTask_attributes = {
        .name = "ControllerTask",
        // The task runs double-precision PID math in software (the Cortex-M4
        // FPU is single-precision only), which is stack hungry, so give it
        // headroom beyond the previous 128*8 = 1024 bytes.
        .stack_size = 512 * 8,
        .priority = (osPriority_t) osPriorityNormal,
        };

    controllers_manager.thread_handler = osThreadNew(StartControllerTask, &controllers_manager.state, &ControllerTask_attributes);
}

uint8_t controllers_manager_is_not_init()
{
    osThreadState_t status = osThreadGetState(controllers_manager.thread_handler);
    return status == osThreadError;
}

void controllers_manager_initialize_controller(uint8_t motor_index, uint8_t encoder_index, double kp, double ki, double kd, bool is_reversed, bool is_encoder_reversed, double encoder_resolution, double integral_limit)
{
    // Initialize controller manager which starts task for all controllers
    if (controllers_manager_is_not_init())
    {
        controllers_manager_init();
    }

    // SetMotorController
    pid_controller_t controller; 
    pid_controller_init(
        &controller,
        ((double)controllers_manager.state.update_interval_ms)/1000.0, // PID sampling time in seconds (matches the controller task rate)
        kp,
        ki,
        kd,
        integral_limit
    );

    controller_info_t controller_info;
    controller_info.state = RUN;
    controller_info.controller = controller;
    controller_info.mIndex = motor_index;
    controller_info.eIndex = encoder_index;

    // Always (re-)apply motor and encoder settings so re-initializing a running
    // controller updates its reverse flags / resolution without a board reset.
    // The hardware timer setup inside these is separately guarded; only the
    // settings are updated. previousEncoderValue is re-read below, so any change
    // to the encoder's reverse frame stays consistent (no glitch tick).
    // Encoder direction is configured independently of the motor via
    // is_encoder_reversed, giving the closed loop negative feedback regardless
    // of how the encoder is wired relative to the motor.
    motor->initialize(controller_info.mIndex, is_reversed);
    encoder->initialize(controller_info.eIndex, encoder_resolution, is_encoder_reversed);

    if (xSemaphoreTake(controllers_manager.state.controller_state_mutex, portMAX_DELAY))
    {
        controllers_manager.state.previousEncoderValue[motor_index] = encoder->get_value(encoder_index);
        controllers_manager.state.Controller_info[motor_index] = controller_info;
        xSemaphoreGive(controllers_manager.state.controller_state_mutex);
    }
}

void controllers_manager_initialize_controller_multiple(uint8_t motor_selection, double kp, double ki, double kd, double integral_limit)
{
    // Initialize controller manager which starts task for all controllers
    if (controllers_manager_is_not_init())
    {
        controllers_manager_init();
    }

    if (xSemaphoreTake(controllers_manager.state.controller_state_mutex, portMAX_DELAY))
    {
        for (uint8_t motor_index = 0; motor_index < NUMBER_MOTORS; motor_index++)
        {
            if (motor_selection & (1 << motor_index))
            {
                // Builing PID controller
                pid_controller_t controller; 
                pid_controller_init(
                    &controller,
                    ((double)controllers_manager.state.update_interval_ms)/1000.0, // PID sampling time in seconds (matches the controller task rate)
                    kp,
                    ki,
                    kd,
                    integral_limit
                );

                // Building controller info
                controller_info_t controller_info;
                controller_info.state = RUN;
                controller_info.controller = controller;
                controller_info.mIndex = motor_index;
                controller_info.eIndex = motor_index;

                controllers_manager.state.previousEncoderValue[motor_index] = encoder->get_value(motor_index);
                controllers_manager.state.Controller_info[motor_index] = controller_info;
            }
        }

    xSemaphoreGive(controllers_manager.state.controller_state_mutex);
    }
}

void controllers_manager_stop_controller_multiple(uint8_t motor_selection)
{
    if (xSemaphoreTake(controllers_manager.state.controller_state_mutex, portMAX_DELAY))
    {
        for (uint8_t motor_index = 0; motor_index < NUMBER_MOTORS; motor_index++)
        {
            if (motor_selection & (1 << motor_index))
            {
                controllers_manager.state.Controller_info[motor_index].state = STOP;
                controllers_manager.state.Controller_info[motor_index].controller = (pid_controller_t){0};

                // Stop motor
                motor->stop(controllers_manager.state.Controller_info[motor_index].mIndex);

                // Set target speed to zero
                controllers_manager.state.target_motor_speed[motor_index] = 0;
            }
        }
        xSemaphoreGive(controllers_manager.state.controller_state_mutex);
    }
}

void controllers_manager_stop_controller(uint8_t motor_index)
{
    // If no controller is running for this motor there is nothing to stop, and
    // the state mutex may not exist yet (manager never initialized), so return
    // before touching it. This mirrors the early-out in
    // controllers_manager_delete_controller and keeps open-loop motor commands
    // safe when no controller was ever created.
    if (controllers_manager.state.Controller_info[motor_index].state == STOP)
    {
        return;
    }

    if (xSemaphoreTake(controllers_manager.state.controller_state_mutex, portMAX_DELAY))
    {
        controllers_manager.state.Controller_info[motor_index].state = STOP;
        controllers_manager.state.Controller_info[motor_index].controller = (pid_controller_t){0};
        controllers_manager.state.target_motor_speed[motor_index] = 0;
        xSemaphoreGive(controllers_manager.state.controller_state_mutex);
    }
}

void controllers_manager_brake_multiple(uint8_t motor_selection)
{
    if (xSemaphoreTake(controllers_manager.state.controller_state_mutex, portMAX_DELAY))
    {
        for (uint8_t motor_index = 0; motor_index < NUMBER_MOTORS; motor_index++)
        {
            if (motor_selection & (1 << motor_index))
            {
                controllers_manager.state.Controller_info[motor_index].state = STOP;
                controllers_manager.state.Controller_info[motor_index].controller = (pid_controller_t){0};

                // Actively brake motor (short brake) so it resists motion
                motor->brake(controllers_manager.state.Controller_info[motor_index].mIndex);

                // Set target speed to zero
                controllers_manager.state.target_motor_speed[motor_index] = 0;
            }
        }
        xSemaphoreGive(controllers_manager.state.controller_state_mutex);
    }
}

// Reset the closed-loop controller for a single motor: clears the PID history
// (windup/derivative/output) and re-zeros the target while keeping the
// controller running with its tuning. No-op if no controller is running.
void controllers_manager_reset_controller(uint8_t motor_index)
{
    // Nothing to reset if no controller is running for this motor. The state
    // mutex may not exist yet (manager never initialized), so return before
    // touching it, mirroring the early-out in the stop/delete helpers.
    if (controllers_manager.state.Controller_info[motor_index].state == STOP)
    {
        return;
    }

    if (xSemaphoreTake(controllers_manager.state.controller_state_mutex, portMAX_DELAY))
    {
        // Clear accumulated PID history (windup, differentiator, output) and
        // re-zero the target, but keep the controller RUNNING with its tuning.
        pid_controller_reset(&controllers_manager.state.Controller_info[motor_index].controller);
        controllers_manager.state.target_motor_speed[motor_index] = 0;
        xSemaphoreGive(controllers_manager.state.controller_state_mutex);
    }
}

void controllers_manager_delete_controller(uint8_t motor_index)
{
    // Check if controller for this motor is running
    if(controllers_manager.state.Controller_info[motor_index].state == STOP)
    {
        return;
    }

    if (xSemaphoreTake(controllers_manager.state.controller_state_mutex, portMAX_DELAY)) {

        controllers_manager.state.Controller_info[motor_index].state = STOP;
        controllers_manager.state.Controller_info[motor_index].controller = (pid_controller_t){0};
        
        // Stop motor
        motor->stop(controllers_manager.state.Controller_info[motor_index].mIndex);

        // Set target speed to zero
        controllers_manager.state.target_motor_speed[motor_index] = 0;

        // Release controller state mutex
        xSemaphoreGive(controllers_manager.state.controller_state_mutex);
    }
}

void controllers_manager_set_target_speed(uint8_t motor_index, double target_speed)
{
    // Check if controller for this motor is running
    if(controllers_manager.state.Controller_info[motor_index].state == STOP)
    {
        return;
    }

    if (xSemaphoreTake(controllers_manager.state.controller_state_mutex, portMAX_DELAY)) {
        controllers_manager.state.target_motor_speed[motor_index] = target_speed;
        xSemaphoreGive(controllers_manager.state.controller_state_mutex);
    }
}

void controllers_manager_set_target_speed_multiple(uint8_t* motor_indexes, double* target_speeds, uint8_t motor_count)
{
    if (xSemaphoreTake(controllers_manager.state.controller_state_mutex, portMAX_DELAY)) {
        for (uint8_t i = 0; i < motor_count; i++)
        {
            controllers_manager.state.target_motor_speed[motor_indexes[i]] = target_speeds[i];
        }
        xSemaphoreGive(controllers_manager.state.controller_state_mutex);
    }

}

motor_controller_state controllers_manager_get_motor_controller_state(uint8_t motor_index)
{
    // Check if controller for this motor is running. A bare `return;` here is
    // undefined behavior in a struct-returning function: the caller then
    // transmits uninitialized stack memory (0xA5 FreeRTOS fill) as the reply.
    // Return a zeroed state instead so a not-running controller reads as zeros.
    if(controllers_manager.state.Controller_info[motor_index].state == STOP)
    {
        return (motor_controller_state){0};
    }

    const pid_controller_t* controller = &controllers_manager.state.Controller_info[motor_index].controller;
    motor_controller_state state = {0};
    if (controllers_manager.state.Controller_info[motor_index].state == RUN)
    {
        if (xSemaphoreTake(controllers_manager.state.controller_state_mutex, portMAX_DELAY)) {
            state.motor_index = motor_index;
            state.kp = controller->kp;
            state.ki = controller->ki;
            state.kd = controller->kd;
            state.target_speed = controller->target_speed;
            state.current_speed = controller->previousSpeed; // Get state should always access after controller update
            state.error = controller->previousError;
            state.output = controller->motorPWM;
            xSemaphoreGive(controllers_manager.state.controller_state_mutex);
        }
    }
    return state;
}

// Set the global controller-loop frequency (Hz): updates the task period and
// every running PID's sampling time, quantized to the 1 ms tick. No-op if 0.
void controllers_manager_set_frequency(uint16_t frequency_hz)
{
    // Clamp to the supported range; 0 stays 0 (invalid) and is ignored below.
    frequency_hz = loop_frequency_clamp_hz(frequency_hz);
    if (frequency_hz == 0)
    {
        return; // Ignore invalid frequency
    }

    // Convert to the RTOS tick period. The task period and the PID sampling
    // time both derive from this quantized value so they stay consistent.
    uint32_t period_ms = loop_frequency_hz_to_period_ms(frequency_hz);

    // If the manager (and its mutex/task) has not been created yet, just record
    // the interval; the task reads it when it starts and new controllers pick
    // up the matching sampling time on init.
    if (controllers_manager_is_not_init())
    {
        controllers_manager.state.update_interval_ms = period_ms;
        return;
    }

    if (xSemaphoreTake(controllers_manager.state.controller_state_mutex, portMAX_DELAY))
    {
        controllers_manager.state.update_interval_ms = period_ms;

        // Keep every running controller's PID sampling time in sync with the
        // new loop rate so its integral/derivative terms stay correct.
        const double T = ((double)period_ms) / 1000.0;
        for (int index = 0; index < NUMBER_MOTORS; index++)
        {
            controllers_manager.state.Controller_info[index].controller.T = T;
        }
        xSemaphoreGive(controllers_manager.state.controller_state_mutex);
    }
}

// Get the current global controller-loop frequency in Hz.
uint16_t controllers_manager_get_frequency()
{
    return loop_period_ms_to_frequency_hz(controllers_manager.state.update_interval_ms);
}