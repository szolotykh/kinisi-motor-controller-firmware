//------------------------------------------------------------
// File name: commands_handler.c
//------------------------------------------------------------
#include "commands.h"
#include "controllers_manager.h"
#include <pid_controller.h>
#include <cmsis_os.h>
#include <message_queue.h>
#include <usbd_cdc_if.h>
#include <hw_gpio.h>
#include "platform.h"
#include "commands_handler.h"
#include "hardware_i2c.h"
#include "stdbool.h"

#define PID_CONTROLLER_UPDATE_INTERVAL 100

controllers_manager_t controllersManager;
controllers_manager_input_t controllers_manager_input = {
    .TargetMotorSpeed = {0},
    .ControllerInfo = {{.state = STOP}, {.state = STOP},{.state = STOP},{.state = STOP}},
    .update_interval_ms = PID_CONTROLLER_UPDATE_INTERVAL,
    .controller_state_mutex = NULL
};

void command_handler(controller_command_t* cmd, void (*command_callback)(uint8_t*, uint8_t))
{
    switch(cmd->commandType)
        {
        case INITIALIZE_MOTOR:
            initialize_motor(cmd->properties.initialize_motor.motor_index, cmd->properties.initialize_motor.is_reversed);
        break;

        case SET_MOTOR_SPEED:
            {
            set_motor_speed(
                cmd->properties.set_motor_speed.motor_index,
                cmd->properties.set_motor_speed.pwm);
            }
        break;

        case STOP_MOTOR:
            {
                stop_motor(cmd->properties.stop_motor.motor_index);
            }
        break;

        case BRAKE_MOTOR:
            {
                brake_motor(cmd->properties.brake_motor.motor_index);
            }
        break;

        case INITIALIZE_MOTOR_CONTROLLER:
            {
            uint8_t motorIndex = cmd->properties.initialize_motor_controller.motor_index;

            // Initialize controller manager which starts task for all controllers
            if (controllers_manager_is_not_init(&controllersManager))
            {
                controllers_manager_init(&controllersManager, &controllers_manager_input);
            }

            // SetMotorController
            pid_controller_t controller; 
            pid_controller_init(
                &controller,
                ((double)PID_CONTROLLER_UPDATE_INTERVAL)/1000.0, // PID controller update interval in seconds
                cmd->properties.initialize_motor_controller.kp,
                cmd->properties.initialize_motor_controller.ki,
                cmd->properties.initialize_motor_controller.kd,
                cmd->properties.initialize_motor_controller.is_reversed,
                cmd->properties.initialize_motor_controller.encoder_resolution
            );

            controller_info_t controllerInfo;
            controllerInfo.state = STOP;
            controllerInfo.controller = controller;
            controllerInfo.mIndex = motorIndex;
            controllerInfo.eIndex = motorIndex;

            initialize_motor(controllerInfo.mIndex, cmd->properties.initialize_motor_controller.is_reversed);
            initialize_encoder(controllerInfo.eIndex);

            controllers_manager_input.ControllerInfo[motorIndex] = controllerInfo;
            controllers_manager_input.ControllerInfo[motorIndex].state = RUN;
            }
        break; 

        case DELETE_MOTOR_CONTROLLER:
            {
            controllers_manager_input.ControllerInfo[cmd->properties.delete_motor_controller.motor_index].state = STOPPING; // STOPPING
            }
        break;

        case SET_MOTOR_TARGET_SPEED:
            {
                if (xSemaphoreTake(controllers_manager_input.controller_state_mutex, portMAX_DELAY)) {
                    controllers_manager_input.TargetMotorSpeed[cmd->properties.set_motor_target_speed.motor_index] = cmd->properties.set_motor_target_speed.speed;
                    xSemaphoreGive(controllers_manager_input.controller_state_mutex);
                }
            }
        break;

        case RESET_MOTOR_CONTROLLER:
            {
            // TODO: Implement
            }
        break;

        case GET_MOTOR_CONTROLLER_STATE:
            {
                uint8_t motor_index = cmd->properties.get_motor_controller_state.motor_index;
                const pid_controller_t* controller = &controllers_manager_input.ControllerInfo[motor_index].controller;
                motor_controller_state state = {0};
                if (controllers_manager_input.ControllerInfo[motor_index].state == RUN)
                {
                    if (xSemaphoreTake(controllers_manager_input.controller_state_mutex, portMAX_DELAY)) {
                        state.motor_index = motor_index;
                        state.kp = controller->kp;
                        state.ki = controller->ki;
                        state.kd = controller->kd;
                        state.target_speed = controller->target_speed;
                        state.current_speed = controller->previousSpeed; // Get state should always access after controller update
                        state.error = controller->previousError;
                        state.output = controller->motorPWM;
                        xSemaphoreGive(controllers_manager_input.controller_state_mutex);
                    }
                }
                command_callback((uint8_t*)&state, sizeof(motor_controller_state));
            }
        break;

        // Encoder commands
        case INITIALIZE_ENCODER:
            initialize_encoder(cmd->properties.initialize_encoder.encoder_index);
        break;

        case GET_ENCODER_VALUE:
            {
            unsigned int value = get_encoder_value(cmd->properties.get_encoder_value.encoder_index);
            command_callback((uint8_t*)&value, sizeof(unsigned int));
            }
        break;

        // GPIO commands
        case INITIALIZE_GPIO_PIN:
            initialize_gpio_pin(cmd->properties.initialize_gpio_pin.pin_number, cmd->properties.initialize_gpio_pin.mode);
        break;

        case SET_GPIO_PIN_STATE:
            set_gpio_pin_state(cmd->properties.set_gpio_pin_state.pin_number, cmd->properties.set_gpio_pin_state.state);
        break;

        case GET_GPIO_PIN_STATE:
            {
            uint8_t state = get_gpio_pin_state(cmd->properties.get_gpio_pin_state.pin_number);
            command_callback((uint8_t*)&state, sizeof(uint8_t));
            }
        break;

        case TOGGLE_GPIO_PIN_STATE:
            toggle_gpio_pin_state(cmd->properties.toggle_gpio_pin_state.pin_number);
        break;

        // Status LED commands
        case SET_STATUS_LED_STATE:
            set_status_led_state(cmd->properties.set_status_led_state.state);
        break;

        case TOGGLE_STATUS_LED_STATE:
            toggle_status_led_state();
        break;

        // Platform commands
        case INITIALIZE_MECANUM_PLATFORM:
            initialize_mecanum_platform(
                cmd->properties.initialize_mecanum_platform.is_reversed_0,
                cmd->properties.initialize_mecanum_platform.is_reversed_1,
                cmd->properties.initialize_mecanum_platform.is_reversed_2,
                cmd->properties.initialize_mecanum_platform.is_reversed_3
            );
        break;

        case INITIALIZE_OMNI_PLATFORM:
            initialize_omni_platform(
                cmd->properties.initialize_omni_platform.is_reversed_0,
                cmd->properties.initialize_omni_platform.is_reversed_1,
                cmd->properties.initialize_omni_platform.is_reversed_2,
                cmd->properties.initialize_omni_platform.wheels_diameter,
                cmd->properties.initialize_omni_platform.robot_radius
            );

        case SET_PLATFORM_CONTROLLER:
            // SetController
        break;

        case SET_PLATFORM_VELOCITY:
            {
            platform_velocity_t platform_velocity = {
                .x = cmd->properties.set_platform_velocity.x,
                .y = cmd->properties.set_platform_velocity.y,
                .t = cmd->properties.set_platform_velocity.t
            };
            set_platform_velocity(platform_velocity);
            }
        break;
        }
}