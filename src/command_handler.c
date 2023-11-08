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

controllers_manager_t controllersManager;
controllers_manager_input_t controllers_manager_input = {
    .TargetVelocity = {0},
    .ControllerInfo = {{.state = STOP}, {.state = STOP},{.state = STOP},{.state = STOP}}
};

void command_handler(controller_command_t* cmd, void (*command_callback)(uint8_t*, uint16_t))
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
                cmd->properties.set_motor_speed.direction,
                cmd->properties.set_motor_speed.speed);
            }
        break;

        case INITIALIZE_MOTOR_CONTROLLER:
            {
            uint8_t motorIndex = cmd->properties.initialize_motor_controller.motor_index;
            if (controllers_manager_is_not_init(&controllersManager))
            {
                controllers_manager_init(&controllersManager, &controllers_manager_input);
            }

            // SetMotorController
            pid_controller_t controller; 
            pid_controller_init(
                &controller,
                cmd->properties.initialize_motor_controller.kp,
                cmd->properties.initialize_motor_controller.ki,
                cmd->properties.initialize_motor_controller.kd
            );

            controller_info_t controllerInfo;
            controllerInfo.state = STOP;
            controllerInfo.controller = controller;
            controllerInfo.mIndex = motorIndex;
            controllerInfo.eIndex = motorIndex;

            initialize_motor(controllerInfo.mIndex, false);
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

        case SET_MOTOR_TARGET_VELOCITY:
            {
            signed short speed = cmd->properties.set_motor_target_velocity.speed;
            signed short direction = cmd->properties.set_motor_target_velocity.direction;
            controllers_manager_input.TargetVelocity[cmd->properties.set_motor_target_velocity.motor_index] = ((direction * 2) - 1) * speed;
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

        case SET_PLATFORM_CONTROLLER:
            // SetController
        break;

        case SET_PLATFORM_VELOCITY_INPUT:
            {
            mecanum_velocity_t mecanumVelocity = get_mecanum_velocities(
                cmd->properties.set_platform_velocity_input.x,
                cmd->properties.set_platform_velocity_input.y,
                cmd->properties.set_platform_velocity_input.t);
            set_velocity_input(mecanumVelocity);
            // Should be part of set target velocity
            // TargetVelocity[MOTOR0] = mecanumVelocity.motor0;
            // TargetVelocity[MOTOR1] = mecanumVelocity.motor1;
            // TargetVelocity[MOTOR2] = mecanumVelocity.motor2;
            // TargetVelocity[MOTOR3] = mecanumVelocity.motor3;
            
            }
        break;
        }
}