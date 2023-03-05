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

controllers_manager_t controllersManager;
controllers_manager_input_t controllers_manager_input = {
    .TargetVelocity = {0},
    .ControllerInfo = {{.state = STOP}, {.state = STOP},{.state = STOP},{.state = STOP}}
};

void command_handler(controller_command_t* cmd)
{
    switch(cmd->commandType)
        {
        case INITIALIZE_MOTOR:
            initialize_motor(cmd->properties.initializeMotor.motorIndex);
        break;

        case SET_MOTOR_SPEED:
            {
            set_motor_speed(
                cmd->properties.setMotorSpeed.motorIndex,
                cmd->properties.setMotorSpeed.direction,
                cmd->properties.setMotorSpeed.speed);
            }
        break;

        case MOTOR_SET_CONTROLLER:
            {
            uint8_t motorIndex = cmd->properties.setMotorController.motorIndex;
            if (controllers_manager_is_not_init(&controllersManager))
            {
                controllers_manager_init(&controllersManager, &controllers_manager_input);
            }

            // SetMotorController
            pid_controller_t controller; 
            pid_controller_init(
                &controller,
                cmd->properties.setMotorController.kp,
                cmd->properties.setMotorController.ki,
                cmd->properties.setMotorController.kd
            );

            controller_info_t controllerInfo;
            controllerInfo.state = STOP;
            controllerInfo.controller = controller;
            controllerInfo.mIndex = motorIndex;
            controllerInfo.eIndex = motorIndex;

            initialize_motor(controllerInfo.mIndex);
            initialize_encoder(controllerInfo.eIndex);

            controllers_manager_input.ControllerInfo[motorIndex] = controllerInfo;
            controllers_manager_input.ControllerInfo[motorIndex].state = RUN;
            }
        break; 

        case MOTOR_DEL_CONTROLLER:
            {
            controllers_manager_input.ControllerInfo[cmd->properties.deleteMotorController.motorIndex].state = STOPPING; // STOPPING
            }
        break;

        case MOTOR_SET_TARGET_VELOCITY:
            {
            signed short speed = cmd->properties.setMotorTargetVelocity.speed;
            signed short direction = cmd->properties.setMotorTargetVelocity.direction;
            controllers_manager_input.TargetVelocity[cmd->properties.setMotorTargetVelocity.motorIndex] = ((direction * 2) - 1) * speed;
            }
        break;

        case INITIALIZE_ENCODER:
            initialize_encoder(cmd->properties.initializeEncoder.encoderIndex);
        break;

        case GET_ENCODER_VALUE:
            {
            unsigned int value = get_encoder_value(cmd->properties.getEncoderValue.encoderIndex);
            CDC_Transmit_FS((uint8_t*)&value, sizeof(unsigned int));
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
            mecanum_velocity_t mecanumVelocity = get_mecanum_velocities(
                cmd->properties.setPlatformVelocityInput.x,
                cmd->properties.setPlatformVelocityInput.y,
                cmd->properties.setPlatformVelocityInput.t);
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