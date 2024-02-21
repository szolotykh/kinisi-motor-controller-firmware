//------------------------------------------------------------
// File name: commands_handler.c
//------------------------------------------------------------
#include "commands.h"
#include "controllers_manager.h"
#include "odometry_manager.h"
#include <pid_controller.h>
#include <cmsis_os.h>
#include <message_queue.h>
#include <usbd_cdc_if.h>
#include <hw_gpio.h>
#include "platform.h"
#include "commands_handler.h"
#include "hardware_i2c.h"
#include "stdbool.h"

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
                controllers_manager_initialize_controller(
                    cmd->properties.initialize_motor_controller.motor_index,
                    cmd->properties.initialize_motor_controller.encoder_index,
                    cmd->properties.initialize_motor_controller.kp,
                    cmd->properties.initialize_motor_controller.ki,
                    cmd->properties.initialize_motor_controller.kd,
                    cmd->properties.initialize_motor_controller.is_reversed,
                    cmd->properties.initialize_motor_controller.encoder_resolution,
                    cmd->properties.initialize_motor_controller.integral_limit);
            }
        break; 

        case DELETE_MOTOR_CONTROLLER:
            {
                controllers_manager_delete_controller(
                    cmd->properties.delete_motor_controller.motor_index);
            }
        break;

        case SET_MOTOR_TARGET_SPEED:
            {
                controllers_manager_set_target_speed(
                    cmd->properties.set_motor_target_speed.motor_index,
                    cmd->properties.set_motor_target_speed.speed);
            }
        break;

        case RESET_MOTOR_CONTROLLER:
            {
            // TODO: Implement
            }
        break;

        case GET_MOTOR_CONTROLLER_STATE:
            {
                motor_controller_state state = controllers_manager_get_motor_controller_state(
                    cmd->properties.get_motor_controller_state.motor_index);

                command_callback((uint8_t*)&state, sizeof(motor_controller_state));
            }
        break;

        // Encoder commands
        case INITIALIZE_ENCODER:
            {
            initialize_encoder(
                cmd->properties.initialize_encoder.encoder_index,
                cmd->properties.initialize_encoder.encoder_resolution,
                cmd->properties.initialize_encoder.is_reversed);
            }
        break;

        case GET_ENCODER_VALUE:
            {
            uint16_t value = get_encoder_value(cmd->properties.get_encoder_value.encoder_index);
            command_callback((uint8_t*)&value, sizeof(uint16_t));
            }
        break;

        case START_ENCODER_ODOMETRY:
            {
            encoder_start_odometry(
                cmd->properties.start_encoder_odometry.encoder_index);
            }
        break;

        case RESET_ENCODER_ODOMETRY:
            {
            encoder_reset_odometry(
                cmd->properties.reset_encoder_odometry.encoder_index);
            }
        break;

        case GET_ENCODER_ODOMETRY:
            {
            double odometry = encoder_get_odometry(
                cmd->properties.get_encoder_odometry.encoder_index);
            command_callback((uint8_t*)&odometry, sizeof(double));
            }
        break;

        case STOP_ENCODER_ODOMETRY:
            {
            encoder_stop_odometry(
                cmd->properties.stop_encoder_odometry.encoder_index);
            }
        break;

        // GPIO commands
        case INITIALIZE_GPIO_PIN:
            initialize_gpio_pin(
                cmd->properties.initialize_gpio_pin.pin_number,
                cmd->properties.initialize_gpio_pin.mode);
        break;

        case SET_GPIO_PIN_STATE:
            set_gpio_pin_state(
                cmd->properties.set_gpio_pin_state.pin_number,
                cmd->properties.set_gpio_pin_state.state);
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
                cmd->properties.initialize_mecanum_platform.is_reversed_3,
                cmd->properties.initialize_mecanum_platform.length,
                cmd->properties.initialize_mecanum_platform.width,
                cmd->properties.initialize_mecanum_platform.wheels_diameter,
                cmd->properties.initialize_mecanum_platform.encoder_resolution
            );
        break;

        case INITIALIZE_OMNI_PLATFORM:
            initialize_omni_platform(
                cmd->properties.initialize_omni_platform.is_reversed_0,
                cmd->properties.initialize_omni_platform.is_reversed_1,
                cmd->properties.initialize_omni_platform.is_reversed_2,
                cmd->properties.initialize_omni_platform.wheels_diameter,
                cmd->properties.initialize_omni_platform.robot_radius,
                cmd->properties.initialize_omni_platform.encoder_resolution
            );
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

        case START_PLATFORM_CONTROLLER:
            {
            plaform_controller_settings_t plaform_controller_settings = {
                .kp = cmd->properties.start_platform_controller.kp,
                .ki = cmd->properties.start_platform_controller.ki,
                .kd = cmd->properties.start_platform_controller.kd,
                .integral_limit = cmd->properties.start_platform_controller.integral_limit
            };

            platform_start_velocity_controller(plaform_controller_settings);
            }
        break;

        case STOP_PLATFORM_CONTROLLER:
            {
            platform_stop_velocity_controller();
            }
        break;

        case SET_PLATFORM_TARGET_VELOCITY:
        {
            platform_velocity_t platform_target_velocity = {
                .x = cmd->properties.set_platform_target_velocity.x,
                .y = cmd->properties.set_platform_target_velocity.y,
                .t = cmd->properties.set_platform_target_velocity.t
            };
            platform_set_target_velocity(platform_target_velocity);
        }
        break;
        
        case START_PLATFORM_ODOMETRY:
        {
            platform_start_odometry();
        }
        break;

        case RESET_PLATFORM_ODOMETRY:
        {
            platform_reset_odometry();
        }
        break;

        case GET_PLATFORM_ODOMETRY:
        {
            platform_odometry_t platform_odometry = platform_get_odometry();
            command_callback((uint8_t*)&platform_odometry, sizeof(platform_odometry_t));
        }
        break;

        case STOP_PLATFORM_ODOMETRY:
        {
            platform_stop_odometry();
        }
        break;
    }
}