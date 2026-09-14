//------------------------------------------------------------
// File name: command_handler.c
// Description: Check resource prerequisites and execute validated controller commands.
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
#include "initialization.h"
#include "connection.h"
#include "hw_clock.h"
#include "command_requirements.h"

/**
 * @brief Execute a validated command after checking its resource prerequisites.
 * @param cmd Request with a valid command ID, payload length, and argument ranges.
 * @param command_callback Receives response payload bytes and must copy them immediately.
 * @return RESPONSE_OK for accepted operations, or a generated protocol error code.
 * @note Called serially inside a connection dispatch; ACK/error framing is handled above.
 */
uint8_t command_handler(controller_command_t* cmd, void (*command_callback)(uint8_t*, uint8_t))
{
    const gpio_interface_t* gpio = get_gpio_interface();
    const hw_motor_interface_t* motor = get_motor_interface();
    const hw_encoder_interface_t* encoder = get_encoder_interface();

    const command_resources_t resources = {
        .motor_owned = platform_owns_motor,
        .motor_initialized = motor->is_initialized,
        .encoder_initialized = encoder->is_initialized,
        .controller_running = controllers_manager_is_running,
        .platform_initialized = platform_is_initialized,
        .platform_controller_running = platform_is_controller_running
    };
    uint8_t prerequisite = command_requirements_check(cmd, &resources);
    if (prerequisite != RESPONSE_OK) return prerequisite;

    switch(cmd->commandType)
    {
        case INIT:
        {
            // Transport callbacks copy the response into their transmit buffer.
            uint8_t error = initialization_validate(cmd);
            if (error != RESPONSE_OK) return error;
            init_response response = initialization_response();
            command_callback((uint8_t *)&response, sizeof(response));
            return RESPONSE_OK;
        }
        break;
        case INITIALIZE_MOTOR:
            {
                motor->initialize(cmd->properties.initialize_motor.motor_index,
                                cmd->properties.initialize_motor.is_reversed);
            }
        break;

        case SET_MOTOR_SPEED:
            {
            {
                motor->set_speed(
                    cmd->properties.set_motor_speed.motor_index,
                    cmd->properties.set_motor_speed.pwm);
            }
            }
        break;

        case STOP_MOTOR:
            {
                uint8_t motor_index = cmd->properties.stop_motor.motor_index;
                {
                    // Take the motor out of closed-loop control (no-op if none
                    // is running) so the PID task stops overriding it, then coast.
                    controllers_manager_stop_controller(motor_index);
                    motor->stop(motor_index);
                }
            }
        break;

        case BRAKE_MOTOR:
            {
                uint8_t motor_index = cmd->properties.brake_motor.motor_index;
                {
                    // Take the motor out of closed-loop control (no-op if none
                    // is running) so the PID task stops overriding it, then brake.
                    controllers_manager_stop_controller(motor_index);
                    motor->brake(motor_index);
                }
            }
        break;

        case INITIALIZE_MOTOR_CONTROLLER:
            {
                {
                controllers_manager_initialize_controller(
                    cmd->properties.initialize_motor_controller.motor_index,
                    cmd->properties.initialize_motor_controller.encoder_index,
                    cmd->properties.initialize_motor_controller.kp,
                    cmd->properties.initialize_motor_controller.ki,
                    cmd->properties.initialize_motor_controller.kd,
                    cmd->properties.initialize_motor_controller.is_reversed,
                    cmd->properties.initialize_motor_controller.is_encoder_reversed,
                    cmd->properties.initialize_motor_controller.encoder_resolution,
                    cmd->properties.initialize_motor_controller.integral_limit);
                }
            }
        break; 

        case DELETE_MOTOR_CONTROLLER:
            {
                {
                controllers_manager_delete_controller(
                    cmd->properties.delete_motor_controller.motor_index);
                }
            }
        break;

        case SET_CONTROLLER_FREQUENCY:
            {
                controllers_manager_set_frequency(
                    cmd->properties.set_controller_frequency.frequency);
            }
        break;

        case GET_CONTROLLER_FREQUENCY:
            {
                uint16_t frequency = controllers_manager_get_frequency();
                command_callback((uint8_t*)&frequency, sizeof(uint16_t));
            }
        break;

        case SET_MOTOR_TARGET_SPEED:
            {
                {
                controllers_manager_set_target_speed(
                    cmd->properties.set_motor_target_speed.motor_index,
                    cmd->properties.set_motor_target_speed.speed);
                }
            }
        break;

        case RESET_MOTOR_CONTROLLER:
            {
            {
                controllers_manager_reset_controller(
                    cmd->properties.reset_motor_controller.motor_index);
            }
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
            encoder->initialize(
                cmd->properties.initialize_encoder.encoder_index,
                cmd->properties.initialize_encoder.encoder_resolution,
                cmd->properties.initialize_encoder.is_reversed);
            }
        break;

        case GET_ENCODER_VALUE:
            {
            uint16_t value = encoder->get_value(cmd->properties.get_encoder_value.encoder_index);
            command_callback((uint8_t*)&value, sizeof(uint16_t));
            }
        break;

        case START_ENCODER_ODOMETRY:
            {
            encoder_start_odometry(cmd->properties.start_encoder_odometry.encoder_index);
            }
        break;

        case RESET_ENCODER_ODOMETRY:
            {
            encoder_reset_odometry(cmd->properties.reset_encoder_odometry.encoder_index);
            }
        break;

        case GET_ENCODER_ODOMETRY:
            {
            const time_sync_t *clock = connection_current_clock();
            uint64_t sampled_us, timestamp;
            double angle;
            if (!clock || !clock->ready) return RESPONSE_CLOCK_NOT_READY;
            uint8_t error = encoder_get_odometry_sample(cmd->properties.get_encoder_odometry.encoder_index, &angle, &sampled_us);
            if (error != RESPONSE_OK) return error;
            if (!time_sync_convert(clock, sampled_us, &timestamp)) return RESPONSE_CLOCK_NOT_READY;
            encoder_odometry_sample sample = {
                .timestamp_us = timestamp, .clock_mode = clock->mode,
                .clock_quality = time_sync_quality(clock, hw_clock_microseconds()), .angle = angle
            };
            command_callback((uint8_t*)&sample, sizeof(sample));
            }
        break;

        case SET_ODOMETRY_FREQUENCY:
            {
                odometry_manager_set_frequency(
                    cmd->properties.set_odometry_frequency.frequency);
            }
        break;

        case GET_ODOMETRY_FREQUENCY:
            {
                uint16_t frequency = odometry_manager_get_frequency();
                command_callback((uint8_t*)&frequency, sizeof(uint16_t));
            }
        break;

        case STOP_ENCODER_ODOMETRY:
            {
            encoder_stop_odometry(cmd->properties.stop_encoder_odometry.encoder_index);
            }
        break;

        // GPIO commands
        case INITIALIZE_GPIO_PIN:
            gpio->initialize_pin(
                cmd->properties.initialize_gpio_pin.pin_number,
                cmd->properties.initialize_gpio_pin.mode);
        break;

        case SET_GPIO_PIN_STATE:
            gpio->set_state(
                cmd->properties.set_gpio_pin_state.pin_number,
                cmd->properties.set_gpio_pin_state.state);
        break;

        case GET_GPIO_PIN_STATE:
            {
            uint8_t state = gpio->get_state(cmd->properties.get_gpio_pin_state.pin_number);
            command_callback((uint8_t*)&state, sizeof(uint8_t));
            }
        break;

        case TOGGLE_GPIO_PIN_STATE:
            gpio->toggle(cmd->properties.toggle_gpio_pin_state.pin_number);
        break;

        // Status LED commands
        case SET_STATUS_LED_STATE:
            gpio->set_status_led(cmd->properties.set_status_led_state.state);
        break;

        case TOGGLE_STATUS_LED_STATE:
            gpio->toggle_status_led();
        break;

        // Platform commands
        case INITIALIZE_MECANUM_PLATFORM:
            initialize_mecanum_platform(
                cmd->properties.initialize_mecanum_platform.is_reversed_0,
                cmd->properties.initialize_mecanum_platform.is_reversed_1,
                cmd->properties.initialize_mecanum_platform.is_reversed_2,
                cmd->properties.initialize_mecanum_platform.is_reversed_3,
                cmd->properties.initialize_mecanum_platform.is_encoder_reversed_0,
                cmd->properties.initialize_mecanum_platform.is_encoder_reversed_1,
                cmd->properties.initialize_mecanum_platform.is_encoder_reversed_2,
                cmd->properties.initialize_mecanum_platform.is_encoder_reversed_3,
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
                cmd->properties.initialize_omni_platform.is_encoder_reversed_0,
                cmd->properties.initialize_omni_platform.is_encoder_reversed_1,
                cmd->properties.initialize_omni_platform.is_encoder_reversed_2,
                cmd->properties.initialize_omni_platform.wheels_diameter,
                cmd->properties.initialize_omni_platform.robot_radius,
                cmd->properties.initialize_omni_platform.encoder_resolution
            );
        break;

        case INITIALIZE_DIFFERENTIAL_PLATFORM:
            initialize_differential_platform(
                cmd->properties.initialize_differential_platform.is_reversed_0,
                cmd->properties.initialize_differential_platform.is_reversed_1,
                cmd->properties.initialize_differential_platform.is_encoder_reversed_0,
                cmd->properties.initialize_differential_platform.is_encoder_reversed_1,
                cmd->properties.initialize_differential_platform.wheel_diameter,
                cmd->properties.initialize_differential_platform.wheel_base,
                cmd->properties.initialize_differential_platform.encoder_resolution
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

        case BRAKE_PLATFORM:
            {
            platform_brake();
            }
        break;

        case COAST_PLATFORM:
            {
            platform_coast();
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
            const time_sync_t *clock = connection_current_clock();
            uint64_t sampled_us, timestamp;
            platform_odometry_t pose;
            if (!clock || !clock->ready) return RESPONSE_CLOCK_NOT_READY;
            uint8_t error = odometry_manager_get_platform_sample(&pose, &sampled_us);
            if (error != RESPONSE_OK) return error;
            if (!time_sync_convert(clock, sampled_us, &timestamp)) return RESPONSE_CLOCK_NOT_READY;
            platform_odometry_sample sample = {
                .timestamp_us = timestamp, .clock_mode = clock->mode,
                .clock_quality = time_sync_quality(clock, hw_clock_microseconds()),
                .x = pose.x, .y = pose.y, .t = pose.t
            };
            command_callback((uint8_t*)&sample, sizeof(sample));
        }
        break;

        case STOP_PLATFORM_ODOMETRY:
        {
            platform_stop_odometry();
        }
        break;
        default: return RESPONSE_UNKNOWN_COMMAND;
    }
    return RESPONSE_OK;
}
