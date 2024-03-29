// ----------------------------------------------------------------------------
// Kinisi motr controller commands.
// This file is auto generated by the commands generator from JSON file.
// Do not edit this file manually.
// Timestamp: 2024-02-20 22:32:00
// Version: 1.0.4
// ----------------------------------------------------------------------------

#pragma once

#include <stdint.h>

// Commands// This command initializes a motor and prepares it for use.
#define INITIALIZE_MOTOR 0x01

// This command sets the speed of the specified motor in PWM.
#define SET_MOTOR_SPEED 0x02

// This command stops motor by setting its speed to 0.
#define STOP_MOTOR 0x03

// This command brakes motor.
#define BRAKE_MOTOR 0x04

// This command sets the controller for the specified motor.
#define INITIALIZE_MOTOR_CONTROLLER 0x05

// This command sets the target speed for the specified motor in radians.
#define SET_MOTOR_TARGET_SPEED 0x06

// This command resets the controller for the specified motor.
#define RESET_MOTOR_CONTROLLER 0x07

// This command gets the state of the controller for the specified motor.
#define GET_MOTOR_CONTROLLER_STATE 0x08

// This command deletes the controller for the specified motor.
#define DELETE_MOTOR_CONTROLLER 0x09

// This command initializes an encoder and prepares it for use.
#define INITIALIZE_ENCODER 0x11

// This command retrieves the current value of the encoder.
#define GET_ENCODER_VALUE 0x12

// This command starts the odometry calculation for the specified encoder.
#define START_ENCODER_ODOMETRY 0x13

// This command resets the odometry calculation for the specified encoder.
#define RESET_ENCODER_ODOMETRY 0x14

// This command stops the odometry calculation for the specified encoder.
#define STOP_ENCODER_ODOMETRY 0x15

// This command retrieves the odometry of the specified encoder.
#define GET_ENCODER_ODOMETRY 0x16

// This command initializes a digital pin and prepares it for use.
#define INITIALIZE_GPIO_PIN 0x20

// This command sets the specified pin to a state.
#define SET_GPIO_PIN_STATE 0x21

// This command gets the state of the specified pin.
#define GET_GPIO_PIN_STATE 0x22

// This command toggles the specified pin.
#define TOGGLE_GPIO_PIN_STATE 0x23

// This command sets the status LED to a state.
#define SET_STATUS_LED_STATE 0x25

// This command toggles the status LED.
#define TOGGLE_STATUS_LED_STATE 0x26

// This command initializes a mecanum platform and prepares it for use.
#define INITIALIZE_MECANUM_PLATFORM 0x30

// This command initializes a omni platform and prepares it for use.
#define INITIALIZE_OMNI_PLATFORM 0x31

// This command sets the velocity for the platform in PWM.
#define SET_PLATFORM_VELOCITY 0x40

// This command sets the controller for the platform.
#define START_PLATFORM_CONTROLLER 0x41

// This command set the target velocity for the platform in meters per second.
#define SET_PLATFORM_TARGET_VELOCITY 0x42

// This command gets the current velocity of the platform in meters per second.
#define GET_PLATFORM_CURRENT_VELOCITY 0x43

// This command stops the controller for the platform.
#define STOP_PLATFORM_CONTROLLER 0x44

// This command starts the odometry calculation for the platform.
#define START_PLATFORM_ODOMETRY 0x45

// This command resets the odometry calculation for the platform.
#define RESET_PLATFORM_ODOMETRY 0x46

// This command stops the odometry calculation for the platform.
#define STOP_PLATFORM_ODOMETRY 0x47

// This command retrieves the odometry of the platform in meters and radians.
#define GET_PLATFORM_ODOMETRY 0x48


// motor_controller_state: The state of the controller for the specified motor.
#pragma pack(push, 1)
typedef struct
{
    int8_t motor_index; // Index of the motor with the controller.
    double kp; // Proportional constant of PID
    double ki; // Integral constant of PID
    double kd; // Derivative constant of PID
    double target_speed; // The target speed of the motor.
    double current_speed; // The current speed of the motor.
    double error; // The error of the motor.
    double output; // The output of the motor.
} motor_controller_state;
#pragma pack(pop)

// platform_velocity: The velocity of the platform in meters per second.
#pragma pack(push, 1)
typedef struct
{
    double x; // X component of platform velocity in meters per second
    double y; // Y component of platform velocity in meters per second
    double t; // Theta component of platform velocity in radians per second
} platform_velocity;
#pragma pack(pop)

// platform_odometry: The odometry of the platform in meters and radians.
#pragma pack(push, 1)
typedef struct
{
    double x; // X component of platform odometry in meters
    double y; // Y component of platform odometry in meters
    double t; // Theta component of platform odometry in radians
} platform_odometry;
#pragma pack(pop)


#pragma pack(push, 1)
typedef struct
{
    uint8_t commandType;  //command type
    union {   //Union to support different properties of each command
        // INITIALIZE_MOTOR: This command initializes a motor and prepares it for use.
        struct {
            uint8_t motor_index; // The index of the motor to initialize.
            uint8_t is_reversed; // Whether or not the motor is reversed.
        } initialize_motor;

        // SET_MOTOR_SPEED: This command sets the speed of the specified motor in PWM.
        struct {
            uint8_t motor_index; // The index of the motor to set the speed for.
            double pwm; // The speed of the motor.
        } set_motor_speed;

        // STOP_MOTOR: This command stops motor by setting its speed to 0.
        struct {
            uint8_t motor_index; // The index of the motor to set the speed for.
        } stop_motor;

        // BRAKE_MOTOR: This command brakes motor.
        struct {
            uint8_t motor_index; // The index of the motor to set the speed for.
        } brake_motor;

        // INITIALIZE_MOTOR_CONTROLLER: This command sets the controller for the specified motor.
        struct {
            uint8_t motor_index; // The index of the motor to set the controller for.
            uint8_t is_reversed; // Whether or not the motor is reversed.
            uint8_t encoder_index; // The index of the encoder to use for the controller.
            double encoder_resolution; // Encoder resolution in ticks per revolution. The value can not be negative or zero.
            double kp; // Proportional constant of PID
            double ki; // Integral constant of PID
            double kd; // Derivative constant of PID
            double integral_limit; // Integral limit of PID controller. The value can not be negative or zero. If the value is zero or negative, the integral limit is disabled.
        } initialize_motor_controller;

        // SET_MOTOR_TARGET_SPEED: This command sets the target speed for the specified motor in radians.
        struct {
            uint8_t motor_index; // The index of the motor to set the target velocity for.
            double speed; // The speed of the motor.
        } set_motor_target_speed;

        // RESET_MOTOR_CONTROLLER: This command resets the controller for the specified motor.
        struct {
            uint8_t motor_index; // The index of the motor to reset the controller for.
        } reset_motor_controller;

        // GET_MOTOR_CONTROLLER_STATE: This command gets the state of the controller for the specified motor.
        struct {
            uint8_t motor_index; // The index of the motor to get the state for.
        } get_motor_controller_state;

        // DELETE_MOTOR_CONTROLLER: This command deletes the controller for the specified motor.
        struct {
            uint8_t motor_index; // The index of the motor to delete the controller for.
        } delete_motor_controller;

        // INITIALIZE_ENCODER: This command initializes an encoder and prepares it for use.
        struct {
            uint8_t encoder_index; // The index of the encoder to initialize.
            double encoder_resolution; // Encoder resolution in ticks per revolution. The value can not be negative or zero.
            uint8_t is_reversed; // Whether or not the encoder is reversed.
        } initialize_encoder;

        // GET_ENCODER_VALUE: This command retrieves the current value of the encoder.
        struct {
            uint8_t encoder_index; // The index of the encoder to retrieve the value for.
        } get_encoder_value;

        // START_ENCODER_ODOMETRY: This command starts the odometry calculation for the specified encoder.
        struct {
            uint8_t encoder_index; // The index of the encoder to start the odometry calculation for.
        } start_encoder_odometry;

        // RESET_ENCODER_ODOMETRY: This command resets the odometry calculation for the specified encoder.
        struct {
            uint8_t encoder_index; // The index of the encoder to reset the odometry calculation for.
        } reset_encoder_odometry;

        // STOP_ENCODER_ODOMETRY: This command stops the odometry calculation for the specified encoder.
        struct {
            uint8_t encoder_index; // The index of the encoder to stop the odometry calculation for.
        } stop_encoder_odometry;

        // GET_ENCODER_ODOMETRY: This command retrieves the odometry of the specified encoder.
        struct {
            uint8_t encoder_index; // The index of the encoder to retrieve the odometry for.
        } get_encoder_odometry;

        // INITIALIZE_GPIO_PIN: This command initializes a digital pin and prepares it for use.
        struct {
            uint8_t pin_number; // The number of the pin to initialize.
            uint8_t mode; // Set digital pin as input or output. Modes: 0 = INPUT_PULLDOWN, 1 = INPUT_PULLUP, 2 = INPUT_NOPULL, 3 = OUTPUT.
        } initialize_gpio_pin;

        // SET_GPIO_PIN_STATE: This command sets the specified pin to a state.
        struct {
            uint8_t pin_number; // The number of the pin to set to a state.
            uint8_t state; // The state of the pin. 0 = LOW, 1 = HIGH.
        } set_gpio_pin_state;

        // GET_GPIO_PIN_STATE: This command gets the state of the specified pin.
        struct {
            uint8_t pin_number; // The number of the pin to get the state for.
        } get_gpio_pin_state;

        // TOGGLE_GPIO_PIN_STATE: This command toggles the specified pin.
        struct {
            uint8_t pin_number; // The number of the pin to toggle.
        } toggle_gpio_pin_state;

        // SET_STATUS_LED_STATE: This command sets the status LED to a state.
        struct {
            uint8_t state; // The state of the status LED. 0 = OFF, 1 = ON.
        } set_status_led_state;

        // TOGGLE_STATUS_LED_STATE: This command toggles the status LED.
        struct {
        } toggle_status_led_state;

        // INITIALIZE_MECANUM_PLATFORM: This command initializes a mecanum platform and prepares it for use.
        struct {
            uint8_t is_reversed_0; // Determins if motor 0 is reversed.
            uint8_t is_reversed_1; // Determins if motor 1 is reversed.
            uint8_t is_reversed_2; // Determins if motor 2 is reversed.
            uint8_t is_reversed_3; // Determins if motor 3 is reversed.
            double length; // Length of the platform in meters.
            double width; // Width of the platform in meters.
            double wheels_diameter; // Diameter of the robot wheels in meters.
            double encoder_resolution; // Encoder resolution in ticks per revolution. The value can not be negative. If platform does not have encoders, the value should be set to zero.
        } initialize_mecanum_platform;

        // INITIALIZE_OMNI_PLATFORM: This command initializes a omni platform and prepares it for use.
        struct {
            uint8_t is_reversed_0; // Determins if motor 0 is reversed.
            uint8_t is_reversed_1; // Determins if motor 1 is reversed.
            uint8_t is_reversed_2; // Determins if motor 2 is reversed.
            double wheels_diameter; // Diameter of the robot wheels in millimeters.
            double robot_radius; // Distance berween the center of the robot and the center of the wheels in millimeters.
            double encoder_resolution; // Encoder resolution in ticks per revolution. The value can not be negative. If platform does not have encoders, the value should be set to zero.
        } initialize_omni_platform;

        // SET_PLATFORM_VELOCITY: This command sets the velocity for the platform in PWM.
        struct {
            double x; // X component of platform velocity in PWM
            double y; // Y component of platform velocity in PWM
            double t; // Theta component of platform velocity in PWM
        } set_platform_velocity;

        // START_PLATFORM_CONTROLLER: This command sets the controller for the platform.
        struct {
            double kp; // Proportional constant of PID
            double ki; // Integral constant of PID
            double kd; // Derivative constant of PID
            double integral_limit; // Integral limit of PID controller. The value can not be negative or zero. If the value is zero or negative, the integral limit is disabled.
        } start_platform_controller;

        // SET_PLATFORM_TARGET_VELOCITY: This command set the target velocity for the platform in meters per second.
        struct {
            double x; // X component of platform velocity in meters per second
            double y; // Y component of platform velocity in meters per second
            double t; // Theta component of platform velocity in radians per second
        } set_platform_target_velocity;

        // GET_PLATFORM_CURRENT_VELOCITY: This command gets the current velocity of the platform in meters per second.
        struct {
        } get_platform_current_velocity;

        // STOP_PLATFORM_CONTROLLER: This command stops the controller for the platform.
        struct {
        } stop_platform_controller;

        // START_PLATFORM_ODOMETRY: This command starts the odometry calculation for the platform.
        struct {
        } start_platform_odometry;

        // RESET_PLATFORM_ODOMETRY: This command resets the odometry calculation for the platform.
        struct {
        } reset_platform_odometry;

        // STOP_PLATFORM_ODOMETRY: This command stops the odometry calculation for the platform.
        struct {
        } stop_platform_odometry;

        // GET_PLATFORM_ODOMETRY: This command retrieves the odometry of the platform in meters and radians.
        struct {
        } get_platform_odometry;

    } properties;
} controller_command_t;
#pragma pack(pop)

