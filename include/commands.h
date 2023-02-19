#pragma once

#include <stdint.h>

// Commands
// Motor commands
#define INITIALIZE_MOTOR 0x01
#define SET_MOTOR_SPEED 0x02
#define MOTOR_SET_CONTROLLER 0x03
#define MOTOR_SET_TARGET_VELOCITY 0x04
#define MOTOR_DEL_CONTROLLER 0x05

// Encoder commands
#define INITIALIZE_ENCODER 0x11
#define GET_ENCODER_VALUE 0x12

// GPIO
#define PIN_LOW 0x20
#define PIN_HIGH 0x21
#define PIN_TOGGLE 0x22

// Status LED
#define STATUS_LED_OFF 0x23
#define STATUS_LED_ON 0x24
#define STATUS_LED_TOGGLE 0x25

// Platform
#define PLATFORM_INITIALIZE 0x30
#define PLATFORM_SET_VELOCITY_INPUT 0x31
#define PLATFORM_SET_CONTROLLER 0x32

#pragma pack(push, 1)
//Motor controller command structure
typedef struct
{
  uint8_t commandType;  //command type
  union {   //Union to support different properties of each command
    struct {
      uint8_t motorIndex;  //index of the motor (0-4)
    } initializeMotor;
    
    struct {
      uint8_t motorIndex;  //index of the motor (0-4)
      uint8_t direction;  //direction of the motor (0 or 1)
      uint16_t speed;       //speed of the motor in PWM (0-840)
    } setMotorSpeed;
    
    struct {
      uint8_t motorIndex;  //index of the motor (0-4)
      double kp;           //proportional constant of PID
      double ki;           //integral constant of PID
      double kd;           //derivative constant of PID
    } setMotorController;
    
    struct {
      uint8_t motorIndex;  //index of the motor (0-4)
      uint8_t direction;  //direction of the motor (0 or 1)
      uint16_t speed;       //speed of the motor in % (0-100)
    } setMotorTargetVelocity;
    
    struct {
      uint8_t motorIndex;  //index of the motor (0-4)
    } deleteMotorController;
    
    struct {
      uint8_t encoderIndex;  //index of the encoder (0-4)
    } initializeEncoder;
    
    struct {
      uint8_t encoderIndex;  //index of the encoder (0-4)
    } getEncoderValue;
    
    struct {
      uint8_t pinNumber;  //pin number of the pin to be set low
    } pinLow;
    
    struct {
      uint8_t pinNumber;  //pin number of the pin to be set high
    } pinHigh;
    
    struct {
      uint8_t pinNumber;  //pin number of the pin to be toggled
    } pinToggle;
    
    struct {
      //no properties for turning off the status LED
    } statusLedOff;
    
    struct {
      //no properties for turning on the status LED
    } statusLedOn;
    
    struct {
      //no properties for toggling the status LED
    } statusLedToggle;
    
    struct {
      //no properties for initializing the platform
    } initializePlatform;
    
    struct {
      int8_t x;  //X component of platform velocity
      int8_t y;  //Y component of platform velocity
      int8_t t;  //T component of platform velocity
    } setPlatformVelocityInput;
    
    struct {
      //no properties for setting the platform controller
    } setPlatformController;
  } properties;
} controller_command_t;
#pragma pack(pop)