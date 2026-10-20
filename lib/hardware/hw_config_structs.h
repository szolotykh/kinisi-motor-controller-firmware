//------------------------------------------------------------
// File name: config_structs.h
//------------------------------------------------------------

#pragma once

#include "stm32f4xx_hal.h"
#include <stdbool.h>

// Numeric selectors are also usable in #if; C enum names alone are not.
#define MOTOR_CONTROL_MODEL_PWM 0
#define MOTOR_CONTROL_MODEL_ENABLE_DIRECTION 1
typedef enum {
	MOTOR_CONTROL_PWM = MOTOR_CONTROL_MODEL_PWM,
	MOTOR_CONTROL_ENABLE_DIRECTION = MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
} motor_control_model_t;

typedef struct gpio_info_t {
	GPIO_TypeDef* port;
	uint16_t pin;
} gpio_info_t;

typedef struct {
	gpio_info_t gpio;
	bool activeLow;
} status_led_info_t;

typedef struct {
	SPI_TypeDef* instance;
	uint32_t alternate;
	gpio_info_t sck;
	gpio_info_t miso;
	gpio_info_t mosi;
} spi_bus_info_t;

typedef struct {
	I2C_TypeDef* instance;
	uint32_t alternate;
	uint32_t frequencyHz;
	gpio_info_t scl;
	gpio_info_t sda;
} i2c_bus_info_t;

typedef enum {
	TEMPERATURE_SENSOR_TMP102 = 1
} temperature_sensor_type_t;

typedef struct {
	temperature_sensor_type_t type;
	uint8_t address; // Unshifted 7-bit I2C address.
} temperature_sensor_info_t;

typedef enum {
	IMU_SENSOR_BMI088 = 1
} imu_sensor_type_t;

typedef struct {
	imu_sensor_type_t type;
	gpio_info_t accelChipSelect;
	gpio_info_t gyroChipSelect;
	// Wiring metadata; the polled driver does not configure EXTI or data sync.
	gpio_info_t accelInterrupt;
	gpio_info_t gyroInterrupt;
} imu_sensor_info_t;

// Circuit/default metadata only; current measurement and IC2/IC4 configuration are not implemented.
typedef enum {
	CURRENT_SENSE_NONE = 0,
	CURRENT_SENSE_ADC = 1
} current_sense_type_t;

typedef struct {
	bool available;
	current_sense_type_t type;
	gpio_info_t input;
	uint16_t senseResistorOhm;
	uint16_t filterCapacitanceNf;
} current_sense_info_t;

typedef struct {
	bool available;
	bool enabled;
	uint16_t thresholdMa;
	const uint16_t* supportedThresholdsMa;
	uint8_t supportedThresholdCount;
} hardware_current_limit_info_t;

typedef enum {
	OVERCURRENT_PROTECTION_NONE = 0,
	OVERCURRENT_PROTECTION_LATCHED_SHUTDOWN,
	OVERCURRENT_PROTECTION_AUTOMATIC_RETRY,
	OVERCURRENT_PROTECTION_REPORT_ONLY,
	OVERCURRENT_PROTECTION_DISABLED
} overcurrent_protection_mode_t;

typedef struct {
	bool available;
	overcurrent_protection_mode_t mode;
	const overcurrent_protection_mode_t* supportedModes;
	uint8_t supportedModeCount;
} overcurrent_protection_info_t;

// PWM channel configuration structure
typedef struct pwm_channel_info_t
{
	uint32_t timerChannel;
	GPIO_TypeDef* port;
	uint16_t pin;
	bool complementary;
}pwm_channel_info_t;

// Motor configuration structure
typedef struct motor_info_t
{
	motor_control_model_t controlModel;
	TIM_TypeDef* timer;
	// Dual-PWM drivers use these two channels.
	pwm_channel_info_t pwmChannel1;
	pwm_channel_info_t pwmChannel2;
	// Enable/direction drivers use one PWM output and a direction GPIO.
	pwm_channel_info_t enable;
	gpio_info_t direction;
	// DRV8873S control/status pins (used by the v4 driver helper).
	gpio_info_t disable;
	gpio_info_t sleep;
	gpio_info_t fault;
	gpio_info_t chipSelect;
	current_sense_info_t currentSense;
	hardware_current_limit_info_t hardwareCurrentLimit;
	overcurrent_protection_info_t overcurrentProtection;

}motor_info_t;

// Encoder configuration structure
typedef struct encoder_info_t
{
	TIM_TypeDef* timer;

	GPIO_TypeDef* portA;
	uint16_t pinA;

	GPIO_TypeDef* portB;
	uint16_t pinB;
}encoder_info_t;
