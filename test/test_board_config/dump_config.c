// Compare generated C mappings with a snapshot taken from the pre-generator header.
#include <stdio.h>
#include <assert.h>
#ifdef BASELINE_CONFIG
#include "hw_config_before_generator.h"
#else
#include "hw_config.h"
#endif

GPIO_TypeDef test_ports[5];
TIM_TypeDef test_timers[12];

static void pin(gpio_info_t p) {
    if (p.port) printf(" P%c:%04x", (int)('A' + (p.port - test_ports)), p.pin);
    else printf(" -:%04x", p.pin);
}
static void pwm(pwm_channel_info_t p) {
    printf(" CH%lu%s", (unsigned long)(p.timerChannel / 4 + 1), p.complementary ? "N" : "");
    pin((gpio_info_t){p.port, p.pin});
}
int main(void) {
#ifndef BASELINE_CONFIG
#ifdef MP_V4
    assert(NUMBER_IMU_SENSORS == 1);
    assert(imu_sensor_info[0].type == IMU_SENSOR_BMI088);
    assert(imu_spi_info.instance == SPI2 && imu_spi_info.alternate == GPIO_AF5_SPI2);
    assert(imu_spi_info.sck.port == GPIOB && imu_spi_info.sck.pin == GPIO_PIN_13);
    assert(imu_spi_info.miso.port == GPIOC && imu_spi_info.miso.pin == GPIO_PIN_2);
    assert(imu_spi_info.mosi.port == GPIOC && imu_spi_info.mosi.pin == GPIO_PIN_3);
    assert(imu_sensor_info[0].accelChipSelect.port == GPIOD && imu_sensor_info[0].accelChipSelect.pin == GPIO_PIN_10);
    assert(imu_sensor_info[0].gyroChipSelect.port == GPIOD && imu_sensor_info[0].gyroChipSelect.pin == GPIO_PIN_12);
    assert(imu_sensor_info[0].accelInterrupt.port == GPIOD && imu_sensor_info[0].accelInterrupt.pin == GPIO_PIN_11);
    assert(imu_sensor_info[0].gyroInterrupt.port == GPIOD && imu_sensor_info[0].gyroInterrupt.pin == GPIO_PIN_9);
    assert(NUMBER_TEMPERATURE_SENSORS == 1);
    assert(temperature_sensor_info[0].type == TEMPERATURE_SENSOR_TMP102);
    assert(temperature_sensor_info[0].address == 0x48);
    assert(temperature_i2c_info.instance == I2C3);
    assert(temperature_i2c_info.alternate == GPIO_AF4_I2C3);
    assert(temperature_i2c_info.frequencyHz == 100000);
    assert(temperature_i2c_info.scl.port == GPIOA && temperature_i2c_info.scl.pin == GPIO_PIN_8);
    assert(temperature_i2c_info.sda.port == GPIOC && temperature_i2c_info.sda.pin == GPIO_PIN_9);
#else
    assert(NUMBER_TEMPERATURE_SENSORS == 0);
    assert(NUMBER_IMU_SENSORS == 0);
#endif
#endif
    printf("0.%d.0 motors=%d encoders=%d gpio=%d pwm=%u model=%d\n",
           KINISI_BOARD_VERSION_MINOR, NUMBER_MOTORS, NUMBER_ENCODERS,
           NUMBER_GPIO_PINS, MOTOR_PWM_FREQUENCY_HZ, MOTOR_CONTROL_MODEL);
    for (unsigned i = 0; i < NUMBER_MOTORS; ++i) {
        const motor_info_t *m = &motor_info[i];
#ifndef BASELINE_CONFIG
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
        const uint16_t current_pins[] = {GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_0, GPIO_PIN_1};
        assert(m->currentSense.available && m->currentSense.input.port == GPIOC);
        assert(m->currentSense.type == CURRENT_SENSE_ADC);
        assert(m->currentSense.input.pin == current_pins[i]);
        assert(m->currentSense.senseResistorOhm == 330 && m->currentSense.filterCapacitanceNf == 1000);
        const uint32_t adc_channels[] = {14, 15, 10, 11};
        assert(NUMBER_CURRENT_SENSORS == 4 && m->currentSense.adcChannel == adc_channels[i]);
        assert(m->currentSense.referenceMillivolts == 3300 && m->currentSense.currentMirrorRatio == 1100);
        assert(m->hardwareCurrentLimit.available && m->hardwareCurrentLimit.enabled);
        assert(m->hardwareCurrentLimit.thresholdMa == 6500);
        const uint16_t thresholds[] = {4000, 5400, 6500, 7000};
        assert(m->hardwareCurrentLimit.supportedThresholdCount == sizeof(thresholds) / sizeof(thresholds[0]));
        assert(m->hardwareCurrentLimit.supportedThresholdsMa != NULL);
        for (unsigned j = 0; j < m->hardwareCurrentLimit.supportedThresholdCount; ++j) {
            assert(m->hardwareCurrentLimit.supportedThresholdsMa[j] == thresholds[j]);
        }
        const overcurrent_protection_mode_t modes[] = {
            OVERCURRENT_PROTECTION_LATCHED_SHUTDOWN, OVERCURRENT_PROTECTION_AUTOMATIC_RETRY,
            OVERCURRENT_PROTECTION_REPORT_ONLY, OVERCURRENT_PROTECTION_DISABLED
        };
        assert(m->overcurrentProtection.available);
        assert(m->overcurrentProtection.mode == OVERCURRENT_PROTECTION_LATCHED_SHUTDOWN);
        assert(m->overcurrentProtection.supportedModeCount == sizeof(modes) / sizeof(modes[0]));
        assert(m->overcurrentProtection.supportedModes != NULL);
        for (unsigned j = 0; j < m->overcurrentProtection.supportedModeCount; ++j) {
            assert(m->overcurrentProtection.supportedModes[j] == modes[j]);
        }
#else
        assert(!m->currentSense.available && !m->hardwareCurrentLimit.available);
        assert(m->currentSense.type == CURRENT_SENSE_NONE);
        assert(m->hardwareCurrentLimit.supportedThresholdCount == 0);
        assert(m->hardwareCurrentLimit.supportedThresholdsMa == NULL);
        assert(!m->overcurrentProtection.available);
        assert(m->overcurrentProtection.mode == OVERCURRENT_PROTECTION_NONE);
        assert(m->overcurrentProtection.supportedModeCount == 0);
        assert(m->overcurrentProtection.supportedModes == NULL);
#endif
#endif
        printf("M%u model=%d TIM%ld", i, m->controlModel, (long)(m->timer - test_timers + 1));
        pwm(m->pwmChannel1); pwm(m->pwmChannel2); pwm(m->enable);
        pin(m->direction); pin(m->disable); pin(m->sleep); pin(m->fault); pin(m->chipSelect);
        puts("");
    }
    for (unsigned i = 0; i < NUMBER_ENCODERS; ++i) {
        const encoder_info_t *e = &encoder_info[i];
        printf("E%u TIM%ld", i, (long)(e->timer - test_timers + 1));
        pin((gpio_info_t){e->portA, e->pinA}); pin((gpio_info_t){e->portB, e->pinB}); puts("");
    }
    for (unsigned i = 0; i < NUMBER_GPIO_PINS; ++i) {
        printf("G%u", i); pin(gpio_info[i]); puts("");
    }
    printf("L0");
#ifdef BASELINE_CONFIG
    pin((gpio_info_t){STATUS_LED_PORT, STATUS_LED_PIN});
#else
    pin(status_led_info[0].gpio);
    if (NUMBER_STATUS_LEDS != 1 || status_led_info[0].activeLow) return 1;
#endif
    puts("");
    return 0;
}
