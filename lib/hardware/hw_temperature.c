#include "hw_temperature.h"
#include "hw_config.h"

#if NUMBER_TEMPERATURE_SENSORS > 0
#include "hw_timers.h"

#define TMP102_TEMPERATURE_REGISTER 0U
#define TMP102_CONFIGURATION_REGISTER 1U
#define TEMPERATURE_I2C_TIMEOUT_MS 10U
#define TEMPERATURE_BUS_CLEAR_TIMEOUT_MS 50U
// Allow a complete 4 Hz conversion interval plus conversion time after setup.
#define TMP102_STARTUP_MS 300U

static I2C_HandleTypeDef sensor_i2c;
static bool bus_ready;
static bool bus_recovery_needed;
static bool configured[NUMBER_TEMPERATURE_SENSORS];
static uint32_t configured_at[NUMBER_TEMPERATURE_SENSORS];

static void invalidate_bus(void)
{
    bus_ready = false;
    bus_recovery_needed = true;
    for (unsigned i = 0; i < NUMBER_TEMPERATURE_SENSORS; ++i) configured[i] = false;
}

static void configure_bus_pins(uint32_t mode)
{
    const gpio_info_t pins[] = {temperature_i2c_info.scl, temperature_i2c_info.sda};
    for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        rcc_gpio_clk_enable(pins[i].port);
        if (mode == GPIO_MODE_OUTPUT_OD)
            HAL_GPIO_WritePin(pins[i].port, pins[i].pin, GPIO_PIN_SET);
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin = pins[i].pin;
        gpio.Mode = mode;
        gpio.Pull = GPIO_NOPULL; // Board provides external pull-ups.
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = temperature_i2c_info.alternate;
        HAL_GPIO_Init(pins[i].port, &gpio);
    }
}

static bool wait_clock_high(uint32_t started)
{
    const gpio_info_t scl = temperature_i2c_info.scl;
    while ((uint32_t)(HAL_GetTick() - started) < TEMPERATURE_BUS_CLEAR_TIMEOUT_MS) {
        if (HAL_GPIO_ReadPin(scl.port, scl.pin) == GPIO_PIN_SET) return true;
        HAL_Delay(1);
    }
    return false;
}

static bool recover_bus(void)
{
    // A HAL timeout can exit before STOP, leaving BUSY set or a slave mid-byte.
    // Release the peripheral, clock out at most one byte/ACK, then issue STOP.
    // This is serialized with sensor reads; it never touches external I2C2.
    const gpio_info_t scl = temperature_i2c_info.scl, sda = temperature_i2c_info.sda;
    (void)HAL_I2C_DeInit(&sensor_i2c);
    configure_bus_pins(GPIO_MODE_OUTPUT_OD);
    uint32_t started = HAL_GetTick();
    bool cleared = false;
    if (!wait_clock_high(started)) goto release;
    for (unsigned pulse = 0; pulse < 9 && HAL_GPIO_ReadPin(sda.port, sda.pin) == GPIO_PIN_RESET; ++pulse) {
        HAL_GPIO_WritePin(scl.port, scl.pin, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(scl.port, scl.pin, GPIO_PIN_SET);
        if (!wait_clock_high(started)) goto release;
        HAL_Delay(1);
    }
    // Change SDA while SCL is low, then release SCL before SDA to generate STOP.
    HAL_GPIO_WritePin(scl.port, scl.pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(sda.port, sda.pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(scl.port, scl.pin, GPIO_PIN_SET);
    if (!wait_clock_high(started)) goto release;
    HAL_Delay(1);
    HAL_GPIO_WritePin(sda.port, sda.pin, GPIO_PIN_SET);
    HAL_Delay(1);
    cleared = (uint32_t)(HAL_GetTick() - started) < TEMPERATURE_BUS_CLEAR_TIMEOUT_MS &&
              HAL_GPIO_ReadPin(scl.port, scl.pin) == GPIO_PIN_SET &&
              HAL_GPIO_ReadPin(sda.port, sda.pin) == GPIO_PIN_SET;
release:
    // Never hold either line low when recovery fails; a later request can retry.
    HAL_GPIO_WritePin(scl.port, scl.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(sda.port, sda.pin, GPIO_PIN_SET);
    return cleared;
}

static bool prepare_bus(void)
{
    if (bus_ready) return true;
    TEMPERATURE_I2C_CLK_ENABLE();
    if (bus_recovery_needed && !recover_bus()) return false;
    configure_bus_pins(GPIO_MODE_AF_OD);
    sensor_i2c.Instance = temperature_i2c_info.instance;
    sensor_i2c.Init.ClockSpeed = temperature_i2c_info.frequencyHz;
    sensor_i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
    sensor_i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    sensor_i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    sensor_i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    sensor_i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    // HAL_Init resets the STM32 peripheral (SWRST), including its BUSY latch.
    bus_ready = HAL_I2C_Init(&sensor_i2c) == HAL_OK;
    if (bus_ready) bus_recovery_needed = false;
    else invalidate_bus();
    return bus_ready;
}

static uint16_t address(uint8_t index)
{
    // STM32 HAL expects the 7-bit address shifted left by one.
    return (uint16_t)temperature_sensor_info[index].address << 1;
}

static bool prepare_sensor(uint8_t index)
{
    if (configured[index]) return true;
    if (!prepare_bus()) return false;
    // Continuous conversion, 4 Hz, normal 12-bit mode. ALERT is unconnected.
    uint8_t config[2] = {0x60, 0x80};
    if (HAL_I2C_Mem_Write(&sensor_i2c, address(index), TMP102_CONFIGURATION_REGISTER,
                         I2C_MEMADD_SIZE_8BIT, config, sizeof(config),
                         TEMPERATURE_I2C_TIMEOUT_MS) != HAL_OK) {
        invalidate_bus();
        return false;
    }
    if (HAL_I2C_Mem_Read(&sensor_i2c, address(index), TMP102_CONFIGURATION_REGISTER,
                        I2C_MEMADD_SIZE_8BIT, config, sizeof(config),
                        TEMPERATURE_I2C_TIMEOUT_MS) != HAL_OK) {
        invalidate_bus();
        return false;
    }
    // Ignore OS/AL status and reserved bits; verify resolution and all settings.
    const uint16_t value = ((uint16_t)config[0] << 8) | config[1];
    if ((value & 0x7FD0U) != 0x6080U) return false;
    configured_at[index] = HAL_GetTick();
    configured[index] = true;
    return true;
}
#endif

void hw_temperature_init(void)
{
#if NUMBER_TEMPERATURE_SENSORS > 0
    for (uint8_t i = 0; i < NUMBER_TEMPERATURE_SENSORS; ++i) {
        (void)prepare_sensor(i);
    }
#endif
}

hw_temperature_status_t hw_temperature_read(uint8_t index, double *celsius)
{
#if NUMBER_TEMPERATURE_SENSORS > 0
    if (index >= NUMBER_TEMPERATURE_SENSORS ||
        temperature_sensor_info[index].type != TEMPERATURE_SENSOR_TMP102)
        return HW_TEMPERATURE_UNAVAILABLE;
    if (!celsius) return HW_TEMPERATURE_IO_ERROR;
    if (!prepare_sensor(index)) return HW_TEMPERATURE_IO_ERROR;
    if ((uint32_t)(HAL_GetTick() - configured_at[index]) < TMP102_STARTUP_MS)
        return HW_TEMPERATURE_NOT_READY;
    uint8_t bytes[2];
    if (HAL_I2C_Mem_Read(&sensor_i2c, address(index), TMP102_TEMPERATURE_REGISTER,
                        I2C_MEMADD_SIZE_8BIT, bytes, sizeof(bytes),
                        TEMPERATURE_I2C_TIMEOUT_MS) != HAL_OK) {
        invalidate_bus(); // Recover the bus before attempting another transfer.
        return HW_TEMPERATURE_IO_ERROR;
    }
    if (bytes[1] & 0x0FU) {
        configured[index] = false; // Retry configuration on the next request.
        return HW_TEMPERATURE_IO_ERROR;
    }
    // TMP102 transmits MSB first, with a signed 12-bit result in bits 15:4.
    int16_t sample = ((uint16_t)bytes[0] << 4) | (bytes[1] >> 4);
    if (sample & 0x0800) sample -= 4096;
    *celsius = (double)sample / 16.0;
    return HW_TEMPERATURE_OK;
#else
    (void)index;
    (void)celsius;
    return HW_TEMPERATURE_UNAVAILABLE;
#endif
}
