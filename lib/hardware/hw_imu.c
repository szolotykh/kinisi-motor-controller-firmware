#include "hw_imu.h"
#include "hw_config.h"

#if NUMBER_IMU_SENSORS > 0
#include "hw_timers.h"
#include <string.h>

// Bosch BMI088 datasheet, rev 1.9, sections 4.1, 5 and 6.1.
// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf
#define IMU_SPI_TIMEOUT_MS 10U
#define IMU_SETTLING_MS 50U
#define ACCEL_CONFIG 0xA8U // 100 Hz, normal filter
#define ACCEL_RANGE 0x01U // +/-6 g
#define GYRO_CONFIG 0x07U // 100 Hz, 32 Hz bandwidth
#define GYRO_RANGE 0x02U // +/-500 degrees/s

static SPI_HandleTypeDef imu_spi;
static bool bus_ready;
static bool configured[NUMBER_IMU_SENSORS];
static uint32_t configured_at[NUMBER_IMU_SENSORS];

static bool prepare_bus(void)
{
    if (bus_ready) return true;
    // Deassert every slave before enabling the bus, including on recovery.
    for (unsigned i = 0; i < NUMBER_IMU_SENSORS; ++i) {
        const gpio_info_t selects[] = {imu_sensor_info[i].accelChipSelect, imu_sensor_info[i].gyroChipSelect};
        for (unsigned j = 0; j < 2; ++j) {
            rcc_gpio_clk_enable(selects[j].port);
            HAL_GPIO_WritePin(selects[j].port, selects[j].pin, GPIO_PIN_SET);
            GPIO_InitTypeDef gpio = {0};
            gpio.Pin = selects[j].pin;
            gpio.Mode = GPIO_MODE_OUTPUT_PP;
            gpio.Pull = GPIO_NOPULL;
            gpio.Speed = GPIO_SPEED_FREQ_HIGH;
            HAL_GPIO_Init(selects[j].port, &gpio);
        }
    }
    IMU_SPI_CLK_ENABLE();
    const gpio_info_t pins[] = {imu_spi_info.sck, imu_spi_info.miso, imu_spi_info.mosi};
    for (unsigned i = 0; i < 3; ++i) {
        rcc_gpio_clk_enable(pins[i].port);
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin = pins[i].pin;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = imu_spi_info.alternate;
        HAL_GPIO_Init(pins[i].port, &gpio);
    }
    imu_spi.Instance = imu_spi_info.instance;
    imu_spi.Init.Mode = SPI_MODE_MASTER;
    imu_spi.Init.Direction = SPI_DIRECTION_2LINES;
    imu_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    imu_spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    imu_spi.Init.CLKPhase = SPI_PHASE_1EDGE; // BMI088 SPI mode 0
    imu_spi.Init.NSS = SPI_NSS_SOFT;
    imu_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; // APB1 / 64 = 656.25 kHz
    imu_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    bus_ready = HAL_SPI_Init(&imu_spi) == HAL_OK;
    return bus_ready;
}

static bool transfer(uint8_t index, bool accel, uint8_t *tx, uint8_t *rx, uint16_t length)
{
    const gpio_info_t cs = accel ? imu_sensor_info[index].accelChipSelect : imu_sensor_info[index].gyroChipSelect;
    HAL_GPIO_WritePin(cs.port, cs.pin, GPIO_PIN_RESET);
    const bool ok = HAL_SPI_TransmitReceive(&imu_spi, tx, rx, length, IMU_SPI_TIMEOUT_MS) == HAL_OK;
    HAL_GPIO_WritePin(cs.port, cs.pin, GPIO_PIN_SET); // Always release CS, including timeout.
    return ok;
}

static bool read_registers(uint8_t index, bool accel, uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t tx[8] = {0}, rx[8];
    // Accel has an extra dummy byte after the address; gyro does not.
    const unsigned offset = accel ? 2 : 1;
    if (length > sizeof(tx) - offset) return false;
    tx[0] = reg | 0x80U;
    if (!transfer(index, accel, tx, rx, length + offset)) return false;
    memcpy(data, rx + offset, length);
    return true;
}

static bool write_register(uint8_t index, bool accel, uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {reg, value}, rx[2];
    const bool ok = transfer(index, accel, tx, rx, sizeof(tx));
    // Also satisfies the 1 ms spacing needed while the accelerometer is suspended.
    HAL_Delay(1);
    return ok;
}

static bool identity_ok(uint8_t index)
{
    uint8_t accel, gyro;
    return read_registers(index, true, 0x00, &accel, 1) && accel == 0x1E &&
           read_registers(index, false, 0x00, &gyro, 1) && gyro == 0x0F;
}

static bool settings_ok(uint8_t index)
{
    uint8_t accel[2], power[2], gyro[3];
    return identity_ok(index) &&
           read_registers(index, true, 0x40, accel, sizeof(accel)) &&
           accel[0] == ACCEL_CONFIG && (accel[1] & 3U) == ACCEL_RANGE &&
           read_registers(index, true, 0x7C, power, sizeof(power)) && power[0] == 0 && power[1] == 4 &&
           read_registers(index, false, 0x0F, gyro, sizeof(gyro)) &&
           (gyro[0] & 7U) == GYRO_RANGE && (gyro[1] & 0x7FU) == GYRO_CONFIG && gyro[2] == 0;
}

static bool prepare_sensor(uint8_t index)
{
    if (configured[index]) return true;
    if (!prepare_bus()) return false;
    uint8_t dummy;
    // First accel read selects SPI; its result is deliberately discarded.
    if (!read_registers(index, true, 0x00, &dummy, 1) || !identity_ok(index)) return false;
    if (!write_register(index, true, 0x7E, 0xB6)) return false;
    const bool gyro_reset_ok = write_register(index, false, 0x14, 0xB6);
    HAL_Delay(30); // No gyro access during reset/startup.
    // A failed transfer may still have reached the device; honor reset time anyway.
    if (!gyro_reset_ok) return false;
    if (!read_registers(index, true, 0x00, &dummy, 1) || !identity_ok(index)) return false;
    if (!write_register(index, true, 0x7C, 0)) return false;
    HAL_Delay(5);
    if (!write_register(index, true, 0x7D, 4)) return false;
    HAL_Delay(5);
    if (!write_register(index, true, 0x40, ACCEL_CONFIG) ||
        !write_register(index, true, 0x41, ACCEL_RANGE) ||
        !write_register(index, false, 0x0F, GYRO_RANGE) ||
        !write_register(index, false, 0x10, GYRO_CONFIG)) return false;
    configured_at[index] = HAL_GetTick();
    configured[index] = true;
    return true;
}

static int32_t signed_sample(const uint8_t *bytes)
{
    const uint16_t raw = (uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8);
    return raw & 0x8000U ? (int32_t)raw - 65536 : raw;
}
#endif

void hw_imu_init(void)
{
#if NUMBER_IMU_SENSORS > 0
    HAL_Delay(30); // Allow gyro power-on startup even on a fast MCU boot.
    for (uint8_t i = 0; i < NUMBER_IMU_SENSORS; ++i) (void)prepare_sensor(i);
#endif
}

hw_imu_status_t hw_imu_read(uint8_t index, hw_imu_sample_t *sample)
{
#if NUMBER_IMU_SENSORS > 0
    if (index >= NUMBER_IMU_SENSORS || imu_sensor_info[index].type != IMU_SENSOR_BMI088)
        return HW_IMU_UNAVAILABLE;
    if (!sample || !prepare_sensor(index)) return HW_IMU_IO_ERROR;
    if ((uint32_t)(HAL_GetTick() - configured_at[index]) < IMU_SETTLING_MS) return HW_IMU_NOT_READY;
    uint8_t accel[6], gyro[6];
    // Detect sensor reset, floating MISO, or changed settings before scaling data.
    // Each burst is coherent for its sensor; accel and gyro are not synchronized.
    if (!settings_ok(index) || !read_registers(index, true, 0x12, accel, sizeof(accel)) ||
        !read_registers(index, false, 0x02, gyro, sizeof(gyro))) {
        configured[index] = false;
        return HW_IMU_IO_ERROR;
    }
    const double accel_scale = 6.0 * 9.80665 / 32768.0;
    const double gyro_scale = 500.0 * 0.017453292519943295 / 32768.0;
    *sample = (hw_imu_sample_t){
        signed_sample(accel) * accel_scale, signed_sample(accel + 2) * accel_scale,
        signed_sample(accel + 4) * accel_scale, signed_sample(gyro) * gyro_scale,
        signed_sample(gyro + 2) * gyro_scale, signed_sample(gyro + 4) * gyro_scale
    };
    return HW_IMU_OK;
#else
    (void)index;
    (void)sample;
    return HW_IMU_UNAVAILABLE;
#endif
}
