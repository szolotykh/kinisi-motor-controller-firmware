#include "hw_imu.h"
#include "hw_config.h"

#if NUMBER_IMU_SENSORS > 0
#include "hw_timers.h"
#include "hw_clock.h"
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
static bool spi_in_use;
typedef struct { uint64_t earliest, latest; double xyz[3]; } axis_event_t;
typedef struct { axis_event_t events[4]; unsigned next; } axis_history_t;
static struct {
    uint32_t epoch;
    uint64_t accel_probe, gyro_probe, verified_at;
} producer;
static struct {
    uint32_t epoch;
    hw_imu_status_t status;
    axis_history_t accel, gyro;
} cache;

// Only the cache metadata or one fixed-size sample is touched with IRQs masked.
// SPI transfers and sensor restart delays run outside this section.
static void invalidate(uint32_t epoch, hw_imu_status_t status)
{
    uint32_t irq = __get_PRIMASK(); __disable_irq();
    cache.epoch = epoch;
    cache.status = status;
    cache.accel.next = cache.gyro.next = 0;
    __set_PRIMASK(irq);
}

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
    memset(&producer, 0, sizeof(producer));
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

static hw_imu_status_t read_direct(uint8_t index, hw_imu_sample_t *sample)
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

hw_imu_status_t hw_imu_read(uint8_t index, hw_imu_sample_t *sample)
{
#if NUMBER_IMU_SENSORS > 0
    if (__atomic_test_and_set(&spi_in_use, __ATOMIC_ACQUIRE)) return HW_IMU_NOT_READY;
#endif
    hw_imu_status_t status = read_direct(index, sample);
#if NUMBER_IMU_SENSORS > 0
    if (index == 0 && sample && status == HW_IMU_IO_ERROR) invalidate(producer.epoch, status);
    __atomic_clear(&spi_in_use, __ATOMIC_RELEASE);
#endif
    return status;
}

#if NUMBER_IMU_SENSORS > 0
static void publish(axis_history_t *history, uint64_t earliest, uint64_t latest, const uint8_t *data, double scale)
{
    // A preempted/late poll provides no tight source time. Leave the previous
    // verified events eligible only for their original lifetime.
    if (!earliest || latest < earliest || latest - earliest > 5000) return;
    axis_event_t event = {.earliest=earliest, .latest=latest};
    for (unsigned i = 0; i < 3; ++i) event.xyz[i] = signed_sample(data + 2*i)*scale;
    uint32_t irq = __get_PRIMASK(); __disable_irq();
    history->events[history->next % 4] = event;
    if (++history->next == 8) history->next = 4;
    cache.status = HW_IMU_OK;
    __set_PRIMASK(irq);
}

static hw_imu_status_t poll_sensor(uint32_t epoch)
{
    if (!prepare_sensor(0)) return HW_IMU_IO_ERROR;
    if ((uint32_t)(HAL_GetTick()-configured_at[0]) < IMU_SETTLING_MS) return HW_IMU_NOT_READY;
    uint8_t ready, count, mode, bytes[6];
    uint64_t now = hw_clock_microseconds();
    if (producer.epoch != epoch) {
        invalidate(epoch, HW_IMU_NOT_READY);
        if (!settings_ok(0) || !write_register(0, false, 0x3E, 0x40) ||
            !read_registers(0, false, 0x3E, &mode, 1) || mode != 0x40 ||
            !read_registers(0, true, 0x1D, &ready, 1)) return HW_IMU_IO_ERROR;
        // Writing the defined FIFO mode both flushes and starts it (Bosch 5.5.16).
        producer.epoch = epoch;
        producer.gyro_probe = producer.accel_probe = producer.verified_at = now;
        return HW_IMU_OK;
    }
    if (now - producer.verified_at >= 20000) {
        if (!settings_ok(0) || !read_registers(0, false, 0x3E, &mode, 1) || mode != 0x40)
            return HW_IMU_IO_ERROR;
        producer.verified_at = now;
    }

    // ACC_INT_STAT_1 clears on read independently of normal GET_IMU data bursts.
    uint64_t begin = hw_clock_microseconds();
    if (!read_registers(0, true, 0x1D, &ready, 1)) return HW_IMU_IO_ERROR;
    uint64_t end = hw_clock_microseconds(), previous = producer.accel_probe;
    producer.accel_probe = begin;
    if (ready & 0x80) {
        if (!read_registers(0, true, 0x12, bytes, 6)) return HW_IMU_IO_ERROR;
        begin = hw_clock_microseconds();
        if (!read_registers(0, true, 0x1D, &ready, 1)) return HW_IMU_IO_ERROR;
        producer.accel_probe = begin;
        // If another payload arrived while reading, neither event is associated
        // unambiguously. Consume the indication and wait for a new event.
        if (!(ready & 0x80)) publish(&cache.accel, previous, end, bytes, 6.0*9.80665/32768.0);
    }

    begin = hw_clock_microseconds();
    if (!read_registers(0, false, 0x0E, &count, 1) || count > 8) return HW_IMU_IO_ERROR;
    end = hw_clock_microseconds(); previous = producer.gyro_probe;
    producer.gyro_probe = begin;
    if (count) {
        // Only remove the frames counted at this inspection. Later frames remain in FIFO.
        for (unsigned i = 0; i < count; ++i)
            if (!read_registers(0, false, 0x3F, bytes, 6)) return HW_IMU_IO_ERROR;
        publish(&cache.gyro, previous, end, bytes, 500.0*0.017453292519943295/32768.0);
    }
    return HW_IMU_OK;
}

static bool newest_before(const axis_history_t *history, uint64_t at, axis_event_t *event)
{
    unsigned count = history->next < 4 ? history->next : 4;
    for (unsigned j = 1; j <= count; ++j) {
        const axis_event_t *p = &history->events[(history->next-j) % 4];
        if (p->latest <= at) { *event = *p; return true; }
    }
    return false;
}
#endif

void hw_imu_poll_observations(uint32_t epoch)
{
#if NUMBER_IMU_SENSORS > 0
    if (!epoch || __atomic_test_and_set(&spi_in_use, __ATOMIC_ACQUIRE)) return;
    hw_imu_status_t status = poll_sensor(epoch);
    if (status != HW_IMU_OK) {
        if (status == HW_IMU_IO_ERROR) configured[0] = false;
        invalidate(epoch, status);
    }
    __atomic_clear(&spi_in_use, __ATOMIC_RELEASE);
#else
    (void)epoch;
#endif
}

hw_imu_status_t hw_imu_read_before(uint32_t epoch, uint64_t at, hw_imu_sample_t *sample,
                                    uint64_t *accel_us, uint64_t *gyro_us)
{
#if NUMBER_IMU_SENSORS > 0
    if (!sample || !accel_us || !gyro_us) return HW_IMU_IO_ERROR;
    axis_event_t a, g;
    uint32_t irq = __get_PRIMASK(); __disable_irq();
    hw_imu_status_t status = epoch && cache.epoch == epoch ? cache.status : HW_IMU_NOT_READY;
    bool both = status == HW_IMU_OK && newest_before(&cache.accel, at, &a) && newest_before(&cache.gyro, at, &g);
    __set_PRIMASK(irq);
    if (!both) return status == HW_IMU_OK ? HW_IMU_NOT_READY : status;
    // Use the worst possible skew across the full windows, not the separation
    // between poll start times. Report the older bound for conservative age.
    if (at-a.earliest > 35000 || at-g.earliest > 35000 ||
        (a.latest > g.earliest && a.latest-g.earliest > 15000) ||
        (g.latest > a.earliest && g.latest-a.earliest > 15000)) return HW_IMU_STALE;
    *sample = (hw_imu_sample_t){a.xyz[0],a.xyz[1],a.xyz[2],g.xyz[0],g.xyz[1],g.xyz[2]};
    *accel_us = a.earliest; *gyro_us = g.earliest;
    return HW_IMU_OK;
#else
    (void)epoch; (void)at; (void)sample; (void)accel_us; (void)gyro_us;
    return HW_IMU_UNAVAILABLE;
#endif
}

hw_imu_status_t hw_imu_read_observation(uint8_t index, hw_imu_sample_t *sample,
                                         uint64_t *accel_us, uint64_t *gyro_us)
{
#if NUMBER_IMU_SENSORS > 0
    if (index != 0) return HW_IMU_UNAVAILABLE;
    uint32_t irq = __get_PRIMASK(); __disable_irq();
    uint32_t epoch = cache.epoch;
    __set_PRIMASK(irq);
    return hw_imu_read_before(epoch, hw_clock_microseconds(), sample, accel_us, gyro_us);
#else
    (void)index; (void)sample; (void)accel_us; (void)gyro_us;
    return HW_IMU_UNAVAILABLE;
#endif
}
