#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "hw_config.h"
#include "hw_imu.h"

GPIO_TypeDef test_ports[5];
TIM_TypeDef test_timers[12];

#if NUMBER_IMU_SENSORS > 0
static uint32_t tick, gyro_reset_at, accel_reset_at, last_write;
static unsigned clocks, gpio_calls, transfers, writes;
static bool fail_init, fail_transfer, bad_id, reject_config, fail_gyro_reset, fail_gyro_sample;
static bool accel_spi, gyro_reset, accel_reset, wrote;
static uint16_t selected;
static uint8_t acc_regs[128], gyro_regs[128];
static uint32_t irq_mask;
static bool sample_race, interleave_direct, snapshot_inside_raw;
static uint64_t cache_at, cached_a, cached_g;
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1; }
void __set_PRIMASK(uint32_t value) { irq_mask = value; }

static void reset_accel(void) {
    memset(acc_regs, 0, sizeof(acc_regs));
    acc_regs[0] = 0x1E; acc_regs[0x40] = 0xA8; acc_regs[0x41] = 1; acc_regs[0x7C] = 3;
    accel_spi = false;
}
static void reset_gyro(void) {
    memset(gyro_regs, 0, sizeof(gyro_regs));
    gyro_regs[0] = 0x0F; gyro_regs[0x10] = 0x80;
}
uint64_t hw_clock_microseconds(void) { return (uint64_t)tick * 1000; }
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }
void test_spi2_clock_enable(void) { ++clocks; }
void rcc_gpio_clk_enable(GPIO_TypeDef *port) { assert(port == GPIOB || port == GPIOC || port == GPIOD); }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio) {
    if (port == GPIOD) {
        assert(gpio->Pin == GPIO_PIN_10 || gpio->Pin == GPIO_PIN_12);
        assert(gpio->Mode == GPIO_MODE_OUTPUT_PP);
        assert(port->ODR & gpio->Pin); // CS goes high before output is enabled.
    } else {
        assert((port == GPIOB && gpio->Pin == GPIO_PIN_13) ||
               (port == GPIOC && (gpio->Pin == GPIO_PIN_2 || gpio->Pin == GPIO_PIN_3)));
        assert(gpio->Mode == GPIO_MODE_AF_PP && gpio->Alternate == GPIO_AF5_SPI2);
    }
    assert(gpio->Pull == GPIO_NOPULL);
    ++gpio_calls;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state) {
    assert(port == GPIOD && (pin == GPIO_PIN_10 || pin == GPIO_PIN_12));
    if (state == GPIO_PIN_RESET) {
        assert(!selected); selected = pin; port->ODR &= ~pin;
    } else {
        assert(!selected || selected == pin);
        selected = 0; port->ODR |= pin;
    }
}
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *bus) {
    assert(bus->Instance == SPI2); // Must not touch motor SPI3.
    assert(bus->Init.CLKPolarity == SPI_POLARITY_LOW && bus->Init.CLKPhase == SPI_PHASE_1EDGE);
    assert(bus->Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_64);
    assert(bus->Init.Mode == SPI_MODE_MASTER && bus->Init.NSS == SPI_NSS_SOFT);
    assert(bus->Init.DataSize == SPI_DATASIZE_8BIT && bus->Init.FirstBit == SPI_FIRSTBIT_MSB);
    return fail_init ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *bus, uint8_t *tx, uint8_t *rx,
                                         uint16_t length, uint32_t timeout) {
    assert(bus->Instance == SPI2 && selected && timeout == 10);
    assert(!irq_mask); // Physical SPI work is always interruptible.
    if (interleave_direct) {
        interleave_direct = false;
        hw_imu_poll_observations(1); // GET owns SPI; this poll must preserve cache.
        hw_imu_sample_t sample;
        assert(hw_imu_read_before(1, cache_at, &sample, &cached_a, &cached_g) == HW_IMU_OK);
        snapshot_inside_raw = true;
    }
    ++transfers;
    if (fail_transfer) return HAL_ERROR;
    bool accel = selected == GPIO_PIN_10;
    if (!accel && gyro_reset) assert((uint32_t)(tick - gyro_reset_at) >= 30);
    if (accel && accel_reset) assert((uint32_t)(tick - accel_reset_at) >= 1);
    uint8_t *regs = accel ? acc_regs : gyro_regs;
    memset(rx, 0xCC, length); // Poison dummy bytes to expose offset mistakes.
    unsigned reg = tx[0] & 0x7F;
    if (tx[0] & 0x80) {
        if (!accel && reg == 0x02 && fail_gyro_sample) return HAL_ERROR;
        const unsigned offset = accel ? 2 : 1;
        assert(length > offset && reg + length - offset <= 128);
        if (accel && !accel_spi) { accel_spi = true; return HAL_OK; }
        memcpy(rx + offset, regs + reg, length - offset);
        if (accel && reg == 0x12 && sample_race) {
            acc_regs[0x1D] = 0x80;
            acc_regs[0x12]++;
            sample_race = false;
        }
        if (accel && reg == 0x1D) acc_regs[reg] = 0; // Read-to-clear DRDY.
        if (!accel && reg == 0x3F) {
            assert(gyro_regs[0x0E] > 0 && length == 7);
            --gyro_regs[0x0E];
        }
        if (reg == 0 && bad_id) rx[offset] = 0xFF;
    } else {
        assert(length == 2);
        if (wrote) assert((uint32_t)(tick - last_write) >= 1);
        last_write = tick; wrote = true; ++writes;
        if (accel) assert(accel_spi);
        if (accel && reg == 0x7E) {
            assert(tx[1] == 0xB6); reset_accel(); accel_reset = true; accel_reset_at = tick;
        } else if (!accel && reg == 0x14) {
            assert(tx[1] == 0xB6); reset_gyro(); gyro_reset = true; gyro_reset_at = tick;
            if (fail_gyro_reset) return HAL_ERROR; // Reset took effect despite a failed transfer.
        } else {
            if (!accel && reg == 0x3E) {
                assert(tx[1] == 0x40); // Only the documented FIFO-mode write.
                gyro_regs[0x0E] = 0;
            }
            if (!reject_config) regs[reg] = tx[1];
            if (!accel && reg == 0x10) regs[reg] |= 0x80; // Read-only bit 7.
        }
    }
    return HAL_OK;
}
static void expect_error(hw_imu_status_t status) {
    hw_imu_sample_t sample = {1, 2, 3, 4, 5, 6}, before = sample;
    assert(hw_imu_read(0, &sample) == status);
    assert(!memcmp(&sample, &before, sizeof(sample)));
    assert(!selected); // Errors never leave either slave selected.
}
static void poll_events(uint32_t epoch, unsigned delay_ms, bool accel, unsigned gyro_frames)
{
    tick += delay_ms;
    if (accel) acc_regs[0x1D] = 0x80;
    gyro_regs[0x0E] = gyro_frames;
    hw_imu_poll_observations(epoch);
}

static void check_continuity(uint32_t epoch)
{
    hw_imu_cursor_t cursor;
    hw_imu_sample_t sample;
    uint64_t a=0,g=0;
    hw_imu_begin_tracking(&cursor,epoch);
    tick+=2; hw_imu_poll_observations(epoch);
    poll_events(epoch,2,true,1);
    assert(hw_imu_read_tracking(&cursor,epoch,hw_clock_microseconds(),&sample,&a,&g)==HW_IMU_OK);
    for (unsigned problem=0;problem<6;++problem) {
        // Each fresh START snapshots the current cursors, including when fusion owns this FIFO.
        hw_imu_begin_tracking(&cursor,epoch);
        if (problem<=1) {
            // Reader sleeps for 20ms; producer remains on schedule. An intermediate
            // axis event may carry saturation even though the final value is ordinary.
            for (unsigned i=0;i<10;++i) {
                bool event = i==4 || i==9;
                if (event) {
                    acc_regs[0x12+1]=gyro_regs[0x3F+1]=i==4 ? 0x7f : 0;
                }
                poll_events(epoch,2,event && (problem==0 || i==9),event && (problem==1 || i==9));
            }
        } else if (problem==2) {
            poll_events(epoch,2,true,2);  // FIFO collapse to the newest of two real samples
        } else if (problem==3) {
            poll_events(epoch,9,true,1);  // wider than the physical acquisition bound
        } else if (problem==4) {
            sample_race=true;
            poll_events(epoch,2,true,1);  // a second accel event between DRDY and payload verification
        } else {
            // Real FIFO reset, then a change back to the same owner before the tracking reader ran.
            tick+=2; hw_imu_poll_observations(epoch+1);
            tick+=2; hw_imu_poll_observations(epoch);
            poll_events(epoch,2,true,1);
        }
        assert(hw_imu_read_tracking(&cursor,epoch,hw_clock_microseconds(),&sample,&a,&g)==HW_IMU_STALE);
        // The discontinuity remains visible after healthy newer events.
        poll_events(epoch,2,true,1);
        assert(hw_imu_read_tracking(&cursor,epoch,hw_clock_microseconds(),&sample,&a,&g)==HW_IMU_STALE);
        assert(hw_imu_read_before(epoch,hw_clock_microseconds(),&sample,&a,&g)==HW_IMU_OK);
    }
    hw_imu_begin_tracking(&cursor,epoch);
    poll_events(epoch,2,true,9);
    assert(hw_imu_read_tracking(&cursor,epoch,hw_clock_microseconds(),&sample,&a,&g)==HW_IMU_IO_ERROR);
    tick+=2; hw_imu_poll_observations(epoch); // Physical reconfigure and settle.
    tick+=52; hw_imu_poll_observations(epoch);
    poll_events(epoch,2,true,1);
    assert(hw_imu_read_before(epoch,hw_clock_microseconds(),&sample,&a,&g)==HW_IMU_OK);
    assert(hw_imu_read_tracking(&cursor,epoch,hw_clock_microseconds(),&sample,&a,&g)==HW_IMU_STALE);
}
static void put_sample(uint8_t *data, int16_t value) {
    data[0] = (uint8_t)value; data[1] = (uint8_t)((uint16_t)value >> 8);
}
#endif

int main(int argc, char **argv) {
#if NUMBER_IMU_SENSORS > 0
    const char *scenario = argc > 1 ? argv[1] : "normal";
    reset_accel(); reset_gyro();
    if (!strcmp(scenario, "wrap")) tick = UINT32_MAX - 100;
    fail_init = !strcmp(scenario, "init_failure");
    fail_transfer = !strcmp(scenario, "transfer_failure");
    bad_id = !strcmp(scenario, "wrong_id");
    reject_config = !strcmp(scenario, "bad_config");
    fail_gyro_reset = !strcmp(scenario, "reset_failure");
    hw_imu_init();
    if (fail_init || fail_transfer || bad_id || fail_gyro_reset) {
        expect_error(HW_IMU_IO_ERROR);
        fail_init = fail_transfer = bad_id = fail_gyro_reset = false;
    }
    expect_error(HW_IMU_NOT_READY);
    tick += 49;
    expect_error(HW_IMU_NOT_READY);
    ++tick;
    if (reject_config) {
        expect_error(HW_IMU_IO_ERROR);
        reject_config = false;
        expect_error(HW_IMU_NOT_READY); tick += 50;
    }
    assert(clocks && gpio_calls >= 5);
    const int16_t values[] = {0, 1, -1, 16384, -16384, 32767, -32768};
    for (unsigned n = 0; n < sizeof(values)/sizeof(values[0]); ++n) {
        for (unsigned axis = 0; axis < 3; ++axis) {
            put_sample(acc_regs + 0x12 + axis*2, values[(n+axis)%7]);
            put_sample(gyro_regs + 0x02 + axis*2, values[(n+axis+3)%7]);
        }
        hw_imu_sample_t sample;
        assert(hw_imu_read(0, &sample) == HW_IMU_OK);
        const double a[] = {sample.acceleration_x, sample.acceleration_y, sample.acceleration_z};
        const double g[] = {sample.angular_velocity_x, sample.angular_velocity_y, sample.angular_velocity_z};
        for (unsigned axis = 0; axis < 3; ++axis) {
            assert(fabs(a[axis] - values[(n+axis)%7] * (58.8399/32768.0)) < 1e-12);
            assert(fabs(g[axis] - values[(n+axis+3)%7] / 65.536 * (3.141592653589793/180)) < 1e-12);
        }
    }
    if (!strcmp(scenario, "normal")) {
        check_continuity(UINT32_C(0x80000001));
        check_continuity(17);
        uint64_t a = 0, g = 0, origin = hw_clock_microseconds();
        hw_imu_sample_t raw;
        hw_imu_poll_observations(1);
        assert(gyro_regs[0x3E] == 0x40 && hw_imu_read_before(1, origin, &raw, &a, &g) == HW_IMU_NOT_READY);
        tick += 2; gyro_regs[0x0E] = 2; acc_regs[0x1D] = 0x80;
        put_sample(gyro_regs + 0x43, 500);
        hw_imu_poll_observations(1);
        cache_at = hw_clock_microseconds();
        assert(hw_imu_read_before(1, cache_at, &raw, &a, &g) == HW_IMU_OK);
        assert(raw.angular_velocity_z > 0 && a == origin && g == origin && !gyro_regs[0x0E]);
        assert(hw_imu_read_before(1, cache_at-1, &raw, &a, &g) == HW_IMU_NOT_READY);
        interleave_direct = true;
        assert(hw_imu_read(0, &raw) == HW_IMU_OK && snapshot_inside_raw && cached_a == a && cached_g == g);
        tick += 2; hw_imu_poll_observations(1);
        tick += 2; acc_regs[0x1D] = 0x80; sample_race = true; hw_imu_poll_observations(1);
        assert(hw_imu_read_observation(0, &raw, &a, &g) == HW_IMU_OK && a == origin && g == origin);
        tick += 2; hw_imu_poll_observations(1); // The racing B payload cannot be counted twice.
        assert(hw_imu_read_observation(0, &raw, &a, &g) == HW_IMU_OK && a == origin);
        tick += 2; acc_regs[0x1D] = 0x80; hw_imu_poll_observations(1);
        assert(hw_imu_read_observation(0, &raw, &a, &g) == HW_IMU_OK && a > origin && g == origin);
        uint64_t stored_a = a;
        tick += 2; gyro_regs[0x0E] = 1; hw_imu_poll_observations(1);
        assert(hw_imu_read_observation(0, &raw, &a, &g) == HW_IMU_OK && a == stored_a && g > a);
        // A delayed poll cannot stamp uncertain events as simultaneous; retain only bounded history.
        uint64_t old_g = g;
        tick += 20; acc_regs[0x1D] = 0x80; gyro_regs[0x0E] = 1; hw_imu_poll_observations(1);
        assert(hw_imu_read_observation(0, &raw, &a, &g) == HW_IMU_OK && a == stored_a && g == old_g);
        tick += 2; gyro_regs[0x0E] = 1; hw_imu_poll_observations(1);
        assert(hw_imu_read_observation(0, &raw, &a, &g) == HW_IMU_STALE); // Frozen accel, disjoint windows.
        tick += 2; acc_regs[0x1D] = 0x80; gyro_regs[0x0E] = 1; hw_imu_poll_observations(1);
        assert(hw_imu_read_observation(0, &raw, &a, &g) == HW_IMU_OK);
        // Individually narrow event windows can still span >15 ms worst-case skew.
        uint64_t epoch_start = hw_clock_microseconds();
        hw_imu_poll_observations(2);
        tick += 2; acc_regs[0x1D] = 0x80; gyro_regs[0x0E] = 1; hw_imu_poll_observations(2);
        assert(hw_imu_read_before(2, hw_clock_microseconds(), &raw, &a, &g) == HW_IMU_OK);
        while (hw_clock_microseconds() - epoch_start < 13000) { tick += 2; hw_imu_poll_observations(2); }
        tick += 4; gyro_regs[0x0E] = 1; hw_imu_poll_observations(2);
        assert(hw_imu_read_before(2, hw_clock_microseconds(), &raw, &a, &g) == HW_IMU_STALE);
        // Standalone tracking takes ownership after fusion stops. A long inactive
        // interval fills the hardware FIFO and leaves an old but readable cache.
        // Every restart has a new high-bit epoch; concurrent fused acquisition uses
        // its own low-bit epoch and both consumers can read one event without consuming it.
        const uint32_t owners[]={0x80000001u,0x80000002u,3u,0x80000003u};
        for (unsigned i=0;i<sizeof(owners)/sizeof(owners[0]);++i) {
            tick+=2000; gyro_regs[0x0E]=32; acc_regs[0x1D]=0x80;
            uint32_t epoch=owners[i];
            assert(hw_imu_read_before(epoch,hw_clock_microseconds(),&raw,&a,&g)==HW_IMU_NOT_READY);
            hw_imu_poll_observations(epoch);
            assert(!gyro_regs[0x0E] && hw_imu_read_before(epoch,hw_clock_microseconds(),&raw,&a,&g)==HW_IMU_NOT_READY);
            tick+=2; gyro_regs[0x0E]=1; acc_regs[0x1D]=0x80;
            hw_imu_poll_observations(epoch);
            assert(hw_imu_read_before(epoch,hw_clock_microseconds(),&raw,&a,&g)==HW_IMU_OK);
            uint64_t preserved_a=a,preserved_g=g;
            assert(hw_imu_read_before(epoch,hw_clock_microseconds(),&raw,&a,&g)==HW_IMU_OK &&
                preserved_a==a && preserved_g==g);
        }
        fail_transfer = true; assert(hw_imu_read(0, &raw) == HW_IMU_IO_ERROR);
        assert(hw_imu_read_observation(0, &raw, &a, &g) == HW_IMU_IO_ERROR);
        fail_transfer = false; assert(hw_imu_read(0, &raw) == HW_IMU_NOT_READY);
        tick += 50;
        assert(hw_imu_read(0, &raw) == HW_IMU_OK);
    }
    unsigned old_writes = writes;
    hw_imu_init();
    assert(writes == old_writes); // Idempotent; doesn't reset a running sensor.
    hw_imu_sample_t sample;
    unsigned old_transfers = transfers;
    assert(hw_imu_read(1, &sample) == HW_IMU_UNAVAILABLE);
    assert(hw_imu_read(255, &sample) == HW_IMU_UNAVAILABLE);
    assert(hw_imu_read(0, NULL) == HW_IMU_IO_ERROR);
    assert(transfers == old_transfers);
    fail_transfer = true; expect_error(HW_IMU_IO_ERROR);
    fail_transfer = false; expect_error(HW_IMU_NOT_READY); tick += 50;
    assert(hw_imu_read(0, &sample) == HW_IMU_OK);
    // Failure after reading acceleration must not publish a partial sample.
    fail_gyro_sample = true; expect_error(HW_IMU_IO_ERROR);
    fail_gyro_sample = false; expect_error(HW_IMU_NOT_READY); tick += 50;
    assert(hw_imu_read(0, &sample) == HW_IMU_OK);
    // A powered sensor resetting must not produce a plausible zero sample.
    reset_gyro(); expect_error(HW_IMU_IO_ERROR);
    expect_error(HW_IMU_NOT_READY); tick += 50;
    assert(hw_imu_read(0, &sample) == HW_IMU_OK);
#else
    (void)argc; (void)argv;
    hw_imu_init();
    hw_imu_sample_t sample = {1, 2, 3, 4, 5, 6};
    assert(hw_imu_read(0, &sample) == HW_IMU_UNAVAILABLE);
    assert(sample.acceleration_x == 1 && sample.angular_velocity_z == 6);
    assert(hw_imu_read(255, NULL) == HW_IMU_UNAVAILABLE);
#endif
    puts("IMU checks passed");
}
