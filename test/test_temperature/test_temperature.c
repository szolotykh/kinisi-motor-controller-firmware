#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "hw_config.h"
#include "hw_temperature.h"

GPIO_TypeDef test_ports[5];
TIM_TypeDef test_timers[12];

#if NUMBER_TEMPERATURE_SENSORS > 0
static uint32_t tick;
static unsigned init_calls, deinit_calls, writes, reads, gpio_calls, clock_calls;
static bool fail_init, fail_write, fail_read;
static uint16_t config_readback = 0x60A0;
static uint16_t temperature = 0x1910; // 25.0625 C

uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms + 1; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState value)
{
    if (value == GPIO_PIN_SET) port->ODR |= pin;
    else port->ODR &= ~pin;
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    return port->ODR & pin ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
void test_i2c_clock_enable(void) { ++clock_calls; }
void rcc_gpio_clk_enable(GPIO_TypeDef *port) { assert(port == GPIOA || port == GPIOC); }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio)
{
    assert((port == GPIOA && gpio->Pin == GPIO_PIN_8) ||
           (port == GPIOC && gpio->Pin == GPIO_PIN_9));
    assert((gpio->Mode == GPIO_MODE_AF_OD || gpio->Mode == GPIO_MODE_OUTPUT_OD) && gpio->Pull == GPIO_NOPULL);
    if (gpio->Mode == GPIO_MODE_AF_OD) assert(gpio->Alternate == GPIO_AF4_I2C3);
    ++gpio_calls;
}
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *bus)
{
    ++init_calls;
    assert(bus->Instance == I2C3 && bus->Init.ClockSpeed == 100000);
    assert(bus->Init.AddressingMode == I2C_ADDRESSINGMODE_7BIT);
    assert(bus->Init.OwnAddress1 == 0 && bus->Init.OwnAddress2 == 0);
    assert(bus->Init.DutyCycle == I2C_DUTYCYCLE_2);
    assert(bus->Init.DualAddressMode == I2C_DUALADDRESS_DISABLE);
    assert(bus->Init.GeneralCallMode == I2C_GENERALCALL_DISABLE);
    assert(bus->Init.NoStretchMode == I2C_NOSTRETCH_DISABLE);
    return fail_init ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *bus)
{
    assert(bus->Instance == I2C3);
    ++deinit_calls;
    return HAL_OK;
}
static void check_transfer(I2C_HandleTypeDef *bus, uint16_t address,
                           uint16_t size, uint16_t length, uint32_t timeout)
{
    assert(bus->Instance == I2C3); // Never use the external I2C2 transport.
    assert(address == 0x90); // Never access the FRAM at 0x50 (shifted 0xA0).
    assert(size == I2C_MEMADD_SIZE_8BIT && length == 2 && timeout == 10);
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *bus, uint16_t address,
        uint16_t reg, uint16_t size, uint8_t *data, uint16_t length, uint32_t timeout)
{
    check_transfer(bus, address, size, length, timeout);
    assert(reg == 1 && data[0] == 0x60 && data[1] == 0x80);
    ++writes;
    return fail_write ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *bus, uint16_t address,
        uint16_t reg, uint16_t size, uint8_t *data, uint16_t length, uint32_t timeout)
{
    check_transfer(bus, address, size, length, timeout);
    assert(reg == 0 || reg == 1);
    ++reads;
    if (fail_read) return HAL_ERROR;
    uint16_t value = reg == 1 ? config_readback : temperature;
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
    return HAL_OK;
}

static void expect_status(hw_temperature_status_t status)
{
    double value = 999.0;
    assert(hw_temperature_read(0, &value) == status);
    assert(value == 999.0); // Failed/not-ready reads never publish an old value.
}
#endif

int main(int argc, char **argv)
{
#if NUMBER_TEMPERATURE_SENSORS > 0
    const char *scenario = argc > 1 ? argv[1] : "normal";
    if (!strcmp(scenario, "wrap")) tick = UINT32_MAX - 100;
    if (!strcmp(scenario, "init_failure")) fail_init = true;
    if (!strcmp(scenario, "write_failure")) fail_write = true;
    if (!strcmp(scenario, "read_failure")) fail_read = true;
    if (!strcmp(scenario, "bad_config")) config_readback = 0x61B0; // Shutdown/extended.
    if (!strcmp(scenario, "status_bits")) config_readback = 0xE080; // OS/AL are not settings.

    hw_temperature_init();
    const bool recovering = fail_init || fail_write || fail_read || config_readback == 0x61B0;
    if (recovering) {
        expect_status(HW_TEMPERATURE_IO_ERROR);
        fail_init = fail_write = fail_read = false;
        config_readback = 0x60A0;
        expect_status(HW_TEMPERATURE_NOT_READY);
    }
    unsigned old_writes = writes, old_reads = reads;
    hw_temperature_init(); // Idempotent: does not restart the sampling delay.
    expect_status(HW_TEMPERATURE_NOT_READY);
    tick += 299;
    expect_status(HW_TEMPERATURE_NOT_READY);
    assert(writes == old_writes && reads == old_reads);
    ++tick;

    const struct { uint16_t raw; double celsius; } examples[] = {
        {0x1910, 25.0625}, {0x0000, 0}, {0xFFF0, -0.0625},
        {0xE700, -25}, {0xD800, -40}, {0x7D00, 125}, {0x0010, 0.0625}
    };
    for (unsigned i = 0; i < sizeof(examples) / sizeof(examples[0]); ++i) {
        double value = 999;
        temperature = examples[i].raw;
        assert(hw_temperature_read(0, &value) == HW_TEMPERATURE_OK);
        assert(value == examples[i].celsius);
    }
    old_reads = reads;
    double value = 999;
    assert(hw_temperature_read(1, &value) == HW_TEMPERATURE_UNAVAILABLE);
    assert(hw_temperature_read(255, &value) == HW_TEMPERATURE_UNAVAILABLE);
    assert(hw_temperature_read(0, NULL) == HW_TEMPERATURE_IO_ERROR);
    assert(value == 999 && old_reads == reads);

    // An interrupted read invalidates configuration and can recover on a later call.
    fail_read = true;
    expect_status(HW_TEMPERATURE_IO_ERROR);
    fail_read = false;
    expect_status(HW_TEMPERATURE_NOT_READY);
    tick += 300;
    assert(hw_temperature_read(0, &value) == HW_TEMPERATURE_OK);

    // Wrong register format/corrupt low bits must not become a plausible temperature.
    temperature = 0xFFFF;
    expect_status(HW_TEMPERATURE_IO_ERROR);
    temperature = 0x1910;
    expect_status(HW_TEMPERATURE_NOT_READY);
    tick += 300;
    assert(hw_temperature_read(0, &value) == HW_TEMPERATURE_OK && value == 25.0625);
    assert(gpio_calls == (init_calls + deinit_calls) * 2 && clock_calls == init_calls);
#else
    (void)argc; (void)argv;
    // No HAL I2C functions are linked in these builds; any hardware access fails linking.
    hw_temperature_init();
    double value = 999;
    assert(hw_temperature_read(0, &value) == HW_TEMPERATURE_UNAVAILABLE);
    assert(hw_temperature_read(255, &value) == HW_TEMPERATURE_UNAVAILABLE);
    assert(value == 999);
#endif
    puts("Temperature tests passed");
    return 0;
}
