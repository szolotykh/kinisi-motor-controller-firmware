// Model HAL timeouts that leave BUSY set, plus a slave holding either bus line.
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "hw_config.h"
#include "hw_temperature.h"

GPIO_TypeDef test_ports[5];
TIM_TypeDef test_timers[12];
static uint32_t tick;
static unsigned init_calls, deinit_calls, transfers, pulses, stops, release_after;
static unsigned fault_register = 255;
static bool fault_write, busy, scl_stuck, sda_stuck, fail_init, scl_gpio, sda_gpio;
static unsigned configured_writes[NUMBER_TEMPERATURE_SENSORS];

uint32_t HAL_GetTick(void) { return tick; }
// STM32 HAL_Delay adds a tick to guarantee the minimum delay.
void HAL_Delay(uint32_t ms) { tick += ms + 1; }
void test_i2c_clock_enable(void) {}
void rcc_gpio_clk_enable(GPIO_TypeDef *port) { assert(port == GPIOA || port == GPIOC); }
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin)
{
    if (port == GPIOA && pin == GPIO_PIN_8)
        return scl_stuck || (scl_gpio && !(port->ODR & pin)) ? GPIO_PIN_RESET : GPIO_PIN_SET;
    assert(port == GPIOC && pin == GPIO_PIN_9);
    return sda_stuck || pulses < release_after || (sda_gpio && !(port->ODR & pin)) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState value)
{
    bool was_high = (port->ODR & pin) != 0;
    if (value == GPIO_PIN_SET) port->ODR |= pin;
    else port->ODR &= ~pin;
    if (port == GPIOA && pin == GPIO_PIN_8) {
        if (scl_gpio && !was_high && value == GPIO_PIN_SET && !scl_stuck && (GPIOC->ODR & GPIO_PIN_9))
            ++pulses;
    } else {
        assert(port == GPIOC && pin == GPIO_PIN_9);
        if (sda_gpio && !was_high && value == GPIO_PIN_SET &&
            HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET &&
            HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET) ++stops;
    }
}
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio)
{
    assert(gpio->Pull == GPIO_NOPULL);
    bool manual = gpio->Mode == GPIO_MODE_OUTPUT_OD;
    assert(manual || (gpio->Mode == GPIO_MODE_AF_OD && gpio->Alternate == GPIO_AF4_I2C3));
    if (manual) assert(port->ODR & gpio->Pin); // Preload released, never drive high push-pull.
    if (port == GPIOA && gpio->Pin == GPIO_PIN_8) scl_gpio = manual;
    else { assert(port == GPIOC && gpio->Pin == GPIO_PIN_9); sda_gpio = manual; }
}
HAL_StatusTypeDef HAL_I2C_DeInit(I2C_HandleTypeDef *bus)
{
    assert(bus->Instance == I2C3);
    ++deinit_calls;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *bus)
{
    assert(bus->Instance == I2C3 && !scl_gpio && !sda_gpio);
    ++init_calls;
    if (fail_init) return HAL_ERROR;
    // Real HAL_Init issues SWRST; a normal memory transfer does not clear BUSY.
    busy = false;
    return HAL_OK;
}
static unsigned check_transfer(I2C_HandleTypeDef *bus, uint16_t address, uint16_t size,
                               uint16_t length, uint32_t timeout)
{
    assert(bus->Instance == I2C3 && size == I2C_MEMADD_SIZE_8BIT && length == 2 && timeout == 10);
    assert(address >= 0x90 && address < 0x90 + NUMBER_TEMPERATURE_SENSORS * 2 && !(address & 1));
    assert(!scl_gpio && !sda_gpio);
    ++transfers;
    return (address - 0x90) / 2;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *bus, uint16_t address,
        uint16_t reg, uint16_t size, uint8_t *data, uint16_t length, uint32_t timeout)
{
    unsigned index = check_transfer(bus, address, size, length, timeout);
    assert(reg == 1 && data[0] == 0x60 && data[1] == 0x80);
    if (fault_write) { busy = true; fault_write = false; return HAL_ERROR; }
    if (busy) return HAL_ERROR;
    ++configured_writes[index];
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *bus, uint16_t address,
        uint16_t reg, uint16_t size, uint8_t *data, uint16_t length, uint32_t timeout)
{
    (void)check_transfer(bus, address, size, length, timeout);
    assert(reg <= 1);
    if (reg == fault_register) { busy = true; fault_register = 255; return HAL_ERROR; }
    if (busy) return HAL_ERROR;
    data[0] = reg ? 0x60 : 0x19;
    data[1] = reg ? 0xA0 : 0x10;
    return HAL_OK;
}
static void expect_status(uint8_t index, hw_temperature_status_t status)
{
    double value = 999;
    assert(hw_temperature_read(index, &value) == status);
    assert(value == 999);
}
static void expect_sample(uint8_t index)
{
    double value = 999;
    assert(hw_temperature_read(index, &value) == HW_TEMPERATURE_OK && value == 25.0625);
}
int main(int argc, char **argv)
{
    assert(argc == 2);
    const char *scenario = argv[1];
    if (!strncmp(scenario, "wrap", 4)) tick = UINT32_MAX - 320;
    hw_temperature_init();
    tick += 300;
    expect_sample(0);

    // Fail a data transfer after START but before STOP; the next call must clear BUSY.
    fault_register = 0;
    expect_status(0, HW_TEMPERATURE_IO_ERROR);
    assert(busy);
    unsigned before_init = init_calls;
    if (!strcmp(scenario, "sda_release") || !strcmp(scenario, "wrap")) release_after = 9;
    if (!strcmp(scenario, "sda_stuck")) sda_stuck = true;
    if (!strcmp(scenario, "scl_stuck") || !strcmp(scenario, "wrap_scl_stuck")) scl_stuck = true;
    if (!strcmp(scenario, "init_failure")) fail_init = true;
    if (sda_stuck || scl_stuck || fail_init) {
        unsigned before_transfers = transfers;
        uint32_t started = tick;
        expect_status(0, HW_TEMPERATURE_IO_ERROR);
        assert((uint32_t)(tick - started) <= 55); // Bounded even with a permanently low clock.
        assert(transfers == before_transfers && pulses <= 9);
        assert((GPIOA->ODR & GPIO_PIN_8) && (GPIOC->ODR & GPIO_PIN_9));
        if (!fail_init) assert(init_calls == before_init);
        sda_stuck = scl_stuck = fail_init = false;
    }
    uint32_t started = tick;
    expect_status(0, HW_TEMPERATURE_NOT_READY);
    assert((uint32_t)(tick - started) <= 55);
    assert(init_calls > before_init && deinit_calls && stops && !busy);
    if (release_after) assert(pulses == 9);
    tick += 299;
    expect_status(0, HW_TEMPERATURE_NOT_READY);
    ++tick;
    expect_sample(0);

    // Configuration writes and configuration readback failures need the same recovery.
    for (unsigned failure = 0; failure < 2; ++failure) {
        fault_register = 0;
        expect_status(0, HW_TEMPERATURE_IO_ERROR);
        if (failure == 0) fault_write = true;
        else fault_register = 1;
        expect_status(0, HW_TEMPERATURE_IO_ERROR);
        assert(busy);
        expect_status(0, HW_TEMPERATURE_NOT_READY);
        tick += 300;
        expect_sample(0);
    }
    // One failed sensor invalidates every configured sensor sharing the bus.
    for (uint8_t i = 1; i < NUMBER_TEMPERATURE_SENSORS; ++i) {
        expect_status(i, HW_TEMPERATURE_NOT_READY);
        assert(configured_writes[i] == 2);
        tick += 300;
        expect_sample(i);
    }
    puts("I2C bus recovery, bounded failures and fresh conversion checks passed");
}
