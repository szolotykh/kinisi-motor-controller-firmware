#include "hw_config.h"
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
#include "hw_drv8873.h"
#include "hw_timers.h"

#if DRV8873_SLEW_RATE_BITS > 7
#error "DRV8873 slew-rate selection must fit IC1 SR[2:0]"
#endif
#define DRV8873_REG_IC1 0x02U
#define DRV8873_IC1_TOFF_40US 0x40U
#define DRV8873_IC1_SR_SHIFT 2U

static bool prepared;
static SPI_HandleTypeDef spi;
static void write_pin(gpio_info_t pin, bool value) {
    HAL_GPIO_WritePin(pin.port, pin.pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void output(gpio_info_t pin, bool value) {
    rcc_gpio_clk_enable(pin.port);
    write_pin(pin, value);
    GPIO_InitTypeDef g = {0};
    g.Pin = pin.pin; g.Mode = GPIO_MODE_OUTPUT_PP; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(pin.port, &g);
}
static bool transfer(unsigned i, uint8_t reg, bool read, uint8_t value, uint8_t *result) {
    uint8_t tx[2] = {(read ? 0x40 : 0) | (reg << 1), value}, rx[2];
    write_pin(motor_info[i].chipSelect, false);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&spi, tx, rx, 2, 10);
    write_pin(motor_info[i].chipSelect, true);
    HAL_Delay(1); // Exceeds the 500 ns minimum between frames.
    if (status != HAL_OK) return false;
    *result = rx[1];
    return true;
}
bool hw_drv8873_prepare(void) {
    if (prepared) return true;
    for (unsigned i=0; i<NUMBER_MOTORS; ++i) {
        output(motor_info[i].disable, true);
        output(motor_info[i].sleep, false);
        output(motor_info[i].chipSelect, true);
        output(motor_info[i].direction, false);
        output((gpio_info_t){motor_info[i].enable.port, motor_info[i].enable.pin}, false);
        rcc_gpio_clk_enable(motor_info[i].fault.port);
        GPIO_InitTypeDef g = {0};
        g.Pin = motor_info[i].fault.pin; g.Mode = GPIO_MODE_INPUT;
        HAL_GPIO_Init(motor_info[i].fault.port, &g);
    }
    __HAL_RCC_SPI3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    g.Mode = GPIO_MODE_AF_PP; g.Alternate = GPIO_AF6_SPI3;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &g);
    spi.Instance = SPI3;
    spi.Init.Mode = SPI_MODE_MASTER;
    spi.Init.Direction = SPI_DIRECTION_2LINES;
    spi.Init.DataSize = SPI_DATASIZE_8BIT;
    spi.Init.CLKPolarity = SPI_POLARITY_LOW;
    spi.Init.CLKPhase = SPI_PHASE_2EDGE;
    spi.Init.NSS = SPI_NSS_SOFT;
    spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    if (HAL_SPI_Init(&spi) != HAL_OK) return false;
    prepared = true;
    return true;
}

bool hw_drv8873_initialize(motorIndex i) {
    if (i >= NUMBER_MOTORS || !hw_drv8873_prepare()) return false;
    write_pin(motor_info[i].disable, true);
    // An explicit reinitialization also clears latched faults through sleep.
    write_pin(motor_info[i].sleep, false);
    HAL_Delay(1); // Exceeds the 60 us maximum sleep-entry time.
    write_pin(motor_info[i].sleep, true);
    HAL_Delay(2); // DRV8873 wake-up maximum: 1.5 ms.
    uint8_t value;
    // PH/EN with physical input pins (MODE=00, SPI_IN=0). Explicitly select SR:
    // the reset SR=100 has long switching delays relative to a 10 us PWM cycle.
    // Keep TOFF=40 us; IC2/IC4 current regulation and protection remain unchanged.
    const uint8_t control = DRV8873_IC1_TOFF_40US |
        (DRV8873_SLEW_RATE_BITS << DRV8873_IC1_SR_SHIFT);
    if (!transfer(i, DRV8873_REG_IC1, false, control, &value) ||
        !transfer(i, DRV8873_REG_IC1, true, 0, &value) || value != control) {
        write_pin(motor_info[i].sleep, false);
        return false; // DISABLE remains asserted.
    }
    return true;
}

bool hw_drv8873_fault_active(motorIndex i) {
    return i >= NUMBER_MOTORS ||
        HAL_GPIO_ReadPin(motor_info[i].fault.port, motor_info[i].fault.pin) == GPIO_PIN_RESET;
}
#endif
