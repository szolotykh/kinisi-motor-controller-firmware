#include "hw_config.h"
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
#include "hw_drv8873.h"
#include "hw_timers.h"
#include <stddef.h>

#if DRV8873_SLEW_RATE_BITS > 7
#error "DRV8873 slew-rate selection must fit IC1 SR[2:0]"
#endif
#define DRV8873_REG_IC1 0x02U
#define DRV8873_REG_IC3 0x04U
#define DRV8873_REG_IC4 0x05U
#define DRV8873_IC3_CLR_FLT 0x80U
#define DRV8873_IC1_TOFF_40US 0x40U
#define DRV8873_IC1_SR_SHIFT 2U

static volatile uint32_t fault_sequence[NUMBER_MOTORS];
static bool prepared;
uint32_t hw_drv8873_event_sequence(motorIndex i) {
    return i < NUMBER_MOTORS ? fault_sequence[i] : 0;
}
void HAL_GPIO_EXTI_Callback(uint16_t pin) {
    for (unsigned i = 0; i < NUMBER_MOTORS; ++i)
        if (motor_info[i].fault.pin == pin) ++fault_sequence[i];
}
static void fault_irq(uint16_t mask) {
    for (unsigned i = 0; i < NUMBER_MOTORS; ++i)
        if (motor_info[i].fault.pin & mask) HAL_GPIO_EXTI_IRQHandler(motor_info[i].fault.pin);
}
void EXTI0_IRQHandler(void) { fault_irq(GPIO_PIN_0); }
void EXTI1_IRQHandler(void) { fault_irq(GPIO_PIN_1); }
void EXTI2_IRQHandler(void) { fault_irq(GPIO_PIN_2); }
void EXTI3_IRQHandler(void) { fault_irq(GPIO_PIN_3); }
void EXTI4_IRQHandler(void) { fault_irq(GPIO_PIN_4); }
void EXTI9_5_IRQHandler(void) { fault_irq(0x03e0); }
void EXTI15_10_IRQHandler(void) { fault_irq(0xfc00); }

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
static bool transfer(unsigned i, uint8_t reg, bool read, uint8_t value, uint8_t *result, uint8_t *status_out) {
    uint8_t tx[2] = {(read ? 0x40 : 0) | (reg << 1), value}, rx[2];
    write_pin(motor_info[i].chipSelect, false);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&spi, tx, rx, 2, 10);
    write_pin(motor_info[i].chipSelect, true);
    HAL_Delay(1); // Exceeds the 500 ns minimum between frames.
    if (status != HAL_OK) return false;
    if (status_out) {
        // The two status-header MSBs are fixed high. Reject an absent/stuck-low peer.
        if ((rx[0] & 0xc0U) != 0xc0U) return false;
        *status_out = rx[0];
    }
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
        g.Pin = motor_info[i].fault.pin; g.Mode = GPIO_MODE_IT_RISING_FALLING;
        HAL_GPIO_Init(motor_info[i].fault.port, &g);
        uint16_t pin = motor_info[i].fault.pin;
        IRQn_Type irq = pin == GPIO_PIN_0 ? EXTI0_IRQn : pin == GPIO_PIN_1 ? EXTI1_IRQn :
            pin == GPIO_PIN_2 ? EXTI2_IRQn : pin == GPIO_PIN_3 ? EXTI3_IRQn :
            pin == GPIO_PIN_4 ? EXTI4_IRQn : pin <= GPIO_PIN_9 ? EXTI9_5_IRQn : EXTI15_10_IRQn;
        HAL_NVIC_SetPriority(irq, 5, 0);
        HAL_NVIC_EnableIRQ(irq);
    }
    MOTOR_SPI_CLK_ENABLE();
    const gpio_info_t pins[] = {motor_spi_info.sck, motor_spi_info.miso, motor_spi_info.mosi};
    for (unsigned i = 0; i < sizeof(pins) / sizeof(pins[0]); ++i) {
        rcc_gpio_clk_enable(pins[i].port);
        GPIO_InitTypeDef g = {0};
        g.Pin = pins[i].pin;
        g.Mode = GPIO_MODE_AF_PP; g.Alternate = motor_spi_info.alternate;
        g.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(pins[i].port, &g);
    }
    spi.Instance = motor_spi_info.instance;
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
    // Keep TOFF=40 us and preserve protection settings. Apply board current-limit defaults below.
    const uint8_t control = DRV8873_IC1_TOFF_40US |
        (DRV8873_SLEW_RATE_BITS << DRV8873_IC1_SR_SHIFT);
    if (!transfer(i, DRV8873_REG_IC1, false, control, &value, NULL) ||
        !transfer(i, DRV8873_REG_IC1, true, 0, &value, NULL) || value != control) {
        write_pin(motor_info[i].sleep, false);
        return false; // DISABLE remains asserted.
    }
    // Report thermal warnings on nFAULT without changing protection or current-regulation reporting.
    uint8_t ic2, header;
    if (!transfer(i, 0x03U, true, 0, &ic2, &header) ||
        !transfer(i, 0x03U, false, ic2 | 0x20U, &value, &header) ||
        !transfer(i, 0x03U, true, 0, &value, &header) || value != (ic2 | 0x20U)) {
        write_pin(motor_info[i].sleep, false);
        return false;
    }
    const hw_motor_current_limit_t setting = {
        motor_info[i].hardwareCurrentLimit.enabled, motor_info[i].hardwareCurrentLimit.thresholdMa
    };
    if (!hw_drv8873_set_current_limit(i, &setting)) {
        write_pin(motor_info[i].sleep, false);
        return false;
    }
    return true;
}

bool hw_drv8873_fault_active(motorIndex i) {
    return i >= NUMBER_MOTORS ||
        HAL_GPIO_ReadPin(motor_info[i].fault.port, motor_info[i].fault.pin) == GPIO_PIN_RESET;
}

void hw_drv8873_read_faults(motorIndex i, hw_motor_faults_t *faults) {
    *faults = (hw_motor_faults_t){0};
    if (!prepared || i >= NUMBER_MOTORS) { faults->sleep_requested = true; return; }
    faults->sleep_requested = HAL_GPIO_ReadPin(motor_info[i].sleep.port, motor_info[i].sleep.pin) == GPIO_PIN_RESET;
    faults->fault_active = hw_drv8873_fault_active(i);
    if (faults->sleep_requested) return;
    uint8_t fault, diag, status1, status2;
    if (!transfer(i, 0x00U, true, 0, &fault, &status1) ||
        !transfer(i, 0x01U, true, 0, &diag, &status2) || (fault & 0x80U)) return;
    // Reserved FAULT bit 7 must be zero: reject an all-high response too.
    uint8_t reported = fault | (status1 & 0x3fU) | (status2 & 0x3fU);
    faults->status_valid = true;
    faults->fault_active |= (fault & 0x40U) != 0 || hw_drv8873_fault_active(i);
    faults->overtemperature_warning = (reported & 0x20U) != 0;
    faults->undervoltage = (reported & 0x10U) != 0;
    faults->charge_pump_undervoltage = (reported & 0x08U) != 0;
    faults->overcurrent = (reported & 0x04U) != 0 || (diag & 0x0fU) != 0;
    faults->overtemperature_shutdown = (reported & 0x02U) != 0;
    faults->open_load = (reported & 0x01U) != 0 || (diag & 0xc0U) != 0;
    faults->current_regulation = (diag & 0x30U) != 0;
}

bool hw_drv8873_clear_faults(motorIndex i, hw_motor_faults_t *faults) {
    if (!faults) return false;
    *faults = (hw_motor_faults_t){0};
    if (!prepared || i >= NUMBER_MOTORS) return false;
    // Do not wake a sleeping/unpowered driver or resume its old PWM output.
    write_pin(motor_info[i].disable, true);
    if (HAL_GPIO_ReadPin(motor_info[i].sleep.port, motor_info[i].sleep.pin) == GPIO_PIN_RESET) return false;
    uint8_t control, value, status;
    if (!transfer(i, DRV8873_REG_IC3, true, 0, &control, &status) ||
        (control & DRV8873_IC3_CLR_FLT)) return false;
    // Preserve lock, half-bridge configuration and SPI input values. CLR_FLT self-clears.
    if (!transfer(i, DRV8873_REG_IC3, false, control | DRV8873_IC3_CLR_FLT, &value, &status) ||
        !transfer(i, DRV8873_REG_IC3, true, 0, &value, &status) || value != control) return false;
    hw_drv8873_read_faults(i, faults);
    // Remaining active faults are a valid result, not a communication error.
    return faults->status_valid;
}

static const uint16_t current_thresholds_ma[] = {4000, 5400, 6500, 7000};

static bool read_current_control(motorIndex i, uint8_t *control) {
    if (!prepared || i >= NUMBER_MOTORS ||
        HAL_GPIO_ReadPin(motor_info[i].sleep.port, motor_info[i].sleep.pin) == GPIO_PIN_RESET) return false;
    uint8_t status;
    return transfer(i, DRV8873_REG_IC4, true, 0, control, &status) && !(*control & 0x80U);
}

bool hw_drv8873_get_current_limit(motorIndex i, hw_motor_current_limit_t *setting) {
    uint8_t control;
    if (!setting || !read_current_control(i, &control)) return false;
    uint8_t disabled = control & 3U;
    // The public motor setting applies to both outputs; don't misreport asymmetric settings.
    if (disabled != 0 && disabled != 3) return false;
    setting->enabled = disabled == 0;
    setting->threshold_ma = current_thresholds_ma[(control >> 2) & 3U];
    return true;
}

bool hw_drv8873_set_current_limit(motorIndex i, const hw_motor_current_limit_t *setting) {
    if (!hw_motor_current_limit_allowed(i, setting)) return false;
    uint8_t level = 0;
    while (level < 4 && current_thresholds_ma[level] != setting->threshold_ma) ++level;
    if (level == 4) return false;
    write_pin(motor_info[i].disable, true);
    uint8_t control, actual, status;
    // Avoid restarting an in-progress open-load test when writing IC4.
    if (!read_current_control(i, &control) || (control & 0x40U)) return false;
    const uint8_t desired = (control & 0x30U) | (level << 2) | (setting->enabled ? 0U : 3U);
    return transfer(i, DRV8873_REG_IC4, false, desired, &actual, &status) &&
        read_current_control(i, &actual) && actual == desired;
}
#endif
