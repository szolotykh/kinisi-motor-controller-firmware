#include "hw_motor.h"
#include "hw_motor_faults.h"
#include "hw_motor_current_limit.h"
#include "hw_config.h"
#include "hw_timers.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
#include "hw_drv8873.h"
#endif

GPIO_TypeDef test_ports[5];
TIM_TypeDef test_timers[12];
static TIM_HandleTypeDef handles[12];
static unsigned timer_inits, delays;
static uint16_t spi_pins;
static int fail_spi_motor = -1;
static uint8_t corrupt_readback;
static uint8_t registers[NUMBER_MOTORS], fault_registers[NUMBER_MOTORS], diag_registers[NUMBER_MOTORS];
static uint8_t diagnostic_header = 0xc0;
static uint8_t control2_registers[NUMBER_MOTORS], control3_registers[NUMBER_MOTORS], control4_registers[NUMBER_MOTORS];
static bool corrupt_control4, lock_current_control;
static unsigned transfer_count, fail_transfer;
static bool persistent_fault, corrupt_control3;
static unsigned spi_reads, spi_writes;
static uint32_t apb2_divider = RCC_HCLK_DIV2, pclk2 = 84000000U;
static uint32_t apb1_divider = RCC_HCLK_DIV4, pclk1 = 42000000U;

uint32_t HAL_RCC_GetPCLK1Freq(void) { return pclk1; }
uint32_t HAL_RCC_GetPCLK2Freq(void) { return pclk2; }
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *clocks, uint32_t *latency) {
    clocks->APB1CLKDivider = apb1_divider;
    clocks->APB2CLKDivider = apb2_divider;
    *latency = 5;
}

TIM_HandleTypeDef *get_timer_handeler(TIM_TypeDef *timer) { return &handles[timer - test_timers]; }
uint32_t get_alternate_function_mapping(TIM_TypeDef *timer) { (void)timer; return 1; }
void rcc_gpio_clk_enable(GPIO_TypeDef *port) { assert(port >= GPIOA && port <= GPIOE); }
void rcc_tim_clk_enable(TIM_TypeDef *timer) { assert(timer >= TIM1 && timer <= TIM12); }
void Error_Handler(void) { assert(!"Unexpected hardware error"); }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state) {
    assert(port >= GPIOA && port <= GPIOE);
    if (state) { port->ODR |= pin; port->IDR |= pin; }
    else { port->ODR &= ~pin; port->IDR &= ~pin; }
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) { return (port->IDR & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET; }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio) {
    assert(port && gpio->Pin);
    if (gpio->Mode == GPIO_MODE_AF_PP && gpio->Alternate == GPIO_AF6_SPI3) {
        assert(port == GPIOC);
        spi_pins |= gpio->Pin;
    }
}
void HAL_Delay(uint32_t duration) { delays += duration; }
int HAL_TIM_Base_GetState(TIM_HandleTypeDef *timer) { return timer->State; }
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *timer) {
    ++timer_inits; timer->State = HAL_TIM_STATE_READY;
    assert(timer->Init.Period == MOTOR_MAX_SPEED - 1);
    bool apb2 = timer->Instance == TIM1 || timer->Instance == TIM8;
    uint32_t timer_clock = apb2 ? pclk2 : pclk1;
    uint32_t divider = apb2 ? apb2_divider : apb1_divider;
    if (divider != RCC_HCLK_DIV1) timer_clock *= 2U;
    uint32_t frequency = timer_clock / (timer->Init.Prescaler + 1) / (timer->Init.Period + 1);
    assert(frequency == 100000U);
    // At 100 kHz, a 25% physical drive duty is a 2.5 us pulse.
    assert((uint64_t)(timer->Init.Prescaler + 1) * 210 * 1000000000ULL / timer_clock == 2500U);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *timer, TIM_ClockConfigTypeDef *clock) {
    (void)timer; (void)clock; return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *timer) { (void)timer; return HAL_OK; }
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *timer, TIM_MasterConfigTypeDef *master) {
    (void)timer; (void)master; return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *timer, TIM_OC_InitTypeDef *oc, uint32_t channel) {
    assert(oc->OCMode == TIM_OCMODE_PWM1 && oc->OCPolarity == TIM_OCPOLARITY_HIGH);
    timer->polarityN[channel/4] = oc->OCNPolarity;
    timer->Instance->compare[channel/4] = oc->Pulse;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel) {
    assert(!(timer->started & (1u << (channel/4))));
    timer->started |= 1u << (channel/4); return HAL_OK;
}
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *timer, uint32_t channel) {
    assert(channel != TIM_CHANNEL_4);
    assert(!(timer->startedN & (1u << (channel/4))));
    timer->startedN |= 1u << (channel/4); return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *spi) {
    assert(spi->Instance == SPI3);
    assert(spi_pins == (GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12));
    assert(spi->Init.CLKPolarity == SPI_POLARITY_LOW && spi->Init.CLKPhase == SPI_PHASE_2EDGE);
    assert(spi->Init.DataSize == SPI_DATASIZE_8BIT);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *tx, uint8_t *rx, uint16_t size, uint32_t timeout) {
    (void)spi; (void)timeout;
    assert(size == 2);
    unsigned reg = (tx[0] & 0x3e) >> 1;
    assert(reg <= 5);
    if (++transfer_count == fail_transfer) return HAL_ERROR;
    int selected = -1;
    for (unsigned i = 0; i < NUMBER_MOTORS; ++i) {
        const gpio_info_t *cs = &motor_info[i].chipSelect;
        if (!(cs->port->ODR & cs->pin)) { assert(selected == -1); selected = i; }
    }
    assert(selected >= 0);
    if (!(tx[0] & 0x40)) assert(motor_info[selected].disable.port->ODR & motor_info[selected].disable.pin);
    if (selected == fail_spi_motor) return HAL_ERROR;
    if (reg == 3) {
        rx[0] = diagnostic_header; rx[1] = control2_registers[selected];
        if (tx[0] & 0x40) ++spi_reads;
        else { ++spi_writes; control2_registers[selected] = tx[1]; }
        return HAL_OK;
    }
    if (reg == 5) {
        rx[0] = diagnostic_header; rx[1] = control4_registers[selected];
        if (tx[0] & 0x40) ++spi_reads;
        else {
            ++spi_writes;
            if (!lock_current_control) control4_registers[selected] = tx[1] ^ (corrupt_control4 ? 0x10 : 0);
        }
        return HAL_OK;
    }
    if (reg == 4) {
        rx[0] = diagnostic_header;
        rx[1] = control3_registers[selected];
        if (tx[0] & 0x40) { ++spi_reads; }
        else {
            ++spi_writes;
            assert(tx[1] == (uint8_t)(control3_registers[selected] | 0x80));
            control3_registers[selected] = (tx[1] & 0x7f) ^ (corrupt_control3 ? 1 : 0);
            if (!persistent_fault) {
                fault_registers[selected] = diag_registers[selected] = 0;
                motor_info[selected].fault.port->IDR |= motor_info[selected].fault.pin;
            }
        }
        return HAL_OK;
    }
    if (reg < 2) {
        assert(tx[0] & 0x40); // Diagnostic reads must never write control/fault registers.
        ++spi_reads;
        rx[0] = diagnostic_header;
        rx[1] = reg == 0 ? fault_registers[selected] : diag_registers[selected];
        return HAL_OK;
    }
    rx[0] = 0xc0;
    rx[1] = registers[selected];
    if (tx[0] & 0x40) rx[1] ^= corrupt_readback;
    else {
        // PH/EN=00, physical pins, SR=000 (53.2 V/us), TOFF=01 (40 us).
        assert(tx[1] == 0x40);
        ++spi_writes;
        registers[selected] = tx[1];
    }
    return HAL_OK;
}
static bool high(gpio_info_t pin) { return (pin.port->ODR & pin.pin) != 0; }
static uint32_t compare(unsigned i, uint32_t channel) { return motor_info[i].timer->compare[channel/4]; }

static void assert_idle(unsigned i, bool braking) {
    const motor_info_t *info = &motor_info[i];
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
    assert(high(info->disable) == !braking);
    assert(compare(i, info->enable.timerChannel) == 0);
#else
    uint32_t expected = braking ? MOTOR_MAX_SPEED : 0;
    assert(compare(i, info->pwmChannel1.timerChannel) == expected);
    assert(compare(i, info->pwmChannel2.timerChannel) == expected);
#endif
}


static void fault_reporting(void) {
    hw_motor_faults_t state;
    assert(!hw_motor_read_faults(255, &state));
    assert(!hw_motor_read_faults(0, NULL));
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
    for (unsigned i = 0; i < NUMBER_MOTORS; ++i) {
        const motor_info_t *info = &motor_info[i];
        HAL_GPIO_WritePin(info->sleep.port, info->sleep.pin, GPIO_PIN_RESET);
        unsigned reads = spi_reads;
        assert(hw_motor_read_faults(i, &state));
        assert(state.sleep_requested && !state.status_valid && spi_reads == reads);
        get_motor_interface()->initialize(i, false);
        info->fault.port->IDR |= info->fault.pin;
        get_motor_interface()->set_speed(i, 25);
        uint32_t duty = compare(i, info->enable.timerChannel);
        uint32_t outputs[5];
        for (unsigned port = 0; port < 5; ++port) outputs[port] = test_ports[port].ODR;
        unsigned writes = spi_writes;
        fault_registers[i] = 0x7f; diag_registers[i] = 0xff;
        assert(hw_motor_read_faults(i, &state));
        assert(state.status_valid && !state.sleep_requested && state.fault_active);
        assert(state.overtemperature_warning && state.undervoltage && state.charge_pump_undervoltage);
        assert(state.overcurrent && state.overtemperature_shutdown && state.open_load && state.current_regulation);
        assert(fault_registers[i] == 0x7f && diag_registers[i] == 0xff && spi_writes == writes);
        assert(compare(i, info->enable.timerChannel) == duty);
        for (unsigned port = 0; port < 5; ++port) assert(outputs[port] == test_ports[port].ODR);
        fault_registers[i] = 0; diag_registers[i] = 0;
        assert(hw_motor_read_faults(i, &state) && state.status_valid && !state.fault_active && !state.overcurrent);
        fault_registers[i] = 0x10; // Supply undervoltage, independently of charge pump and current.
        assert(hw_motor_read_faults(i, &state) && state.undervoltage && !state.charge_pump_undervoltage && !state.overcurrent);
        fault_registers[i] = 0x08;
        assert(hw_motor_read_faults(i, &state) && state.charge_pump_undervoltage && !state.undervoltage);
        fault_registers[i] = 0x20; // A warning is distinct from thermal shutdown.
        assert(hw_motor_read_faults(i, &state) && state.overtemperature_warning && !state.overtemperature_shutdown);
        fault_registers[i] = 0x02;
        assert(hw_motor_read_faults(i, &state) && state.overtemperature_shutdown && !state.overtemperature_warning);
        fault_registers[i] = 0x04;
        assert(hw_motor_read_faults(i, &state) && state.overcurrent && !state.open_load);
        fault_registers[i] = 0x01;
        assert(hw_motor_read_faults(i, &state) && state.open_load && !state.overcurrent);
        fault_registers[i] = 0;
        // Each register bit is independent; DIAG regulation is not overcurrent.
        diag_registers[i] = 0x30;
        assert(hw_motor_read_faults(i, &state) && state.current_regulation && !state.overcurrent);
        diag_registers[i] = 0x01;
        assert(hw_motor_read_faults(i, &state) && state.overcurrent && !state.current_regulation);
        diag_registers[i] = 0xc0;
        assert(hw_motor_read_faults(i, &state) && state.open_load && !state.overcurrent);
        diag_registers[i] = 0;
        fail_spi_motor = i;
        assert(hw_motor_read_faults(i, &state) && !state.status_valid && !state.overcurrent);
        fail_spi_motor = -1;
        diagnostic_header = 0;
        assert(hw_motor_read_faults(i, &state) && !state.status_valid);
        diagnostic_header = 0xff; fault_registers[i] = 0xff;
        assert(hw_motor_read_faults(i, &state) && !state.status_valid);
        diagnostic_header = 0xc0; fault_registers[i] = 0;
        assert(hw_motor_read_faults(i, &state) && state.status_valid);
        assert(spi_writes == writes);
    }
#else
    assert(!hw_motor_read_faults(0, &state));
#endif
}


static void clear_faults(void) {
    hw_motor_faults_t state;
    assert(!hw_motor_clear_faults(255, &state));
    assert(!hw_motor_clear_faults(0, NULL));
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
    const hw_motor_interface_t *motor = get_motor_interface();
    for (unsigned i = 0; i < NUMBER_MOTORS; ++i) {
        const motor_info_t *info = &motor_info[i];
        // Exercise preserving both locked and unlocked IC3 settings and input bits.
        control3_registers[i] = i % 2 ? 0x3b : 0x40;
        uint8_t control = control3_registers[i], mode = registers[i];
        info->fault.port->IDR |= info->fault.pin;
        motor->set_speed(i, 25);
        fault_registers[i] = 0x44; diag_registers[i] = 1;
        info->fault.port->IDR &= ~info->fault.pin;
        bool direction = high(info->direction);
        uint32_t other_duties[NUMBER_MOTORS];
        for (unsigned j = 0; j < NUMBER_MOTORS; ++j)
            other_duties[j] = compare(j, motor_info[j].enable.timerChannel);
        unsigned writes = spi_writes;
        assert(hw_motor_clear_faults(i, &state));
        assert(state.status_valid && !state.fault_active && !state.overcurrent);
        assert(spi_writes == writes + 1 && control3_registers[i] == control && registers[i] == mode);
        assert_idle(i, false); assert(high(info->sleep) && high(info->direction) == direction);
        for (unsigned j = 0; j < NUMBER_MOTORS; ++j)
            if (j != i) assert(compare(j, motor_info[j].enable.timerChannel) == other_duties[j]);
        // Persistent conditions are reported honestly after a successful clear operation.
        persistent_fault = true;
        fault_registers[i] = 0x44; diag_registers[i] = 1;
        info->fault.port->IDR &= ~info->fault.pin;
        assert(hw_motor_clear_faults(i, &state));
        assert(state.status_valid && state.fault_active && state.overcurrent);
        assert_idle(i, false); persistent_fault = false;
        // Every transaction can fail: control read, write, readback, FAULT and DIAG.
        for (unsigned failure = 1; failure <= 5; ++failure) {
            info->fault.port->IDR |= info->fault.pin;
            motor->set_speed(i, 25); transfer_count = 0; fail_transfer = failure;
            assert(!hw_motor_clear_faults(i, &state));
            assert_idle(i, false); assert(high(info->sleep));
        }
        fail_transfer = 0;
        corrupt_control3 = true;
        assert(!hw_motor_clear_faults(i, &state)); assert_idle(i, false);
        corrupt_control3 = false;
        diagnostic_header = 0; writes = spi_writes;
        assert(!hw_motor_clear_faults(i, &state) && spi_writes == writes); assert_idle(i, false);
        diagnostic_header = 0xc0;
        control3_registers[i] = 0xff;
        assert(!hw_motor_clear_faults(i, &state) && spi_writes == writes);
        control3_registers[i] = control;
        HAL_GPIO_WritePin(info->sleep.port, info->sleep.pin, GPIO_PIN_RESET);
        transfer_count = 0;
        assert(!hw_motor_clear_faults(i, &state) && !transfer_count && !high(info->sleep));
        assert_idle(i, false);
        motor->initialize(i, false);
    }
#else
    assert(!hw_motor_clear_faults(0, &state));
#endif
}

static void current_limit_tests(void) {
    hw_motor_current_limit_t value = {true, 5400}, actual;
    assert(!hw_motor_current_limit_allowed(255, &value));
    assert(!hw_motor_current_limit_allowed(0, NULL));
    assert(!hw_motor_get_current_limit(255, &actual));
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
    const hw_motor_interface_t *motor = get_motor_interface();
    const uint16_t levels[] = {4000, 5400, 6500, 7000};
    for (unsigned i = 0; i < NUMBER_MOTORS; ++i) {
        const motor_info_t *info = &motor_info[i];
        assert(hw_motor_get_current_limit(i, &actual));
        assert(actual.enabled && actual.threshold_ma == info->hardwareCurrentLimit.thresholdMa);
        // Keep unrelated open-load settings, all other drivers and direction unchanged.
        control4_registers[i] |= 0x30;
        uint8_t controls[NUMBER_MOTORS]; memcpy(controls, control4_registers, sizeof(controls));
        for (unsigned enabled = 0; enabled <= 1; ++enabled) for (unsigned level = 0; level < 4; ++level) {
            value = (hw_motor_current_limit_t){enabled != 0, levels[level]};
            motor->set_speed(i, 25);
            bool direction = high(info->direction);
            uint32_t duty = compare(i, info->enable.timerChannel);
            assert(hw_motor_get_current_limit(i, &actual));
            assert(compare(i, info->enable.timerChannel) == duty); // GET does not stop or change outputs.
            assert(hw_motor_current_limit_allowed(i, &value));
            assert(hw_motor_set_current_limit(i, &value));
            assert_idle(i, false); assert(high(info->sleep) && high(info->direction) == direction);
            assert(control4_registers[i] == (0x30 | (level << 2) | (enabled ? 0 : 3)));
            assert(hw_motor_get_current_limit(i, &actual));
            assert(actual.enabled == value.enabled && actual.threshold_ma == value.threshold_ma);
            for (unsigned j = 0; j < NUMBER_MOTORS; ++j) if (i != j) assert(control4_registers[j] == controls[j]);
        }
        value = (hw_motor_current_limit_t){true, 5000};
        unsigned writes = spi_writes;
        motor->set_speed(i, 25);
        uint32_t duty = compare(i, info->enable.timerChannel);
        assert(!hw_motor_set_current_limit(i, &value));
        assert(spi_writes == writes && compare(i, info->enable.timerChannel) == duty);
        value.threshold_ma = 5400;
        for (unsigned failure = 1; failure <= 3; ++failure) {
            transfer_count = 0; fail_transfer = failure;
            assert(!hw_motor_set_current_limit(i, &value)); assert_idle(i, false);
        }
        fail_transfer = 0;
        corrupt_control4 = true;
        assert(!hw_motor_set_current_limit(i, &value)); assert_idle(i, false);
        corrupt_control4 = false;
        control4_registers[i] = 8; lock_current_control = true;
        assert(!hw_motor_set_current_limit(i, &value)); assert_idle(i, false);
        lock_current_control = false;
        for (unsigned invalid = 0; invalid < 3; ++invalid) {
            control4_registers[i] = invalid == 0 ? 0xff : invalid == 1 ? 0x40 : 8;
            diagnostic_header = invalid == 2 ? 0 : 0xc0;
            writes = spi_writes;
            assert(!hw_motor_set_current_limit(i, &value) && spi_writes == writes);
        }
        diagnostic_header = 0xc0; control4_registers[i] = 1;
        assert(!hw_motor_get_current_limit(i, &actual)); // Asymmetric disable cannot be represented.
        HAL_GPIO_WritePin(info->sleep.port, info->sleep.pin, GPIO_PIN_RESET);
        transfer_count = 0;
        assert(!hw_motor_get_current_limit(i, &actual) && !hw_motor_set_current_limit(i, &value));
        assert(!transfer_count && !high(info->sleep));
        control4_registers[i] = 0;
        motor->initialize(i, false); // Initialization reapplies board defaults after a runtime setting.
        assert(motor->is_initialized(i) && hw_motor_get_current_limit(i, &actual));
        assert(actual.enabled && actual.threshold_ma == 6500);
        corrupt_control4 = true;
        motor->initialize(i, false);
        assert(!motor->is_initialized(i) && !high(info->sleep));
        assert(!hw_motor_get_current_limit(i, &actual) && !hw_motor_set_current_limit(i, &value));
        corrupt_control4 = false; motor->initialize(i, false);
        assert(motor->is_initialized(i));
    }
#else
    assert(!hw_motor_current_limit_allowed(0, &value));
    assert(!hw_motor_get_current_limit(0, &actual) && !hw_motor_set_current_limit(0, &value));
#endif
}

int main(int argc, char **argv) {
    if (argc > 1 && strcmp(argv[1], "apb2_div1") == 0) {
        apb2_divider = RCC_HCLK_DIV1; pclk2 = 168000000U;
    } else if (argc > 1 && strcmp(argv[1], "apb2_div4") == 0) {
        apb2_divider = RCC_HCLK_DIV4; pclk2 = 42000000U;
    } else if (argc > 1 && strcmp(argv[1], "apb1_div1") == 0) {
        apb1_divider = RCC_HCLK_DIV1; pclk1 = 84000000U;
    }
    const hw_motor_interface_t *motor = get_motor_interface();
    for (unsigned i = 0; i < 5; ++i) test_ports[i].IDR = 0xffff;
    for (unsigned i = 0; i < NUMBER_MOTORS; ++i) registers[i] = 0x51; // IC1 reset value.
    hw_motor_init();
    for (unsigned i = 0; i < NUMBER_MOTORS; ++i) {
        assert(!motor->is_initialized(i));
        hw_motor_faults_t diagnostic;
        assert(!hw_motor_clear_faults(i, &diagnostic));
        motor->set_speed(i, 100); // Uninitialized motors do not start timers.
        assert(timer_inits == 0);
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
        assert(high(motor_info[i].disable) && !high(motor_info[i].sleep));
        assert(high(motor_info[i].chipSelect));
#endif
    }
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
    // An unsuccessful SPI operation or wrong readback cannot enable a motor.
    fail_spi_motor = 0;
    motor->initialize(0, false);
    assert(!motor->is_initialized(0) && high(motor_info[0].disable) && !high(motor_info[0].sleep));
    fail_spi_motor = -1;
    corrupt_readback = 1; // Wrong control mode must fail initialization.
    motor->initialize(0, false);
    assert(!motor->is_initialized(0) && high(motor_info[0].disable));
    corrupt_readback = 4; // A wrong slew-rate bit must fail too.
    motor->initialize(0, false);
    assert(!motor->is_initialized(0) && high(motor_info[0].disable));
    assert(!high(motor_info[0].sleep) && timer_inits == 0);
    corrupt_readback = 0;
#endif
    for (unsigned i = 0; i < NUMBER_MOTORS; ++i) {
        const motor_info_t *info = &motor_info[i];
        assert(info->controlModel == MOTOR_CONTROL_MODEL);
        motor->initialize(i, false);
        assert(motor->is_initialized(i) && !motor->is_reversed(i));
        TIM_HandleTypeDef *timer = get_timer_handeler(info->timer);
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
        assert(registers[i] == 0x40 && high(info->disable));
        // There must be no software duty threshold at 50%. This checks requested
        // EN duty only; actual bridge pulses and motor startup need hardware tests.
        const unsigned duties[] = {1, 10, 25, 49, 50, 60, 70, 100};
        for (unsigned d = 0; d < sizeof(duties)/sizeof(duties[0]); ++d) {
            motor->set_speed(i, duties[d]);
            assert(compare(i, info->enable.timerChannel) == duties[d] * MOTOR_MAX_SPEED / 100);
            assert(!high(info->disable));
        }
#endif
        unsigned before_delay = delays;
        motor->set_speed(i, 25);
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
        assert(high(info->direction) && !high(info->disable));
        assert(compare(i, info->enable.timerChannel) == 210);
        unsigned mask = 1u << (info->enable.timerChannel/4);
        assert(timer->noPreload & mask);
        if (info->enable.complementary) {
            assert(timer->startedN & mask);
            assert(timer->polarityN[info->enable.timerChannel/4] == TIM_OCNPOLARITY_LOW);
        } else assert(timer->started & mask);
#else
        assert(compare(i, info->pwmChannel1.timerChannel) == 630);
        assert(compare(i, info->pwmChannel2.timerChannel) == 840);
        assert(timer->startedN == 0);
#endif
        motor->set_speed(i, -25);
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
        assert(!high(info->direction) && compare(i, info->enable.timerChannel) == 210);
#else
        assert(compare(i, info->pwmChannel1.timerChannel) == 840);
        assert(compare(i, info->pwmChannel2.timerChannel) == 630);
#endif
        unsigned before = timer_inits;
        motor->initialize(i, true);
        assert(motor->is_reversed(i) && timer_inits == before);
        before_delay = delays;
        motor->set_speed(i, -200);
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
        assert(high(info->direction) && compare(i, info->enable.timerChannel) == 840);
#else
        assert(compare(i, info->pwmChannel1.timerChannel) == 0);
        assert(compare(i, info->pwmChannel2.timerChannel) == 840);
#endif
        motor->stop(i);
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
        assert(high(info->disable) && compare(i, info->enable.timerChannel) == 0);
#else
        assert(compare(i, info->pwmChannel1.timerChannel) == 0 && compare(i, info->pwmChannel2.timerChannel) == 0);
#endif
        motor->brake(i);
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
        assert(!high(info->disable) && compare(i, info->enable.timerChannel) == 0);
        info->fault.port->IDR &= ~info->fault.pin;
        assert(hw_drv8873_fault_active(i));
        motor->set_speed(i, 50);
        assert(high(info->disable));
        info->fault.port->IDR |= info->fault.pin;
        assert(!hw_drv8873_fault_active(i));
#else
        assert(compare(i, info->pwmChannel1.timerChannel) == 840 && compare(i, info->pwmChannel2.timerChannel) == 840);
#endif
        motor->set_speed(i, 0);
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
        assert(high(info->disable));
#else
        assert(compare(i, info->pwmChannel1.timerChannel) == 840 && compare(i, info->pwmChannel2.timerChannel) == 840);
#endif
        motor->set_speed(i, NAN);
        motor->set_speed(i, INFINITY);
        assert(delays == before_delay); // Speed, stop and brake never block the control loop.
    }
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
    assert(timer_inits == 1); // All four EN outputs share TIM1 without resetting siblings.
    motor->set_speed(0, 50);
    motor->set_speed(1, -75);
    assert(compare(0, TIM_CHANNEL_4) == 420 && compare(1, TIM_CHANNEL_3) == 630);
#endif
    // A driver can reset while USB keeps the MCU and its timer state alive.
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
    for (unsigned i = 0; i < NUMBER_MOTORS; ++i) {
        motor->set_speed(i, 70);
        registers[i] = 0x51;
        unsigned before = timer_inits;
        motor->initialize(i, false);
        assert(registers[i] == 0x40 && motor->is_initialized(i));
        assert_idle(i, false);
        assert(timer_inits == before);
        motor->set_speed(i, 70);
        fail_spi_motor = i;
        motor->initialize(i, false);
        assert(!motor->is_initialized(i));
        hw_motor_faults_t diagnostic;
        assert(!hw_motor_clear_faults(i, &diagnostic));
        assert_idle(i, false);
        assert(!high(motor_info[i].sleep) && high(motor_info[i].chipSelect));
        motor->set_speed(i, 70);
        assert_idle(i, false);
        fail_spi_motor = -1;
        motor->initialize(i, false);
        assert(motor->is_initialized(i) && registers[i] == 0x40);
        assert_idle(i, false);
        assert(timer_inits == before);
    }
#endif
    motor->initialize(255, false); motor->set_speed(255, 10); motor->stop(255); motor->brake(255);
    assert(!motor->is_initialized(255) && !motor->is_reversed(255));

    // Brake directly from motion, then resume and coast, in each direction and
    // reverse setting. A stop on a shared timer must leave sibling duties alone.
    for (unsigned i = 0; i < NUMBER_MOTORS; ++i) {
        const motor_info_t *info = &motor_info[i];
        for (unsigned reverse = 0; reverse < 2; ++reverse) {
            motor->initialize(i, reverse);
            for (int sign = -1; sign <= 1; sign += 2) {
                motor->set_speed(i, sign * 75);
                uint32_t before[12][4];
                for (unsigned t = 0; t < 12; ++t)
                    for (unsigned c = 0; c < 4; ++c)
                        before[t][c] = test_timers[t].compare[c];
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
                bool phase = high(info->direction);
#endif
                unsigned before_delay = delays;
                motor->brake(i);
                assert_idle(i, true);
                motor->brake(i);
                assert_idle(i, true);
                motor->set_speed(i, sign * 50);
                motor->stop(i);
                assert_idle(i, false);
                motor->stop(i);
                assert_idle(i, false);
                assert(delays == before_delay);
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
                assert(high(info->direction) == phase);
                assert(high(info->sleep)); // Stop/brake must not reset SPI mode through sleep.
#endif
                for (unsigned j = 0; j < NUMBER_MOTORS; ++j) {
                    if (i == j) continue;
                    const motor_info_t *other = &motor_info[j];
                    unsigned timer_index = other->timer - test_timers;
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
                    assert(compare(j, other->enable.timerChannel) == before[timer_index][other->enable.timerChannel/4]);
#else
                    assert(compare(j, other->pwmChannel1.timerChannel) == before[timer_index][other->pwmChannel1.timerChannel/4]);
                    assert(compare(j, other->pwmChannel2.timerChannel) == before[timer_index][other->pwmChannel2.timerChannel/4]);
#endif
                }
            }
        }
    }
    fault_reporting();
    clear_faults();
    current_limit_tests();
#if MOTOR_CONTROL_MODEL == MOTOR_CONTROL_MODEL_ENABLE_DIRECTION
    unsigned before_irq = transfer_count;
    for (unsigned i=0; i<NUMBER_MOTORS; ++i) {
        uint32_t sequence = hw_motor_fault_event_sequence(i);
        HAL_GPIO_EXTI_Callback(motor_info[i].fault.pin);
        HAL_GPIO_EXTI_Callback(motor_info[i].fault.pin);
        assert(hw_motor_fault_event_sequence(i) == sequence + 2);
    }
    assert(transfer_count == before_irq); // ISR never performs SPI or changes drive state.
#endif
    puts("Motor model, duty, reversal, coast/brake, current-limit settings and initialization checks passed");
    return 0;
}
