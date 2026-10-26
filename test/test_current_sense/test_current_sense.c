#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "hw_config.h"
#include "hw_current_sense.h"

GPIO_TypeDef test_ports[5];
TIM_TypeDef test_timers[12];

#if NUMBER_CURRENT_SENSORS > 0
static unsigned init_calls, gpio_calls, conversions, stops, channel;
static unsigned fail_at; // 1 init, 2 configure, 3 start, 4 poll, 5 stop, 6 deinit
static uint32_t raw = 1117;
void test_adc_clock_enable(void) {}
void rcc_gpio_clk_enable(GPIO_TypeDef *port) { assert(port == GPIOC); }
void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *gpio) {
    assert(port == GPIOC);
    assert(gpio->Mode == GPIO_MODE_ANALOG && gpio->Pull == GPIO_NOPULL);
    assert(gpio->Pin == GPIO_PIN_0 || gpio->Pin == GPIO_PIN_1 || gpio->Pin == GPIO_PIN_4 || gpio->Pin == GPIO_PIN_5);
    ++gpio_calls;
}
// Any motor enable, sleep, PWM, or SPI operation would fail to link this test.
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *adc) {
    ++init_calls;
    assert(adc->Instance == ADC1 && adc->Init.ClockPrescaler == ADC_CLOCK_SYNC_PCLK_DIV4);
    assert(adc->Init.Resolution == ADC_RESOLUTION_12B && adc->Init.NbrOfConversion == 1);
    assert(!adc->Init.ContinuousConvMode && !adc->Init.ScanConvMode && !adc->Init.DMAContinuousRequests);
    return fail_at == 1 ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_DeInit(ADC_HandleTypeDef *adc) { assert(adc->Instance == ADC1); return fail_at == 6 ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *adc, ADC_ChannelConfTypeDef *config) {
    assert(adc->Instance == ADC1 && config->Rank == 1 && config->SamplingTime == ADC_SAMPLETIME_480CYCLES);
    channel = config->Channel; conversions = 0;
    return fail_at == 2 ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *adc) { (void)adc; return fail_at == 3 ? HAL_ERROR : HAL_OK; }
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *adc, uint32_t timeout) {
    (void)adc; assert(timeout == 1); ++conversions;
    return fail_at == 4 ? HAL_ERROR : HAL_OK;
}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *adc) {
    (void)adc;
    if (conversions == 1) return 4095; // Mux residue must not pollute the measurement.
    return raw + ((raw && raw < 4095 && conversions % 2) ? 2 : 0);
}
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *adc) { (void)adc; ++stops; return fail_at == 5 ? HAL_ERROR : HAL_OK; }
#endif

int main(void) {
    double amps = -123;
#if NUMBER_CURRENT_SENSORS > 0
    fail_at = 1; hw_current_sense_init();
    assert(hw_current_sense_read(0, &amps) == HW_CURRENT_SENSE_IO_ERROR && amps == -123);
    fail_at = 0;
    const unsigned channels[] = {14, 15, 10, 11};
    for (uint8_t i = 0; i < 4; ++i) {
        assert(hw_current_sense_read(i, &amps) == HW_CURRENT_SENSE_OK);
        assert(channel == channels[i] && conversions == 17);
        assert(fabs(amps - 1118.0 * 11.0 / 4095.0) < 1e-9);
    }
    assert(gpio_calls == init_calls * 4);
    assert(hw_current_sense_read(4, &amps) == HW_CURRENT_SENSE_UNAVAILABLE);
    assert(hw_current_sense_read(255, &amps) == HW_CURRENT_SENSE_UNAVAILABLE);
    assert(hw_current_sense_read(0, NULL) == HW_CURRENT_SENSE_IO_ERROR);
    raw = 0; assert(hw_current_sense_read(0, &amps) == HW_CURRENT_SENSE_OK && amps == 0);
    raw = 4095; amps = -123;
    assert(hw_current_sense_read(0, &amps) == HW_CURRENT_SENSE_OUT_OF_RANGE && amps == -123);
    raw = 1117;
    for (unsigned failure = 2; failure <= 5; ++failure) {
        fail_at = failure; unsigned previous_stops = stops, previous_init = init_calls;
        assert(hw_current_sense_read(0, &amps) == HW_CURRENT_SENSE_IO_ERROR && amps == -123);
        assert(stops > previous_stops); // No conversion remains running after failure.
        if (failure == 4) assert(conversions == 1); // Fail fast, bounded polling.
        fail_at = 6;
        assert(hw_current_sense_read(0, &amps) == HW_CURRENT_SENSE_IO_ERROR && amps == -123);
        fail_at = 0;
        assert(hw_current_sense_read(0, &amps) == HW_CURRENT_SENSE_OK && init_calls > previous_init);
        amps = -123;
    }
#else
    hw_current_sense_init();
    for (uint8_t i = 0; i < 4; ++i)
        assert(hw_current_sense_read(i, &amps) == HW_CURRENT_SENSE_UNAVAILABLE && amps == -123);
#endif
    puts("Current sensing: channel mapping, conversion, averaging, saturation, recovery and legacy absence passed");
    return 0;
}
