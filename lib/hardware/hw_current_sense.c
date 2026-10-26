#include "hw_current_sense.h"
#include "hw_config.h"

#if NUMBER_CURRENT_SENSORS > 0
#include "hw_timers.h"

#define CURRENT_ADC_SAMPLES 16U
#define CURRENT_ADC_FULL_SCALE 4095U
#define CURRENT_ADC_TIMEOUT_MS 1U
static ADC_HandleTypeDef current_adc;
static bool adc_ready;

static bool prepare_adc(void)
{
    if (adc_ready) return true;
    __HAL_RCC_ADC1_CLK_ENABLE();
    current_adc.Instance = ADC1;
    // Clear a failed conversion before retrying. ADC2/3 are not reset.
    if (HAL_ADC_DeInit(&current_adc) != HAL_OK) return false;
    for (unsigned i = 0; i < NUMBER_MOTORS; ++i) {
        const current_sense_info_t *sense = &motor_info[i].currentSense;
        if (!sense->available || sense->type != CURRENT_SENSE_ADC) continue;
        rcc_gpio_clk_enable(sense->input.port);
        GPIO_InitTypeDef gpio = {0};
        gpio.Pin = sense->input.pin;
        gpio.Mode = GPIO_MODE_ANALOG;
        gpio.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(sense->input.port, &gpio);
    }
    // PCLK2 = 84 MHz; divide by 4 keeps ADC at 21 MHz (below 36 MHz maximum).
    current_adc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    current_adc.Init.Resolution = ADC_RESOLUTION_12B;
    current_adc.Init.ScanConvMode = DISABLE;
    current_adc.Init.ContinuousConvMode = DISABLE;
    current_adc.Init.DiscontinuousConvMode = DISABLE;
    current_adc.Init.NbrOfDiscConversion = 0;
    current_adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    current_adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    current_adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    current_adc.Init.NbrOfConversion = 1;
    current_adc.Init.DMAContinuousRequests = DISABLE;
    current_adc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    adc_ready = HAL_ADC_Init(&current_adc) == HAL_OK;
    return adc_ready;
}

static hw_current_sense_status_t io_error(void)
{
    (void)HAL_ADC_Stop(&current_adc);
    adc_ready = false; // Recover on the next request, without blocking boot.
    return HW_CURRENT_SENSE_IO_ERROR;
}
#endif

void hw_current_sense_init(void)
{
#if NUMBER_CURRENT_SENSORS > 0
    (void)prepare_adc();
#endif
}

hw_current_sense_status_t hw_current_sense_read(uint8_t index, double *amps)
{
#if NUMBER_CURRENT_SENSORS > 0
    if (index >= NUMBER_MOTORS) return HW_CURRENT_SENSE_UNAVAILABLE;
    const current_sense_info_t *sense = &motor_info[index].currentSense;
    if (!sense->available || sense->type != CURRENT_SENSE_ADC) return HW_CURRENT_SENSE_UNAVAILABLE;
    if (!amps) return HW_CURRENT_SENSE_IO_ERROR;
    if (!prepare_adc()) return HW_CURRENT_SENSE_IO_ERROR;
    ADC_ChannelConfTypeDef channel = {0};
    channel.Channel = sense->adcChannel;
    channel.Rank = 1;
    channel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&current_adc, &channel) != HAL_OK) return io_error();
    uint32_t sum = 0;
    // Discard the first conversion after switching the mux, then average 16.
    // The external RC already filters PWM; no duty-cycle compensation is applied.
    for (unsigned i = 0; i <= CURRENT_ADC_SAMPLES; ++i) {
        if (HAL_ADC_Start(&current_adc) != HAL_OK ||
            HAL_ADC_PollForConversion(&current_adc, CURRENT_ADC_TIMEOUT_MS) != HAL_OK)
            return io_error();
        uint32_t raw = HAL_ADC_GetValue(&current_adc);
        if (HAL_ADC_Stop(&current_adc) != HAL_OK) return io_error();
        if (!i) continue;
        if (raw >= CURRENT_ADC_FULL_SCALE) return HW_CURRENT_SENSE_OUT_OF_RANGE;
        sum += raw;
    }
    *amps = ((double)sum / CURRENT_ADC_SAMPLES / CURRENT_ADC_FULL_SCALE) *
        ((double)sense->referenceMillivolts / 1000.0) *
        sense->currentMirrorRatio / sense->senseResistorOhm;
    return HW_CURRENT_SENSE_OK;
#else
    (void)index;
    (void)amps;
    return HW_CURRENT_SENSE_UNAVAILABLE;
#endif
}
