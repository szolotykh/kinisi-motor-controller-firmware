//------------------------------------------------------------
// File name: loop_frequency.c
//------------------------------------------------------------
#include "loop_frequency.h"

uint32_t loop_frequency_hz_to_period_ms(uint16_t frequency_hz)
{
    if (frequency_hz == 0)
    {
        return 0; // Invalid frequency
    }

    uint32_t period_ms = 1000u / frequency_hz;
    if (period_ms == 0)
    {
        period_ms = 1; // Clamp to the 1 ms tick (max 1000 Hz)
    }
    return period_ms;
}

uint16_t loop_period_ms_to_frequency_hz(uint32_t period_ms)
{
    if (period_ms == 0)
    {
        return 0;
    }
    return (uint16_t)(1000u / period_ms);
}
