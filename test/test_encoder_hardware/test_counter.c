#include <assert.h>
#include <stdio.h>
#include "encoder_timer_sampling.h"

int main(void)
{
    TIM_TypeDef expected={0};
    expected.CR1=1;
    expected.SMCR=3;
    expected.CCMR1=0x0101;
    expected.CCER=0x0011;
    expected.ARR=65535;
    assert(encoder_timer_sampling_valid(&expected));
    // Use the independent STM32 register bit layout. Testing individual bits
    // detects polarity flips, missing edges, unexpected divide or filter modes.
    static const unsigned capture_bits[]={0,1,3,4,5,7};
    for (unsigned i=0;i<sizeof(capture_bits)/sizeof(*capture_bits);i++) {
        TIM_TypeDef t=expected;t.CCER^=1U<<capture_bits[i];assert(!encoder_timer_sampling_valid(&t));
    }
    for (unsigned i=0;i<16;i++) {
        TIM_TypeDef t=expected;t.CCMR1^=1U<<i;assert(!encoder_timer_sampling_valid(&t));
    }
    for (unsigned i=0;i<3;i++) {
        TIM_TypeDef t=expected;t.SMCR^=1U<<i;assert(!encoder_timer_sampling_valid(&t));
    }
    TIM_TypeDef t=expected;t.SMCR|=1U<<14;assert(!encoder_timer_sampling_valid(&t));
    t=expected;t.CR1=0;assert(!encoder_timer_sampling_valid(&t));
    t=expected;t.ARR--;assert(!encoder_timer_sampling_valid(&t));
    t=expected;t.PSC++;assert(!encoder_timer_sampling_valid(&t));
    t=expected;t.CR1|=1U<<4;t.CNT=16384;assert(encoder_timer_sampling_valid(&t));
    puts("PASS STM32F405 quadrature register model, physical polarity, input filter/prescale, timing and live reverse direction");
}
