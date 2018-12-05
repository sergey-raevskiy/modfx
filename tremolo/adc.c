#include "adc.h"
#include <avr/io.h>

#define ADC_CHANGE_THRESHOLD 10

/* Current ADC values. */
static uint8_t adcvec[ADC_TOTAL];

void adc_init()
{
    /* Configure ADC. */
    ADCSRA = (1 << ADEN) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);
}

uint8_t adc_is_changed(uint8_t adc)
{
    uint8_t prev = adcvec[adc];
    uint8_t val;

    /* Read ADC. */
    ADMUX = (1 << ADLAR) | (1 << REFS0) | adc;
    ADCSRA |= (1 << ADSC);
    loop_until_bit_is_clear(ADCSRA, ADSC);
    val = ADCH;

    if ( ((prev > val) && (prev - val) > ADC_CHANGE_THRESHOLD)
        || ((val > prev) && (val - prev) > ADC_CHANGE_THRESHOLD) )
    {
        /* If difference is above threshold save the new
           value and return true. */
        adcvec[adc] = val;
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8_t adc_get(uint8_t adc)
{
    return adcvec[adc];
}
