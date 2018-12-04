#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

enum
{
    ADC_MODE,
    ADC_TEMPO,
    ADC_NOTE,
    ADC_TOTAL
};

void adc_init();
uint8_t adc_is_changed(uint8_t adc);
uint8_t adc_get(uint8_t adc);

#endif /* ADC_H_ */
