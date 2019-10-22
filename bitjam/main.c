#define F_CPU (16000000UL)

#include <avr/io.h>
#include <avr/interrupt.h>

static uint8_t adc;
static uint8_t mask;

ISR(ADC_vect)
{
    adc = ADCH;
    adc = adc & mask;
    PORTD = adc;
}

int main(void)
{
    DDRD = 0b11111111;

    TCCR1A = (1 << WGM11);
    TCCR1B = (1 << CS10) | (1 << WGM12 ) | (1 << WGM13);
    ICR1 = F_CPU / 440;

    ADCSRA = (1 << ADEN) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2) | (1 << ADIE) | (1 << ADATE);
    ADCSRB = (1 << ADTS1) | (1 << ADTS2);
    ADMUX = (1 << ADLAR) | (1 << REFS0);

    mask = 0xff;

    sei();

    while (1)
        ;

    return 0;
}
