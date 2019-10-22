#define F_CPU (16000000UL)

#include <avr/io.h>
#include <avr/interrupt.h>

static uint8_t get_bit_mask(uint8_t nbits)
{
    return 0xff << (8 - nbits);
}

static uint8_t adc;
static uint8_t mask;

ISR(TIMER1_OVF_vect)
{
    //PORTD++;
}

ISR(ADC_vect)
{
    adc = ADCH;
    adc = adc & mask;
    PORTD = adc;
}

int main(void)
{
    DDRD = 0xff;

    TCCR1A = (1 << WGM11);
    TCCR1B = (1 << CS10) | (1 << WGM12) | (1 << WGM13);
    ICR1 = 400;
    TIMSK1 |= (1 << TOIE1);

    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADIE) | (1 << ADATE);
    ADCSRB = (1 << ADTS1) | (1 << ADTS2);
    ADMUX = (1 << ADLAR) | (1 << REFS0);

    mask = get_bit_mask(3);

    sei();

    while (1)
        ;

    return 0;
}
