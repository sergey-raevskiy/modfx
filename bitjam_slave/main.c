#include <avr/io.h>

ISR(SPI0_STC_vect)
{
    PORTD = SPDR;
    SPDR = ADCH;

    /* Start new conversion */
    ADCSRA |= (1 << ADSC);
}

int main(void)
{
    /* Init PORTD */
    DDRD = 0xff;

    /* Init ADC */
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADIE) | (1 << ADATE);
    ADCSRB = (1 << ADTS1) | (1 << ADTS2);
    ADMUX = (1 << ADLAR) | (1 << REFS0);

    /* Set MISO output, all others input */
    DDRB = (1 << PORTB4);

    /* Enable SPI */
    SPCR = (1 << SPE) | (1 << SPIE);
    sei();

    while (1)
    {
        /* loop forever */
    }
}
