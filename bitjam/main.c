#define F_CPU (16000000UL)

#include <avr/io.h>
#include <avr/interrupt.h>

static uint8_t get_bit_mask(uint8_t nbits)
{
    return 0xff << (8 - nbits);
}

static uint8_t adc;
static uint8_t mask;

void SPI_MasterInit(void)
{
    /* Set MOSI and SCK output, all others input */
    DDRB = (1 << PORTB3) | (1 << PORTB5);
    /* Enable SPI, Master, set clock rate fck/16 */
    SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}

uint8_t SPI_MasterTransmit(char cData)
{
    /* Start transmission */
    SPDR = cData;

    /* Wait for transmission complete */
    while(!(SPSR & (1<<SPIF)))
        ;

    return SPDR;
}

ISR(TIMER1_OVF_vect)
{
    //adc = adc & mask;
    adc = SPI_MasterTransmit(adc);
}

int main(void)
{
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
