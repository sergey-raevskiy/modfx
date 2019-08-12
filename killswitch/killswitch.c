// 9.6 MHz, built in resonator
#define F_CPU 9600000

#include <avr/io.h>
#include <util/delay.h>

/* IO configuration */

#define OUT_PORT PORTB
#define OUT_DDR  DDRB
#define OUT_PIN  (1 << PINB0)

#define SW_PORT PORTB
#define SW_PINS PINB
#define SW_PIN  (1 << PINB0)

static uint8_t lfsr()
{
    static uint32_t S = 0x00000001;
    S = ((((S >> 31) ^ (S >> 30) ^ (S >> 29) ^ (S >> 27) ^ (S >> 25) ^ S ) & 0x00000001 ) << 31 ) | (S >> 1);
    return S & 1;
}

int main ()
{
    OUT_DDR |= OUT_PIN;

    /* pull-up */
    SW_PORT |= SW_PIN;

    while (1)
    {
        uint8_t b = lfsr();

        if (SW_PINS & SW_PIN)
            b = 0;

        if (b)
            OUT_PORT |= OUT_PIN;
        else
            OUT_PORT &= ~OUT_PIN;

        _delay_ms(50);
    }

    return 0;
}
