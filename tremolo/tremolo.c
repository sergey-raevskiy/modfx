#define F_CPU 8000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

static uint8_t map_exp(uint8_t val)
{
    static const uint8_t lookup[] PROGMEM = {
          0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
          0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
          1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,
          2,   2,   2,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   3,   3,
          3,   3,   3,   3,   3,   4,   4,   4,   4,   4,   4,   4,   4,   4,   5,   5,
          5,   5,   5,   5,   5,   5,   6,   6,   6,   6,   6,   6,   6,   7,   7,   7,
          7,   7,   8,   8,   8,   8,   8,   9,   9,   9,   9,   9,  10,  10,  10,  10,
         11,  11,  11,  11,  12,  12,  12,  12,  13,  13,  13,  14,  14,  14,  15,  15,
         15,  16,  16,  16,  17,  17,  18,  18,  18,  19,  19,  20,  20,  21,  21,  22,
         22,  23,  23,  24,  24,  25,  25,  26,  26,  27,  28,  28,  29,  30,  30,  31,
         32,  32,  33,  34,  35,  35,  36,  37,  38,  39,  40,  40,  41,  42,  43,  44,
         45,  46,  47,  48,  49,  51,  52,  53,  54,  55,  56,  58,  59,  60,  62,  63,
         64,  66,  67,  69,  70,  72,  73,  75,  77,  78,  80,  82,  84,  86,  88,  90,
         91,  94,  96,  98, 100, 102, 104, 107, 109, 111, 114, 116, 119, 122, 124, 127,
        130, 133, 136, 139, 142, 145, 148, 151, 155, 158, 161, 165, 169, 172, 176, 180,
        184, 188, 192, 196, 201, 205, 210, 214, 219, 224, 229, 234, 239, 244, 250, 255
    };

    return pgm_read_byte(&lookup[val]);
}

#define set_bit(reg, bit, state) do { \
    if (state)                        \
        reg |= (1 << bit);            \
    else                              \
        reg &= ~(1 << bit);           \
} while (0)

ISR(TIMER0_OVF0_vect)
{
    static uint8_t i = 0;
    OCR1A = map_exp(i++);
    set_bit(PORTB, PB0, i < 0x3f);
}

int main(void)
{
    // TIMER0 used as waveform generator.
    TCCR0 = (CS01 << 1);

    // Configuring TIMER1 as PWM driver for vactrol.
    TCCR1B  = (1 << CS10) | (1 << CTC1);
    TCCR1A = (1 << PWM1A) | (1 << COM1A1);
    OCR1C = 0xff;

    // Enable TIMER0 interrupt.
    TIMSK = (1 << TOIE0);

    DDRB = (1 << PB0) | (1 << PB1);

    sei();
    while (1);
}