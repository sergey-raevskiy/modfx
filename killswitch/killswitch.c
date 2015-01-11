// 9.6 MHz, built in resonator
#define F_CPU 9600000

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>

uint8_t adc_read(uint8_t nadc)
{
    ADMUX = (1 << ADLAR) | nadc;

    // Start the conversion
    ADCSRA |= (1 << ADSC);

    // Wait for it to finish
    while (ADCSRA & (1 << ADSC));

    return ADCH;
}

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

int main ()
{
    /* PWM setup */

    // PB0 & PB1 is PWM outputs for LEDs.
    DDRB |= (1 << PB0) | (1 << PB1);

    TCCR0B |= (1 << CS00);

    // Set to 'Fast PWM' mode
    TCCR0A = (1 << WGM01) | (1 << WGM00) | (1 << COM0B1) | (1 << COM0A1);

    /* ADC setup */

    // Set the prescaler to clock/128 & enable ADC
    // At 9.6 MHz this is 75 kHz.
    // See ATtiny13 datasheet, Table 14.4.
    ADCSRA = (1 << ADPS1) | (0 << ADPS0) | (1 << ADEN);

    /* Switch setup */
    DDRB &= ~(1 << PB2);

    while (1) {
        if (!(PINB & (1 << PB2))) {
            uint8_t rate = adc_read(2);
            uint8_t depth = adc_read(3);

            if (rand() % 4) {
                depth = 0;
            }

            OCR0A = OCR0B = ~map_exp(0xff - depth);

            int delay = (0xff - map_exp(rate) + 30) / 4;
            while (delay--) {
                _delay_us(100);
            }
        } else {
            OCR0B = ~map_exp(0x7f);
            OCR0A = 0xff;
        }
    }
}
