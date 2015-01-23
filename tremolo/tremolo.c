#define F_CPU 8000000 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#define unlikely(x) __builtin_expect(x, 0)

enum {
    ADC_RATE,
    ADC_NOTE,
    ADC_DEPTH,
    ADC_WAVE
};

typedef enum {
    WF_RAMPUP,
    WF_RAMPDOWN,
    WF_SQUARE,
    WF_TRIANGLE,
    WF_RES2,
    WF_RES3,
    WF_RES4,
    WF_RES5,
    WF_RES6,
    WF_RES7,
    WF_RES8,
    WF_RES9,

    WF_TOTAL
} waveform_t;

_Static_assert(WF_TOTAL == 12, "Total waveforms mismatch");

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

static inline long map_lin(long x, long xmin, long xmax, long ymin, long ymax)
{
    return (x - xmin) * (ymax - ymin) / (xmax - xmin) + ymin;
}

#define set_bit(reg, bit, state) do { \
    if (state)                        \
        reg |= (1 << bit);            \
    else                              \
        reg &= ~(1 << bit);           \
} while (0)

static uint16_t tempo = 400;
static waveform_t wave = WF_RAMPUP;

static uint8_t wave_func(uint8_t i)
{
    switch (wave)
    {
    case WF_RAMPUP:
        return i;
    case WF_RAMPDOWN:
        return ~i;
    case WF_SQUARE:
        return (i & 0x80) ? 0 : 0xff;
    case WF_TRIANGLE:
        return (i & 0x80) ? (0xff - (i << 1)) : ((i - 0x80) << 1);
    default:
        return i;
    }
}

static uint8_t tapst = 0;

static struct {
    uint8_t y;
    uint16_t next;
    uint16_t cnt;
} lfo;

static void reset_cycle()
{
    lfo.y = 0;
    lfo.cnt = 0;
    lfo.next = 0;
}

ISR(TIMER1_CMPA_vect)
{
    if (unlikely(lfo.cnt >= tempo))
        reset_cycle();

    if (unlikely(lfo.cnt >= lfo.next))
    {
        OCR1A = map_exp(wave_func(lfo.y));

        lfo.y++;
        lfo.next = map_lin(lfo.y, 0, 255, 0, tempo);
    }

    set_bit(PORTB, PB0, (lfo.y < 0x3f) || tapst);

    lfo.cnt++;
}

SIGNAL(TIMER0_OVF0_vect)
{
    static uint16_t cnt = 0;
    static uint8_t btst = 0;

    btst <<= 1;
    btst |= (PINB & (1 << PB2)) ? 0 : 1;

    if (tapst && cnt >= 8192)
        tapst = 0;

    if (tapst == 2 * 4 /* 4 taps */)
    {
        // Got new tempo.
        tapst = 0;

        cli();
        tempo = cnt * 8;
        sei();
    }
    else if (!(tapst & 1) && btst == 0xff)
    {
        // Tap button pressed.
        tapst++;

        // Sync.
        reset_cycle();

        // Reset the counter.
        cnt = 0;
    }
    else if ((tapst & 1) && btst == 0x00)
    {
        // Button unpressed.
        tapst++;
    }

    cnt++;
}

static uint8_t adc_read(uint8_t nadc)
{
    ADMUX = (1 <<ADLAR) | nadc;
    ADCSR |= (1 << ADSC);
    while (ADCSR & (1 << ADSC));
    return ADCH;
}

static uint8_t map_rotary(uint8_t val, uint8_t npos)
{
    uint8_t pos;
    for (pos = 0; pos < npos; pos++) {
        uint16_t cmp = (255 * pos + 127) / (npos - 1);
        if (val < cmp)
            break;
    }
    return pos;
}

int main(void)
{
    // Configuring TIMER1 as PWM driver for vactrol.
    TCCR1B  = (1 << CS10) | (1 << CTC1);
    TCCR1A = (1 << PWM1A) | (1 << COM1A1);
    OCR1C = 0xff;

    TCCR0 = (1 << CS01);

    // Enable OCRA interrupt.
    TIMSK = (1 << OCIE1A) | (1 << TOIE0);

    PORTB = (1 << PB2);
    DDRB = (1 << PB0) | (1 << PB1);

    // Enable ADC.
    ADCSR = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    reset_cycle();
    sei();

    tempo = 1;
    uint8_t old_rate = 0;

    while (1)
    {
        // Tempo
        uint8_t new_rate = adc_read(ADC_RATE);

        if (new_rate != old_rate)
        {
            uint16_t new_tempo = map_lin(0xff - new_rate, 0, 255, 1, 65535);
            old_rate = new_rate;

            cli();
            tempo = new_tempo;
            sei();
        }

        // Waveform
        wave = map_rotary(adc_read(ADC_WAVE), WF_TOTAL);
    }
}
