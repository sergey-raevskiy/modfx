#define F_CPU 8000000 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "defs.h"
#include "map_exp.h"
#include "wave.h"

enum {
    ADC_RATE,
    ADC_NOTE,
    ADC_DEPTH,
    ADC_WAVE
};

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

static uint8_t tapst = 0;

static uint24_t phase;
static uint24_t phase_inc;

/* Occurs when PWM overflow happens. */
ISR(TIMER1_CMPA_vect)
{
    /* Use the higest byte as phase for wave_func(). */
    uint8_t phase_hi = phase >> 16;

    /* Generate new value and write it to PWM. */
    uint8_t val = wave_func(phase_hi);
    OCR1A = map_exp(val);

    /* Increment the phase. */
    phase = phase + phase_inc;

    // FIXME: Reset cycle?

    set_bit(PORTB, PB0, (phase_hi < 0x3f) || tapst);
}

SIGNAL(TIMER0_OVF0_vect)
{
    static uint16_t cnt = 0;
    static uint8_t btst = 0;

    btst <<= 1;
    btst |= (PINB & (1 << PB2)) ? 0 : 1;

    if (tapst && cnt >= 8192)
        tapst = 0;

    else if (!(tapst & 1) && btst == 0xff)
    {
        // Tap button pressed.
        tapst++;

        cli();

        // Sync.
        phase = 0;

        if (tapst == 7)
        {
            // Last lap. Set the new tempo.
            // FIXME
            //tempo = cnt * 8;
        }
        else
        {
            // Reset the counter.
            cnt = 0;
        }

        sei();
    }
    else if ((tapst & 1) && btst == 0x00)
    {
        // Button unpressed.
        tapst++;

        // Sequence completed.
        if (tapst == 8)
        {
            tapst = 0;
        }
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

#define ROTARY_CMP_VAL(pos, npos) ((255 * pos + 127) / (npos - 1))

static void set_wave()
{
    uint8_t adc = adc_read(ADC_WAVE);

    if (adc < ROTARY_CMP_VAL(0, 12))
        wave_set(WF_RAMPUP);
    else if (adc < ROTARY_CMP_VAL(1, 12))
        wave_set(WF_RAMPDOWN);
    else if (adc < ROTARY_CMP_VAL(2, 12))
        wave_set(WF_SQUARE);
    else if (adc < ROTARY_CMP_VAL(3, 12))
        wave_set(WF_TRIANGLE);
    else
        wave_set(WF_RAMPUP);
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

    // Set initial waveform.
    wave_set(WF_RAMPUP);

    phase = 0;
    sei();

    // FIXME: Initial value.
    phase_inc = 100;
    uint8_t old_rate = 0;

    while (1)
    {
        // Tempo
        uint8_t new_rate = adc_read(ADC_RATE);

        if (new_rate != old_rate)
        {
            uint16_t new_tempo = map_lin(0xff - new_rate, 0, 255, 255, 65535);
            old_rate = new_rate;

            cli();
            phase_inc = new_tempo;
            sei();
        }

        // Waveform
        set_wave();
    }
}
