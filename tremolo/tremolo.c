#define F_CPU 8000000 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>

#include "adc.h"
#include "defs.h"
#include "wave.h"

static uint8_t tapst = 0;

static uint24_t phase;
static uint24_t phase_inc;

/* Occurs when PWM overflow happens. Since PWM is running at full CK, this
   interrupt occurs at 8 MHz / 256 = 31250 Hz. */
ISR(TIMER0_OVF_vect)
{
    /* Use the higest byte as phase for wave_func(). */
    uint8_t phase_hi = phase >> 16;

    /* Generate new value and write it to PWM. */
    OCR1A = wave_func(phase_hi);

    /* Increment the phase. */
    phase = phase + phase_inc;

    // FIXME: Reset cycle?

    /* Set the tap led state. */
    if ((phase_hi < 0x3f) || tapst)
        PORTB |= (1 << PB0);
    else
        PORTB &= ~(1 << PB0);
}

//SIGNAL(TIMER0_OVF0_vect)
//{
//    static uint16_t cnt = 0;
//    static uint8_t btst = 0;
//
//    btst <<= 1;
//    btst |= (PINB & (1 << PB2)) ? 0 : 1;
//
//    if (tapst && cnt >= 8192)
//        tapst = 0;
//
//    else if (!(tapst & 1) && btst == 0xff)
//    {
//        // Tap button pressed.
//        tapst++;
//
//        cli();
//
//        // Sync.
//        phase = 0;
//
//        if (tapst == 7)
//        {
//            // Last lap. Set the new tempo.
//            // FIXME
//            //tempo = cnt * 8;
//        }
//        else
//        {
//            // Reset the counter.
//            cnt = 0;
//        }
//
//        sei();
//    }
//    else if ((tapst & 1) && btst == 0x00)
//    {
//        // Button unpressed.
//        tapst++;
//
//        // Sequence completed.
//        if (tapst == 8)
//        {
//            tapst = 0;
//        }
//    }
//
//    cnt++;
//}

static uint8_t adc_read(uint8_t nadc)
{
    ADMUX = (1 <<ADLAR) | nadc;
    //ADCSR |= (1 << ADSC);
    //loop_until_bit_is_set(ADCSR, ADSC);
    return ADCH;
}

#define ROTARY_CMP_VAL(pos, npos) ((255 * pos + 127) / (npos - 1))

static void set_wave()
{
    uint8_t adc = adc_get(ADC_MODE);

    if (adc < ROTARY_CMP_VAL(0, 12))
        wf_set_rampup();
    else if (adc < ROTARY_CMP_VAL(1, 12))
        wf_set_rampdown();
    else if (adc < ROTARY_CMP_VAL(2, 12))
        wf_set_square();
    else if (adc < ROTARY_CMP_VAL(3, 12))
        wf_set_triangle();
    else
        wf_set_rampup();
}

static void set_tempo()
{
    /* */
}

static void set_note()
{
    /* */
}

static void init_timers()
{
    /* PWM driver. */
    TCCR0A = (1 << COM0A1) | (1 << WGM00) | (1 << WGM01);
    TCCR0B = (1 << CS00);

    // /* Configure TIMER0 for regular operation at CK/8 (1MHz). */
    // TCCR0 = (1 << CS01);
    // 
    // /* Enable output compare interrupt for PWM and overflow interrupt
    //    for TIMER0. */
    // TIMSK = (1 << OCIE1A) | (1 << TOIE0);

    TIMSK0 = (1 << TOIE0);
}

int main(void)
{
    init_timers();
    // 
    // /* Set pullup for tap button. */
    // PORTB = (1 << PB2);
    // 
    // /* Set PB0 (tap led) and PB1 (PWM output) as outputs. */
    // DDRB = (1 << PB0) | (1 << PB1);
    // 
    adc_init();

    /* Set ramp-up waveform by default. */
    wf_set_rampup();

    phase = 0;
    sei();

    phase_inc = 100;

    while (1)
    {
        if (adc_is_changed(ADC_MODE))
            set_wave();
        if (adc_is_changed(ADC_TEMPO))
            set_tempo();
        if (adc_is_changed(ADC_NOTE))
            set_note();
    }
}
