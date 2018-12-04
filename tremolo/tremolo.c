#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/pgmspace.h>

#include "adc.h"
#include "defs.h"
#include "wave.h"

/* Helper for rotary switches. */
#define ROTARY_CMP_VAL(pos, npos) ((255 * pos + 127) / (npos - 1))

/* BPM led */
#define BPM_LED_PORT  PORTB
#define BPM_LED_DDR   DDRB
#define BPM_LED_PIN   PINB5
#define BPM_LED_ON()  (BPM_LED_PORT |= (1 << BPM_LED_PIN))
#define BPM_LED_OFF() (BPM_LED_PORT &= ~(1 << BPM_LED_PIN))

static uint8_t tapst = 0;

static uint8_t phase;

/* Use 16-bit timer/counter for phase increment. */
ISR(TIMER1_OVF_vect)
{
    /* Generate new value and write it to PWM. */
    OCR0A = wave_func(phase);

    /* Set the tap led state. */
    if ((phase < 0x3f) || tapst)
        BPM_LED_ON();
    else
        BPM_LED_OFF();

    phase++;
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
    static const uint16_t icr[] PROGMEM =
    {
        50000, 49728, 49459, 49191, 48924, 48659, 48395, 48132, 47872, 47612, 47354, 47097, 46842, 46588, 46335, 46084,
        45834, 45586, 45338, 45093, 44848, 44605, 44363, 44123, 43883, 43646, 43409, 43174, 42939, 42707, 42475, 42245,
        42016, 41788, 41561, 41336, 41112, 40889, 40667, 40447, 40228, 40009, 39793, 39577, 39362, 39149, 38937, 38725,
        38515, 38307, 38099, 37892, 37687, 37483, 37279, 37077, 36876, 36676, 36477, 36280, 36083, 35887, 35693, 35499,
        35307, 35115, 34925, 34736, 34547, 34360, 34174, 33988, 33804, 33621, 33439, 33257, 33077, 32898, 32719, 32542,
        32365, 32190, 32015, 31842, 31669, 31498, 31327, 31157, 30988, 30820, 30653, 30487, 30321, 30157, 29993, 29831,
        29669, 29508, 29348, 29189, 29031, 28874, 28717, 28561, 28406, 28252, 28099, 27947, 27795, 27645, 27495, 27346,
        27197, 27050, 26903, 26757, 26612, 26468, 26325, 26182, 26040, 25899, 25758, 25619, 25480, 25342, 25204, 25068,
        24932, 24796, 24662, 24528, 24395, 24263, 24132, 24001, 23871, 23741, 23612, 23484, 23357, 23230, 23104, 22979,
        22855, 22731, 22607, 22485, 22363, 22242, 22121, 22001, 21882, 21763, 21645, 21528, 21411, 21295, 21180, 21065,
        20951, 20837, 20724, 20612, 20500, 20389, 20278, 20168, 20059, 19950, 19842, 19734, 19627, 19521, 19415, 19310,
        19205, 19101, 18998, 18895, 18792, 18690, 18589, 18488, 18388, 18288, 18189, 18090, 17992, 17895, 17798, 17701,
        17605, 17510, 17415, 17320, 17227, 17133, 17040, 16948, 16856, 16765, 16674, 16583, 16493, 16404, 16315, 16227,
        16139, 16051, 15964, 15877, 15791, 15706, 15621, 15536, 15452, 15368, 15285, 15202, 15119, 15037, 14956, 14875,
        14794, 14714, 14634, 14555, 14476, 14397, 14319, 14242, 14164, 14088, 14011, 13935, 13860, 13785, 13710, 13635,
        13562, 13488, 13415, 13342, 13270, 13198, 13126, 13055, 12984, 12914, 12844, 12774, 12705, 12636, 12568, 12500,
    };

    ICR1 = pgm_read_word(&icr[adc_get(ADC_TEMPO)]);
}

static void set_note()
{
    /* Note lengths:
        1/1
        1/2
        1/4
        1/8
        1/16 ???
      */
}

static void init_gpio()
{
    BPM_LED_DDR |= (1 << BPM_LED_PIN);
}

static void init_timers()
{
    /* PWM driver. Use OC1A (PB2) as output. */
    TCCR0A = (1 << COM0A1) | (1 << WGM00) | (1 << WGM01);
    TCCR0B = (1 << CS00);
    DDRB |= (1 << DDB2);

    // /* Configure TIMER0 for regular operation at CK/8 (1MHz). */
    // TCCR0 = (1 << CS01);
    // 
    // /* Enable output compare interrupt for PWM and overflow interrupt
    //    for TIMER0. */
    // TIMSK = (1 << OCIE1A) | (1 << TOIE0);

    /* Configure timer1 as plain timer. Ck/1, TOP value is ICR1. */
    ICR1 = 10000;
    TCCR1A = (1 << WGM11);
    TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10);

    /* Enable timer1 overflow interrupt. */
    TIMSK1 = (1 << TOIE1);
}

int main(void)
{
    init_gpio();
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
