#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/pgmspace.h>

#include "adc.h"
#include "wave.h"

/* Helper for rotary switches. */
#define ROTARY_CMP_VAL(pos, npos) ((255 * pos + 127) / (npos - 1))

/* BPM led */
#define BPM_LED_PORT  PORTB
#define BPM_LED_DDR   DDRB
#define BPM_LED_PIN   PINB5
#define BPM_LED_ON()  (BPM_LED_PORT |= (1 << BPM_LED_PIN))
#define BPM_LED_OFF() (BPM_LED_PORT &= ~(1 << BPM_LED_PIN))

/* Tap button */
#define TAP_TEMPO_PORT   PORTB
#define TAP_TEMPO_DDR    DDRB
#define TAP_TEMPO_PIN    PINB4
#define TAP_IS_PRESSED() bit_is_clear(PINB, TAP_TEMPO_PIN)

/* Tap button state. */
static uint8_t tapst = 0;

/* LFO phase. */
static uint8_t phase;

/* Use 16-bit timer/counter for phase increment. */
/* freq = tempo / 60 * 256 */
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

/* Tap-tempo monitor routine. */
/* freq = fcpu / 256 / 8 = 7812,5 Hz */
ISR(TIMER2_OVF_vect)
{
    static uint16_t tap_counter = 0;
    static uint8_t tap_debounce = 0;

    /* Increment the tap counter. */
    tap_counter++;

    /* Update debounce vector. */
    tap_debounce <<= 1;
    tap_debounce |= TAP_IS_PRESSED() ? 1 : 0;

    if (tapst != 0 && tap_counter >= 8192)
    {
        /* Prevent overflow. */
        tapst = 0;
    }

    switch (tapst)
    {
    case 0:
        if (tap_debounce == 0xff)
        {
            /* Tap button pressed first time. */

            /* Sync. */
            phase = 0;

            /* Reset counter. */
            tap_counter = 0;

            /* Go to next state. */
            tapst++;
        }
        break;
    case 1:
        if (tap_debounce == 0x00)
        {
            /* Tap button unpressed. */
            tapst++;
        }
        break;
    case 2:
        if (tap_debounce == 0xff)
        {
            /* Tap button pressed second time. */

            /* Set new tempo. */
            ICR1 = tap_counter * 8;

            tapst++;
        }
    case 3:
        if (tap_debounce == 0x00)
        {
            tapst = 0;
        }
        break;
    }
}

static void set_wave()
{
    if (adc_is_changed(ADC_MODE))
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
        else if (adc < ROTARY_CMP_VAL(4, 12))
            wf_set_sine();
        else
            wf_set_rampup();
    }
}

static void set_tempo()
{
    static const uint16_t icr[] PROGMEM =
    {
        50000, 49728, 49459, 49191, 48924, 48659, 48395, 48132,
        47872, 47612, 47354, 47097, 46842, 46588, 46335, 46084,
        45834, 45586, 45338, 45093, 44848, 44605, 44363, 44123,
        43883, 43646, 43409, 43174, 42939, 42707, 42475, 42245,
        42016, 41788, 41561, 41336, 41112, 40889, 40667, 40447,
        40228, 40009, 39793, 39577, 39362, 39149, 38937, 38725,
        38515, 38307, 38099, 37892, 37687, 37483, 37279, 37077,
        36876, 36676, 36477, 36280, 36083, 35887, 35693, 35499,
        35307, 35115, 34925, 34736, 34547, 34360, 34174, 33988,
        33804, 33621, 33439, 33257, 33077, 32898, 32719, 32542,
        32365, 32190, 32015, 31842, 31669, 31498, 31327, 31157,
        30988, 30820, 30653, 30487, 30321, 30157, 29993, 29831,
        29669, 29508, 29348, 29189, 29031, 28874, 28717, 28561,
        28406, 28252, 28099, 27947, 27795, 27645, 27495, 27346,
        27197, 27050, 26903, 26757, 26612, 26468, 26325, 26182,
        26040, 25899, 25758, 25619, 25480, 25342, 25204, 25068,
        24932, 24796, 24662, 24528, 24395, 24263, 24132, 24001,
        23871, 23741, 23612, 23484, 23357, 23230, 23104, 22979,
        22855, 22731, 22607, 22485, 22363, 22242, 22121, 22001,
        21882, 21763, 21645, 21528, 21411, 21295, 21180, 21065,
        20951, 20837, 20724, 20612, 20500, 20389, 20278, 20168,
        20059, 19950, 19842, 19734, 19627, 19521, 19415, 19310,
        19205, 19101, 18998, 18895, 18792, 18690, 18589, 18488,
        18388, 18288, 18189, 18090, 17992, 17895, 17798, 17701,
        17605, 17510, 17415, 17320, 17227, 17133, 17040, 16948,
        16856, 16765, 16674, 16583, 16493, 16404, 16315, 16227,
        16139, 16051, 15964, 15877, 15791, 15706, 15621, 15536,
        15452, 15368, 15285, 15202, 15119, 15037, 14956, 14875,
        14794, 14714, 14634, 14555, 14476, 14397, 14319, 14242,
        14164, 14088, 14011, 13935, 13860, 13785, 13710, 13635,
        13562, 13488, 13415, 13342, 13270, 13198, 13126, 13055,
        12984, 12914, 12844, 12774, 12705, 12636, 12568, 12500,
    };

    if (adc_is_changed(ADC_TEMPO))
    {
        ICR1 = pgm_read_word(&icr[adc_get(ADC_TEMPO)]);
    }
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

    if (adc_is_changed(ADC_NOTE))
    {
        // TODO
    }
}

static void init_gpio()
{
    BPM_LED_DDR |= (1 << BPM_LED_PIN);

    /* Clear DDR and enable pull-up. */
    TAP_TEMPO_DDR &= ~(1 << TAP_TEMPO_PIN);
    TAP_TEMPO_PORT |= (1 << TAP_TEMPO_PIN);
}

static void init_timers()
{
    /* PWM driver. Use OC1A (PB2) as output. */
    TCCR0A = (1 << COM0A1) | (1 << WGM00) | (1 << WGM01);
    TCCR0B = (1 << CS00);
    DDRD |= (1 << DDD6);

    /* Configure timer1 as plain timer. Ck/1, TOP value is ICR1. */
    ICR1 = 24932; /* 150 BPM */
    TCCR1A = (1 << WGM11);
    TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS10);

    /* Enable timer1 overflow interrupt. */
    TIMSK1 = (1 << TOIE1);

    /* Configure timer2 as plain timer. Ck/8, TOP value is 0xff. */
    TCCR2B = (1 << CS21);

    /* Enable timer2 overflow interrupt. */
    TIMSK2 = (1 << TOIE2);
}

int main(void)
{
    init_gpio();
    init_timers();
    adc_init();

    /* Set ramp-up waveform by default. */
    wf_set_rampup();

    sei();

    while (1)
    {
        set_wave();
        set_tempo();
        set_note();
    }
}
