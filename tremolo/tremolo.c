#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/pgmspace.h>

#include "adc.h"
#include "wave.h"

/* Helper for rotary switches. */
#define ROTARY_CMP_VAL(pos, npos) ((255 * pos + 127) / (npos - 1))

/* BPM led */
#define BPM_LED_PORT  PORTD
#define BPM_LED_DDR   DDRD
#define BPM_LED_PIN   PIND7
#define BPM_LED_ON()  (BPM_LED_PORT |= (1 << BPM_LED_PIN))
#define BPM_LED_OFF() (BPM_LED_PORT &= ~(1 << BPM_LED_PIN))

/* Tap button */
#define TAP_TEMPO_PORT   PORTB
#define TAP_TEMPO_DDR    DDRB
#define TAP_TEMPO_PIN    PINB0
#define TAP_IS_PRESSED() bit_is_clear(PINB, TAP_TEMPO_PIN)

/* Tap button state. */
static uint8_t tapst = 0;

/* Note length multiplier in 16th. */
static uint8_t note_multiplier;

/* LFO phase. */
static uint8_t lfo_phase;
static uint8_t lfo_subcounter;

/* BPM led phase. */
static uint16_t bpm_phase;

static void update_lfo()
{
    /* Generate new value and write it to PWM. */
    OCR0A = wave_func(lfo_phase);

    /* Frequency division. */
    if (++lfo_subcounter >= note_multiplier)
    {
        lfo_phase++;
        lfo_subcounter = 0;
    }
}

static void update_bpm_led()
{
    /* Set the tap led state. */
    if (((bpm_phase & 0x300) == 0) || tapst)
        BPM_LED_ON();
    else
        BPM_LED_OFF();

    bpm_phase++;
}

/* Use 16-bit timer/counter for phase increment. */
/* freq = tempo / 60 * 256 */
ISR(TIMER1_OVF_vect)
{
    update_lfo();
    update_bpm_led();
}

static void reset_phase()
{
    cli();
    lfo_phase = 0;
    lfo_subcounter = 0;
    bpm_phase = 0;
    sei();
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
            reset_phase();

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
        break;
    case 3:
        if (tap_debounce == 0x00)
        {
            tapst = 0;
        }
        break;
    }
}

static void set_wave(uint8_t val)
{
    if (val < ROTARY_CMP_VAL(0, 12))
        wf_set_rampup();
    else if (val < ROTARY_CMP_VAL(1, 12))
        wf_set_rampdown();
    else if (val < ROTARY_CMP_VAL(2, 12))
        wf_set_square();
    else if (val < ROTARY_CMP_VAL(3, 12))
        wf_set_triangle();
    else if (val < ROTARY_CMP_VAL(4, 12))
        wf_set_sine();
    else
        wf_set_rampup();
}

static void set_tempo(uint8_t val)
{
    /* See tempo.py */
    static const uint16_t icr[] PROGMEM =
    {
        12500, 12432, 12364, 12297, 12231, 12164, 12098, 12033,
        11968, 11903, 11838, 11774, 11710, 11647, 11583, 11521,
        11458, 11396, 11334, 11273, 11212, 11151, 11090, 11030,
        10970, 10911, 10852, 10793, 10734, 10676, 10618, 10561,
        10504, 10447, 10390, 10334, 10278, 10222, 10166, 10111,
        10057, 10002,  9948,  9894,  9840,  9787,  9734,  9681,
         9628,  9576,  9524,  9473,  9421,  9370,  9319,  9269,
         9219,  9169,  9119,  9070,  9020,  8971,  8923,  8874,
         8826,  8778,  8731,  8684,  8636,  8590,  8543,  8497,
         8451,  8405,  8359,  8314,  8269,  8224,  8179,  8135,
         8091,  8047,  8003,  7960,  7917,  7874,  7831,  7789,
         7747,  7705,  7663,  7621,  7580,  7539,  7498,  7457,
         7417,  7377,  7337,  7297,  7257,  7218,  7179,  7140,
         7101,  7063,  7024,  6986,  6948,  6911,  6873,  6836,
         6799,  6762,  6725,  6689,  6653,  6617,  6581,  6545,
         6510,  6474,  6439,  6404,  6370,  6335,  6301,  6267,
         6233,  6199,  6165,  6132,  6098,  6065,  6033,  6000,
         5967,  5935,  5903,  5871,  5839,  5807,  5776,  5744,
         5713,  5682,  5651,  5621,  5590,  5560,  5530,  5500,
         5470,  5440,  5411,  5382,  5352,  5323,  5295,  5266,
         5237,  5209,  5181,  5153,  5125,  5097,  5069,  5042,
         5014,  4987,  4960,  4933,  4906,  4880,  4853,  4827,
         4801,  4775,  4749,  4723,  4698,  4672,  4647,  4622,
         4597,  4572,  4547,  4522,  4498,  4473,  4449,  4425,
         4401,  4377,  4353,  4330,  4306,  4283,  4260,  4237,
         4214,  4191,  4168,  4145,  4123,  4101,  4078,  4056,
         4034,  4012,  3991,  3969,  3947,  3926,  3905,  3884,
         3863,  3842,  3821,  3800,  3779,  3759,  3739,  3718,
         3698,  3678,  3658,  3638,  3619,  3599,  3579,  3560,
         3541,  3522,  3502,  3483,  3465,  3446,  3427,  3408,
         3390,  3372,  3353,  3335,  3317,  3299,  3281,  3263,
         3246,  3228,  3211,  3193,  3176,  3159,  3142,  3125,
    };

    ICR1 = pgm_read_word(&icr[val]);
}

static void set_note(uint8_t val)
{
    if (val < ROTARY_CMP_VAL(0, 6))
    {
        /* 1/1 note in 16th */
        note_multiplier = 16;
    }
    else if (val < ROTARY_CMP_VAL(1, 6))
    {
        /* 1/2 note in 16th */
        note_multiplier = 8;
    }
    else if (val < ROTARY_CMP_VAL(2, 6))
    {
        /* 1/4 note in 16th */
        note_multiplier = 4;
    }
    else if (val < ROTARY_CMP_VAL(3, 6))
    {
        /* 1/8p note in 16th */
        note_multiplier = 3;
    }
    else if (val < ROTARY_CMP_VAL(4, 6))
    {
        /* 1/8 note */
        note_multiplier = 2;
    }
    else
    {
        /* 1/16 note in 16th */
        note_multiplier = 1;
    }

    /* FIXME: Allow reset. */
    reset_phase();
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
    /* PWM driver. Use OC0A (PD6) as output. */
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
    TCCR2B = (1 << CS21) | (1 << CS22);

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

    /* Set initial values. */
    adc_update_all();

    set_wave(adc_get(ADC_MODE));
    set_tempo(adc_get(ADC_TEMPO));
    set_note(adc_get(ADC_NOTE));

    /* Main loop. */
    while (1)
    {
        adc_update_all();

        if (adc_is_changed(ADC_MODE))
            set_wave(adc_get(ADC_MODE));
        if (adc_is_changed(ADC_TEMPO))
            set_tempo(adc_get(ADC_TEMPO));
        if (adc_is_changed(ADC_NOTE))
            set_note(adc_get(ADC_NOTE));
    }
}
