#pragma once
#include <stdint.h>

/* Waveform selectors. */
void wf_set_rampup();
void wf_set_rampdown();
void wf_set_square();
void wf_set_triangle();
void wf_set_sine();

/* Wave function. */
static uint8_t wave_func(uint8_t phase)
{
    extern uint8_t wf_wf[256];
    return wf_wf[phase];
}
