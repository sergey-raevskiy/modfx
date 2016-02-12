#pragma once
#include <stdint.h>

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

void wave_set(waveform_t w);
uint8_t wave_func(uint8_t i);
