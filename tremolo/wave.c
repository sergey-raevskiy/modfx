#include "wave.h"

/* Current waveform buffer. */
uint8_t wf_wf[256];

void wf_set_rampup()
{
    for (uint16_t i = 0; i < 256; i++)
        wf_wf[i] = i;
}

void wf_set_rampdown()
{
    for (uint16_t i = 0; i < 256; i++)
        wf_wf[i] = 0xFF - i;
}

void wf_set_square()
{
    for (uint16_t i = 0; i < 128; i++)
        wf_wf[i] = 0;
    for (uint16_t i = 128; i < 256; i++)
        wf_wf[i] = 0xFF;
}

void wf_set_triangle()
{
    for (uint16_t i = 0; i < 128; i++)
        wf_wf[i] = 255 - (i * 2);
    for (uint16_t i = 0; i < 128; i++)
        wf_wf[i + 128] = i * 2;
}
