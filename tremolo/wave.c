#include "wave.h"

static waveform_t wave;

void wave_set(waveform_t w)
{
    wave = w;
}

uint8_t wave_func(uint8_t i)
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
