#include <avr/io.h>
#include <avr/interrupt.h>

/*
           _____   ___ ___
           RESET -|   U   |- VCC
     (Mode) ADC3 -| AT13A |- ADC1 (Depth)
    (Speed) ADC2 -|       |- OC0B (PWM Output)
             GND -|_______|- PB0  (Button)
*/

enum {
    ADC_DEPTH = 1,
    ADC_SPEED = 2,
    ADC_MODE = 3,
};

int main()
{
    while(1)
    {
        cli();
    }
}