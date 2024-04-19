/*
LEDIndicatorTest.c

LED Indicator is set high
*/

// External libraries
#include <msp430.h>

// Local libraries
#include "LEDIndicator.h"

void main()
{
    //Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    // Setup output port
    setupLEDIndicator();

    while(1)
    {
        LEDIndicatorOn();                   // Turn on the indicator LED connected to P2.5
    }
}
