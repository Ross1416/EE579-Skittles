// External libraries
#include <msp430.h>

#ifndef LEDINDICATOR_H
#define LEDINDICATOR_H

// Define Hardware ports
#define LED_INDICATOR BIT5 			// Located at P2.5

// Setup the LED Indicator
void setupLEDIndicator()
{
    P2DIR |= LED_INDICATOR;         // Set P2.5 as an output
    P2OUT &= ~LED_INDICATOR;        // Turn off indicator LED when initialised
}

// Toggle LED Indicator
void toggleLEDIndicator()
{
    P2OUT ^= LED_INDICATOR;         // Toggle P2.5 output
}

// Turn on the LED Indicator
void LEDIndicatorOn()
{
    P2OUT |= LED_INDICATOR;
}

// Turn off the LED Indicator
void LEDIndicatorOff()
{
    P2OUT &= ~LED_INDICATOR;
}

#endif // LEDINDICATOR_H
