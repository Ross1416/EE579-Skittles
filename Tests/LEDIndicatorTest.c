/*
LEDIndicatorTest.c

LED Indicator is set high
*/

// External libraries
#include <msp430.h>

/*// Local libraries
#include "TestingSetup.h"
*/

// Hardware ports
#define LED_INDICATOR 		BIT5 										// Located at P2.5

void main()
{
	//Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

	// Setup output port
    setupLEDIndicator();
	
	while(1)
	{
		P2OUT |= LED_INDICATOR;												// Turn on the indicator LED connected to P2.5
	}
}

// Setup the LED Indicator
void setupLEDIndicator()
{
	P2DIR |= LED_INDICATOR; 											// Set P2.5 as an output
	P2OUT &= ~LED_INDICATOR;											// Turn off indicator LED when initialised
}