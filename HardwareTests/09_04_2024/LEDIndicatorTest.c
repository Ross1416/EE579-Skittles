#include <msp430.h>

#include "TestingSetup.h"

void main()
{
	//Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

	// Setup output port
	setupLEDIndicator();

	P2OUT |= LED_PIN;			// Turn on the indicator LED connected to P2.5
}