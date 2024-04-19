/*
Filename    : HardwareTests/LEDIndicatorTest.c
Author      : Andrew Law
Project     : EE579 Skittles Knocking Over Project
Date        : 9/4/24
Description : Script to test the indicator LED component

			Indicator LED is turned on
			
			indicatorLedSetup() sets up the indicator LED, which is 
			then turned on via indicatorLEDOn().			
--------------------------------------------------------------------------------
Functions Present
--------------------------------------------------------------------------------
indicatorLEDSetup()
indicatorLEDOn()
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
09-APR-2024 andrewlaw9178 created to modularise the indicator LED
xx-APR-2024 andrewwlaw9178 added ...
xx-APR-2024 andrewwlaw9178 added ...
--------------------------------------------------------------------------------
*/

// External libraries
#include <msp430.h>

// Local libraries
#include "IndicatorLED.h"

// Define Hardware ports
#define IND_LED				BIT5					// Located at P2.5

// Indicator LED
struct IndicateLED indicatorLED = {2, IND_LED};

void main()
{
    //Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    // Setup output port
	indicatorLEDSetup(&indicatorLED);               // Set up the Indicator LED
    
	while(1)
    {
		indicatorLEDOn(&indicatorLED);				// Turn on indicator LED
    }
}
