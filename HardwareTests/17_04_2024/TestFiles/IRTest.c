/*
Filename    : HardwareTests/IRTest.c
Author      : Andrew Law
Project     : EE579 Skittles Knocking Over Project
Date        : 11/4/24
Description : Script to test the IR sensor component
			
			When infrared sensor detects a black skittle or 
			environment, turn off indicator LED, and when detects
			white skittle turn indicator LED on.
			
			At start, indicator LED and infrared sensor are set up,
			and if the IR sensor detects a white skittle the indicator
			LED will turn on, and if it detects a black skittle or the 
			environment ten the indicator LED will turn off.
--------------------------------------------------------------------------------
Functions Present
--------------------------------------------------------------------------------
setupIR()
indicatorLEDSetup()
indicatorLEDOn()
indicatorLEDOff()
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-APR-2024 andrewlaw9178 created to modularise the IR sensor
12-APR-2024 andrewwlaw9178 added ...
16-APR-2024 andrewwlaw9178 added ...
17-APR-2024 andrewwlaw9178 added ...
--------------------------------------------------------------------------------
*/

/*
InfraredTest.c



At start,

*/

// External libraries
#include <msp430.h>

// Local libraries
#include "Infrared.h"
#include "IndicatorLED.h"

// Define Hardware ports
#define IND_LED				BIT5					// Located at P2.5
#define IR_PIN              BIT3                    // Located at P2.3

// Infrared Sensor
struct Infrared irFront = {2, IR_PIN, 2};

// Indicator LED
struct IndicateLED indicatorLED = {2, IND_LED};

int main(void)
{    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    // Setup device
    setupIR(&irFront);                              // Selects IR input
	indicatorLEDSetup(&indicatorLED);               // Set up the Indicator LED

    while (1)
    {
        readIR(&irFront);
        if (irFront.colour)
        {
            indicatorLEDOn(&indicatorLED);			// Turn on indicator LED
        }
        else
        {
            indicatorLEDOff(&indicatorLED);			// Turn off indicator LED
        }
    }
}
