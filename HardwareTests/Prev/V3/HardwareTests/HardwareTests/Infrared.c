/*
Filename    : HardwareTest/Infrared.c
Author      : Andrew Law
Project     : EE579 Skittles Knocking Over Project
Date        : 02/02/24
Description : Infrared Sensor Functionality
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
02-FEB-2024 andrewlaw9178 created to implement functionality of Infrared code
12-FEB-2024 andrewlaw9178 created to read digital output through ADC
03-MAR-2024 andrewlaw9178 created to fix issues to allow use of digital output
01-MAR-2024 andrewlaw9178 created to provide structure to the infrared code
03-APR-2024 andrewlaw9178 created to fix error in Infrared code
09-APR-2024 Ross Inglis updated code to allow different ports selected, renamed
            functions and removed debugging LEDs.
12-APR-2024 andrewlaw9178 updated logic that was wrong way round
18-APR-2024 andrewlaw9178 changed logic to prev version as 12-APR update was 
			now wrong 
--------------------------------------------------------------------------------
*/

// Local libraries
#include "Infrared.h"

// Setup the IR sensor
void setupIR(struct Infrared *ir)
{
    if (ir->port == 1)
    {
        P1DIR &= ~ir->pin;                          // Set P1.x as an input pin
    }
    else if (ir->port == 2)
    {
        P2DIR &= ~ir->pin;                          // Set P2.x as an input pin
    }
}

// Read the output of the IR sensor
void readIR(struct Infrared *ir)
{
    if (ir->port == 1)
    {
        if (P1IN & (ir->pin))                       // Black skittle or environment detected
        {
            ir->colour = 0;                         // 0 for black skittle
        }
        else                                        // White skittle detected
        {
            ir->colour = 1;                         // 1 for white skittle
        }
    }
    else if (ir->port == 2)
    {
        if (P2IN & (ir->pin))                       // Black skittle or environment detected
        {
            ir->colour = 0;                         // 0 for black skittle or environment
        }
        else                                        // White skittle detected
        {
            ir->colour = 1;                         // 1 for white skittle
        }
    }
}
//==============================================================================
// End of File :  HardwareTest/Infrared.c
