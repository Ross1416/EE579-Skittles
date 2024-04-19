/*
InfraredTest.c

When infrared sensor detects a black skittle or environment, turn off indicator LED, and when detects white skittle
turn indicator LED on
*/

// External libraries
#include <msp430.h>

// Local libraries
#include "LEDIndicator.h"
#include "Infrared.h"

// Define Hardware ports
#define IR_PIN              BIT3                    // Located at P2.3
#define IR_PORT             BIT2                    // Located at P2

struct Infrared irFront = {2, IR_PIN, IR_PORT};

int main(void)
{    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    // Setup device
    setupIR(&irFront);                               // Selects IR input
    setupLEDIndicator();                            // Sets the LED indicator to be used for testing

    while (1)
    {
        readIR(&irFront);
        if (irFront.colour)
        {
            LEDIndicatorOn();
        }
        else
        {
            LEDIndicatorOff();
        }
    }
}
