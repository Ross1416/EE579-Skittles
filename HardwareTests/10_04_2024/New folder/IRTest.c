/*
InfraredTest.c

When infrared sensor detects a black skittle or environment, turn off indicator LED, and when detects white skittle
turn indicator LED on
*/

// External libraries
#include <msp430.h>

// Local libraries
#include "TestingSetup.h"

// Hardware ports
#define LED_INDICATOR 		BIT5 					// Located at P2.5
#define IR_PIN				BIT3					// Located at P2.3

//  Define variable containing infrared sensor information
struct Infrared{
  char colour;
  int pin;
};

struct Infrared irFront = {2, LED_PIN};

int main(void)
{    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    // Setup device
    setupIR(irFront);								// Selects IR input
    setupLEDIndicator();							// Sets the LED indicator to be used for testing
	
	while (1)
	{
		IRRead();
	    if (ir.colour)
	    {
			LEDIndicatorOn();
	    }
	    else
	    {
			LEDIndicatorOff();
	    }
	}
	return 0;
}

// Setup the IR sensor
void setupIR(struct Infrared *ir)
{
    P2DIR &= ~ir->pin;								// Set P2.3 as an input pin
	ir->colour = 2;									// Initialise the colour as 2: 0 = white, 1 = black, 2 = init
}

// Read the output of the IR sensor
void readIR(struct Infrared *ir)
{
	if (P1IN & (ir->pin))                         	// Black skittle or environemnt detected
    {
		ir->colour = 1;                        	 	// 1 for black skittle
	}
    else                                          	// White skittle detected
    {
		ir->colour = 0;                         	// 0 for white skittle
	}
}

// Setup the LED Indicator
void setupLEDIndicator ()
{
	P2DIR |= LED_INDICATOR; 						// Set P2.5 as an output
	P2OUT &= ~LED_INDICATOR;						// Turn off indicator LED when initialised
}

// Toggle LED Indicator
void toggleLEDIndicator()
{
	P2OUT ^= LED_INDICATOR;							// Toggle P2.5 output
}

// Setup the LED Indicator
void setupLEDIndicator()
{
	P2DIR |= LED_INDICATOR; 						// Set P2.5 as an output
	P2OUT &= ~LED_INDICATOR;						// Turn off indicator LED when initialised
}

// Turn indicator LED on
void LEDIndicatorOn()
{
	P2OUT |= LED_INDICATOR;						// Toggle P2.5 output
}

// Turn indicator LED off
void LEDIndicatorOff()
{
	P2OUT &= ~LED_INDICATOR;						// Toggle P2.5 output
}