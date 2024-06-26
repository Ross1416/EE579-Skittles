/*
Filename    : HardwareTests/UserInterface.c
Author      : Andrew Law
Project     : EE579 Skittles Knocking Over Project
Date        : 09/4/24
Description : Buttons Functionality
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
01-MAY-2024 andrewlaw9178 edited Button.c and IndicatorLED.c and combined them
            such that this file is functionally the same as CarReorganised.
02-MAY-2024 andrewlaw9178 changed comments on switch read.
--------------------------------------------------------------------------------
*/

// Local libraries 
#include "UserInterface.h"

// Sets up the start stop switch
void setupButton(struct Button *butt)
{
	if(butt->port == 1)
	{
		// Setup button for input and interrupt
		P1DIR &= ~butt->pin;										// Sets P1.x as an input

		// Pull up so when pressed will go high to low
		P1OUT |= butt->pin;											// Selects pull up resistor on P1.x
		P1REN |= butt->pin;											// Enables pull up resistor on P1.x 
		P1IE |= butt->pin;                                    		// Enable interrupt for P1.x
		P1IES |= butt->pin;                                     	// High to Low transition
		P1IFG &= ~butt->pin;                                    	// Clear interrupt for P1.x
	}
	else if(butt->port == 2)
	{
		// Setup button for input and interrupt
		P2DIR &= ~butt->pin;										// Sets P1.x as an input

		// Pull up so when pressed will go high to low
		P2OUT |= butt->pin;											// Selects pull up resistor on P2.x
		P2REN |= butt->pin;											// Enables pull up resistor on P2.x
		P2IE |= butt->pin;                                    		// Enable interrupt for P2.x
		P2IES |= butt->pin;                                     	// High to Low transition
		P2IFG &= ~butt->pin;                                    	// Clear interrupt for P2.x
	}
}

// Setup the switch
void setupSwitch(struct Switch *sw)
{
	if(sw->port == 1)
	{
		P1DIR &= ~sw->pin;       								// Sets P1.x as input
		P1OUT |= sw->pin;      									// Selects pull down resistor on P1.x
		P1REN |= sw->pin;       								// Enables pull down resistor on P1.x
	}
	else if(sw->port == 2)
	{
		P2DIR &= ~sw->pin;       								// Sets P2.x as input
		P2OUT |= sw->pin;      									// Selects pull down resistor on P2.x
		P2REN |= sw->pin;       								// Enables pull down resistor on P2.x
	}
}

// Read value of button
void readButton(struct Button *butt)
{
	if(butt->port == 1)
	{
		if(P1IN & (butt->pin))                         		    // Start button hasn't been pressed
        {
			butt->val = 0;                         			    // Start button is 0 (not pressed)
        }
        else
        {
			butt->val = 1;                         			    // Start button is 1 (pressed)
        }
	}
    else if(butt->port == 2)
    {
		if(P2IN & (butt->pin))                         		    // Start button hasn't been pressed
        {
			butt->val = 1;                         			    // Start button is 0 (not pressed)
        }
        else
        {
            butt->val = 0;                         			    // Start button is 1 (pressed)
        }
	}
}

// Read value of switch
void readSwitch(struct Switch *sw)
{
	if(sw->port == 1)
	{
		if((P1IN & (sw->pin)) == sw->pin)
		{
			sw->val = 1;                         				// 1 for Left
		}
		else
		{
			sw->val = 0;                         				// 0 for Right
		}
	}
	else if(sw->port == 2)
	{
		if(P2IN & (sw->pin))                         
		{
			sw->val = 1;                         				// 1 for Left
		}
		else                                          
		{
			sw->val = 0;                         				// 0 for Right
		}
	}
}

// Set up indicator LED
void indicatorLEDSetup(struct IndicateLED *LED)
{
    if(LED->port == 1)
    {
        P1DIR |= LED->pin;						// Set P1.x as an output
        P1SEL &=~ LED->pin;						// Selects I/O functionality with P1SEL2
        P1SEL2 &=~ LED->pin;					// Selects I/O functionality with P1SEL
		P1OUT &=~ LED->pin;						// Sets P1.x as LOW to turn off indictor LED
    }
    else if(LED->port == 2)
    {
        P2DIR |= LED->pin;						// Set P2.x as an output
        P2SEL &=~ LED->pin;						// Selects I/O functionality with P1SEL2
        P2SEL2 &=~ LED->pin;					// Selects I/O functionality with P1SEL
		P2OUT &=~ LED->pin;						// Sets P2.x as LOW to turn off indictor LED
    }
}

// Turn indicator LED on
void indicatorLEDOn(struct IndicateLED *LED)
{
    if(LED->port == 1)
    {
        P1OUT |= LED->pin;						// Set P1.x as HIGH
    }
    else if(LED->port == 2)
    {
        P2OUT |= LED->pin;						// Set P2.x as HIGH
    }
}

// Turn indicator LED off
void indicatorLEDOff(struct IndicateLED *LED)
{
    if(LED->port == 1)
    {
        P1OUT &= ~LED->pin;						// Set P1.x as LOW
    }
    else if(LED->port == 2)
    {
        P2OUT &= ~LED->pin;						// Set P2.x as LOW
    }
}

// Toggle indicator LED 
void indicatorLEDToggle(struct IndicateLED *LED)
{
    if(LED->port == 1)
    {
        P1OUT ^= LED->pin;						// Toggle P1.x
    }
    else if(LED->port == 2)
    {
        P2OUT ^= LED->pin;						// Toggle P2.x
    }
}


//==============================================================================
// End of File :  HardwareTests/UserInterface.c
