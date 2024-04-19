/*
Filename    : Testing/Button.c
Author      : Andrew Law
Project     : EE579 Skittles Knocking Over Project
Date        : 09/4/24
Description : Buttons Functionality
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
09-APR-2024 andrewlaw91788 created to add modularity to buttons
17-APR-2024 andrewlaw91788 updated with comments
--------------------------------------------------------------------------------
*/

// Local libraries 
#include "Button.h"

// Sets up the start stop switch
void setupStartStop(struct Button *butt)
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

// Sets up the black switch
void setupBlackButton(struct Button *butt)
{
	if(butt->port == 1)
	{
		// Setup button for input and interrupt (P2.7)
		P1DIR &= ~butt->pin;										// Sets P1.x as an input

		// Pull up so when pressed will go high to low
		P1OUT |= butt->pin;											// Selects pull up resistor on P1.x
		P1REN |= butt->pin;											// Enables pull up resistor on P1.x
		P1IE |= butt->pin;                                          // Enable interrupt on P1.x
		P1IES |= butt->pin;                                         // High to Low transition
		P1IFG &= ~butt->pin;                                        // Clear interrupt on P1.x
	}
	else if(butt->port = 2)
	{
		//  Setup button for input and interrupt (P2.7)
		P2DIR &= ~butt->pin;										// Sets P2.x as an input

		// Pull up so when pressed will go high to low
		P2OUT |= butt->pin;											// Selects pull up resistor on P2.x
		P2REN |= butt->pin;											// Enables pull up resistor on P2.x
		P2IE |= butt->pin;                                          // Enable interrupt on P2.x
		P2IES |= butt->pin;                                         // High to Low transition
		P2IFG &= ~butt->pin;                                        // Clear interrupt on P2.x
	}   
}

// Sets up the white switch
void setupWhiteButton(struct Button *butt)
{
	if(butt->port == 1)
	{
		// Setup button for input and interrupt (P2.7)
		P1DIR &= ~butt->pin;										// Sets P1.x as an input

		// Pull up so when pressed will go high to low
		P1OUT |= butt->pin;											// Selects pull up resistor on P1.x
		P1REN |= butt->pin;											// Enables pull up resistor on P1.x
		P1IE |= butt->pin;                                          // Enable interrupt on P1.x
		P1IES |= butt->pin;                                         // High to Low transition
		P1IFG &= ~butt->pin;                                        // Clear interrupt on P1.x
	}
	else if(butt->port = 2)
	{
		//  Setup button for input and interrupt (P2.7)
		P2DIR &= ~butt->pin;										// Sets P2.x as an input

		// Pull up so when pressed will go high to low
		P2OUT |= butt->pin;											// Selects pull up resistor on P2.x
		P2REN |= butt->pin;											// Enables pull up resistor on P2.x
		P2IE |= butt->pin;                                          // Enable interrupt on P2.x
		P2IES |= butt->pin;                                         // High to Low transition
		P2IFG &= ~butt->pin;                                        // Clear interrupt on P2.x
	}   
}

// Setup the switch for mode - slide_select 
void setupModeSwitch(struct Button *butt)
{
	if(butt->port == 1)
	{
		P1DIR |= MODE_SWITCH;       								// Sets P1.x as input
		P1OUT |= MODE_SWITCH;      									// Selects pull down resistor on P1.x
		P1REN |= MODE_SWITCH;       								// Enables pull down resistor on P1.x
	}
	else if(butt->port == 2)
	{
		P2DIR |= MODE_SWITCH;       								// Sets P2.x as input
		P2OUT |= MODE_SWITCH;      									// Selects pull down resistor on P2.x
		P2REN |= MODE_SWITCH;       								// Enables pull down resistor on P2.x
	}
}
