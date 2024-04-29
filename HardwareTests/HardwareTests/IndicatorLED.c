/*
Filename    : HardwareTests/IndicatorLED.c
Author      : Samuel Kliskey
Project     : EE579 Skittles Knocking Over Project
Date        : 31/3/24
Description : Indicator LED Functionality
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
04-APR-2024 SARK created to restructure AutonomousCar project
17-APR-2024 andrewlaw9178 added comments and fixed the toggle function
18-APR-2024 andrewlaw9178 added code such the LED is off on initalisation
--------------------------------------------------------------------------------
*/

// Local libraries
#include "IndicatorLED.h"

// Set up indicator LED
void indicatorLEDSetup(struct IndicateLED *LED)
{
    if (LED->port == 1)
    {
        P1DIR |= LED->pin;						// Set P1.x as an output
        P1SEL &=~ LED->pin;						// Selects I/O functionality with P1SEL2
        P1SEL2 &=~ LED->pin;					// Selects I/O functionality with P1SEL
		P1OUT &=~ LED->pin;						// Sets P1.x as LOW to turn off indictor LED
    }
    else if (LED->port == 2)
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
    if (LED->port == 1)
    {
        P1OUT |= LED->pin;						// Set P1.x as HIGH
    }
    else if (LED->port == 2)
    {
        P2OUT |= LED->pin;						// Set P2.x as HIGH
    }
}

// Turn indicator LED off
void indicatorLEDOff(struct IndicateLED *LED)
{
    if (LED->port == 1)
    {
        P1OUT &= ~LED->pin;						// Set P1.x as LOW
    }
    else if (LED->port == 2)
    {
        P2OUT &= ~LED->pin;						// Set P2.x as LOW
    }
}

// Toggle indicator LED 
void indicatorLEDToggle(struct IndicateLED *LED)
{
    if (LED->port == 1)
    {
        P1OUT ^= LED->pin;						// Toggle P1.x
    }
    else if (LED->port == 2)
    {
        P2OUT ^= LED->pin;						// Toggle P2.x
    }
}

//==============================================================================
// End of File :  HardwareTest/IndicatorLED.h
