/*
Filename    : CarReorganised/IndicatorLED.c
Author      : Samuel Kliskey
Project     : EE579 Skittles Knocking Over Project
Date        : 31/3/24
Description : Indicator LED Functionality
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
04-APR-2024 SARK created to restructure AutonomousCar project
--------------------------------------------------------------------------------
*/
#include "IndicatorLED.h"

void indicatorLEDSetup(struct IndicateLED *LED)
{
    if (LED->port == 1)
    {
        P1DIR |= LED->pin;
        P1SEL &=~ LED->pin;
        P1SEL2 &=~ LED->pin;
    }
    else if (LED->port == 2)
    {
        P2DIR |= LED->pin;
        P2SEL &=~ LED->pin;
        P2SEL2 &=~ LED->pin;
    }
}

void indicatorLEDOn(struct IndicateLED *LED)
{
    if (LED->port == 1)
    {
        P1OUT |= LED->pin;
    }
    else if (LED->port == 2)
    {
        P2OUT |= LED->pin;
    }
}

void indicatorLEDOff(struct IndicateLED *LED)
{
    if (LED->port == 1)
    {
        P1OUT &= ~LED->pin;
    }
    else if (LED->port == 2)
    {
        P2OUT &= ~LED->pin;
    }
}

void indicatorLEDToggle(struct IndicateLED *LED)
{
    if (LED->port == 1)
    {
        P1OUT &= ~LED->pin;
    }
    else if (LED->port == 2)
    {
        P2OUT &= ~LED->pin;
    }
}




//==============================================================================
// End of File :  CarReorganised/IndicatorLED.h
