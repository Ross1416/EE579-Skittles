/*
Filename    : CarReorganised/Ultrasonic.c
Author      : Samuel Kliskey
Project     : EE579 Skittles Knocking Over Project
Date        : 31/3/24
Description : Ultrasonic Functionality
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
31-MAR-2024 SARK created to restructure AutonomousCar project
--------------------------------------------------------------------------------
*/
#include "Ultrasonic.h"

void ultrasonicSetup(struct Ultrasonic *ultra)
{

    //Echo pin
    P2SEL |= ultra->echoPin;

    //Trig pin
    P2DIR |= ultra->trigPin;
    P2OUT &= ~ultra->trigPin;

    //Timer A1 settings
    //Capture rising, input B (P2.2), capture mode, enable interrupt, sync to clock rising edge
    TA1CCTL1 |= CM_1 + CCIS_1 + CAP + CCIE + SCS;
    TA1CCTL1 &= ~CCIFG;     //Clear ultrasonic interrupt

    //Timer Setup
    if(CLOCK_USED_ULTRASONIC == SMCK_FREQ)
    {
        TA1CTL |= TASSEL_2;
    }
    else
    {
        TA1CTL |= TASSEL_1;
    }

    TA1CTL &= ~TAIFG;
    TA1CTL &= ~TAIE;

    TA1CCTL0 &= ~(CCIFG+CCIE);
    TA1CCTL2 &= ~CCIE;
    TA1CCTL2 &= ~CCIFG;

    TA1CCTL1 |= CM_1 + CCIS_1 + CAP + CCIE + SCS;
    TA1CCTL1 &= ~(CCIFG+SCCI);

    TA1CTL |= MC_2;
}

void ultrasonicTrigger(struct Ultrasonic *ultra)
{
    P2OUT |= ultra->trigPin;
    __delay_cycles(20);
    P2OUT &= ~ultra->trigPin;
}

//==============================================================================
// End of File :  CarReorganised/Ultrasonic.h
