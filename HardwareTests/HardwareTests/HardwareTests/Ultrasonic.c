/*
Filename    : HardwareTest/Ultrasonic.c
Author      : Samuel Kliskey
Project     : EE579 Skittles Knocking Over Project
Date        : 31/3/24
Description : Ultrasonic Functionality
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
31-MAR-2024 SARK created to restructure AutonomousCar project
15-APR-2024 andrewlaw9178 set P2.0 to always be the trigger pin
17-APR-2024 andrewlaw9178 added comments,
02-MAY-2024 andrewlaw9178 altered line 29-32 SONAR trig
--------------------------------------------------------------------------------
*/

// Local libraries
#include "Ultrasonic.h"

// Setup the ultrasonic sensor
void ultrasonicSetup(struct Ultrasonic *ultra)
{
    if (ultra->port == 1)                                       // Non-stationary ultrasonic
    {
        // Echo pin
        P1SEL |= ultra->echoPin;                                // Connects input to the timer capture register
        P1DIR &= ~ultra->echoPin;                               // Sets pin as an input (voltage divider)

        // Trig pin
        P1DIR |= ultra->trigPin;                                // Sets pin as an output
        P1OUT &= ~ultra->trigPin;                               // Sets pin as LOW
        //P2DIR |= BIT0;
        //P2OUT &= ~BIT0;

        // Set capture compare to the ultrasonic being used
        if (ultra->echoPin == BIT2)
        {
            TA0CCTL1 |= CCIS_0;                                 // Set capture compare input A to recieve ultrasonic signal
        }
        // Capture rising edge, capture mode, enable interrupt, sync to clock rising edge
        TA0CCTL1 |= CM_1 + CAP + CCIE + SCS;
        TA0CCTL1 &= ~(CCIFG+SCCI+CCI);                          // Clear interrupt, disable synchronised input and capture mode ?????????????
    }

    if (ultra->port == 2)                                       // Stationary ultrasonic
    {
        // Echo pin
        P2SEL |= ultra->echoPin;                                // Connects input to the time capture register

        // Trig pin
        P2DIR |= ultra->trigPin;                                // Sets pin as an output
        P2OUT &= ~ultra->trigPin;                               // Sets pin as LOW

        // Timer A1 settings
        if (ultra->echoPin == BIT2)                             // Left ultrasonic
        {
            TA1CCTL1 &= ~CCIS_3;
            TA1CCTL1 |= CCIS_1;                                 // Set capture compare input B to recieve ultrasonic signal
        }
        else if (ultra->echoPin == BIT1)                        // Right ultrasonic
        {
            TA1CCTL1 &= ~CCIS_3;
            TA1CCTL1 |= CCIS_0;                                 // Set capture compare input A to recieve ultrasonic signal
        }

        // Capture rising edge, capture mode, enable interrupt, sync to clock rising edge
        TA1CCTL1 |= CM_1 + CAP + CCIE + SCS;
        TA1CCTL1 &= ~(CCIFG+SCCI+CCI);  // Clear interrupt, disable synchronised input and capture mode ?????????????????????????????????????
    }
}

// Send a signal to the ultrasonic sensor to get a reading from it

void ultrasonicTrigger(struct Ultrasonic *ultra)
{
    if (ultra->port == 1)                                       // Non-stationary ultrasonic
    {
        //  Set capture compare to the ultrasonic being used
        if (ultra->echoPin == BIT2)
        {
            TA0CCTL1 |= CCIS_0;                                 // Set capture compare input A to recieve ultrasonic signal
        }

        // Trigger ultrasonic to get distance reading
        P1OUT |= ultra->trigPin;                                // Set output pin as HIGH
        __delay_cycles(20);
        P1OUT &= ~ultra->trigPin;                               // Then set it LOW
    }

    if (ultra->port == 2)                                       // Stationary ultrasonic
    {
        //Set capture compare to the ultrasonic being used
        if (ultra->echoPin == BIT2)                             // Left ultrasonic
        {
           TA1CCTL1 &= ~CCIS_3;
           TA1CCTL1 |= CCIS_1;                                  // Set capture compare input B to recieve ultrasonic signal
        }
        else if (ultra->echoPin == BIT1)                        // Right ultrasonic
        {
            TA1CCTL1 &= ~CCIS_3;
            TA1CCTL1 |= CCIS_0;                                 // Set capture compare input A to recieve ultrasonic signal
        }

        // Trigger ultrasonic to get distance reading
        P2OUT |= ultra->trigPin;                                // Set output as HIGH
        __delay_cycles(20);
        P2OUT &= ~ultra->trigPin;                               // Then set to LOW
    }
}

//==============================================================================
// End of File :  HardwareTest/Ultrasonic.c
