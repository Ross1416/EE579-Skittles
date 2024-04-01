/*
Filename    : CarReorganised/Servo.c
Author      : Samuel Kliskey
Project     : EE579 Skittles Knocking Over Project
Date        : 31/3/24
Description : Servo Functionality
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
31-MAR-2024 SARK created to restructure AutonomousCar project
--------------------------------------------------------------------------------
*/
#include "Servo.h"

void servoSetup(struct Servo *servoMotor)
{
    //Servo Timer settings
    TA1CCR0 = 655; //20 ms at ACLK (Max count value)
    TA1CCR2 = 49;  //1.5 ms

    //Output B (P2.4), Reset/Set output
    TA1CCTL2 &= ~(CCIE + CCIFG + CAP);      //Clear/disable Timer1.2 interrupt and set compare mode
    TA1CCTL2 |= CCIS_0 + OUTMOD_7;

    //Servo Trigger pin settings
    P2DIR |= servoMotor->pwmPin; //Bit 4
    P2SEL |= servoMotor->pwmPin;
}

void servoTurn(struct Servo *servoMotor)
{
    if (TA1CCR2 >= 80)
    {
        servoMotor->direction = -1;
        P2OUT &= ~RGB_WHITE;
        P2OUT |= RGB_BLUE;
    }
    else if (TA1CCR2 <= 35)
    {
        servoMotor->direction = 1;
        P2OUT &= ~RGB_WHITE;
        P2OUT |= RGB_GREEN;
    }
    TA1CCR2 += servoMotor->direction;
}
//==============================================================================
// End of File :  CarReorganised/Servo.h
