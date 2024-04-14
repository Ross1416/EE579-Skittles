/*
Filename    : CarReorganised/DCMotor.c
Author      : Samuel Kliskey
Project     : EE579 Skittles Knocking Over Project
Date        : 31/3/24
Description : DC Motor Functionality
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
31-MAR-2024 SARK created to restructure AutonomousCar project
--------------------------------------------------------------------------------
*/
#include "DCMotor.h"

void motorSetup(struct MotorDC *motor)
{
    P1DIR |= motor->pinA + motor->pinB;
}


void motorOutput(struct MotorDC *motor)
{
    if (motor->pwm.state == 0)
    {
        //P1OUT |= motor->pinA + motor->pinB;
        P1OUT &= ~(motor->pinA + motor->pinB);
    }
    else
    {
        switch(motor->direction)
        {
        case 0:
            P1OUT &= ~(motor->pinA + motor->pinB);
            //P1OUT |= motor->pinA + motor->pinB;
            break;
        case 1:
            P1OUT |= motor->pinA;
            P1OUT &= ~motor->pinB;
            break;
        case 2:
            P1OUT |= motor->pinB;
            P1OUT &= ~motor->pinA;
            break;
        }
    }
}


//==============================================================================
// End of File :  CarReorganised/DCMotor.h
