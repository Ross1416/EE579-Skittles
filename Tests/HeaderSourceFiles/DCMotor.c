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
12th - andrewlaw9178 added comments
--------------------------------------------------------------------------------
*/
#include "DCMotor.h"

void motorSetup(struct MotorDC *motor)
{
    P1DIR |= motor->pinA + motor->pinB;				// Sets pin to output 
}


void motorOutput(struct MotorDC *motor)
{
    if (motor->pwm.state == 0)						 
    {
        P1OUT &= ~(motor->pinA + motor->pinB);		
													// Drive - Turn off motor so stay stationary
													// Steer - Turn off motor to keep wheels straight
    }
    else
    {
        switch(motor->direction)
        {
        case 0:
            P1OUT &= ~(motor->pinA + motor->pinB);	
													// Drive - Turn off motor to stay stationary
													// Steer - Turn off motor to keep wheels straight
            break;
        case 1:
            P1OUT |= motor->pinA;					
            P1OUT &= ~motor->pinB;					
													// Drive - Turn on motor to go forwards
													// Steer - Turn on motor to go right
            break;
        case 2:
            P1OUT |= motor->pinB;					
            P1OUT &= ~motor->pinA;					
													// Drive - Turn on motor to go backwards
													// Steer - Turn on motor to go left
            break;
        }
    }
}

//==============================================================================
// End of File :  CarReorganised/DCMotor.h
