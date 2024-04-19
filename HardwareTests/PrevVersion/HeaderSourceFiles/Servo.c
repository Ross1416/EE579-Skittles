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

// Setup the servo 
void servoSetup(struct Servo *servoMotor)
{
	//Servo Timer settings
    TA1CCR0 = 20000;                        // Max count value before Timer1 is triggered - 20 ms at SMCLK
    TA1CCR2 = 1500;                         // Triggers Timer1 after 1.5 ms

    //Output B (P2.4), Reset/Set output
    TA1CCTL2 &= ~(CCIE + CCIFG + CAP);      // Disable Timer 1.2 intterupt, disable interupt flag, set compare mode
    TA1CCTL2 |= CCIS_0 + OUTMOD_7;          // Set compare input select to CCI1A and set output mode to reset/set

    //Servo Trigger pin settings
    P2DIR |= servoMotor->pwmPin;            // Output PWM signal to Servo - Bit 4
    P2SEL |= servoMotor->pwmPin;            // Connects output to the timer compare register
}

// Turn servo
void servoTurn(struct Servo *servoMotor)
{
    //Rotate
    if (servoMotor->direction == 1)                     // Turn clockwise
    {
        TA1CCR2 += servoMotor->speed;                   // Adjusts limit such that it moves clockwise
    }
    else                                                                // Turn anticlockwise
    {
        TA1CCR2 -= servoMotor->speed;                                   // Adjusts limit such that it moves anticlockwise
    }

    //If breaching limits keep to limits.
    if (TA1CCR2 > PWM_SERVO_UPPER)                                      // If beyond maximum clockwise angle
    {
        TA1CCR2 = PWM_SERVO_UPPER;
    }
    else if (TA1CCR2 < PWM_SERVO_LOWER)                                 // If beyon maximim anticlockwise angle
    {
        TA1CCR2 = PWM_SERVO_LOWER;
    }
}

void servoCenter()
{
    TA1CCR2 = 1500;
}
//==============================================================================
// End of File :  CarReorganised/Servo.h