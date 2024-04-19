/*
Filename    : CarReorganised/Servo.h
Author      : Samuel Kliskey
Project     : EE579 Skittles Knocking Over Project
Date        : 31/3/24
Description : Header file for Servo Functionality
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
31-MAR-2024 SARK created to restructure AutonomousCar project
--------------------------------------------------------------------------------
*/
#include <msp430.h>

#ifndef SERVO_H
#define SERVO_H

//Define variable containing servo motor information
struct Servo{
    char pwmPin;
    unsigned int speed;
    char direction;
};

extern void    servoSetup(struct Servo *servoMotor);
extern void    servoTurn(struct Servo *servoMotor);
extern void    servoCenter();

//SERVO PWM LIMITS
#define PWM_SERVO_UPPER   2400				// Left most
#define PWM_SERVO_LOWER   450				// Right most angle


#endif // SERVO_H
//==============================================================================
// End of File :  CarReorganised/Servo.h