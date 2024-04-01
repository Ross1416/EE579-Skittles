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
    int direction;
};

extern void    servoSetup(struct Servo *servoMotor);
extern void    servoTurn(struct Servo *servoMotor);

//RGB colours (P2OUT |= RGB_XX;)
#define RGB_RED     0x02
#define RGB_GREEN   0x08
#define RGB_BLUE    0x20
#define RGB_PURPLE  0x22
#define RGB_CYAN    0x28
#define RGB_YELLOW  0xA
#define RGB_WHITE   0x2A
//RGB off command: (P2OUT &= ~0x2A;)

#endif // SERVO_H
//==============================================================================
// End of File :  CarReorganised/Servo.h
