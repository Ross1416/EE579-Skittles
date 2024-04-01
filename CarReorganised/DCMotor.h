/*
Filename    : CarReorganised/DCMotor.h
Author      : Samuel Kliskey
Project     : EE579 Skittles Knocking Over Project
Date        : 31/3/24
Description : Header file for DC Motor Functionality
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
31-MAR-2024 SARK created to restructure AutonomousCar project
--------------------------------------------------------------------------------
*/
#include <msp430.h>

#ifndef DCMOTOR_H
#define DCMOTOR_H

//Define PWM information for motor speed
struct PWM {
    int sec;
    int ms;
    int aSec;
    int aMs;
    char state;
};

//Define variable which contains motor information
struct MotorDC {
    char direction; //Off (0), Forward (1) or Back(2)
    int pinA;
    int pinB;
    struct PWM pwm;
};

extern void    motorOutput(struct MotorDC *motor);
extern void    motorSetup(struct MotorDC *motor);

//Motor steer commands
#define STRAIGHT    0
#define RIGHT       1
#define LEFT        2

//Motor drive commands
#define OFF         0
#define FORWARD     1
#define BACK        2

#endif // DCMOTOR_H
//==============================================================================
// End of File :  CarReorganised/DCMotor.h
