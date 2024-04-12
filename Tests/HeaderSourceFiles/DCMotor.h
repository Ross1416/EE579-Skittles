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
12th - andrewlaw9178 added comments
--------------------------------------------------------------------------------
*/
#include <msp430.h>

#ifndef DCMOTOR_H
#define DCMOTOR_H

/*  There are Drive and Steer:
							  - Motor assocaited with drive allows car to move forward, backward or remain stationary
							  - Motor assocaited with steer allows car to move left, right or straight ahead
*/

//Define PWM information for motor speed
struct PWM {
    int sec;											// Period of PWM signal (num of seconds)
    int ms;												// Period of PWM signal (num of milliseconds)
    int aSec;											// Time within period that PWM signal is HIGH (num of seconds)
    int aMs;											// Time within period that PWM signal is HIGH (num of milliseconds)
    char state;											// Determines if the the PWM signal is HIGH or LOW
};

//Define variable which contains motor information
struct MotorDC {
    char direction; 									// Determines direction of car, 0 = OFF, 1 = FORWARD, 2 = BACK
    int pinA;
    int pinB;
    struct PWM pwm;										// Contains the PWM signal
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
