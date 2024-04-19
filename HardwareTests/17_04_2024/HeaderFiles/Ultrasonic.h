/*
Filename    : CarReorganised/Ultrasonic.h
Author      : Samuel Kliskey
Project     : EE579 Skittles Knocking Over Project
Date        : 31/3/24
Description : Header file for Ultrasonic Functionality
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
31-MAR-2024 SARK created to restructure AutonomousCar project
17-APR-2024 andrewlaw9178 added comments
--------------------------------------------------------------------------------
*/
#include <msp430.h>

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

//Define variable containing ultrasonic sensor information
struct Ultrasonic{
    int distance;					// Time between two pulses
    int time[2];					// Time values 
    char timeNumber;				// Which side of CCR1
    char trigPin;					// Trigger an ultrasonic reading
    char echoPin;					// Recieve an ultrasonic readaing
    char port;						// Port it is connected to
};

extern void    ultrasonicSetup(struct Ultrasonic *ultra);
extern void    ultrasonicTrigger(struct Ultrasonic *ultra);

#define SMCK_FREQ      1000000
#define ACLK_FREQ      32768
#define CLOCK_USED_ULTRASONIC   SMCK_FREQ
#define dist2pulse(d)           ((CLOCK_USED_ULTRASONIC/100)*d*2/SOUND_SPEED)     // Converts a distance (cm) to ultrasonic sensor output pulse length

#endif // ULTRASONIC_H
//==============================================================================
// End of File :  CarReorganised/Ultrasonic.h
