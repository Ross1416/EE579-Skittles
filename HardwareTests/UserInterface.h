// All code is functionally the same to the headers and sources in the car code.
//Combine history of IndicatorLED and Button

/*
Filename    : HardwareTests/UserInterface.h
Author      : Andrew Law
Project     : EE579 Skittles Knocking Over Project
Date        : 17/04/24
Description : Header file for Button functionality
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
01-MAY-2024 andrewlaw9178 edited Button.h and IndicatorLED.h and combined them
            such that this file is functionally the same as CarReorganised.
--------------------------------------------------------------------------------
*/

// External libraries
#include <msp430.h>

#ifndef USERINTERFACE_H
#define USERINTERFACE_H

// Define variable which contains motor information
struct IndicateLED {
    char port;			// Port of indicator LED
    int pin;			// Pin of indicator LED
};

// Define variable containing button information
struct Button{
	char port;			// Port of button
	char pin;			// Pin of button
	char val;           // Value read
};

// Define variable containing switch information
struct Switch{
    char port;          // Port of switch
    char pin;           // Pin of switch
    char val;           // Value read

};

extern void setupButton(struct Button *butt);
extern void setupSwitch(struct Switch *sw);
extern void readButton(struct Button *butt);
extern void readSwitch(struct Switch *sw);
extern void indicatorLEDSetup(struct IndicateLED *LED);
extern void indicatorLEDOn(struct IndicateLED *LED);
extern void indicatorLEDOff(struct IndicateLED *LED);
extern void indicatorLEDToggle(struct IndicateLED *LED);

#endif // USERINTERFACE_H

