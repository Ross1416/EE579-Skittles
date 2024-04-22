/*
Filename    : CarReorganised/UserInterface.h
Author      : Samuel Kliskey
Project     : EE579 Skittles Knocking Over Project
Date        : 31/3/24
Description : Header file for User Interface Functionality
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
09-APR-2024 SARK created to make Indicator LED modular
17-APR-2024 SARK altered for buttons and switches with AL code
--------------------------------------------------------------------------------
*/
#include <msp430.h>

#ifndef USERINTERFACE_H
#define USERINTERFACE_H

//Define variable which contains motor information
struct IndicateLED {
    char port;
    int pin;
};

//Define variable containing button information
struct Button{
    char port;          // Port of button
    char pin;           // Pin of button
    char val;           //Value read
};

//Define variable containing button information
struct Switch{
    char port;          // Port of button
    char pin;           // Pin of button
    char val;           //Value read
};

extern void setupButton(struct Button *button);
extern void setupSwitch(struct Switch *sw);
extern void readButton(struct Button *button);
extern void readSwitch(struct Switch *sw);
extern void indicatorLEDSetup(struct IndicateLED *LED);
extern void indicatorLEDOn(struct IndicateLED *LED);
extern void indicatorLEDOff(struct IndicateLED *LED);
extern void indicatorLEDToggle(struct IndicateLED *LED);

#endif  // USERINTERFACE_H
//==============================================================================
// End of File :  CarReorganised/UserInterface.h
