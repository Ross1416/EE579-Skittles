/*
Filename    : HardwareTests/IndicatorLED.h
Author      : Samuel Kliskey
Project     : EE579 Skittles Knocking Over Project
Date        : 31/3/24
Description : Header file for Indicator LED Functionality
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
09-APR-2024 SARK created to make Indicator LED modular
17-APR-2024 andrewlaw9178 added comments
--------------------------------------------------------------------------------
*/

// External libraries
#include <msp430.h>

#ifndef INDICATORLED_H
#define INDICATORLED_H

//Define variable which contains motor information
struct IndicateLED {
    char port;				// Port of indicator LED
    int pin;				// Pin of indicator LED
};

extern void indicatorLEDSetup(struct IndicateLED *LED);
extern void indicatorLEDOn(struct IndicateLED *LED);
extern void indicatorLEDOff(struct IndicateLED *LED);
extern void indicatorLEDToggle(struct IndicateLED *LED);

#endif  // INDICATORLED_H
//==============================================================================
// End of File :  HardwareTest/IndicatorLED.h
