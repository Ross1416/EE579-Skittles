/*
Filename    : Testing/Button.h
Author      : Andrew Law
Project     : EE579 Skittles Knocking Over Project
Date        : 17/04/24
Description : Header file for Button functionality
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
09-APR-2024 andrewlaw91788 created to add modularity to buttons
17-APR-2024 andrewlaw91788 updated with comments
--------------------------------------------------------------------------------
*/

// External libraries
#include <msp430.h>

#ifndef BUTTON_H
#define BUTTON_H

//Define variable containing button information
struct Button{
	char port;			// Port of button
	char pin;			// Pin of button
};

extern void setupBlackButton(struct Button *butt)
extern void setupWhiteButton(struct Button *butt)
extern void setupStartStop(struct Button *butt)
extern void setupModeSwitch(struct Button *butt)

#endif // BUTTON_H