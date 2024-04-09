/*
Filename    : CarReorganised/Infrared.h
Author      : Andrew Law
Project     : EE579 Skittles Knocking Over Project
Date        : 01/04/24
Description : Header file for Infrared Sensor Functionality
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
01-APR-2024 andrewlaw9178 created to have a structured system for Infrared code
02-APR-2024 andrewlaw9178 created to fix and refine header file of Infrared Sensor
03-APR-2024 andrewlaw9178 created to fix  error in header file of Infrared code
09-APR-2024 Ross Inglis updated with renamed functions
--------------------------------------------------------------------------------
*/
#ifndef INFRARED_H
#define INFRARED_H

//  Define variable containing infrared sensor information
struct Infrared{
  char colour;
  char pin;
  char port;
};

extern void IRSetup(struct Infrared *ir);
extern void readIR(struct Infrared *ir);

#endif // INFRARED_H
//==============================================================================
// End of File :  CarReorganised/Infrared.h
