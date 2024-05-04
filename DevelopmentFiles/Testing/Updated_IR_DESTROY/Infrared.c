/*
Filename    : CarReorganised/Infrared.c
Author      : Andrew Law
Project     : EE579 Skittles Knocking Over Project
Date        : 02/2/24
Description : Infrared Sensor Functionality
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
02-FEB-2024 andrewlaw9178 created to implement functionality of Infrared code 
12-FEB-2024 andrewlaw9178 created to read digital output through ADC
03-MAR-2024 andrewlaw9178 created to fix issues to allow use of digital output
08-MAR-2024 andrewlaw9178 created to 
01-MAR-2024 andrewlaw9178 created to provide structure to the infrared code
03-APR-2024 andrewlaw9178 created to fix stupid error in Infrared code
--------------------------------------------------------------------------------
*/
#include <msp430.h>
#include "Infrared.h"
#define GREEN_LED BIT0
#define RED_LED BIT6

void irSensorSetup(struct Infrared *infraSense)
{
    /* Setup Ports */
    P1DIR |= (GREEN_LED + RED_LED);                 // GREEN signifies black and RED signifies white
    P1OUT &= ~(GREEN_LED + RED_LED);                // Turn debugging lights off

    /* Setup Port Interrupt if desired*/
    P2DIR |= 0x00;
    P2OUT |= 0x00;
    P2IES &= ~(infraSense->pin);                        // Low to High - When detect white skittle it triggers response (as need to not hit it)
    P2IFG &= ~(infraSense->pin);                        // Clears interrupt flag
    P2IE |= (infraSense->pin);                          // Interrupt enabled for P1
}

void read_ir_sensor(struct Infrared *ir)
{
  if (P2IN & (ir->pin))                         // Black skittle or environemnt detected
  {
      ir->colour = 'b';                         // 'b' for black
  }
  else                                          // White skittle detected
  {
      ir->colour = 'w';                         // 'w' for white
  }

  if (ir->colour == 'b'){
      P1OUT |= GREEN_LED;                       // Signify black skittle 
      P1OUT &= ~ RED_LED;                       // It is not a white skittle
  }
  else{
      P1OUT &= ~ GREEN_LED;                     // It is not a black skittle 
      P1OUT |= RED_LED;                         // Signify white skittle
  }
}
