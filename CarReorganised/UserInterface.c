/*
Filename    : CarReorganised/UserInterface.c
Author      : Samuel Kliskey
Project     : EE579 Skittles Knocking Over Project
Date        : 31/3/24
Description : User Interface Functionality
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
04-APR-2024 SARK created to restructure AutonomousCar project
17-APR-2024 SARK altered for buttons and switches with AL code
--------------------------------------------------------------------------------
*/
#include "UserInterface.h"


void setupButton(struct Button *button)
{
    if(button->port == 1)
    {
        // Setup button for input and interrupt
        P1DIR &= ~button->pin;                                        // Sets P1.x as an input

        // Pull up so when pressed will go high to low
        P1OUT |= button->pin;                                         // Selects pull up resistor on P1.x
        P1REN |= button->pin;                                         // Enables pull up resistor on P1.x
        P1IE |= button->pin;                                          // Enable interrupt on P1.x
        P1IES |= button->pin;                                         // High to Low transition
        P1IFG &= ~button->pin;                                        // Clear interrupt on P1.x
    }
    else if(button->port == 2)
    {
        //  Setup button for input and interrupt (P2.7)
        P2DIR &= ~button->pin;                                        // Sets P2.x as an input

        // Pull up so when pressed will go high to low
        P2OUT |= button->pin;                                         // Selects pull up resistor on P2.x
        P2REN |= button->pin;                                         // Enables pull up resistor on P2.x
        P2IE |= button->pin;                                          // Enable interrupt on P2.x
        P2IES |= button->pin;                                         // High to Low transition
        P2IFG &= ~button->pin;                                        // Clear interrupt on P2.x
    }
}

void setupSwitch(struct Switch *sw)
{
    if(sw->port == 1)
    {
        P1DIR &= ~sw->pin;                                       // Sets P1.x as input
        P1OUT |= sw->pin;                                       // Selects pull up resistor on P1.x
        P1REN |= sw->pin;                                       // Enables pull up resistor on P1.x
    }
    else if(sw->port == 2)
    {
        P2DIR &= ~sw->pin;                                       // Sets P2.x as input
        P2OUT |= sw->pin;                                       // Selects pull up resistor on P2.x
        P2REN |= sw->pin;                                       // Enables pull up resistor on P2.x
    }
}

void readButton(struct Button *button)
{
    if (button->port == 1)
       {
         if (P1IN & (button->pin))                         // Black skittle or environment detected
         {
             button->val = 1;                         // 1 for black
         }
         else                                          // White skittle detected
         {
             button->val = 0;                         // 0 for white
         }
       }
       else if (button->port == 2)
       {
           if (P2IN & (button->pin))                         // Black skittle or environment detected
           {
               button->val = 1;                         // 1 for black
           }
           else                                          // White skittle detected
           {
               button->val = 0;                         // 0 for white
           }
       }
}

void readSwitch(struct Switch *sw)
{
    if (sw->port == 1)
   {
     if ((P1IN & (sw->pin)) == sw->pin)
     {
         sw->val = 1;                         // 1 for Right
     }
     else
     {
         sw->val = 0;                         // 0 for Left
     }
   }
   else if (sw->port == 2)
   {
       if (P2IN & (sw->pin))                         // Black skittle or environment detected
       {
           sw->val = 1;                         // 1 for black
       }
       else                                          // White skittle detected
       {
           sw->val = 0;                         // 0 for white
       }
   }
}


void indicatorLEDSetup(struct IndicateLED *LED)
{
    if (LED->port == 1)
    {
        P1DIR |= LED->pin;
        P1SEL &=~ LED->pin;
        P1SEL2 &=~ LED->pin;
    }
    else if (LED->port == 2)
    {
        P2DIR |= LED->pin;
        P2SEL &=~ LED->pin;
        P2SEL2 &=~ LED->pin;
    }
}

void indicatorLEDOn(struct IndicateLED *LED)
{
    if (LED->port == 1)
    {
        P1OUT |= LED->pin;
    }
    else if (LED->port == 2)
    {
        P2OUT |= LED->pin;
    }
}

void indicatorLEDOff(struct IndicateLED *LED)
{
    if (LED->port == 1)
    {
        P1OUT &= ~LED->pin;
    }
    else if (LED->port == 2)
    {
        P2OUT &= ~LED->pin;
    }
}

void indicatorLEDToggle(struct IndicateLED *LED)
{
    if (LED->port == 1)
    {
        P1OUT &= ~LED->pin;
    }
    else if (LED->port == 2)
    {
        P2OUT &= ~LED->pin;
    }
}

//==============================================================================
// End of File :  CarReorganised/UserInterface.h
