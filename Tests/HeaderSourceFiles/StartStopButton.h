// External libraries
#include <msp430.h>

#ifndef STOPSTARTBUTTON_H
#define STOPSTARTBUTTON_H

// Hardware ports
#define START_STOP_BUTTON   BIT3                        // Located at P1.3

// Sets up the start stop switch
void setupStartStop()
{
    //Setup button for input and interrupt (P1.3)
    P1DIR &= ~START_STOP_BUTTON;

    //Pull up so when pressed will go high to low
    P1REN |= START_STOP_BUTTON;
    P1OUT |= START_STOP_BUTTON;
    P1IE |= START_STOP_BUTTON;                          // Enable interrupts
    P1IES |= START_STOP_BUTTON;                         // High to Low transition
    P1IFG &= ~START_STOP_BUTTON;                        // Clear interrupts
}

#endif // STOPSTARTBUTTON_H
