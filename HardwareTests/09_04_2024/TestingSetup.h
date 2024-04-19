#include <msp430.h>

#ifndef TESTINGSETUP_H
#define TESTINGSETUP_H

#define START_STOP_SWITCH 	BIT3					// Located at P1.3
#define MODE_SWITCH			BIT1					// Located at P1.1
#define BLACK_BUTTON		BIT7					// Located at P2.7
#define WHITE_BUTTON		BIT6					// Located at P2.6

#define LED_INDICATOR BIT5 		// Located at P2.5

// Debounce wait has finished
    if(flag.debounce)
    {
        if((P1IN & MODE_SWITCH) != MODE_SWITCH)   									//Button still pressed after debounce ---------------------------------- either 0x08 or BIT3 or SWITCH choose!
        {
			startStopButtonPressed = 1;											// Used for other stuff in fulll program main.c
			toggleLEDIndicator();											// Toggle LED inidicator to show button has been pressed
        }
        Schedule.debounce.sec = 0;
        Schedule.debounce.ms = -1;
        flag.debounce = 0;
    }

//==========================================================================================
//LEDIndicatorTest
//==========================================================================================

#define LED_INDICATOR BIT5 		// Located at P2.5

// Setup the LED Indicator
void setupLEDIndicator ()
{
	P2DIR |= LED_INDICATOR; 	// Set P2.5 as an output
}

//==========================================================================================
//StartButtonTest
//==========================================================================================


// Toggle LED Indicator
void toggleLEDIndicator()
{
	P2OUT ^= LED_INDICATOR;		// Toggle P2.5 output
}

// Defines structure for program timing - required for all tests bar LEDIndicatorTest
struct Time {
    int sec;
    int ms;
};

// Defines flags for hardware and software source to alert system to changes or readings  - required for all tests bar LEDIndicatorTest
struct flags {
    // When button pressed
    char button;
	
	// When button debounce finished
    char debounce;

    // When schedule timer ticks
    char timerA0;
};

// Defines structure containing all timings that events occur on  - required for all tests bar LEDIndicatorTest
struct Scheduler {
    // Time when debouncing will finish
    struct Time debounce;
};
void checkSchedule()
{
    int incSec = 0;
    int incMs = 0;

	// Time to check if time to test button debounce
    if(isTime(Schedule.debounce))
    {
		// Carry out check in checkFlags()
        flag.debounce = 1;
		
		//Disable schedule
		Schedule.debounce.sec = 0;
        Schedule.debounce.ms = -1;
    }
}

void checkFlags()
{
	// Debounce wait has finished
    if(flag.debounce)
    {
        if((P1IN & BIT3) != BIT3)   									//Button still pressed after debounce ---------------------------------- either 0x08 or BIT3 choose!
        {
			startStopButtonPressed = 1;											// Used for other stuff in fulll program main.c
			toggleLEDIndicator();											// Toggle LED inidicator to show button has been pressed
        }
        Schedule.debounce.sec = 0;
        Schedule.debounce.ms = -1;
        flag.debounce = 0;
    }
	
	//On button press start debounce
    if (flag.button)
    {
        if(Schedule.debounce.sec == 0 && Schedule.debounce.ms == -1)   	// If debounce not currently occurring
        {
            timeIncrement(&Schedule.debounce, 0, 20);  					// 20 ms debounce check
        }
        flag.button = 0;
    }

void timeIncrement(struct Time *time, int sec, int ms)
{
    //Always on next increment amount
    ms += TIMER_INC_MS;
    ms -= ms % TIMER_INC_MS;

	//Add to current time and if more then max ms add to seconds
    time->ms = currentTime.ms + ms;
    while(time->ms >= SECOND_COUNT)
    {
        time->ms -= SECOND_COUNT;
        sec++;
    }

	//Add to current time and if more then max s wrap back to 0
    time->sec = currentTime.sec + sec;
    while(time->sec >= 60)
    {
        time->sec -= 60;
    }
}

void setupButton()
{
    //Setup button for input and interrupt (P1.3)
    P1DIR &= ~BIT3;
	
    //Pull up so when pressed will go high to low
    P1REN |= BIT3;
    P1OUT |= BIT3;
    P1IE |= BIT3;    												// Enable interrupts
    P1IES |= BIT3;   												// High to Low transition
    P1IFG &= ~BIT3;  												// Clear interrupts
}

void setupTimerSchedule()
{
    if(CLOCK_USED_SCHEDULER == SMCK_FREQ)
    {
        TA0CTL |= TASSEL_2 + MC_1;              					// SMCK  so f = 1 MHz, operating in up mode to TA0CCR0
    }
    else
    {
        TA0CTL |= TASSEL_1 + MC_1;              					// ACLK  so f = 32768 Hz, operating in up mode to TA0CCR0
    }
    TA0CCTL0 |= 0x10;                       						// Interrupt occurs when TA0R reaches TA0CCR0
    TA0CCR0 = CLOCK_USED_SCHEDULER*TIMER_INC_MS/1000; 				// Set the count to schedule time, e.g 1 MHz*5ms = 5000
    TA0CCTL0 &= ~CCIFG;                     						// Clear interrupt flags
	
	TA0CTL &= ~TAIFG;	//Clear interrupt
    TA0CTL &= ~TAIE;	//Disable interrupt on timer edge
}
#endif // TESTINGSETUP_H
