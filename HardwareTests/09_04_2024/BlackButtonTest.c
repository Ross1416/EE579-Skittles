// External libraries
#include <msp430.h>

// Local libraries
#include "TestingSetup.h"

// Available clock information
#define SMCK_FREQ      1000000
#define ACLK_FREQ      32768

// Scheduler information
#define CLOCK_USED_SCHEDULER    SMCK_FREQ
#define SECOND_COUNT   			1000		//1000 ms = 1s
#define isTime(X) ((currentTime.sec == X.sec) && (currentTime.ms == X.ms))

// Scheduler control
#define TIMER_INC_MS    2           //Scheduler interrupts period (2 ms)

#define LED_INDICATOR 		BIT5 					// Located at P2.5
#define BLACK_BUTTON		BIT7					// Located at P2.7



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

//char startStopButtonPressed = 0;

//Scheduling info
struct Time currentTime     =   {0, 0};    //Running count of time
struct Scheduler Schedule   =   {0};       //Schedule when events needing attended
struct flags flag           =   {0};       //Flag when something ready to be attended

//Flags for after main flags are dealt with and then results are used for stateControl()
char whiteButtonPressed = 0;

//Flag that button has been pressed
#pragma vector=PORT2_VECTOR
__interrupt void Port2_ISR(void)
{
    flag.button = 1;													// Tells program that button has been pressed and debouncing check must occur
    __low_power_mode_off_on_exit();
    P2IFG &= ~(BLACK_BUTTON);													// Assuming that BIT3 is the switch and BIT4 is the 
    return;
}

// Flag that for every 2ms passed update current time
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR(void)
{
        currentTime.ms += TIMER_INC_MS;
        if(currentTime.ms >= SECOND_COUNT)    							// Increment timer
        {
            currentTime.ms -= SECOND_COUNT;
            if(++currentTime.sec == 60) currentTime.sec = 0;
        }
        flag.timerA0 = 1;												// Tells checkFlags to see if program needs to action anything

        __low_power_mode_off_on_exit();
        TA0CCTL0 &= ~CCIFG;
}

int main(void)
{
    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    // Setup device
    setupButton();														// Selects button input and sets up port 1 intterput for button
    setupTimerSchedule();												// Sets up scheduling for Time 0A0 interrupt which produces the 2ms clock cycle which program runs off of

    // Disable schedules
    Schedule.debounce.sec = 0;
    Schedule.debounce.ms = -1;											// Waits until button is pressed

	// Initialize indicator LED for testing
    setupLEDIndicator();												// Sets the LED indicator to be used for testing

	//Enable global interrupts
    __bis_SR_register(GIE);

	//Main loop
    while(1)
    {
        if (flag.timerA0)   											// Every 2ms timer intterupt triggers (producing soft clock)
        {
            checkSchedule();    										// If button has been pressed wait for debouncing time to verify
            flag.timerA0 = 0;
        }
        else
        {
            checkFlags();       										// Check if button has been pressed, and if still pressed after 20ms 
        }
    }
}

void checkFlags()
{
	// Debounce wait has finished
    if(flag.debounce)
    {
        if((P1IN & BLACK_BUTTON) != BLACK_BUTTON)   									//Button still pressed after debounce ---------------------------------- either 0x08 or BIT3 or SWITCH choose!
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
    P2DIR &= ~BLACK_BUTTON;
	
    //Pull up so when pressed will go high to low
    P2REN |= BLACK_BUTTON;
    P2OUT |= BLACK_BUTTON;
    P2IE |= BLACK_BUTTON;    												// Enable interrupts
    P2IES |= BLACK_BUTTON;   												// High to Low transition
    P2IFG &= ~BLACK_BUTTON;  												// Clear interrupts
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
// Setup the LED Indicator
void setupLEDIndicator ()
{
	P2DIR |= LED_INDICATOR; 	// Set P2.5 as an output
}

// Toggle LED Indicator
void toggleLEDIndicator()
{
	P2OUT ^= LED_INDICATOR;		// Toggle P2.5 output
}