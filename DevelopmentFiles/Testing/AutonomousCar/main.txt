/*
Filename    : AutonomousCar/main.c
Author      : Samuel Kliskey
Project     : EE579 Skittles Knocking Over Project
Date        : 20/1/24
Description : Main script to control functions of the autonomous vehicle
              to as quickly as possible knock over black skittles and avoid
              white skittles.

--------------------------------------------------------------------------------
Functions Present
--------------------------------------------------------------------------------
Port1_ISR()
Timer0_A0_ISR()

main(void);
checkFlags();
checkSchedule();
setupPins();
setupScheduleTimer();
timeIncrement();

--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
20-JAN-2024 SARK created general structure
--------------------------------------------------------------------------------
*/

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include <msp430.h>

//==============================================================================
// User-Defined Types
//------------------------------------------------------------------------------
//Define time variable
struct Time {
    int sec;
    int ms;
};

//Define Flags
struct flags {
	char button;
	char debounce;
	char timerA0;
};

//Define Scheduler
struct Scheduler {
	struct Time debounce;
};

//==============================================================================
// Function Prototypes
//------------------------------------------------------------------------------
__interrupt void Port1_ISR(void);
__interrupt void Timer0_A0_ISR(void);

int     main(void);
void    checkFlags();
void    checkSchedule();
void    setupPins();
void    setupScheduleTimer();
void    timeIncrement(struct Time *time, int sec, int ms);

//==============================================================================
// MACRO
//------------------------------------------------------------------------------
#define isTime(X) ((currentTime.sec == X.sec) && (currentTime.ms == X.ms))

#define SMCK_FREQ      1000000
#define ACLK_FREQ      32768

#define CLOCK_USED     SMCK_FREQ
#define LOW_POWER      LPM0
#define SECOND_COUNT   1000
#define TIMER_INC_MS   20

//Frequency Tones
#define LOW_FREQ    74

//RGB colours (P2OUT |= RGB_XX;)
#define RGB_RED     0x02
#define RGB_GREEN   0x08
#define RGB_BLUE    0x20
#define RGB_PURPLE  0x22
#define RGB_CYAN    0x28
#define RGB_YELLOW  0xA
#define RGB_WHITE   0x2A
//RGB off command: (P2OUT &= ~0x2A;)

//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------
struct Time currentTime     =   {0, 0};    //Running count of time
struct Scheduler Schedule   =   {0};       //Schedule when events needing attended
struct flags flag           =   {0};       //Flag when something ready to be attended

//==============================================================================
// Functions
//------------------------------------------------------------------------------

//Flag that button has been pressed and take out of LPM
#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR(void)
{
    flag.button = 1;
    __low_power_mode_off_on_exit();
    P1IFG &= ~BIT3;
    return;
}

//Flag that time has passed, update current time and take out of LPM
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR(void)
{
    currentTime.ms += TIMER_INC_MS;
    if(currentTime.ms >= SECOND_COUNT)    //Increment timer
    {
        currentTime.ms -= SECOND_COUNT;
        if(++currentTime.sec == 60) currentTime.sec = 0;
    }

    flag.timerA0 = 1;

    __low_power_mode_off_on_exit();

    TA0CCTL0 &= ~CCIFG;
}


int main(void)
{
	//Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    //Setup device
    setupPins();
    setupScheduleTimer();

    //Other setup
    Schedule.debounce.sec = 0;
    Schedule.debounce.ms = -1;

    __bis_SR_register(GIE);

    while(1)
    {
        if (flag.timerA0)	//When timer ticks
        {
            checkSchedule();	//Check if time to do anything
            flag.timerA0 = 0;
        }
        else if(flag.debounce || flag.button)	//If something needs attended / or more flags as needed in condition
        {
			checkFlags();		//Deal with what needs attended
        }
        else
        {
            LOW_POWER;
        }
    }
}

void checkSchedule()
{
	if(isTime(Schedule.debounce))  //Debounce button check
    {
        flag.debounce = 1;
    }
}

void checkFlags()
{
	if(flag.debounce)
	{
		if((P1IN & 0x08) != 0x08)   //Button still pressed after debounce
		{
			P2OUT ^= 0x2A;
		}
		Schedule.debounce.sec = 0;
		Schedule.debounce.ms = -1;
		flag.debounce = 0;
	}

	if (flag.button)
    {
        if(Schedule.debounce.sec == 0 && Schedule.debounce.ms == -1)   //If debounce not currently occurring
        {
			timeIncrement(&Schedule.debounce, 0, 20);  //20 ms debounce
        }
        flag.button = 0;
    }
}

void timeIncrement(struct Time *time, int sec, int ms)
{
    //Always on next increment amount
    ms += TIMER_INC_MS;
    ms -= ms % TIMER_INC_MS;

    time->ms = currentTime.ms + ms;
    while(time->ms >= SECOND_COUNT)
    {
        time->ms -= SECOND_COUNT;
        sec++;
    }

    time->sec = currentTime.sec + sec;
    while(time->sec >= 60)
    {
        time->sec -= 60;
    }
}

void setupPins()
{
    //LEDs
    P1DIR |= BIT6 + BIT0;  //Red & Driver's LED
    P2DIR |= 0x2A;  //RGB


    //Setup button for input and interrupt (P1.3)
    P1DIR &= ~BIT3;
    //Pull up so when pressed will go high to low
    P1REN |= BIT3;
    P1OUT |= BIT3;
    P1IE |= BIT3;    //Enable interrupts
    P1IES |= BIT3;   //High to Low transition
    P1IFG &= ~BIT3;  //Clear interrupts

	//Motor control pins

	//Any sensor pins
    return;
}

void setupScheduleTimer()
{
    if(CLOCK_USED == SMCK_FREQ)
    {
        TA0CTL |= TASSEL_2 + MC_1;              //SMCK  so f = 1 MHz, operating in up mode to TA0CCR0
    }
    else
    {
        TA0CTL |= TASSEL_1 + MC_1;              //ACLK  so f = 32768 Hz, operating in up mode to TA0CCR0
    }
    TA0CCTL0 |= 0x10;                       //Interrupt occurs when TA0R reaches TA0CCR0
    TA0CCR0 = CLOCK_USED*TIMER_INC_MS/1000; //Set the count to value (5 ms) f*5ms = 5000
    TA0CCTL0 &= ~CCIFG;                     //Clear interrupt flags

    return;
}

//==============================================================================
// End of File : AutonomousCar/main.c
