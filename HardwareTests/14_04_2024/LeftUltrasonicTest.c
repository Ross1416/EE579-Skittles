/*
UltrasonicTest.c

When program starts, ultrasonic sensor will be triggered every 0.5 seconds , with the LED Indicator turning on when the
ultrasonic sensor is triggered and when the ultrasonic sensor outputs a result to the msp430.

At start, after 0.5s checkSchedule() rasises a flag, which
schedule triggers a trigger, which timer sets flag, and flag used to read value
*/

// External libraries
#include <msp430.h>

// Local libraries
#include "Ultrasonic.h"
#include "LEDIndicator.h"

// Available clock information
#define SMCK_FREQ      1000000
#define ACLK_FREQ      32768

// Scheduler information
#define CLOCK_USED_SCHEDULER    SMCK_FREQ
#define SECOND_COUNT            1000        //1000 ms = 1s
#define isTime(X) ((currentTime.sec == X.sec) && (currentTime.ms == X.ms))

// Scheduler control
#define TIMER_INC_MS    2           //Scheduler interrupts period (2 ms)

// Important Ultrasonic Info
#define SOUND_SPEED 343
#define CLOCK_USED_ULTRASONIC   SMCK_FREQ

// Hardware ports
#define ULTRA_LEFT_TRIG     BIT2                                        // Located at P2.2
#define ULTRA_LEFT_ECHO     BIT0                                        // Located at P2.0

// Defines structure for program timing - required for all tests bar LEDIndicatorTest
struct Time {
    int sec;
    int ms;
};

// Defines flags for hardware and software source to alert system to changes or readings  - required for all tests bar LEDIndicatorTest
struct flags {
    // When schedule timer ticks
    char timerA0;

    // Read left wall ultrasonic when result available
    char ultraLeftWallRead;
};

// Defines structure containing all timings that events occur on  - required for all tests bar LEDIndicatorTest
struct Scheduler {
    // Time to start a left ultrasonic reading
    struct Time ultraLeftStart;
};

// Scheduling information
struct Time currentTime     =   {0, 0};    //Running count of time
struct Scheduler Schedule   =   {0};       //Schedule when events needing attended
struct flags flag           =   {0};       //Flag when something ready to be attended

// Wall Ultrasonic info (Port 2)
struct Ultrasonic ultraLeft = {0, {0, 0}, 0, ULTRA_LEFT_TRIG, ULTRA_LEFT_ECHO, 2};

// Sets up the Timer0 A0 to create a soft clock of 2ms period
void setupTimerSchedule()
{
    if(CLOCK_USED_SCHEDULER == SMCK_FREQ)
    {
        TA0CTL |= TASSEL_2 + MC_1;                                      // SMCK  so f = 1 MHz, operating in up mode to TA0CCR0
    }
    else
    {
        TA0CTL |= TASSEL_1 + MC_1;                                      // ACLK  so f = 32768 Hz, operating in up mode to TA0CCR0
    }
    TA0CCTL0 |= 0x10;                                                   // Interrupt occurs when TA0R reaches TA0CCR0
    TA0CCR0 = CLOCK_USED_SCHEDULER*TIMER_INC_MS/1000;                   // Set the count to schedule time, e.g 1 MHz*5ms = 5000
    TA0CCTL0 &= ~CCIFG;                                                 // Clear interrupt flags

    TA0CTL &= ~TAIFG;   //Clear interrupt
    TA0CTL &= ~TAIE;    //Disable interrupt on timer edge
}

// Set up Timer1 A1 interrupt
void setupTimerRADAR()
{
    if(CLOCK_USED_ULTRASONIC == SMCK_FREQ)
    {
        TA1CTL |= TASSEL_2;			// f = 1 MHz
    }
    else
    {
        TA1CTL |= TASSEL_1;			// f = 32.768 kHz
    }

    TA1CTL &= ~TAIFG;	//Clear interrupt
    TA1CTL &= ~TAIE;	//Disable interrupt on timer edge

    TA1CCTL0 &= ~(CCIFG+CCIE);
    TA1CCTL2 &= ~CCIE;
    TA1CCTL2 &= ~CCIFG;
	
    TA1CCTL0 |= CCIE;   //Enable interrupt to know when wraps

	//Count to TA1CCR0 (Defined in servo setup)
    TA1CTL |= MC_1;
}

// Used to schedule events
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

// Checks if it is time to get a reading from the ultrasonic
void checkSchedule()
{
    // Check if time to get left ultrasonic reading
    if (isTime(Schedule.ultraLeftStart))
    {
        ultrasonicTrigger(&ultraLeft);
        LEDIndicatorOn();


        //Disable schedule
        Schedule.ultraLeftStart.sec = 0;
        Schedule.ultraLeftStart.ms = -1;
    }
}

// Checks if a reading from the ultrasonic is ready
void checkFlags()
{
    // Left wall ultrasonic has reading ready
    if (flag.ultraLeftWallRead)
    {
        flag.ultraLeftWallRead = 0;
        LEDIndicatorOff();
        P1DIR |= BIT0;
                P1OUT |= BIT0;
        // Schedule next reading from left ultrasonic
        timeIncrement(&(Schedule.ultraLeftStart), 0, 500);
    }
}

int main(void)
{
    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    // Setup device
    ultrasonicSetup(&ultraLeft);                                        // Set up pin for the left ultrasonic
    setupLEDIndicator();                                                // Set up the LED Iindicator
    setupTimerSchedule();                                               // Sets up scheduling for Time0 A0 interrupt which produces the 2ms clock cycle which program runs off of

    // Disable schedule
    Schedule.ultraLeftStart.sec = 0;
    Schedule.ultraLeftStart.ms = -1;

    // Start getting first measurement after 0.5 seconds
    timeIncrement(&(Schedule.ultraLeftStart), 0, 500);

    // Disable flag
    flag.ultraLeftWallRead = 0;

    //Enable global interrupts
    __bis_SR_register(GIE);

    //Main loop
    while(1)
    {
        if (flag.timerA0)                                               // Every 2ms timer intterupt triggers (producing soft clock)
        {
            checkSchedule();                                            // If button has been pressed wait for debouncing time to verify
            flag.timerA0 = 0;
        }
        else
        {
            checkFlags();                                               // Check if button has been pressed, and if still pressed after 20ms
        }
    }
}

// Flag that for every 2ms passed update current time
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR(void)
{
        currentTime.ms += TIMER_INC_MS;
        if(currentTime.ms >= SECOND_COUNT)                              // Increment timer
        {
            currentTime.ms -= SECOND_COUNT;
            if(++currentTime.sec == 60) currentTime.sec = 0;
        }
        flag.timerA0 = 1;                                               // Tells checkFlags to see if program needs to action anything

        __low_power_mode_off_on_exit();
        TA0CCTL0 &= ~CCIFG;
}

//
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR (void)
{
    //If wrap occurs during ultrasonic pulse
    if (ultraLeft.timeNumber != 0)
    {
        ultraLeft.time[1] += TA1CCR0;
    }
}

// Ultrasonic capture compare
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer1_A1_ISR (void)
{
    switch(TA1IV)
    {
    case 0xA:   //OVERFLOW
        TA1CTL &= ~TAIFG;												// Clear overflow interrupt flag
        break;
    case TA1IV_TACCR1:  												// TA1CCR1 (Wall Ultrasonic)
        if(TA1CCTL1 & CCIS_1)   										// LEFT Ultrasonic being used
        {
            ultraLeft.time[ultraLeft.timeNumber] += TA1CCR1;
            ultraLeft.timeNumber++;
            if (ultraLeft.timeNumber==2)       							// After up/down edges of feedback
            {
                ultraLeft.distance = ultraLeft.time[1]-ultraLeft.time[0];
                flag.ultraLeftWallRead = 1;								// Tells program via checkFlags() that a reading is ready
                ultraLeft.time[0] = 0;
                ultraLeft.time[1] = 0;
                ultraLeft.timeNumber=0;

                TA1CCTL1 |= CM_1;   									// Capture on rising edge
            }
            else
            {
                TA1CCTL1 |= CM_2;   									// Capture on falling edge
            }
            TA1CCTL1 &= ~CCIFG;											// Clear CCR1 interrupt flag
        }
        break;

    case TA1IV_TACCR2:  												// TA1CCR2 (No interrupt as used in servo PWM)
        TA1CCTL2 &= ~CCIFG;												// Clear CCR2 interrupt flag
        break;
    }
}