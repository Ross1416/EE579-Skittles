/*
Filename    : HardwareTests/UltrasonicTest.c
Author      : Andrew Law
Project     : EE579 Skittles Knocking Over Project
Date        : 12/4/24
Description : Script to test the ultrasonic sensor components

            Either SONAR, LEFT or RIGHT ultrasonic can be tested
            When program starts, ultrasonic sensor will be
            triggered every 0.12 seconds , with the indicator
            LED turning on when the ultrasonic sensor is triggered
            and turns off when the ultrasonic sensor outputs a result
            to the msp430.

            At start, the ultrasonic is set up depending on which
            sensor is used, where SONAR = 0, LEFT = 1 and RIGHT = 2,
            schedule is timed for 0.5s, at which point checkSchedule
            turns on Indicator LED and sends a pulse to the ultrasonic
            sensor with Timer0_A1 raising a flag when two measurements
            are obtained, with checkFlags() turning of the Indicator LED
            and removing the flag.
--------------------------------------------------------------------------------
Functions Present
--------------------------------------------------------------------------------
ultrasonicSetup()
indicatorLEDSetup()
setupTimerSchedule()
ultrasonicTrigger()
indicatorLEDOn()
indicatorLEDOff()

SONAR requires setupTimerScheduler()
LEFT and RIGHT requires setupTimerScheduler() and setupTimerSONAR()
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
12-APR-2024 andrewlaw9178 created to modularise the ultrasonic sensor
13-APR-2024 andrewwlaw9178 edited the code
14-APR-2024 andrewwlaw9178 made individual versions for SONAR, LEFT, RIGHT
15-APR-2024 andrewwlaw9178 tried to determine issues with timers, and 
			formatted the code - issue were incorrect hardware 
			connects, 
16-APR-2024 andrewwlaw9178 used servoInputChecks to ensure everything was
			working, SONAR and LEFT working, and successfully combined
17-APR-2024 andrewwlaw9178 got RIGHT working and successfully combined
			everything together, the fixed structure and added variable 
			containing indicator LED information
--------------------------------------------------------------------------------
*/

// External libraries
#include <msp430.h>

// Local libraries
#include "Ultrasonic.h"
#include "IndicatorLED.h"

// Available clock information
#define SMCK_FREQ      1000000
#define ACLK_FREQ      32768

// Scheduler information
#define CLOCK_USED_SCHEDULER    SMCK_FREQ
#define SECOND_COUNT            1000
#define isTime(X) ((currentTime.sec == X.sec) && (currentTime.ms == X.ms))

// Scheduler control
#define TIMER_INC_MS    2                                               // Scheduler interrupts period (2 ms)

// Important Ultrasonic Info
#define SOUND_SPEED 343
#define CLOCK_USED_ULTRASONIC   SMCK_FREQ

// Hardware ports
#define IND_LED             BIT5                                        // Located at P2.5
#define SONAR_TRIG          BIT0                                        // Located at P1.0
#define SONAR_ECHO          BIT2                                        // Located at P1.2
#define ULTRA_LEFT_TRIG     BIT0                                        // Located at P2.0
#define ULTRA_LEFT_ECHO     BIT2                                        // Located at P2.2
#define ULTRA_RIGHT_TRIG    BIT0                                        // Located at P2.0
#define ULTRA_RIGHT_ECHO    BIT1                                        // Located at P2.1
char ultraUsed;                                                          // 0 = SONAR, 1 = LEFT, 2 = RIGHT

// Defines structure for program timing
struct Time {
    int sec;
    int ms;
};

// Defines flags for hardware and software source to alert system to changes or readings
struct flags {
    // When schedule timer ticks
    char timerA0;

    // Read left wall ultrasonic when result available
    char ultraLeftWallRead;

    // Read right wall ultrasonic when result available
    char ultraRightWallRead;

    // Read SONAR ultrasonic when result avialable
    char ultraSONARRead;
};

// Defines structure containing all timings that events occur on
struct Scheduler {
    // Time to start a left ultrasonic reading
    struct Time ultraLeftStart;

    // Time to start a right ultrasonic reading
    struct Time ultraRightStart;

    // Time to start a SONAR ultrasonic reading
    struct Time ultraSONARStart;
};

// Scheduling information
struct Time currentTime     =   {0, 0};                                 // Variable containing soft clock - running count of time
struct Scheduler Schedule   =   {0};                                    // Schedule when events needing attended
struct flags flag           =   {0};                                    // Flag when something ready to be attended

// SONAR Ultrasonic information - Port 1
struct Ultrasonic ultraSONAR = {0, {0, 0}, 0, SONAR_TRIG, SONAR_ECHO, 1};

// Wall Ultrasonic information - Port 2
struct Ultrasonic ultraLeft = {0, {0, 0}, 0, ULTRA_LEFT_TRIG, ULTRA_LEFT_ECHO, 2};
struct Ultrasonic ultraRight = {0, {0, 0}, 0, ULTRA_RIGHT_TRIG, ULTRA_RIGHT_ECHO, 2};

// Indicator LED
struct IndicateLED indicatorLED = {2, IND_LED};

// Sets up the Timer0 A0 to create a soft clock of 2ms period
void setupTimerSchedule()
{
    if(CLOCK_USED_SCHEDULER == SMCK_FREQ)
    {
        TA0CTL |= TASSEL_2 + MC_1;                                      // SMCK selected, so f = 1 MHz, operating in up mode to TA0CCR0
    }
    else
    {
        TA0CTL |= TASSEL_1 + MC_1;                                      // ACLK selected, so f = 32768 Hz, operating in up mode to TA0CCR0
    }
    TA0CCTL0 |= CCIE;                                                   // Interrupt occurs when TA0R reaches TA0CCR0 - enables CCR0 interrup
    TA0CCR0 = CLOCK_USED_SCHEDULER*TIMER_INC_MS/1000;                   // Set the count to schedule time, e.g 1 MHz*2ms = 2000 so 2ms
    TA0CCTL0 &= ~CCIFG;                                                 // Clear CCR0 interrupt flags

    TA0CTL &= ~TAIFG;                                                   // Clear interrupt
    TA0CTL &= ~TAIE;                                                    // Disable interrupt on timer edge
}

// Set up Timer1 A1 interrupt
void setupTimerSONAR()
{
    if(CLOCK_USED_ULTRASONIC == SMCK_FREQ)
    {
        TA1CTL |= TASSEL_2;                                             // SMCLK selected, so f = 1 MHz
    }
    else
    {
        TA1CTL |= TASSEL_1;                                             // ACLK selected , so f = 32.768 kHz
    }

    TA1CTL &= ~TAIFG;                                                   // Clear interrupt
    TA1CTL &= ~TAIE;                                                    // Disable interrupt on timer edge

    TA1CCTL0 &= ~(CCIFG+CCIE);
    TA1CCTL2 &= ~CCIE;                                                  // Disable CCR2 interrupt
    TA1CCTL2 &= ~CCIFG;                                                 // Clear CCR2 interrupt flag

    TA1CCTL0 |= CCIE;                                                   // Enable interrupt to know when wraps

                                                                        // Count to TA1CCR0 (Defined in servo setup) ??????????????????????????
    TA1CTL |= MC_1;
}

// Used to schedule events
void timeIncrement(struct Time *time, int sec, int ms)
{
    // Always on next increment amount
    ms += TIMER_INC_MS;
    ms -= ms % TIMER_INC_MS;

    // Add to current time and if more then max ms add to seconds
    time->ms = currentTime.ms + ms;
    while(time->ms >= SECOND_COUNT)
    {
        time->ms -= SECOND_COUNT;
        sec++;
    }

    // Add to current time and if more then max s wrap back to 0
    time->sec = currentTime.sec + sec;
    while(time->sec >= 60)
    {
        time->sec -= 60;
    }
}

// Checks if it is time to get a reading from the ultrasonic
void checkSchedule()
{
    // Check if time to get SONAR ultrasonic reading
    if (isTime(Schedule.ultraSONARStart))
    {
        ultrasonicTrigger(&ultraSONAR);
        indicatorLEDOn(&indicatorLED);          // Turn on indicator LED
        // Disable schedule
        Schedule.ultraSONARStart.sec = 0;
        Schedule.ultraSONARStart.ms = -1;
    }

    // Check if time to get left ultrasonic reading
    if (isTime(Schedule.ultraLeftStart))
    {
        ultrasonicTrigger(&ultraLeft);
        indicatorLEDOn(&indicatorLED);          // Turn on indicator LED
        // Disable schedule
        Schedule.ultraLeftStart.sec = 0;
        Schedule.ultraLeftStart.ms = -1;
    }

    // Check if time to get right ultrasonic reading
    if (isTime(Schedule.ultraRightStart))
    {
        ultrasonicTrigger(&ultraRight);
        indicatorLEDOn(&indicatorLED);          // Turn on indicator LED
        // Disable schedule
        Schedule.ultraRightStart.sec = 0;
        Schedule.ultraRightStart.ms = -1;
    }
}

// Checks if a reading from the ultrasonic is ready
void checkFlags()
{
    // SONAR ultrasonic has reading ready
    if (flag.ultraSONARRead)
    {
        indicatorLEDOff(&indicatorLED);         // Turn off indicator LED
        flag.ultraSONARRead = 0;
        // Schedule next reading from left ultrasonic
        timeIncrement(&(Schedule.ultraSONARStart), 0, 120);
    }

    // Left wall ultrasonic has reading ready
    if (flag.ultraLeftWallRead)
    {
        indicatorLEDOff(&indicatorLED);         // Turn off indicator LED
        flag.ultraLeftWallRead = 0;
        // Schedule next reading from left ultrasonic
        timeIncrement(&(Schedule.ultraLeftStart), 0, 120);
    }

    // Right wall ultrasonic has reading ready
    if (flag.ultraRightWallRead)
    {
        indicatorLEDOff(&indicatorLED);         // Turn off indicator LED
        flag.ultraRightWallRead = 0;
        // Schedule next reading from right ultrasonic
        timeIncrement(&(Schedule.ultraRightStart), 0, 120);
    }
}

int main(void)
{
    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    ultraUsed = 2;                                                          // 0 = SONAR (P1.2), 1 = LEFT (P2.2), 2 = RIGHT (P2.1)

    if(ultraUsed == 0)
    {
        // Setup device
        ultrasonicSetup(&ultraSONAR);                                       // Set up pin for the left ultrasonic
        indicatorLEDSetup(&indicatorLED);                                   // Set up the Indicator LED
        setupTimerSchedule();                                               // Sets up scheduling for Time0 A0 interrupt which produces the 2ms clock cycle which program runs off of

        // Disable schedule
        Schedule.ultraSONARStart.sec = 0;
        Schedule.ultraSONARStart.ms = -1;

        // Start getting first measurement after 0.12 seconds
        timeIncrement(&(Schedule.ultraSONARStart), 0, 120);

        // Disable flag
        flag.ultraSONARRead = 0;
    }
    else if(ultraUsed == 1)
    {
        // Setup device
        ultrasonicSetup(&ultraLeft);                                        // Set up pin for the left ultrasonic
        indicatorLEDSetup(&indicatorLED);                                   // Set up the Indicator LED
        setupTimerSchedule();                                               // Sets up scheduling for Time0 A0 interrupt which produces the 2ms clock cycle which program runs off of
        setupTimerSONAR();

        // Disable schedule
        Schedule.ultraLeftStart.sec = 0;
        Schedule.ultraLeftStart.ms = -1;

        // Start getting first measurement after 0.12 seconds
        timeIncrement(&(Schedule.ultraLeftStart), 0, 120);

        // Disable flag
        flag.ultraLeftWallRead = 0;
    }

    else if(ultraUsed == 2)
    {
        // Setup device
        ultrasonicSetup(&ultraRight);                                       // Set up pin for the right ultrasonic
        indicatorLEDSetup(&indicatorLED);                                   // Set up the Indicator LED
        setupTimerSchedule();                                               // Sets up scheduling for Time0 A0 interrupt which produces the 2ms clock cycle which program runs off of
        setupTimerSONAR();

        // Disable schedule
        Schedule.ultraRightStart.sec = 0;
        Schedule.ultraRightStart.ms = -1;

        // Start getting first measurement after 0.12 seconds
        timeIncrement(&(Schedule.ultraRightStart), 0, 120);

        // Disable flag
        flag.ultraRightWallRead = 0;
    }

    //Enable global interrupts
    __bis_SR_register(GIE);

    //Main loop
    while(1)
    {
        if (flag.timerA0)                                                   // Every 2ms timer intterupt triggers (producing soft clock)
        {
            checkSchedule();                                                // If button has been pressed wait for debouncing time to verify
            flag.timerA0 = 0;
        }
        else
        {
            checkFlags();                                                   // Check if button has been pressed, and if still pressed after 20ms
        }
    }
}

// Flag that for every 2ms passed update current time
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR(void)
{
    currentTime.ms += TIMER_INC_MS;
    if(currentTime.ms >= SECOND_COUNT)                                      // Increment timer
    {
        currentTime.ms -= SECOND_COUNT;
        if(++currentTime.sec == 60) currentTime.sec = 0;
    }
    flag.timerA0 = 1;                                                       // Tells checkFlags to see if program needs to action anything

    // If wrap occurs during ultrasonic pulse
    if (ultraSONAR.timeNumber != 0)
    {
        ultraSONAR.time[1] += TA0CCR0;
    }

    __low_power_mode_off_on_exit();
    TA0CCTL0 &= ~CCIFG;
}

// Determines SONAR ultrasonic pulse duration and raises flag - using capture mode
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer0_A1_ISR(void)
{
    switch(TA0IV)
    {
        case TA0IV_TACCR1:                                                  // TA0CCR1 (SONAR Ultrasonic)
            ultraSONAR.time[ultraSONAR.timeNumber] += TA0CCR1;
            ultraSONAR.timeNumber++;
            if (ultraSONAR.timeNumber==2)                                   // After up/down edges of feedback
            {
                ultraSONAR.distance = ultraSONAR.time[1]-ultraSONAR.time[0];    // Calculates distance of object
                flag.ultraSONARRead = 1;                                    // Tells program via checkFlags() that a reading is ready
                ultraSONAR.time[0] = 0;
                ultraSONAR.time[1] = 0;
                ultraSONAR.timeNumber=0;
                TA0CCTL1 |= CM_1;                                           // Capture on rising edge
            }
            else
            {
                TA0CCTL1 |= CM_2;                                           // Capture on falling edge
            }
            TA0CCTL1 &= ~CCIFG;                                             // Clear CCR1 interrupt flag
            break;

        case TA0IV_TACCR2:                                                  // TA0CCR2
            TA0CCTL2 &= ~CCIFG;                                             // Clear CCR2 interrupt flag
            break;

        case 0xA:   break;
    }
}

//
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR (void)
{
    // If wrap occurs during ultrasonic pulse
    if (ultraLeft.timeNumber != 0)
    {
        ultraLeft.time[1] += TA1CCR0;
    }
}

// Determines LEFT/RIGHT ultrasonic pulse duration and raises flag - using capture mode
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer1_A1_ISR (void)
{
    switch(TA1IV)
    {
    case 0xA:                                                               //OVERFLOW
        TA1CTL &= ~TAIFG;                                                   // Clear overflow interrupt flag
        break;
    case TA1IV_TACCR1:                                                      // TA1CCR1 (Wall Ultrasonic)
        if(TA1CCTL1 & CCIS_1)                                               // LEFT Ultrasonic being used
        {
            ultraLeft.time[ultraLeft.timeNumber] += TA1CCR1;
            ultraLeft.timeNumber++;
            if (ultraLeft.timeNumber==2)                                    // After up/down edges of feedback
            {
                ultraLeft.distance = ultraLeft.time[1]-ultraLeft.time[0];   // Calculates distance of object
                flag.ultraLeftWallRead = 1;                                 // Tells program via checkFlags() that a reading is ready
                ultraLeft.time[0] = 0;
                ultraLeft.time[1] = 0;
                ultraLeft.timeNumber=0;

                TA1CCTL1 |= CM_1;                                           // Capture on rising edge
            }
            else
            {
                TA1CCTL1 |= CM_2;                                           // Capture on falling edge
            }
            TA1CCTL1 &= ~CCIFG;                                             // Clear CCR1 interrupt flag
        }
        else                                                                // RIGHT Ultrasonic being used
        {
            ultraRight.time[ultraRight.timeNumber] += TA1CCR1;
            ultraRight.timeNumber++;
            if (ultraRight.timeNumber==2)                                   // After up/down edges of feedback
            {
                ultraRight.distance = ultraRight.time[1]-ultraRight.time[0];   // Calculates distance of object
                flag.ultraRightWallRead = 1;                                 // Tells program via checkFlags() that a reading is ready
                ultraRight.time[0] = 0;
                ultraRight.time[1] = 0;
                ultraRight.timeNumber=0;

                TA1CCTL1 |= CM_1;                                           // Capture on rising edge
            }
            else
            {
                TA1CCTL1 |= CM_2;                                           // Capture on falling edge
            }
            TA1CCTL1 &= ~CCIFG;                                             // Clear CCR1 interrupt flag
        }
        break;

    case TA1IV_TACCR2:                                                      // TA1CCR2 (No interrupt as used in servo PWM)
        TA1CCTL2 &= ~CCIFG;                                                 // Clear CCR2 interrupt flag
        break;
    }
}

//==============================================================================
// End of File :  HardwareTest/UltrasonicTest.c
