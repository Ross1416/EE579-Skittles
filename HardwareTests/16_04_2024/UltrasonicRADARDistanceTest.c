/*
UltrasonicRADARDistanceTest.c

When program starts, ultrasonic sensor will be triggered every 0.5 seconds, with the LED Indicator turning on when the
ultrasonic sensor detects an object within 30 cm, and turns off when no object is within 30 cm.

At start, schedule is timed for 0.5s, at which point checkSchedule turns on LED Indicator and
sends a pulse to the ultrasonic sensor with Timer0_A1 raising a flag when two measurements are obtained, with checkFlags()
determining the distance of the object, turning on LED Indicator if it is within 30cm or off it is not, then removing the flag.
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
#define SECOND_COUNT            1000
#define isTime(X) ((currentTime.sec == X.sec) && (currentTime.ms == X.ms))

// Scheduler control
#define TIMER_INC_MS    2                                               // Scheduler interrupts period (2 ms)

// Important Ultrasonic Info
#define SOUND_SPEED 343
#define CLOCK_USED_ULTRASONIC   SMCK_FREQ
#define WALL_READINGS   5

// RADAR Reading control
#define RADAR_READINGS   5
unsigned int i;

// Hardware ports
#define RADAR_TRIG          BIT0                                        // Located at P1.0
#define RADAR_ECHO          BIT2                                        // Located at P1.2

// Defines structure for program timing - required for all tests bar LEDIndicatorTest
struct Time {
    int sec;
    int ms;
};

// Defines flags for hardware and software source to alert system to changes or readings  - required for all tests bar LEDIndicatorTest
struct flags {
    // When schedule timer ticks
    char timerA0;

    // Read RADAR ultrasonic when result avialable
    char ultraRADARRead;
};

// Defines structure containing all timings that events occur on  - required for all tests bar LEDIndicatorTest
struct Scheduler {
    // Time to start a RADAR ultrasonic reading
    struct Time ultraRADARStart;
};

// Scheduling information
struct Time currentTime     =   {0, 0};    //Running count of time
struct Scheduler Schedule   =   {0};       //Schedule when events needing attended
struct flags flag           =   {0};       //Flag when something ready to be attended

//RADAR Ultrasonic info (Port 1)
struct Ultrasonic ultraRADAR = {0, {0, 0}, 0, RADAR_TRIG, RADAR_ECHO, 1};
unsigned int RADARDistances[RADAR_READINGS] = {[0 ... RADAR_READINGS-1] = 2000};

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
    TA0CCTL0 |= CCIE;                                                   // Interrupt occurs when TA0R reaches TA0CCR0 - enables CCR0 interrup
    TA0CCR0 = CLOCK_USED_SCHEDULER*TIMER_INC_MS/1000;                   // Set the count to schedule time, e.g 1 MHz*2ms = 2000 so 2ms
    TA0CCTL0 &= ~CCIFG;                                                 // Clear CCR0 interrupt flags

    TA0CTL &= ~TAIFG;                                                   // Clear interrupt
    TA0CTL &= ~TAIE;                                                    // Disable interrupt on timer edge
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
    // Check if time to get RADAR ultrasonic reading
    if (isTime(Schedule.ultraRADARStart))
    {
        ultrasonicTrigger(&ultraRADAR);
        //Disable schedule
        Schedule.ultraRADARStart.sec = 0;
        Schedule.ultraRADARStart.ms = -1;
    }
}

// Checks if a reading from the ultrasonic is ready
void checkFlags()
{
    // RADAR ultrasonic has reading ready
    if (flag.ultraRADARRead)
    {
        // Update array of wall readings
        for(i = RADAR_READINGS-1; i > 0; i--)
        {
            RADARDistances[i] = RADARDistances[i - 1];                  // Shift each value along hence dropping oldest value
        }
        RADARDistances[0] = ultraRADAR.distance;                        // Newest value is added

        // Check whether a single reading is an outlier & if so ignore it
        if (((RADARDistances[1] >= RADARDistances[2]+2000) || (RADARDistances[1] <= RADARDistances[2]-2000))
            && ((RADARDistances[1] >= RADARDistances[0]+2000) || (RADARDistances[1] <= RADARDistances[0]-2000)))
        {
            RADARDistances[1] = (RADARDistances[0]+RADARDistances[2]) >> 1;
        }

        if(RADARDistances[0] < 700)
        {
            LEDIndicatorOn();
        }
        else
        {
            LEDIndicatorOff();
        }

        flag.ultraRADARRead = 0;
        // Schedule next reading from left ultrasonic
        timeIncrement(&(Schedule.ultraRADARStart), 0, 120);
    }
}

int main(void)
{
    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    // Setup device
    ultrasonicSetup(&ultraRADAR);                                       // Set up pin for the left ultrasonic
    setupLEDIndicator();                                                // Set up the LED Iindicator
    setupTimerSchedule();                                               // Sets up scheduling for Time0 A0 interrupt which produces the 2ms clock cycle which program runs off of

    // Disable schedule
    Schedule.ultraRADARStart.sec = 0;
    Schedule.ultraRADARStart.ms = -1;

    // Start getting first measurement after 0.5 seconds
    timeIncrement(&(Schedule.ultraRADARStart), 0, 120);

    // Disable flag
    flag.ultraRADARRead = 0;

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
    if(currentTime.ms >= SECOND_COUNT)                                  // Increment timer
    {
        currentTime.ms -= SECOND_COUNT;
        if(++currentTime.sec == 60) currentTime.sec = 0;
    }
    flag.timerA0 = 1;                                                   // Tells checkFlags to see if program needs to action anything

    // If wrap occurs during ultrasonic pulse
    if (ultraRADAR.timeNumber != 0)
    {
        ultraRADAR.time[1] += TA0CCR0;
    }

    __low_power_mode_off_on_exit();
    TA0CCTL0 &= ~CCIFG;
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer0_A1_ISR(void)
{
    switch(TA0IV)
    {
        case TA0IV_TACCR1:                                              // TA0CCR1 (RADAR Ultrasonic)
            ultraRADAR.time[ultraRADAR.timeNumber] += TA0CCR1;
            ultraRADAR.timeNumber++;
            if (ultraRADAR.timeNumber==2)                               // After up/down edges of feedback
            {
                ultraRADAR.distance = ultraRADAR.time[1]-ultraRADAR.time[0];    // Calculates distance of object
                flag.ultraRADARRead = 1;                                // Tells program via checkFlags() that a reading is ready
                ultraRADAR.time[0] = 0;
                ultraRADAR.time[1] = 0;
                ultraRADAR.timeNumber=0;
                TA0CCTL1 |= CM_1;                                       // Capture on rising edge
            }
            else
            {
                TA0CCTL1 |= CM_2;                                       // Capture on falling edge
            }
            TA0CCTL1 &= ~CCIFG;                                         // Clear CCR1 interrupt flag
            break;

        case TA0IV_TACCR2:                                              // TA0CCR2
            TA0CCTL2 &= ~CCIFG;                                         // Clear CCR2 interrupt flag
            break;

        case 0xA:   break;
    }
}
