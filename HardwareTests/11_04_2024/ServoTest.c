/*
ServoTest.c

When program starts, servo is centred for 200ms, rotates anticlockwise till at max, then rotates clockwise
till max, then repeats process forever.

At start, servo is set to centre, with checkSchedule() after 120ms raising flag for servo, which causes checkFlags() to turn the servo,
then 120ms later checkSchedule() raises flag for servo again and process repeats.
*/

// External libraries
#include <msp430.h>

// Local libraries
#include "Servo.h"

// Available clock information
#define SMCK_FREQ      1000000
#define ACLK_FREQ      32768

// Scheduler information
#define CLOCK_USED_SCHEDULER    SMCK_FREQ
#define SECOND_COUNT            1000        //1000 ms = 1s
#define isTime(X) ((currentTime.sec == X.sec) && (currentTime.ms == X.ms))

// Scheduler control
#define TIMER_INC_MS    2           //Scheduler interrupts period (2 ms)

// Other information for servo
#define CLOCK_USED_ULTRASONIC   SMCK_FREQ

// Servo PWM limits
#define NUMBER_OF_ANGLES_CHECKED 15 //15

// Hardware ports
#define SERVO_PIN   BIT4

// Defines structure for program timing - required for all tests bar LEDIndicatorTest
struct Time {
    int sec;
    int ms;
};

// Defines flags for hardware and software source to alert system to changes or readings  - required for all tests bar LEDIndicatorTest
struct flags {
    // When schedule timer ticks
    char timerA0;

    // When angle of servo should change
    char servoMove;
};

// Defines structure containing all timings that events occur on  - required for all tests bar LEDIndicatorTest
struct Scheduler {
    // Change angle of servo?
    struct Time turnServo;
};

// Scheduling information
struct Time currentTime     =   {0, 0};    //Running count of time
struct Scheduler Schedule   =   {0};       //Schedule when events needing attended
struct flags flag           =   {0};       //Flag when something ready to be attended

// Servo info (Port 2)
struct Servo servoA = {SERVO_PIN, (PWM_SERVO_UPPER-PWM_SERVO_LOWER)/NUMBER_OF_ANGLES_CHECKED, 0};    //PWM on Port 2.4, angle to be turned, initially turn anti-clockwise

// Set up Timer1 A1 interrupt
void setupTimerRADAR()
{
    if(CLOCK_USED_ULTRASONIC == SMCK_FREQ)
    {
        TA1CTL |= TASSEL_2;         // f = 1 MHz
    }
    else
    {
        TA1CTL |= TASSEL_1;         // f = 32.768 kHz
    }

    TA1CTL &= ~TAIFG;   //Clear interrupt
    TA1CTL &= ~TAIE;    //Disable interrupt on timer edge

    TA1CCTL0 &= ~(CCIFG+CCIE);
    TA1CCTL2 &= ~CCIE;
    TA1CCTL2 &= ~CCIFG;

    TA1CCTL2 |= CCIE;   //Enable interrupt to know when wraps

    //Count to TA1CCR0 (Defined in servo setup)
    TA1CTL |= MC_1;
}

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

// Checks if it is time for servo to move
void checkSchedule()
{
    // Check if time to move servo
    if(isTime(Schedule.turnServo))
    {
        // Carry out check in checkFlags()
        flag.servoMove = 1;

        //Disable schedule
        Schedule.turnServo.sec = 0;
        Schedule.turnServo.ms = -1;
    }
}

// Checks if servo should move
void checkFlags()
{
    // Turn servo
    if(flag.servoMove)
    {
        if(TA1CCR2 >= PWM_SERVO_UPPER)                                  // If reached upper bound
        {
            servoA.direction = 0;                                       // Turn anticlockwise
        }
        else if (TA1CCR2 <= PWM_SERVO_LOWER)
        {
            servoA.direction = 1;                                       // Turn anticlockwise
        }

        servoTurn(&servoA);

        // Move servo after
        timeIncrement(&Schedule.turnServo, 0, 120);

        flag.servoMove = 0;
    }
}

int main(void)
{
    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    // Setup device
    servoSetup(&servoA);                                                // Set up port and timing for the servo and set servo to turn clockwise
    setupTimerSchedule();                                               // Sets up scheduling for Time0 A0 interrupt which produces the 2ms clock cycle which program runs off of
    setupTimerRADAR();                                                  // Set up Timer1 A1
    // Disable flag
    flag.servoMove = 0;

    // Start servo at centre and wait 0.5s
    servoCenter();
    timeIncrement(&Schedule.turnServo, 0, 120);

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

// Used to move servo - pwm signal determines position of servo
//Ultrasonic capture compare
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer1_A1_ISR (void)
{
    switch(TA1IV)
    {
    case 0xA:   //OVERFLOW
        TA1CTL &= ~TAIFG;
        break;
    case TA1IV_TACCR2:  //TA1CCR2 (No interrupt as used in servo PWM)
        TA1CCTL2 &= ~CCIFG;
        break;
    }
}