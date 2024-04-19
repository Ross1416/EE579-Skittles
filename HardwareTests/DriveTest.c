/*
Filename    : HardwareTests/DriveTest.c
Author      : Andrew Law
Project     : EE579 Skittles Knocking Over Project
Date        : 11/4/24
Description : Script to test the back DC motor and H-Bridge components
			
			When program starts, car will move forward for 0.5
			seconds, then reverse backwards for 0.5 seconds. This
			will occur forever, alternating between two speeds, 
			with the indicator LED used to show the faster speed.

			At start, PWM signal will be set up, with it using
			checkSchedule() to create a PWM signal by alternating
			the state. The other section in the checkSchedule() 
			raises a flag every 0.5 seconds to change the direction
			the back wheels move, with checkFlags() making the change.
--------------------------------------------------------------------------------
Functions Present
--------------------------------------------------------------------------------
motorSetup()
setupTimerSchedule()
indicatorLEDSetup()
indicatorLEDOn()
indicatorLEDOff()
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
12-APR-2024 andrewlaw9178 created to modularise the driving
17-APR-2024 andrewwlaw9178 updated comments
			and added variable containing indicator LED
			information
--------------------------------------------------------------------------------
*/

// External libraries
#include <msp430.h>

// Local libraries
#include "DCMotor.h"
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

// Information for drive motor speed
#define MOTOR_PWM_PERIOD    50                                          //PWM period of motors
#define SPEED_TOP           0.9*MOTOR_PWM_PERIOD                        //Top speed of the motors as percentage of PWM.
#define SPEED_SLOW          0.4*MOTOR_PWM_PERIOD                        //Slower speed when changing direction.

// Hardware ports
#define IND_LED					BIT5									// Located at P2.5
#define MOTOR_DRIVE_ANODE       BIT5                                    // Located at P1.4
#define MOTOR_DRIVE_CATHODE     BIT4                                    // Located at P1.5

unsigned int movementStage;                                             // To go through each stage

// Defines structure for program timing - required for all tests bar LEDIndicatorTest
struct Time {
    int sec;
    int ms;
};

// Defines flags for hardware and software source to alert system to changes or readings  - required for all tests bar LEDIndicatorTest
struct flags {
    // When schedule timer ticks
    char timerA0;

    // When state of drive motor needs changed (e.g FORWARD to STOP)
    char motorDrive;
};

// Defines structure containing all timings that events occur on  - required for all tests bar LEDIndicatorTest
struct Scheduler {
    // Time when drive motor PWM should change
    struct Time pwmMotorDrive;

    // Initially start car moving forward
    struct Time initiallyMoveForward;

    // Time when next movement should occur
    struct Time nextMovement;
};

// Scheduling information
struct Time currentTime     =   {0, 0};                                 //Running count of time
struct Scheduler Schedule   =   {0};                                    //Schedule when events needing attended
struct flags flag           =   {0};                                    //Flag when something ready to be attended

// DC Motor info (On Port 1)
struct MotorDC motorDrive = {0, MOTOR_DRIVE_ANODE, MOTOR_DRIVE_CATHODE, {0, MOTOR_PWM_PERIOD, 0, SPEED_TOP, 1}};
                                                                        // Sets a ms period & duty cycle, initializing to HIGH

// Indicator LED
struct IndicateLED indicatorLED = {2, IND_LED};

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

// Checks if it is time to alter PWM for drive DC motor
void checkSchedule()
{
    // Initalise the temp mem allocation for the PWM duty cycle
    int incSec = 0;
    int incMs = 0;

    // Creating and controlling PWM for drive DC motor
    if(isTime(Schedule.pwmMotorDrive))
    {
        if(motorDrive.pwm.state)                                        // If PWM is HIGH
        {
            // Determine time for PWM signal to be set LOW based on duration of HIGH and PWM period
            incSec = motorDrive.pwm.sec-motorDrive.pwm.aSec;
            incMs = motorDrive.pwm.ms-motorDrive.pwm.aMs;

            // Set time when PWM signal will go HIGH again to create a PWM
            timeIncrement(&(Schedule.pwmMotorDrive), incSec, incMs);

            // Set PWM signal LOW
            motorDrive.pwm.state = 0;

            // Turn off drive DC motor in checkFlags() as PWM is now LOW
            flag.motorDrive = 1;
        }
        else                                                            // If PWM LOW
        {
            // Set time for PWM signal to be set LOW again
            timeIncrement(&Schedule.pwmMotorDrive, motorDrive.pwm.aSec, motorDrive.pwm.aMs);

            //  Set PWM signal HIGH
            motorDrive.pwm.state = 1;

            //  Turn on drive DC motor in checkFlags() as OWM signal is now HIGH
            if (motorDrive.pwm.aMs == 0)
            {
                // If PWM is always LOW do not change as scheduler takes
                // short time to then make LOW again so motor is on momentarily
                flag.motorDrive = 0;
            }
            else
            {
                flag.motorDrive = 1;
            }
        }
    }

    // Get car to start moving
    if(isTime(Schedule.initiallyMoveForward))
    {
        // Disable schedule
        Schedule.initiallyMoveForward.sec = 0;
        Schedule.initiallyMoveForward.ms = -1;

        // Start driving forward
        motorDrive.direction = FORWARD;
        flag.motorDrive = 1; // Used to change the state of the drive motor
        motorDrive.pwm.aMs = SPEED_TOP;
        indicatorLEDOn(&indicatorLED);											// Turn on indicator LED


        // Schedule next movement
        timeIncrement(&Schedule.nextMovement, 0, 500);
    }

    // Get car to next movement
    if(isTime(Schedule.nextMovement))
    {
        if(movementStage == 0)
        {
            // Stop car
            motorDrive.direction = OFF;
            flag.motorDrive = 1;
        }
        else if(movementStage == 1)
        {
            // Start driving backwards
            motorDrive.direction = BACK;
            flag.motorDrive = 1;
        }
        else if(movementStage == 2)
        {
            // Stop car
            motorDrive.direction = OFF;
            flag.motorDrive = 1;
        }
        else if(movementStage == 3)
        {
            // Stop driving forwards slower
            motorDrive.direction = FORWARD;
            flag.motorDrive = 1;
            motorDrive.pwm.aMs = SPEED_SLOW;
            indicatorLEDOff(&indicatorLED);								// Turn off indicator LED
        }
        else if(movementStage == 4)
        {
            // Stop car
            motorDrive.direction = OFF;
            flag.motorDrive = 1;
        }
        else if(movementStage == 5)
        {
            // Start driving backwards
            motorDrive.direction = BACK;
            flag.motorDrive = 1;
        }
        else if(movementStage == 6)
        {
            // Stop car
            motorDrive.direction = OFF;
            flag.motorDrive = 1;
        }
        else if(movementStage == 7)
        {
            // Start driving forwards faster
            motorDrive.direction = FORWARD;
            flag.motorDrive = 1;
            motorDrive.pwm.aMs = SPEED_TOP;
            indicatorLEDOn(&indicatorLED);								// Turn on indicator LED
        }

        if(movementStage == 7)
        {
            movementStage = 0;                                          // Means that movementStage doesnt have to be negative reducing memory
        }
        else
        {
            // Move on to next stage
            movementStage++;
        }

        // Schedule next movement
        timeIncrement(&Schedule.nextMovement, 0, 500);
    }
}

// Checks if servo should move
void checkFlags()
{
    // Drive DC motor needs to change behaviour
    if (flag.motorDrive)
    {
        // Alter behaviour as has been updated
        motorOutput(&motorDrive);                                       // Sets the pins LOW/HIGH to create the PWM signal
        flag.motorDrive = 0;
    }
}

int main(void)
{
    // Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    // Setup device
    motorSetup(&motorDrive);                                            // Set up ports and timing for the motor for driving
    indicatorLEDSetup(&indicatorLED);              					 	// Set up the Indicator LED
	setupTimerSchedule();                                               // Sets up scheduling for Time0 A0 interrupt which produces the 2ms clock cycle which program runs off of

    // Start drive DC motor PWM schedules but set its output to do nothing
    timeIncrement(&Schedule.pwmMotorDrive, motorDrive.pwm.aSec, motorDrive.pwm.aMs); // Sets time when duty cycle occurs to set PWM LOW/HIGH
    motorDrive.pwm.state = 1;                                           // Sets PWM signal initally to HIGH
    flag.motorDrive = 1;
    motorDrive.direction = OFF;                                         // At the beginning the car will stay stationary

    // Set half second timer for motor to drive car forward
    timeIncrement(&Schedule.initiallyMoveForward, 0, 500);
    movementStage = 0;

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
//==============================================================================
// End of File :  HardwareTests/DriveTest.c
