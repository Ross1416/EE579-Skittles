/*
Filename    : HardwareTest/Common.c
Author      : Andrew Law
Project     : EE579 Skittles Knocking Over Project
Date        : 11/4/24
Description : Script to test all sensor components

            Comment out all functions that arent to be tested.
            One function is used for testing each component.
            Select a word into the function to determine which
            component is to be tested.
--------------------------------------------------------------------------------
Functions Present
--------------------------------------------------------------------------------
selectComponent()
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
22-APR-2024 andrewlaw9178 created to have hardware component test in 1 location
25-APR-2024 andrewlaw9178 fixed LED and button and added drive tests
26-APR-2024 andrewlaw9178 added steer and servo test
--------------------------------------------------------------------------------
*/
// External libraries
#include <msp430.h>

// Local libraries
#include "Common.h"

// Scheduler information
#define isTime(X) ((currentTime.sec == X.sec) && (currentTime.ms == X.ms))

// Scheduling information
struct Time currentTime = {0, 0};                                       // Variable containing soft clock - running count of time
struct Scheduler Schedule= {{0, -1}};                                   // Schedule when events needing attended
struct Flags flag = {0, 0, 0};                                          // Flag when something ready to be attended

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

// Checks if debouncing has occured
void checkSchedule()
{
    // Initalise the temp mem allocation for the PWM duty cycle
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
        indicatorLEDOn(&indicatorLED);                                          // Turn on indicator LED


        // Schedule next movement
        timeIncrement(&Schedule.nextMovement, 0, 500);
    }

    // Get car to next drive movement
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
            indicatorLEDOff(&indicatorLED);                             // Turn off indicator LED
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
            indicatorLEDOn(&indicatorLED);                              // Turn on indicator LED
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

    // Creating and controlling PWM for steer DC motor
    if(isTime(Schedule.pwmMotorSteer))
    {
        if(motorSteer.pwm.state)                                        // If PWM is HIGH
        {
            // Determine time for PWM signal to be set LOW based on duration of HIGH and PWM period
            incSec = motorSteer.pwm.sec-motorSteer.pwm.aSec;
            incMs = motorSteer.pwm.ms-motorSteer.pwm.aMs;

            // Set time when PWM signal will go HIGH again to create a PWM
            timeIncrement(&(Schedule.pwmMotorSteer), incSec, incMs);

            // Set PWM signal LOW
            motorSteer.pwm.state = 0;

            // Turn off steer DC motor in checkFlags() as PWM is now LOW
            flag.motorSteer = 1;
        }
        else
        {                                                               // If PWM LOW
            // Set time for PWM signal to be set LOW again
            timeIncrement(&Schedule.pwmMotorSteer, motorSteer.pwm.aSec, motorSteer.pwm.aMs);

             // Set PWM signal HIGH
             motorSteer.pwm.state = 1;

             // Turn on drive DC motor in checkFlags() as PWM signal is now HIGH
             if (motorSteer.pwm.aMs == 0)
             {
                 // If PWM is always LOW do not change as scheduler takes
                 // short time to then make LOW again so motor is on momentarily
                 flag.motorSteer = 0;
             }
             else
             {
                 flag.motorSteer = 1;
             }
        }
    }

    // Get car to next steer movement
    if(isTime(Schedule.nextMovementSteer))
    {
        if(movementStageSteer == 0)
        {
            // Turn LEFT
            motorSteer.direction = LEFT;
            flag.motorSteer = 1;
        }
        else if(movementStageSteer == 1)
        {
            // Turn STRAIGHT
            motorSteer.direction = STRAIGHT;
            flag.motorSteer = 1;
        }
        else if(movementStageSteer == 2)
        {
            // Turn LEFT
            motorSteer.direction = RIGHT;
            flag.motorSteer = 1;
        }
        else if(movementStageSteer == 3)
        {
            // Turn LEFT
            motorSteer.direction = STRAIGHT;
            flag.motorSteer = 1;
        }
        if(movementStageSteer == 3)
        {
            movementStageSteer = 0;                                          // Means that movementStageSteer doesnt have to be negative reducing memory
        }
        else
        {
            // Move on to next stage
            movementStageSteer++;
        }

        // Schedule next movement
        timeIncrement(&Schedule.nextMovementSteer, 0, 500);
    }

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

// Checks if button has been pressed or if debouncing has occured
void checkFlags()
{
    // Debounce wait has finished
    if(flag.debounce)
    {
        if((P1IN & startButton.pin) != startButton.pin)                 // Button still pressed after debounce
        {
            indicatorLEDToggle(&indicatorLED);                          // Toggle indicator LED
        }
        Schedule.debounce.sec = 0;
        Schedule.debounce.ms = -1;
        flag.debounce = 0;
    }

    //On button press start debounce
    if(flag.button)
    {
        if(Schedule.debounce.sec == 0 && Schedule.debounce.ms == -1)    // If debounce not currently occurring
        {
            timeIncrement(&Schedule.debounce, 0, 20);                   // 20 ms debounce check
        }
        flag.button = 0;
    }

    // Drive DC motor needs to change behaviour
    if (flag.motorDrive)
    {
        // Alter behaviour as has been updated
        motorOutput(&motorDrive);                                       // Sets the pins LOW/HIGH to create the PWM signal
        flag.motorDrive = 0;
    }

    // Steering DC motor needs to change behaviour
    if (flag.motorSteer)
    {
        //Alter behaviour as has been updated
        motorOutput(&motorSteer);
        flag.motorSteer = 0;
    }

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

// Initalise indicator LED for all tests bar indicator LED test
void initLEDPHY(int port, int pin)
{
    indicatorLED.port = port;
    indicatorLED.pin = pin;
}

// Execute indicator LED test
void indicatorLEDTest(int port, int pin)
{
    indicatorLED.port = port;
    indicatorLED.pin = pin;

    // Setup output port
    indicatorLEDSetup(&indicatorLED);               // Set up the Indicator LED
    
    while(1)
    {
        indicatorLEDOn(&indicatorLED);              // Turn on indicator LED
    }
}

// Execute start button test
void startButtonTest(int port, int pin)
{
    startButton.port = port;
    startButton.pin = pin;

    // Setup device
    setupStartStop(&startButton);                                         // Selects start stop button input and sets up port 1 intterput for button
    indicatorLEDSetup(&indicatorLED);                                   // Set up the Indicator LED
    setupTimerSchedule();                                               // Sets up scheduling for Time0 A0 interrupt which produces the 2ms clock cycle which program runs off of

    // Disable sche dules
    Schedule.debounce.sec = 0;
    Schedule.debounce.ms = -1;                                          // Waits until button is pressed

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

// Execute drive test
void driveTest(int anode, int cathode )
{
    // DC Motor info (On Port 1) and sets a ms period & duty cycle, initializing to HIGH
    motorDrive.direction = 0;
    motorDrive.pinA = anode;
    motorDrive.pinB = cathode;
    motorDrive.pwm.sec = 0;
    motorDrive.pwm.ms = MOTOR_PWM_PERIOD;
    motorDrive.pwm.aSec = 0;
    motorDrive.pwm.aMs = SPEED_TOP;
    motorDrive.pwm.state = 1;
                                                                        // Sets a ms period & duty cycle, initializing to HIGH
    // Setup device
    motorSetup(&motorDrive);                                            // Set up ports and timing for the motor for driving
    indicatorLEDSetup(&indicatorLED);                                   // Set up the Indicator LED
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

// Execute steer test
void steerTest(int anode, int cathode)
{
    // DC Motor info (On Port 1) and sets a ms period & duty cycle, initializing to HIGH
    motorSteer.direction = 0;
    motorSteer.pinA = anode;
    motorSteer.pinB = cathode;
    motorSteer.pwm.sec = 0;
    motorSteer.pwm.ms = MOTOR_PWM_PERIOD;
    motorSteer.pwm.aSec = 0;
    motorSteer.pwm.aMs = MOTOR_PWM_PERIOD;
    motorSteer.pwm.state = 1;

    // Setup device
    motorSetup(&motorSteer);                                                            // Set up ports and timing for the motor for driving
    setupTimerSchedule();                                                               // Sets up scheduling for Time0 A0 interrupt which produces the 2ms clock cycle which program runs off of

    // Start steer DC motor PWM schedules but set its output to do nothing
    timeIncrement(&Schedule.pwmMotorSteer, motorSteer.pwm.aSec, motorSteer.pwm.aMs);
    motorSteer.pwm.state = 1;                                                           // Sets PWM signal to HIGH
    motorSteer.direction = STRAIGHT;                                                    // At the beginning the car will point STRAIGHT
    flag.motorSteer = 1;


    // Set half second timer for motor to drive car forward
    timeIncrement(&Schedule.nextMovementSteer, 0, 500);
    movementStageSteer = 0;

    //Enable global interrupts
    __bis_SR_register(GIE);

    //Main loop
    while(1)
    {
        if (flag.timerA0)                                                           // Every 2ms timer intterupt triggers (producing soft clock)
        {
            checkSchedule();                                                        // If button has been pressed wait for debouncing time to verify
            flag.timerA0 = 0;
        }
        else
        {
            checkFlags();                                                           // Check if button has been pressed, and if still pressed after 20ms
        }
    }
}

// Execute servo test
void servoTest(int port, int pin)
{
    // Servo info (Port 2) and PWM on Port 2.4, angle to be turned, initially turn anti-clockwise
    servoA.pwmPort = port;
    servoA.pwmPin = pin;
    servoA.speed = (PWM_SERVO_UPPER-PWM_SERVO_LOWER)/NUMBER_OF_ANGLES_CHECKED;
    servoA.direction = 0;

    // Setup device
    servoSetup(&servoA);                                                // Set up port and timing for the servo and set servo to turn clockwise
    setupTimerSchedule();                                               // Sets up scheduling for Time0 A0 interrupt which produces the 2ms clock cycle which program runs off of
    setupTimerRADAR();                                                  // Set up Timer1 A1

    // Disable flag
    flag.servoMove = 0;

    // Start servo at centre and wait 0.12s
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

// Flag that button has been pressed
#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR(void)
{
    flag.button = 1;
    __low_power_mode_off_on_exit();
    P1IFG &= ~(startButton.pin);                                                  // Assuming that BIT3 is the START_STOP_BUTTON and BIT4 is the
    return;
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

// Used for Ultrasonic capture compare - to get distance
// Also used to move servo - pwm signal determines position of servo
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer1_A1_ISR (void)
{
    switch(TA1IV)
    {
    case 0xA:                                                       // OVERFLOW
        TA1CTL &= ~TAIFG;                                           // Clear CCR1 flag
        break;
    case TA1IV_TACCR2:                                              // TA1CCR2 (No interrupt as used in servo PWM)
        TA1CCTL2 &= ~CCIFG;                                         // Clear CCR2 flag
        break;
    }
}

//==============================================================================
// End of File :  HardwareTest/Common.c