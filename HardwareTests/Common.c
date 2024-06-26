/*
Filename    : HardwareTest/Common.c
Author      : Andrew Law
Project     : EE579 Skittles Knocking Over Project
Date        : 11/4/24
Description : Script to test all sensor components

            There are functions to set up component,
            set up timers and port interrupts, and to
            test the component itself.
--------------------------------------------------------------------------------
Functions Present
--------------------------------------------------------------------------------
setupTimerSchedule()
timeIncrement()
setupTimerRADAR()
indicatorLEDTest()
startButtonTest()
initLEDPHY()
driveTest()
steerTest()
servoTest()
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
22-APR-2024 andrewlaw9178 created to have hardware component test in 1 location
25-APR-2024 andrewlaw9178 fixed LED and button and added drive tests
26-APR-2024 andrewlaw9178 added steer and servo test
29-APR-2024 andrewlaw9178 changed comments and added ultrasonic
01-MAY-2024 andrewlaw9178 changed comments, fixed ultrasonics and added IR and
            side switches
02-MAY-2024 andrewlaw9178 edited comments and updated drive code
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

// Used to determine which test is being executed
unsigned char compUsed;

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

// Set up Timer1 A1 interrupt for SONAR ultrasonic
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

// Set up Timer1 A1 interrupt for LEFT RIGHT ultrasonics
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

    // Get car to next drive movement
    if(isTime(Schedule.nextMovement))
    {
        if(movementStage == 0)
        {
            // Stop car
            motorDrive.direction = OFF;
            flag.motorDrive = 1;                                                // Used to change the state of the drive motor
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
            indicatorLEDOff(&blueLED);                                          // Turn the blue indicator LED off
            indicatorLEDOn(&redLED);                                            // Turn the red  indicator LED on
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

            indicatorLEDOn(&blueLED);                                           // Turn the blue indicator LED on
            indicatorLEDOff(&redLED);                                           // Turn the red  indicator LED off
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
        timeIncrement(&Schedule.nextMovement, 3, 0);
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
            // Turn RIGHT
            motorSteer.direction = RIGHT;
            flag.motorSteer = 1;
        }
        else if(movementStageSteer == 3)
        {
            // Turn STRAIGHT
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
        timeIncrement(&Schedule.nextMovementSteer, 3, 0);
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

    // Check if time to get SONAR ultrasonic reading
    if (isTime(Schedule.ultraSONARStart))
    {
        ultrasonicTrigger(&ultraSONAR);
        indicatorLEDOn(&redLED);                                        // Turn on red indicator LED
        indicatorLEDOn(&blueLED);                                       // Turn on blue indicator LED
        // Disable schedule
        Schedule.ultraSONARStart.sec = 0;
        Schedule.ultraSONARStart.ms = -1;
    }

    // Check if time to get left ultrasonic reading
    if (isTime(Schedule.ultraLeftStart))
    {
        ultrasonicTrigger(&ultraLeft);
        indicatorLEDOn(&redLED);                                       // Turn on red indicator LED
        // Disable schedule
        Schedule.ultraLeftStart.sec = 0;
        Schedule.ultraLeftStart.ms = -1;
    }

    // Check if time to get right ultrasonic reading
    if (isTime(Schedule.ultraRightStart))
    {
        ultrasonicTrigger(&ultraRight);
        indicatorLEDOn(&blueLED);                                        // Turn on blue indicator LED
        // Disable schedule
        Schedule.ultraRightStart.sec = 0;
        Schedule.ultraRightStart.ms = -1;
    }
}

// Checks if button has been pressed or if debouncing has occured
void checkFlags()
{
    // Debounce wait has finished
    if(flag.debounce)
    {
        readButton(&startButton);
        if(startButton.val)                                             // If start button still pressed after debounce
        {
            indicatorLEDToggle(&redLED);                           // Toggle red LED
            indicatorLEDToggle(&blueLED);                          // Toggle blue LED
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

    // SONAR ultrasonic has reading ready
    if (flag.ultraSONARRead)
    {
        indicatorLEDOff(&redLED);           // Turn off red indicator LED
        indicatorLEDOff(&blueLED);          // Turn off blue indicator LED
        flag.ultraSONARRead = 0;
        // Schedule next reading from left ultrasonic
        timeIncrement(&(Schedule.ultraSONARStart), 0, 120);
    }

    // Left wall ultrasonic has reading ready
    if (flag.ultraLeftWallRead)
    {
        indicatorLEDOff(&redLED);         // Turn off blue indicator LED
        flag.ultraLeftWallRead = 0;
        // Schedule next reading from left ultrasonic
        timeIncrement(&(Schedule.ultraLeftStart), 0, 120);
    }

    // Right wall ultrasonic has reading ready
    if (flag.ultraRightWallRead)
    {
        indicatorLEDOff(&blueLED);         // Turn off blue indicator LED
        flag.ultraRightWallRead = 0;
        // Schedule next reading from right ultrasonic
        timeIncrement(&(Schedule.ultraRightStart), 0, 120);
    }
}

// Execute indicator LED test
void indicatorLEDsTest(int redPort, int redPin, int bluePort, int bluePin)
{
    // Indicator LEDs information
    redLED.port = redPort;
    redLED.pin = redPin;
    blueLED.port = bluePort;
    blueLED.pin = bluePin;

    compUsed = '0';

    // Setup indicator LEDs
    indicatorLEDSetup(&redLED);                                         // Set up the red indicator LED
    indicatorLEDSetup(&blueLED);                                        // Set up the blue indicator LED
    
    while(1)
    {
        indicatorLEDOn(&redLED);                                        // Turn on the red indicator LED
        indicatorLEDOn(&blueLED);                                       // Turn on the blue indicator LED
    }
}

// Initalise indicator LED for all tests bar indicator LED test
void initLEDsPHY(int redPort, int redPin, int bluePort, int bluePin)
{
    // Indicator LEDs information
    redLED.port = redPort;
    redLED.pin = redPin;
    blueLED.port = bluePort;
    blueLED.pin = bluePin;
}

// Execute start button test
void startButtonTest(int port, int pin)
{
    // Start button information
    startButton.port = port;
    startButton.pin = pin;
    startButton.val = 0;

    compUsed = '1';

    // Setup start button, indicator LEDs and timer
    setupButton(&startButton);                                          // Selects start button input and sets up port 1 interrupt for button
    indicatorLEDSetup(&redLED);                                         // Set up the red indicator LED
    indicatorLEDSetup(&blueLED);                                        // Set up the blue indicator LED
    setupTimerSchedule();                                               // Sets up scheduling for Time0 A0 interrupt which produces the 2ms clock cycle which program runs off of

    // Disable schedules
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

// Initalise indicator LED for all tests bar indicator LED test
void initStartButtonPHY(int port, int pin)
{
    // Start button information
    startButton.port = port;
    startButton.pin = pin;
    startButton.val = 0;
}

// Execute slide switch test
void sideSelectSwitchTest(char port, int pin)
{
    // Set slide switch information
    sideSelect.port = port;
    sideSelect.pin = pin;
    sideSelect.val = 0;

    compUsed = '2';

    // Setup slide switch, start button and indicator LED
    setupSwitch(&sideSelect);                                           // Selects side switch
    indicatorLEDSetup(&redLED);                                         // Set up the red indicator LED
    indicatorLEDSetup(&blueLED);                                        // Set up the blue indicator LED

        //Enable global interrupts
    __bis_SR_register(GIE);


    //Main loop
    while(1)
    {
        readSwitch(&sideSelect);
        if(sideSelect.val == 1)                                 // If side select switch is Left
        {
            indicatorLEDOn(&blueLED);                           // Turn blue LED on
            indicatorLEDOff(&redLED);                           // Turn red LED off
        }
        else
        {
            indicatorLEDOff(&blueLED);                          // Turn blue LED off
            indicatorLEDOn(&redLED);                            // Turn red LED on
        }
    }
}

// Execute drive test
void driveTest(int anode, int cathode )
{
    // DC motor info and sets a ms period & duty cycle, initializing to HIGH
    motorDrive.direction = 0;
    motorDrive.pinA = anode;
    motorDrive.pinB = cathode;
    motorDrive.pwm.sec = 0;
    motorDrive.pwm.ms = MOTOR_PWM_PERIOD;
    motorDrive.pwm.aSec = 0;
    motorDrive.pwm.aMs = SPEED_TOP;

    compUsed = '3';

    // Setup H-Bridge and rear DC motor
    motorSetup(&motorDrive);                                            // Set up ports and timing for the motor for driving
    indicatorLEDSetup(&blueLED);                                        // Set up the blue indicator LED
    indicatorLEDSetup(&redLED);                                         // Set up the red indicator LED
    setupTimerSchedule();                                               // Sets up scheduling for Time0 A0 interrupt which produces the 2ms clock cycle which program runs off of

    // Start drive DC motor PWM schedules but set its output to do nothing
    timeIncrement(&Schedule.pwmMotorDrive, motorDrive.pwm.aSec, motorDrive.pwm.aMs); // Sets time when duty cycle occurs to set PWM LOW/HIGH
    motorDrive.pwm.state = 1;                                           // Sets PWM signal initally to HIGH
    flag.motorDrive = 1;
    motorDrive.direction = OFF;                                         // At the beginning the car will stay stationary

    // Set half second timer for motor to drive car forward
    timeIncrement(&Schedule.nextMovement, 3, 0);
    movementStage = 7;

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
    // DC Motor info and sets a ms period & duty cycle, initializing to HIGH
    motorSteer.direction = 0;
    motorSteer.pinA = anode;
    motorSteer.pinB = cathode;
    motorSteer.pwm.sec = 0;
    motorSteer.pwm.ms = MOTOR_PWM_PERIOD;
    motorSteer.pwm.aSec = 0;
    motorSteer.pwm.aMs = MOTOR_PWM_PERIOD;
    motorSteer.pwm.state = 1;

    compUsed = '4';

    // Setup H-Bridge and front DC motor
    motorSetup(&motorSteer);                                                            // Set up ports and timing for the motor for driving
    setupTimerSchedule();                                                               // Sets up scheduling for Time0 A0 interrupt which produces the 2ms clock cycle which program runs off of

    // Start steer DC motor PWM schedules but set its output to do nothing
    timeIncrement(&Schedule.pwmMotorSteer, motorSteer.pwm.aSec, motorSteer.pwm.aMs);
    motorSteer.pwm.state = 1;                                                           // Sets PWM signal to HIGH
    motorSteer.direction = STRAIGHT;                                                    // At the beginning the car will point STRAIGHT
    flag.motorSteer = 1;

    // Set half second timer for motor to drive car forward
    timeIncrement(&Schedule.nextMovementSteer, 3, 0);
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
    // Servo info and PWM on Port 2.4, angle to be turned, initially turn anti-clockwise
    servoA.pwmPort = port;
    servoA.pwmPin = pin;
    servoA.speed = (PWM_SERVO_UPPER-PWM_SERVO_LOWER)/NUMBER_OF_ANGLES_CHECKED;
    servoA.direction = 0;

    compUsed = '5';

    // Setup servo
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

// Execute SONAR ultrasonic test
extern void sonarTest(int trig, int echoPin, int echoPort)
{
    // Set SONAR ultrasonic info
    //ultraSONAR = {0, {0, 0}, 0, SONAR_TRIG, SONAR_ECHO, 1};
    ultraSONAR.distance = 0;
    ultraSONAR.time[0] = 0;
    ultraSONAR.time[1] = 0;
    ultraSONAR.timeNumber = 0;
    ultraSONAR.trigPin = trig;
    ultraSONAR.echoPin = echoPin;
    ultraSONAR.port = echoPort;

    compUsed = '6';

    // Setup device
    ultrasonicSetup(&ultraSONAR);                                       // Set up pin for the left ultrasonic
    indicatorLEDSetup(&blueLED);                                        // Set up the blue indicator LED
    indicatorLEDSetup(&redLED);                                         // Set up the red indicator LED
    setupTimerSchedule();                                               // Sets up scheduling for Time0 A0 interrupt which produces the 2ms clock cycle which program runs off of
//    P1DIR |= BIT0;
//    P1SEL &= ~(BIT0);
//    P1OUT &= ~(BIT0+BIT2);
    // Disable schedule
    Schedule.ultraSONARStart.sec = 0;
    Schedule.ultraSONARStart.ms = -1;

    // Start getting first measurement after 1seconds
    timeIncrement(&(Schedule.ultraSONARStart), 1, 0);

    // Disable flag
    flag.ultraSONARRead = 0;

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

// Execute LEFT ultrasonic test
extern void leftTest(int trig, int echoPin, int echoPort)
{
    // Set LEFT ultrasonic info
    ultraLeft.distance = 0;
    ultraLeft.time[0] = 0;
    ultraLeft.time[1] = 0;
    ultraLeft.timeNumber = 0;
    ultraLeft.trigPin = trig;
    ultraLeft.echoPin = echoPin;
    ultraLeft.port = echoPort;

    compUsed = '7';

    // Setup device
    ultrasonicSetup(&ultraLeft);                                        // Set up pin for the left ultrasonic
    indicatorLEDSetup(&redLED);                                        // Set up the blue indicator LED
    setupTimerSchedule();                                               // Sets up scheduling for Time0 A0 interrupt which produces the 2ms clock cycle which program runs off of
    setupTimerSONAR();

    // Disable schedule
    Schedule.ultraLeftStart.sec = 0;
    Schedule.ultraLeftStart.ms = -1;

    // Start getting first measurement after 1 seconds
    timeIncrement(&(Schedule.ultraLeftStart), 1, 0);

    // Disable flag
    flag.ultraLeftWallRead = 0;

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

// Execute right ultrasonic test
extern void rightTest(int trig, int echoPin, int echoPort)
{
    // Set right ultrasonic info
    ultraRight.distance = 0;
    ultraRight.time[0] = 0;
    ultraRight.time[1] = 0;
    ultraRight.timeNumber = 0;
    ultraRight.trigPin = trig;
    ultraRight.echoPin = echoPin;
    ultraRight.port = echoPort;

    compUsed = '8';

    // Setup right ultrasonic
    ultrasonicSetup(&ultraRight);                                       // Set up pin for the right ultrasonic
    indicatorLEDSetup(&blueLED);                                        // Set up the blue indicator LED
    setupTimerSchedule();                                               // Sets up scheduling for Time0 A0 interrupt which produces the 2ms clock cycle which program runs off of
    setupTimerSONAR();

    // Disable schedule
    Schedule.ultraRightStart.sec = 0;
    Schedule.ultraRightStart.ms = -1;

    // Start getting first measurement after 1 seconds
    timeIncrement(&(Schedule.ultraRightStart), 1, 0);

    // Disable flag
    flag.ultraRightWallRead = 0;

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

// Execute infrared test
void irTest(int port, int pin)
{
    // Set infrared info
    irFront.port = port;
    irFront.pin = pin;
    irFront.colour = 2;

    compUsed = '9';

    // Setup infrared
    setupIR(&irFront);                                                  // Selects IR input
    indicatorLEDSetup(&blueLED);                                        // Set up the blue indicator LED
    indicatorLEDSetup(&redLED);                                         // Set up the red indicator LED
    while (1)
    {
        readIR(&irFront);
        if(irFront.colour == 1)
        {
            indicatorLEDOn(&blueLED);                                   // Turn on indicator blue LED
            indicatorLEDOn(&redLED);                                    // Turn on indicator red LED
        }
        else if(irFront.colour == 0)
        {
            indicatorLEDOff(&blueLED);                                   // Turn off indicator blue LED
            indicatorLEDOff(&redLED);                                    // Turn off indicator red LED
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

    if(compUsed == '6' || compUsed == '7' || compUsed == '8')
    {
        // If wrap occurs during ultrasonic pulse
        if (ultraSONAR.timeNumber != 0)
        {
            ultraSONAR.time[1] += TA0CCR0;
        }
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

//        case 0xA:   break;
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
// Also used to move servo - pwm signal determines position of servo
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
// End of File :  HardwareTest/Common.c
