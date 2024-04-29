/*
Filename    : HardwareTest/Common.h
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
#ifndef COMMON_H
#define COMMON_H

// Local libraries
#include "IndicatorLED.h"
#include "Button.h"
#include "DCMotor.h"
#include "Servo.h"

// Available clock information
#define SMCK_FREQ      1000000
#define ACLK_FREQ      32768

// Scheduler information
#define CLOCK_USED_SCHEDULER    SMCK_FREQ
#define SECOND_COUNT            1000

// Scheduler control
#define TIMER_INC_MS    2                            // Scheduler interrupts period (2 ms)

// Information for drive motor speed
#define MOTOR_PWM_PERIOD    50                                          //PWM period of motors
#define SPEED_TOP           0.9*MOTOR_PWM_PERIOD                        //Top speed of the motors as percentage of PWM.
#define SPEED_SLOW          0.4*MOTOR_PWM_PERIOD                        //Slower speed when changing direction.
unsigned int movementStage;                                             // To go through each drive stage
unsigned int movementStageSteer;                                        // To go through each steer stage


// Other information for servo
#define CLOCK_USED_ULTRASONIC   SMCK_FREQ

// Servo PWM limits
#define NUMBER_OF_ANGLES_CHECKED 15

// Defines structure for program timing
struct Time {
    int sec;
    int ms;
};

// Defines structure containing all timings that events occur on
struct Scheduler {
    // Time when debouncing will finish
    struct Time debounce;

    // Time when drive motor PWM should change
    struct Time pwmMotorDrive;

    // Initially start car moving forward
    struct Time initiallyMoveForward;

    // Time when next movement should occur
    struct Time nextMovement;

    // Time when steer motor PWM should change
    struct Time pwmMotorSteer;

    // Time when next steer movement should occur
    struct Time nextMovementSteer;

    // Change angle of servo
    struct Time turnServo;
};

// Defines flags for hardware and software source to alert system to changes or readings
struct Flags {
    // When button pressed
    char button;

    // When button debounce finished
    char debounce;

    // When schedule timer ticks
    char timerA0;

    // When state of drive motor needs changed (e.g FORWARD to STOP)
    char motorDrive;

    // When state of steer motor needs changed (e.g FORWARD to STOP)
    char motorSteer;

    // When angle of servo should change
    char servoMove;
};

// Indicator LED
struct IndicateLED indicatorLED;

// Start Button
struct Button startButton;

// DC Motor info (On Port 1)
struct MotorDC motorDrive;
struct MotorDC motorSteer;

// Servo info (Port 2)
struct Servo servoA;

//Prototypes for common.c (background)
extern void setupTimerSchedule();
extern void timeIncrement(struct Time *time, int sec, int ms);
extern void setupTimerRADAR();

// Prototypes for main.c
extern void indicatorLEDTest(int port, int pin);
extern void startButtonTest(int port, int pin);
extern void initLEDPHY(int port, int pin);
extern void driveTest(int anode, int diode);
extern void steerTest(int anode, int diode);
extern void servoTest(int port, int pin);

#endif // COMMON_H

//==============================================================================
// End of File :  HardwareTests/Common.h