/*
Filename    : HardwareTest/Common.h
Author      : Andrew Law
Project     : EE579 Skittles Knocking Over Project
Date        : 11/4/24
Description : Protoypes and definitions required by
              code to test all components

            Defines all functions that will be used
            as well as all structures required.
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
02-MAY-2024 andrewlaw9178 added minor changes
--------------------------------------------------------------------------------
*/
#ifndef COMMON_H
#define COMMON_H

// Local libraries
#include "UserInterface.h"
#include "DCMotor.h"
#include "Servo.h"
#include "Ultrasonic.h"
#include "Infrared.h"

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

// Important Ultrasonic Info
#define SOUND_SPEED 343
#define CLOCK_USED_ULTRASONIC   SMCK_FREQ
unsigned char compUsed;

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

    // Time to start a left ultrasonic reading
    struct Time ultraLeftStart;

    // Time to start a right ultrasonic reading
    struct Time ultraRightStart;

    // Time to start a SONAR ultrasonic reading
    struct Time ultraSONARStart;
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

    // Read left wall ultrasonic when result available
    char ultraLeftWallRead;

    // Read right wall ultrasonic when result available
    char ultraRightWallRead;

    // Read SONAR ultrasonic when result avialable
    char ultraSONARRead;
};

// Indicator LED
struct IndicateLED redLED;
struct IndicateLED blueLED;

// Start Button
struct Button startButton;

// Slide switch
struct Switch sideSelect;

// DC Motor info
struct MotorDC motorDrive;
struct MotorDC motorSteer;

// Servo info
struct Servo servoA;

// SONAR Ultrasonic information
struct Ultrasonic ultraSONAR;

// Wall Ultrasonic information
struct Ultrasonic ultraLeft;
struct Ultrasonic ultraRight;

// Infrared information
struct Infrared irFront;

//Prototypes for common.c (background)
extern void setupTimerSchedule();
extern void timeIncrement(struct Time *time, int sec, int ms);
extern void setupTimerRADAR();
extern void setupTimerSONAR();
extern void initLEDsPHY(int redPort, int redPin, int bluePort, int bluePin);
void initStartButtonPHY(int port, int pin);

// Prototypes for main.c
extern void selectComponent(int component);
extern void indicatorLEDsTest(int redPort, int redPin, int bluePort, int bluePin);
extern void startButtonTest(int port, int pin);
extern void sideSelectSwitchTest(char port, int pin);
extern void driveTest(int anode, int diode);
extern void steerTest(int anode, int diode);
extern void servoTest(int port, int pin);
extern void sonarTest(int trig, int echoPin, int echoPort);
extern void leftTest(int trig, int echoPin, int echoPort);
extern void rightTest(int trig, int echoPin, int echoPort);
extern void irTest(int port, int pin);

#endif // COMMON_H

//==============================================================================
// End of File :  HardwareTests/Common.h
