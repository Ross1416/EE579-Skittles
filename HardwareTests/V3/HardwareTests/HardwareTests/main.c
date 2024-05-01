/*
Filename    : HardwareTest/HardwareComponentTests.c
Author      : Andrew Law
Project     : EE579 Skittles Knocking Over Project
Date        : 11/4/24
Description : Script to test all sensor components

            When a component is selected, a function
            is called which executes all required
            code to test the component in question

            At start, enter 1-9 depending on
            the component to be tested, which
            is used by a switch statement to
            determine which test function will be
            called.

            Usage - go through test 1 - 10, updating
            the macros (pin & port values) as you go along
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
29-APR-2024 andrewlaw9178 changed comments and added ultrasonic
01-MAY-2024 andrewlaw9178 changed comments, fixed ultrasonics and added IR
--------------------------------------------------------------------------------
*/

// Local libraries
#include "Common.h"

// Define Hardware ports
#define IND_LED_PORT            2               // Located at Port 2
#define IND_LED_PIN             BIT5            // Located at P2.5
#define START_BUTTON_PORT       1               // Located at Port 1
#define START_BUTTON_PIN        BIT3            // Located at P1.3
#define MOTOR_DRIVE_ANODE       BIT5            // Located at P1.4
#define MOTOR_DRIVE_CATHODE     BIT4            // Located at P1.5
#define MOTOR_STEER_ANODE       BIT6            // Located at P1.6
#define MOTOR_STEER_CATHODE     BIT7            // Located at P1.7
#define SERVO_PORT              2               // Located at Port 2
#define SERVO_PIN               BIT4            // Located at P2.4
#define ULTRA_SONAR_TRIG        BIT0            // Located at P2.0
#define ULTRA_SONAR_ECHO        BIT2            // Located at P1.2
#define ULTRA_SONAR_PORT        1               // Located at Port 1
#define ULTRA_LEFT_TRIG         BIT0            // Located at P2.0
#define ULTRA_LEFT_ECHO         BIT2            // Located at P2.2
#define ULTRA_LEFT_PORT         2               // Located at Port 2
#define ULTRA_RIGHT_TRIG        BIT0            // Located at P2.0
#define ULTRA_RIGHT_ECHO        BIT1            // Located at P2.1
#define ULTRA_RIGHT_PORT        2               // Located at Port 2
#define IR_PORT                 2               // Located at Port 2
#define IR_PIN                  BIT3            // Located at P2.3

void selectComponent(char component)
{
    if(component > '5' && component < '9')
    {
        isUltrasonicUsed('1');
    }
    else
    {
        isUltrasonicUsed('0');
    }

    switch(component)
    {
        case '1':
            // Indicator LED test has been selected
            indicatorLEDTest(IND_LED_PORT, IND_LED_PIN);
            break;
        case '2':
            // Start button test has been selected
            initLEDPHY(IND_LED_PORT, IND_LED_PIN);
            startButtonTest(START_BUTTON_PORT, START_BUTTON_PIN);
            break;
        case '3':
            // Drive test has been selected
            initLEDPHY(IND_LED_PORT, IND_LED_PIN);
            driveTest(MOTOR_DRIVE_ANODE, MOTOR_DRIVE_CATHODE);
            break;
        case '4':
            // Steer test has been selected
            steerTest(MOTOR_STEER_ANODE, MOTOR_STEER_CATHODE);
            break;
        case '5':
            // Servo test has been selected
            servoTest(SERVO_PORT, SERVO_PIN);
            break;
        case '6':
            // SONAR ultrasonic test has been selected
            initLEDPHY(IND_LED_PORT, IND_LED_PIN);
            sonarTest(ULTRA_SONAR_TRIG, ULTRA_SONAR_ECHO, ULTRA_SONAR_PORT);
            break;
        case '7':
            // Right ultrasonic test has been selected
            initLEDPHY(IND_LED_PORT, IND_LED_PIN);
            leftTest(ULTRA_LEFT_TRIG, ULTRA_LEFT_ECHO, ULTRA_LEFT_PORT);
            break;
        case '8':
            // Right ultrasonic test has been selected
            initLEDPHY(IND_LED_PORT, IND_LED_PIN);
            rightTest(ULTRA_RIGHT_TRIG, ULTRA_RIGHT_ECHO, ULTRA_RIGHT_PORT);
            break;
        case '9':
            // Infrared sensor test has been selected
            initLEDPHY(IND_LED_PORT, IND_LED_PIN);
            irTest(IR_PORT, IR_PIN);
            break;
        /*case '10':
            // Slide switch test has been selected
            initLEDPHY(IND_LED_PORT, IND_LED_PIN);
            initButtonPHY();
            irTest(IR_PORT, IR_PIN);
            break;*/
        default:
            // Handle invalid option
            break;
    }
}

int main(void)
{
    //Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    while(1)
    {
        // Enter 1 for indicator LED test
        // Enter 2 for start button test
        // Enter 3 for steer test
        // Enter 4 for drive test
        // Enter 5 for servo test
        // Enter 6 for SONAR ultrasonic test
        // Enter 7 for LEFT ultrasonic test
        // Enter 8 for RIGHT ultrasonic test
        // Enter 9 for Infrared test
        // Enter 10 for slide switch

        selectComponent('9');
    }
}

//==============================================================================
// End of File :  HardwareTests/HardwareComponentTests.c
