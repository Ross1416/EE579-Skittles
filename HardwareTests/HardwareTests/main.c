/*
Filename    : HardwareTest/HardwareComponentTests.c
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

// Local libraries
#include "Common.h"

// Define Hardware ports
#define IND_LED_PORT            2               // Located at Port 2
#define IND_LED_PIN             BIT5            // Located at P2.5
#define START_STOP_BUTTON_PORT  1               // Located at Port 1
#define START_STOP_BUTTON_PIN   BIT3            // Located at P1.3
#define MOTOR_DRIVE_ANODE       BIT5            // Located at P1.4
#define MOTOR_DRIVE_CATHODE     BIT4            // Located at P1.5
#define MOTOR_STEER_ANODE       BIT6            // Located at P1.6
#define MOTOR_STEER_CATHODE     BIT7            // Located at P1.7
#define SERVO_PORT              2               // Located at Port 2
#define SERVO_PIN               BIT4            // Located at P2.4
void selectComponent(char component)
{
    switch(component)
    {
        case '1':
            // Indicator LED test has been selected
            indicatorLEDTest(IND_LED_PORT, IND_LED_PIN);
            break;
        case '2':
            // Start button test has been selected
            initLEDPHY(IND_LED_PORT, IND_LED_PIN);
            startButtonTest(START_STOP_BUTTON_PORT, START_STOP_BUTTON_PIN);
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
            // Serv test has been selected
            servoTest(SERVO_PORT, SERVO_PIN);
            break;
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

        selectComponent('5');
    }
}

//==============================================================================
// End of File :  HardwareTests/HardwareComponentTests.c
