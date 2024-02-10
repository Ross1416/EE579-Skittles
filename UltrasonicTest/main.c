/*
Filename    : AutonomousCar/main.c
Author      : Samuel Kliskey
Project     : EE579 Skittles Knocking Over Project
Date        : 20/1/24
Description : Main script to control functions of the autonomous vehicle
              to as quickly as possible knock over black skittles and avoid
              white skittles.

--------------------------------------------------------------------------------
Functions Present
--------------------------------------------------------------------------------
Port1_ISR()
Timer0_A0_ISR()

main(void);
checkFlags();
checkSchedule();
setupPins();
setupScheduleTimer();
timeIncrement();

--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
20-JAN-2024 SARK created general structure
24-JAN-2024 SARK added DC motor / PWM functionality
06-FEB-2024 SARK added RI functionality for ultrasonic readings
--------------------------------------------------------------------------------
*/

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include <msp430.h>

//==============================================================================
// User-Defined Types
//------------------------------------------------------------------------------
//Define time variable
struct Time {
    int sec;
    int ms;
};

//Define Flags
struct flags {
    char button;
    char debounce;

    char stateChange;

    char timerA0;

    char motorA;
    char motorB;

    char ultraStart;
    char ultrasonicRead;

};

//Define Scheduler
struct Scheduler {
    struct Time debounce;

    struct Time stateChange;

    struct Time pwmMotorA;
    struct Time pwmMotorB;

    struct Time ultraStart;
};

//Define variable with PWM info
struct PWM {
    int sec;
    int ms;
    int aSec;
    int aMs;
    char state;
};

//Define variable which contains motor information
struct MotorDC {
    char direction; //Off (0), Forward (1) or Back(2)
    int pinA;
    int pinB;
    struct PWM pwm;
};

//Define variable containing ultrasonic sensor information
struct Ultrasonic{
    int distance;
    int time[2];
    char timeNumber;
    char trigPin;
    char echoPin;
};

//==============================================================================
// Function Prototypes
//------------------------------------------------------------------------------
__interrupt void Port1_ISR(void);
__interrupt void Timer1_A1_ISR (void);

int     main(void);
void    setupPins();
void    ultrasonicSetup(struct Ultrasonic ultra);

//==============================================================================
// MACRO
//------------------------------------------------------------------------------
#define isTime(X) ((currentTime.sec == X.sec) && (currentTime.ms == X.ms))

#define SMCK_FREQ      1000000
#define ACLK_FREQ      32768

#define CLOCK_USED     SMCK_FREQ
#define LOW_POWER      LPM0
#define SECOND_COUNT   1000
#define TIMER_INC_MS   2

//RGB colours (P2OUT |= RGB_XX;)
#define RGB_RED     0x02
#define RGB_GREEN   0x08
#define RGB_BLUE    0x20
#define RGB_PURPLE  0x22
#define RGB_CYAN    0x28
#define RGB_YELLOW  0xA
#define RGB_WHITE   0x2A
//RGB off command: (P2OUT &= ~0x2A;)

//Ultrasonic distance info
#define TOLERANCE   20
#define CANDIST     200

//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------
//Ultrasonic info (Port 2)
struct Ultrasonic ultraA = {0, {0, 0}, 0, BIT0, BIT2};
volatile int startingDistance;

volatile char flag = 0;

//==============================================================================
// Functions
//------------------------------------------------------------------------------

void ultrasonicSetup(struct Ultrasonic ultra)
{

    //echo
    P2SEL |= BIT2;

    //trig
    P2DIR |= BIT0;
    P2OUT = 0;

    //timer setup
    TA1CTL |= TASSEL_1;
    TA1CTL &= ~TAIFG;
    TA1CTL &= ~TAIE;

    TA1CCTL0 &= ~(CCIFG+CCIE);
    TA1CCTL2 &= ~CCIE;
    TA1CCTL2 &= ~CCIFG;

    TA1CCTL1 |= CM_3 + CCIS_1 + CAP + CCIE + SCS;
    TA1CCTL1 &= ~CCIFG;

    TA1CTL |= MC_2;
}

void trig()
{
    P2OUT |= BIT0;
    __delay_cycles(20);
    P2OUT &= ~BIT0;
}

//Flag that button has been pressed and take out of LPM
#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR(void)
{
    flag = 1;
    P1IFG &= ~BIT3;
    return;
}

//Ultrasonic capture compare
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer1_A1_ISR (void)
{
//    P1OUT |= BIT6;
    switch(TA1IV)
    {
    case 0xA:
        TA1CTL &= ~TAIFG;
        break;
    case 0x02:
        ultraA.time[ultraA.timeNumber] = TA1CCR1;
        ultraA.timeNumber++;
        if (ultraA.timeNumber==2)
        {
            ultraA.distance = ultraA.time[1]-ultraA.time[0];
            ultraA.timeNumber=0;
        }
        TA1CCTL1 &= ~CCIFG;
        break;
    case 0x04:
        TA1CCTL2 &= ~CCIFG;
        break;
    }
}


int main(void)
{
    //Stop watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    //Setup device
    setupPins();
    ultrasonicSetup(ultraA);

    //P2OUT &= ~0x2A;
    //P2OUT |= RGB_RED;

    __bis_SR_register(GIE);

    while(1)
    {
        if (flag == 1)
        {
            __delay_cycles(300);
            P2OUT |= BIT0;
            __delay_cycles(20);
            P2OUT &= ~BIT0;
            flag = 0;
        }
    }
}

void setupPins()
{
    //LEDs
    //P2DIR |= 0x2A;  //RGB
    //P2OUT &= ~0x2A;

    //Setup button for input and interrupt (P1.3)
    P1DIR &= ~BIT3;
    //Pull up so when pressed will go high to low
    P1REN |= BIT3;
    P1OUT |= BIT3;
    P1IE |= BIT3;    //Enable interrupts
    P1IES |= BIT3;   //High to Low transition
    P1IFG &= ~BIT3;  //Clear interrupts

    //Any sensor pins
    return;
}

//==============================================================================
// End of File : AutonomousCar/main.c
