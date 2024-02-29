/*
Filename    : AutonomousCar/main.c
Author      : Samuel Kliskey
Project     : EE579 Skittles Knocking Over Project
Date        : 20/1/24
Description : Main script to control servo tracking can

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
28-FEB-2024 SARK made to test servo moving in maximum arc
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

//Define variable containing servo motor information
struct Servo{
    int distance;
    int time[2];
    char timeNumber;
    char trigPin;
    char echoPin;
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
__interrupt void Timer0_A0_ISR(void);
__interrupt void Timer1_A1_ISR (void);

int     main(void);
void    checkFlags();
void    checkSchedule();
void    setupPins();
void    setupScheduleTimer();
void    timeIncrement(struct Time *time, int sec, int ms);
void    ultrasonicSetup(struct Ultrasonic ultra);
void    ultrasonicTrigger();
void    motorOutput(struct MotorDC motor);
void    motorSetup(struct MotorDC motor);
void	servoTurn();
void    setupUltraServoTimer();

//==============================================================================
// MACRO
//------------------------------------------------------------------------------
#define SOUND_SPEED 343

#define SMCK_FREQ      1000000
#define ACLK_FREQ      32768

#define CLOCK_USED     SMCK_FREQ
#define LOW_POWER      LPM0
#define SECOND_COUNT   1000
#define TIMER_INC_MS   2

#define ULTRASONIC_CLOCK_USED ACLK_FREQ

#define isTime(X) ((currentTime.sec == X.sec) && (currentTime.ms == X.ms))
#define dist2pulse(d) ((ULTRASONIC_CLOCK_USED/100)*d*2/SOUND_SPEED)     // Converts a distance (cm) to ultrasonic sensor output pulse length

#define WALLREADINGS  3

//RGB colours (P2OUT |= RGB_XX;)
#define RGB_RED     0x02
#define RGB_GREEN   0x08
#define RGB_BLUE    0x20
#define RGB_PURPLE  0x22
#define RGB_CYAN    0x28
#define RGB_YELLOW  0xA
#define RGB_WHITE   0x2A
//RGB off command: (P2OUT &= ~0x2A;)

//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------
//Scheduling info
struct Time currentTime     =   {0, 0};    //Running count of time
struct Scheduler Schedule   =   {0};       //Schedule when events needing attended
struct flags flag           =   {0};       //Flag when something ready to be attended

//What tracking should be doing
char stateServo  =  0;

//Ultrasonic info (Port 2)
struct Ultrasonic ultraA = {0, {0, 0}, 0, BIT0, BIT2};
volatile int startingDistance;

//Distance to wall values
int wallTolerance = dist2pulse(5);
int canDetectDist = dist2pulse(20);

//Wall drive control
//char turnState = STRAIGHT;
int  wallDistances[WALLREADINGS] = {0};
char turnStateTime = 0;

int  servoDirection = 1;



//==============================================================================
// Functions
//------------------------------------------------------------------------------
//Flag that time has passed, update current time and take out of LPM
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR(void)
{
    currentTime.ms += TIMER_INC_MS;
    if(currentTime.ms >= SECOND_COUNT)    //Increment timer
    {
        currentTime.ms -= SECOND_COUNT;
        if(++currentTime.sec == 60) currentTime.sec = 0;
    }

    flag.timerA0 = 1;

    __low_power_mode_off_on_exit();

    TA0CCTL0 &= ~CCIFG;
}

//Ultrasonic capture compare
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer1_A1_ISR (void)
{
    switch(TA1IV)
    {
    case 0xA:
        TA1CTL &= ~TAIFG;
        break;
    case 0x02:
        ultraA.time[ultraA.timeNumber] = TA1CCR1;
        ultraA.timeNumber++;
        if (ultraA.timeNumber==2)       //After up/down edges of feedback
        {
            ultraA.distance = ultraA.time[1]-ultraA.time[0];
            if (ultraA.distance < 0)    //When timer wrapped
            {
                ultraA.distance += 0xFFFF;
            }
            flag.ultrasonicRead = 1;
            ultraA.timeNumber=0;
            TA1CCTL1 |= CM_1;   //Capture on rising edge
        }
        else
        {
            TA1CCTL1 |= CM_2;   //Capture on falling edge
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
	P2DIR |= 0x2A;
    ultrasonicSetup(ultraA);
    setupScheduleTimer();
    setupUltraServoTimer();

    Schedule.ultraStart.sec = 0;
    Schedule.ultraStart.ms = 20;

    __bis_SR_register(GIE);

    while(1)
    {
        if (flag.timerA0)   //When timer ticks
        {
            checkSchedule();    //Check if time to do anything
            flag.timerA0 = 0;
        }
        else if(flag.ultraStart || flag.ultrasonicRead) //If something needs attended / add more flags as needed in condition
        {
            checkFlags();       //Deal with what needs attended
        }
        else
        {
        }
    }
}

void checkSchedule()
{

    if(isTime(Schedule.ultraStart))  //Debounce button check
    {
        flag.ultraStart = 1;
    }
}

void checkFlags()
{
    char i = 0;



    if (flag.ultraStart)
    {
        ultrasonicTrigger();
        flag.ultraStart = 0;
        timeIncrement(&Schedule.ultraStart, 0, 20);
    }

    if (flag.ultrasonicRead)            //When reading from ultrasonic has returned
    {
        for(i = 1; i < WALLREADINGS; i++)
        {
            wallDistances[i] = wallDistances[i - 1];
        }
        wallDistances[0] = ultraA.distance;

        flag.ultrasonicRead = 0;
		
		//timeIncrement(&Schedule.ultraStart, 0, 20);
		
		servoTurn();
    }

    //if (flag.stateChange)    //On button press start debounce
    //{
    //    state++;
    //    if (state == GO)
    //    {
    //        //P2OUT &= ~0x2A;
    //        //P2OUT |= RGB_GREEN;
	//
    //        //Start driving forward
    //        motorA.direction = FORWARD;
    //        flag.motorA = 1;
	//
    //        //Initiate first ultrasonic reading
    //        ultrasonicTrigger();
	//
    //    }
    //    else if (state == STOP)
    //    {
    //        //P2OUT &= ~0x2A;
    //        //P2OUT |= RGB_BLUE;
	//
    //        motorA.direction = OFF;
    //        flag.motorA = 1;
    //    }
    //    else
    //    {
    //        state = START;
    //    }
    //    flag.stateChange = 0;
    //}
}

void timeIncrement(struct Time *time, int sec, int ms)
{
    //Always on next increment amount
    ms += TIMER_INC_MS;
    ms -= ms % TIMER_INC_MS;

    time->ms = currentTime.ms + ms;
    while(time->ms >= SECOND_COUNT)
    {
        time->ms -= SECOND_COUNT;
        sec++;
    }

    time->sec = currentTime.sec + sec;
    while(time->sec >= 60)
    {
        time->sec -= 60;
    }
}

void setupScheduleTimer()
{
    if(CLOCK_USED == SMCK_FREQ)
    {
        TA0CTL |= TASSEL_2 + MC_1;              //SMCK  so f = 1 MHz, operating in up mode to TA0CCR0
    }
    else
    {
        TA0CTL |= TASSEL_1 + MC_1;              //ACLK  so f = 32768 Hz, operating in up mode to TA0CCR0
    }
    TA0CCTL0 |= 0x10;                       //Interrupt occurs when TA0R reaches TA0CCR0
    TA0CCR0 = CLOCK_USED*TIMER_INC_MS/1000; //Set the count to value (5 ms) f*5ms = 5000
    TA0CCTL0 &= ~CCIFG;                     //Clear interrupt flags

    return;
}

void setupUltraServoTimer()
{
    if(ULTRASONIC_CLOCK_USED == SMCK_FREQ)
    {
        TA1CTL |= TASSEL_2;			// f = 1 MHz
    }
    else
    {
        TA1CTL |= TASSEL_1;			// f = 32.768 kHz
    }

    TA1CTL &= ~TAIFG;	//Clear interrupt
    TA1CTL &= ~TAIE;	//Disable interrupt on timer edge

    TA1CCTL0 &= ~(CCIFG+CCIE);	//Clear and disable timer overflow interrupt
    TA1CCTL2 &= ~CCIE;			//Clear and disable timer2 interrupt
    TA1CCTL2 &= ~CCIFG;

	//Ultrasonic settings
	//Capture rising, input B (P2.2), capture mode, enable interrupt, sync to clock rising edge
    TA1CCTL1 |= CM_1 + CCIS_1 + CAP + CCIE + SCS;
    TA1CCTL1 &= ~CCIFG;		//Clear ultrasonic interrupt
	
	//Servo settings
	TA1CCR0 = 655; //20 ms at ACLK (Max count value
	TA1CCR2 = 49;  //1.5 ms
	//Output B (P2.4), Reset/Set output
	TA1CCTL2 &= ~(CCIE + CCIFG + CAP);		//Clear/disable Timer1.2 interrupt and set compare mode
	TA1CCTL2 |= CCIS_0 + OUTMOD_7;
	
	//Servo pin
	P2DIR |= BIT4;
	P2SEL |= BIT4;
	
	//Start counting to TA1CCR0 (655)
    TA1CTL |= MC_1;
}

void ultrasonicSetup(struct Ultrasonic ultra)
{
    //Echo pin
    P2SEL |= BIT2;

    //Trig pin
    P2DIR |= BIT0;
    P2OUT = 0;
}

void ultrasonicTrigger()
{
    P2OUT |= BIT0;
    __delay_cycles(20);
    P2OUT &= ~BIT0;
}

void servoTurn()
{
	if (TA1CCR2 >= 80)
	{
		servoDirection = -1;
		P2OUT &= ~RGB_WHITE;
		P2OUT |= RGB_BLUE;
	}
	else if (TA1CCR2 <= 25)
	{
		servoDirection = 1;
		P2OUT &= ~RGB_WHITE;
		P2OUT |= RGB_GREEN;
	}
	TA1CCR2 += servoDirection * 2;
	
}

//==============================================================================
// End of File : AutonomousCar/main.c
