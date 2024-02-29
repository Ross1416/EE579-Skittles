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
11-FEB-2024 RI added distance 2 pulse length macro
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
__interrupt void Timer0_A0_ISR(void);
__interrupt void Timer1_A1_ISR (void);

int     main(void);
void    checkFlags();
void    checkSchedule();
void    setupPins();
void    setupScheduleTimer();
void    timeIncrement(struct Time *time, int sec, int ms);
void	ultrasonicSetup(struct Ultrasonic ultra);
void    ultrasonicTrigger();
void 	motorOutput(struct MotorDC motor);
void 	motorSetup(struct MotorDC motor);

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

//States
#define START		0
#define GO			1
#define STOP		2

//RGB colours (P2OUT |= RGB_XX;)
#define RGB_RED     0x02
#define RGB_GREEN   0x08
#define RGB_BLUE    0x20
#define RGB_PURPLE  0x22
#define RGB_CYAN    0x28
#define RGB_YELLOW  0xA
#define RGB_WHITE   0x2A
//RGB off command: (P2OUT &= ~0x2A;)

//Motor direction commands
#define STRAIGHT    0
#define RIGHT       1
#define LEFT        2

#define OFF         0
#define FORWARD     1
#define BACK        2

//Ultrasonic distance info
//#define TOLERANCE   dist2pulse(5)
//#define CANDIST   	dist2pulse(20)

int TOLERANCE = dist2pulse(1);
int CANDIST = dist2pulse(10);

//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------
//Scheduling info
struct Time currentTime     =   {0, 0};    //Running count of time
struct Scheduler Schedule   =   {0};       //Schedule when events needing attended
struct flags flag           =   {0};       //Flag when something ready to be attended

//What car should be doing
char state	=	START;

//Motor info (On Port 1)
struct MotorDC motorA = {0, BIT4, BIT7, {0, 100, 0, 100, 1}};    //Drive
//struct MotorDC motorB = {0, BIT0, BIT1, {0, 100, 0, 100, 1}};   //Direction
struct MotorDC motorB = {0, BIT5, BIT6, {0, 100, 0, 100, 1}};   //Direction

//Ultrasonic info (Port 2)
struct Ultrasonic ultraA = {0, {0, 0}, 0, BIT0, BIT2};
volatile int startingDistance;


//==============================================================================
// Functions
//------------------------------------------------------------------------------

//Flag that button has been pressed and take out of LPM
#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR(void)
{
    flag.button = 1;
    __low_power_mode_off_on_exit();
    P1IFG &= ~BIT3;
    return;
}

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
            flag.ultrasonicRead = 1;
            ultraA.timeNumber=0;
            TA1CCTL1 |= CM_1;
        }
        else{
            TA1CCTL1 |= CM_2;
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
	motorSetup(motorA);
	motorSetup(motorB);
	ultrasonicSetup(ultraA);
    setupScheduleTimer();


    //Start / disable schedules
    Schedule.debounce.sec = 0;
    Schedule.debounce.ms = -1;

	Schedule.stateChange.sec = 0;
	Schedule.stateChange.ms = -1;

	Schedule.ultraStart.sec = 0;
	Schedule.ultraStart.ms = -1;

    //Start PWM of Motors
    timeIncrement(&Schedule.pwmMotorA, motorA.pwm.aSec, motorA.pwm.aMs);
    motorA.pwm.state = 1;
    flag.motorA = 1;
    motorA.direction = 0;

//    timeIncrement(&Schedule.pwmMotorB, motorB.pwm.aSec, motorB.pwm.aMs);
    motorB.pwm.state = 1;
    flag.motorB = 0;
    motorB.direction = 0;

    //P2OUT &= ~0x2A;
	//P2OUT |= RGB_RED;

    __bis_SR_register(GIE);

    while(1)
    {
        if (flag.timerA0)	//When timer ticks
        {
            checkSchedule();	//Check if time to do anything
            flag.timerA0 = 0;
        }
        else if(flag.debounce || flag.button || flag.motorA || flag.motorB ||
				flag.ultraStart || flag.ultrasonicRead || flag.stateChange)	//If something needs attended / add more flags as needed in condition
        {
			checkFlags();		//Deal with what needs attended
        }
        else
        {
        }
    }
}

void checkSchedule()
{
    int incSec = 0;
    int incMs = 0;

	if(isTime(Schedule.ultraStart))  //Debounce button check
    {
        flag.ultraStart = 1;
    }

	if(isTime(Schedule.stateChange))  //Debounce button check
    {
        flag.stateChange = 1;
        Schedule.stateChange.sec = 0;
        Schedule.stateChange.ms = -1;
    }

    if(isTime(Schedule.debounce))  //Debounce button check
    {
        flag.debounce = 1;
    }

	if(isTime(Schedule.pwmMotorA))  //When PWM for driving motor changes state
    {
		if(motorA.pwm.state)        //If PWM high
		{
		    //Find time to be low for based on length of on time and total PWM period
			incSec = motorA.pwm.sec-motorA.pwm.aSec;
			incMs = motorA.pwm.ms-motorA.pwm.aMs;
			timeIncrement(&(Schedule.pwmMotorA), incSec, incMs);
			motorA.pwm.state = 0;
			flag.motorA = 1;
		}
		else                        //If PWM low
		{
			timeIncrement(&Schedule.pwmMotorA, motorA.pwm.aSec, motorA.pwm.aMs);
			motorA.pwm.state = 1;
			if (motorA.pwm.aMs == 0) //If no on time to PWM
			{
			    flag.motorA = 0;
			}
			else
			{
			    flag.motorA = 1;
			}
		}
    }

	if(isTime(Schedule.pwmMotorB))  //When PWM for steering motor changes state
    {
		if(motorB.pwm.state)        //If PWM high
		{
			incSec = motorB.pwm.sec-motorB.pwm.aSec;
			incMs = motorB.pwm.ms-motorB.pwm.aMs;
			timeIncrement(&(Schedule.pwmMotorB), incSec, incMs);
			motorB.pwm.state = 0;
			flag.motorB = 1;
		}
		else                        //If PWM low
		{
			timeIncrement(&Schedule.pwmMotorB, motorB.pwm.aSec, motorB.pwm.aMs);
			motorB.pwm.state = 1;
            if (motorA.pwm.aMs == 0) //If no on time to PWM
            {
                flag.motorB = 0;
            }
            else
            {
                flag.motorB = 1;
            }
		}
    }

}

void checkFlags()
{
	if(flag.debounce)
	{
		if((P1IN & 0x08) != 0x08)   //Button still pressed after debounce
		{

			//Wait 2 seconds before starting to move
			if (state == 0)
			{
			    timeIncrement(&Schedule.stateChange, 2, 0);
			    ultrasonicTrigger();
			}
			if (state == 1)
			{
			    flag.stateChange = 1;
			}
		}
		Schedule.debounce.sec = 0;
		Schedule.debounce.ms = -1;
		flag.debounce = 0;
	}

	if (flag.ultraStart)
	{
		ultrasonicTrigger();
		flag.ultraStart = 0;
	}

	if (flag.ultrasonicRead)            //When reading from ultrasonic has returned
	{
		if(state == START)
		{
			startingDistance = ultraA.distance;
		}
		else if (state == GO)
		{
			timeIncrement(&Schedule.ultraStart, 0, 20);
		}

	    if(ultraA.distance < startingDistance-CANDIST)            //When reading is suddenly closer
	    {
			flag.stateChange = 1;
	    }
	    else if(ultraA.distance < startingDistance-TOLERANCE)     //When drifted closer to wall
	    {
	        motorB.direction = RIGHT;
	        P1OUT &= ~BIT5;
	        P1OUT |= BIT6;
	    }
	    else if(ultraA.distance > startingDistance+TOLERANCE)     //When drifted further from wall
	    {
	        motorB.direction = LEFT;
	        P1OUT |= BIT5;
	        P1OUT &= ~BIT6;
	    }
	    else       //If right distance from wall drive straight
	    {
	        motorB.direction = STRAIGHT;
	        P1OUT &= ~BIT5;
	        P1OUT &= ~BIT6;
	    }
	    flag.motorB = 1;
		flag.ultrasonicRead = 0;
	}

	if (flag.button)    //On button press start debounce
    {
        if(Schedule.debounce.sec == 0 && Schedule.debounce.ms == -1)   //If debounce not currently occurring
        {
			timeIncrement(&Schedule.debounce, 0, 20);  //20 ms debounce
        }
        flag.button = 0;
    }

	if (flag.motorA)    //If driving motor needs to change
    {
		motorOutput(motorA);
		flag.motorA = 0;
	}

	if (flag.motorB)    //If steering motor needs to change
    {
//		motorOutput(motorB);
		flag.motorB = 0;
	}

	if (flag.stateChange)    //On button press start debounce
    {
        state++;
		if (state == GO)
		{
			//P2OUT &= ~0x2A;
			//P2OUT |= RGB_GREEN;

			//Start driving forward
			motorA.direction = FORWARD;
	        flag.motorA = 1;
			
			//Initiate first ultrasonic reading
			ultrasonicTrigger();

		}
		else if (state == STOP)
		{
			//P2OUT &= ~0x2A;
			//P2OUT |= RGB_BLUE;

			motorA.direction = OFF;
	        flag.motorA = 1;
		}
		else
		{
			state = START;
		}
        flag.stateChange = 0;
    }
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

void ultrasonicSetup(struct Ultrasonic ultra)
{

    //echo
    P2SEL |= BIT2;

    //trig
    P2DIR |= BIT0;
    P2OUT = 0;

    //timer setup

    if(ULTRASONIC_CLOCK_USED == SMCK_FREQ)
    {
        TA1CTL |= TASSEL_2;
    }
    else
    {
        TA1CTL |= TASSEL_1;
    }


	TA1CTL &= ~TAIFG;
	TA1CTL &= ~TAIE;

	TA1CCTL0 &= ~(CCIFG+CCIE);
	TA1CCTL2 &= ~CCIE;
	TA1CCTL2 &= ~CCIFG;

	TA1CCTL1 |= CM_1 + CCIS_1 + CAP + CCIE + SCS;
	TA1CCTL1 &= ~CCIFG;

	TA1CTL |= MC_2;
}

void ultrasonicTrigger()
{
	P2OUT |= BIT0;
	__delay_cycles(20);
	P2OUT &= ~BIT0;
}


void motorSetup(struct MotorDC motor)
{
	P1DIR |= motor.pinA + motor.pinB;
	P1OUT &= ~motor.pinA;
	P1OUT &= ~motor.pinB;
}


void motorOutput(struct MotorDC motor)
{
	if (motor.pwm.state == 0)
	{
		P1OUT &= ~(motor.pinA + motor.pinB);
	}
	else
	{
		switch(motor.direction)
		{
		case 0:
			P1OUT &= ~(motor.pinA + motor.pinB);
			break;
		case 1:
			P1OUT |= motor.pinA;
			P1OUT &= ~motor.pinB;
			break;
		case 2:
			P1OUT |= motor.pinB;
			P1OUT &= ~motor.pinA;
			break;
		}
	}
}

//==============================================================================
// End of File : AutonomousCar/main.c
