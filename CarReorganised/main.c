/*
Filename    : CarReorganised/main.c
Author      : Samuel Kliskey
Project     : EE579 Skittles Knocking Over Project
Date        : 31/3/24
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
setupButton();
setupScheduleTimer();
timeIncrement();

--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
31-MAR-2024 SARK created to restructure AutonomousCar project
01-APR-2024 SARK continued to restructure to make readable
--------------------------------------------------------------------------------
*/

//==============================================================================
// External Files
//------------------------------------------------------------------------------
//External
#include <msp430.h>

//Local
#include "DCMotor.h"
#include "Servo.h"
#include "Ultrasonic.h"

//==============================================================================
// User-Defined Types
//------------------------------------------------------------------------------
//Define time structure
struct Time {
    int sec;
    int ms;
};

//Contains flags from various hardware sources to inform changes or readings
struct flags {
    //When button is pressed
    char button;

    //When button debounce has finished
    char debounce;

    //When the nature of the car changes
    char stateChange;

    //When schedule timer ticks
    char timerA0;

    //When motor needs to change state
    char motorDrive;
    char motorSteer;

    //To trigger ultrasonic
    char ultraStart;

    //To read ultrasonic when result available
    char ultrasonicRead;
};

//Structure contains all timings that events occur on
struct Scheduler {
    //Time at which debouncing finishes
    struct Time debounce;

    //Time at which car state should change
    struct Time stateChange;

    //Time at which motor PWM should change
    struct Time pwmmotorDrive;
    struct Time pwmmotorSteer;

    //Time at which ultrasonic should be triggered
    struct Time ultraStart;
};

//==============================================================================
// Function Prototypes
//------------------------------------------------------------------------------
//Interrupt Service Routines
__interrupt void Port1_ISR(void);
__interrupt void Timer0_A0_ISR(void);
__interrupt void Timer1_A1_ISR (void);

//Main body of code
int     main(void);
void    checkFlags();
void	stateControl();

//Hardware setup
void    setupButton();
void    setupTimerSchedule();
void 	setupTimerRADAR();

//Scheduler functions
void    checkSchedule();
void    timeIncrement(struct Time *time, int sec, int ms);

//Other Functionality
void    alignToWall();

//==============================================================================
// MACRO
//------------------------------------------------------------------------------
//Available clock information
#define SMCK_FREQ      1000000
#define ACLK_FREQ      32768

// Important Ultrasonic Info
#define SOUND_SPEED 343
#define CLOCK_USED_ULTRASONIC   ACLK_FREQ
#define dist2pulse(d)           ((CLOCK_USED_ULTRASONIC/100)*d*2/SOUND_SPEED)     // Converts a distance (cm) to ultrasonic sensor output pulse length

//Scheduler information
#define CLOCK_USED_SCHEDULER    SMCK_FREQ
#define SECOND_COUNT   			1000		//1000 ms = 1s
#define TIMER_INC_MS   			2			//Schedule clock ticks every 2 ms
#define isTime(X) ((currentTime.sec == X.sec) && (currentTime.ms == X.ms))

//Car States
#define START       0
#define GO          1
#define STOP        2

//Wall alignment
#define WALLREADINGS  	3	//Number of wall readings
#define STRAIGHT    	0
#define AWAY        	1
#define CLOSE       	2
#define STRAIGHTEN  	3

//RGB colours (P2OUT |= RGB_XX;)
//RGB off command: (P2OUT &= ~0x2A;)
#define RGB_RED     0x02
#define RGB_GREEN   0x08
#define RGB_BLUE    0x20
#define RGB_PURPLE  0x22
#define RGB_CYAN    0x28
#define RGB_YELLOW  0xA
#define RGB_WHITE   0x2A

//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------
//Scheduling info
struct Time currentTime     =   {0, 0};    //Running count of time
struct Scheduler Schedule   =   {0};       //Schedule when events needing attended
struct flags flag           =   {0};       //Flag when something ready to be attended

//What car should be doing
char state  =   START;

//Motor info (On Port 1)
struct MotorDC motorDrive = {0, BIT4, BIT7, {0, 100, 0, 100, 1}};
struct MotorDC motorSteer = {0, BIT5, BIT6, {0, 100, 0, 100, 1}};

//Ultrasonic info (Port 2)
struct Ultrasonic ultraWall = {0, {0, 0}, 0, BIT0, BIT2};

//Servo info (Port 2)
struct Servo servoA = {BIT4, 1};

//Wall alignment
volatile int startingDistance;			//Initial distance to maintain to wall
int wallTolerance = dist2pulse(5);		//Distance +- correct distance from wall
int canDetectDist = dist2pulse(20);		//Distance closer then wall
char turnState = STRAIGHT;				//Wall alignment turning instruction
int  wallDistances[WALLREADINGS] = {0};	//Distance readings to wall
char turnStateTime = 0;					//Time spent turning to correct for wall


//==============================================================================
// Functions
//------------------------------------------------------------------------------

//Flag that button has been pressed
#pragma vector=PORT1_VECTOR
__interrupt void Port1_ISR(void)
{
    flag.button = 1;
    __low_power_mode_off_on_exit();
    P1IFG &= ~BIT3;
    return;
}

//Flag that time has passed, update current time
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
        ultraWall.time[ultraWall.timeNumber] = TA1CCR1;
        ultraWall.timeNumber++;
        if (ultraWall.timeNumber==2)       //After up/down edges of feedback
        {
            ultraWall.distance = ultraWall.time[1]-ultraWall.time[0];
            if (ultraWall.distance < 0)    //When timer wrapped
            {
                ultraWall.distance += 0xFFFF;
            }
            flag.ultrasonicRead = 1;
            ultraWall.timeNumber=0;
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
    setupButton();
    motorSetup(&motorDrive);
    motorSetup(&motorSteer);
    ultrasonicSetup(&ultraWall);
	servoSetup(&servoA);
	setupTimerRADAR();
    setupTimerSchedule();

    //Initial schedules
    Schedule.debounce.sec = 0;
    Schedule.debounce.ms = -1;

    Schedule.stateChange.sec = 0;
    Schedule.stateChange.ms = -1;

    Schedule.ultraStart.sec = 0;
    Schedule.ultraStart.ms = -1;

    timeIncrement(&Schedule.pwmmotorDrive, motorDrive.pwm.aSec, motorDrive.pwm.aMs);
    motorDrive.pwm.state = 1;
    flag.motorDrive = 1;
    motorDrive.direction = 0;

    timeIncrement(&Schedule.pwmmotorSteer, motorSteer.pwm.aSec, motorSteer.pwm.aMs);
    motorSteer.pwm.state = 1;
    flag.motorSteer = 1;
    motorSteer.direction = 0;

	//Enable global interrupts
    __bis_SR_register(GIE);

	//Main loop
    while(1)
    {
        if (flag.timerA0)   //When timer ticks
        {
            checkSchedule();    //Check if time to do anything
            flag.timerA0 = 0;
        }
        else
        {
            checkFlags();       //Deal with what needs attended
			stateControl();
        }
    }
}

void checkSchedule()
{
    int incSec = 0;
    int incMs = 0;

    if(isTime(Schedule.ultraStart))  //Time to start ultrasonic reading
    {
        flag.ultraStart = 1;
    }

    if(isTime(Schedule.stateChange))  //Time for car to change behaviour check
    {
        flag.stateChange = 1;
        Schedule.stateChange.sec = 0;
        Schedule.stateChange.ms = -1;
    }

    if(isTime(Schedule.debounce))  //Debounce button check
    {
        flag.debounce = 1;
    }

    if(isTime(Schedule.pwmmotorDrive))  //When PWM for driving motor changes state
    {
        if(motorDrive.pwm.state)        //If PWM high
        {
            //Find time to be low for based on length of on time and total PWM period
            incSec = motorDrive.pwm.sec-motorDrive.pwm.aSec;
            incMs = motorDrive.pwm.ms-motorDrive.pwm.aMs;
            timeIncrement(&(Schedule.pwmmotorDrive), incSec, incMs);
            motorDrive.pwm.state = 0;
            flag.motorDrive = 1;
        }
        else                        //If PWM low
        {
            timeIncrement(&Schedule.pwmmotorDrive, motorDrive.pwm.aSec, motorDrive.pwm.aMs);
            motorDrive.pwm.state = 1;
            if (motorDrive.pwm.aMs == 0) //If no on time to PWM
            {
                flag.motorDrive = 0;
            }
            else
            {
                flag.motorDrive = 1;
            }
        }
    }

    if(isTime(Schedule.pwmmotorSteer))  //When PWM for steering motor changes state
    {
        if(motorSteer.pwm.state)        //If PWM high
        {
            incSec = motorSteer.pwm.sec-motorSteer.pwm.aSec;
            incMs = motorSteer.pwm.ms-motorSteer.pwm.aMs;
            timeIncrement(&(Schedule.pwmmotorSteer), incSec, incMs);
            motorSteer.pwm.state = 0;
            flag.motorSteer = 1;
        }
        else                        //If PWM low
        {
            timeIncrement(&Schedule.pwmmotorSteer, motorSteer.pwm.aSec, motorSteer.pwm.aMs);
            motorSteer.pwm.state = 1;
            if (motorDrive.pwm.aMs == 0) //If no on time to PWM
            {
                flag.motorSteer = 0;
            }
            else
            {
                flag.motorSteer = 1;
            }
        }
    }

}

void checkFlags()
{
    char i = 0;

	//Debounce wait has finished
    if(flag.debounce)
    {
        if((P1IN & 0x08) != 0x08)   //Button still pressed after debounce
        {
            //Wait 2 seconds before starting to move
            if (state == 0)
            {
                timeIncrement(&Schedule.stateChange, 2, 0);
                ultrasonicTrigger(&ultraWall);
            }
			//If pressed in other state then will stop the car
            if (state == 1)
            {
                flag.stateChange = 1;
            }
        }
        Schedule.debounce.sec = 0;
        Schedule.debounce.ms = -1;
        flag.debounce = 0;
    }

	//It's time to start an altrasonic distance reading
    if (flag.ultraStart)
    {
        ultrasonicTrigger(&ultraWall);
        flag.ultraStart = 0;
    }

	//Ultrasonic has reading ready
    if (flag.ultrasonicRead)            //When reading from ultrasonic has returned
    {
        for(i = 1; i < WALLREADINGS; i++)
        {
            wallDistances[i] = wallDistances[i - 1];
        }
        wallDistances[0] = ultraWall.distance;

        if(state == START)
        {
            startingDistance = wallDistances[0];    //On start get distance to wall
        }
        else if (state == GO)
        {
            timeIncrement(&Schedule.ultraStart, 0, 20); //In follow wall state get new reading in 20 ms
        }

        if(wallDistances[0] < startingDistance-canDetectDist)            //When reading is suddenly closer
        {
            flag.stateChange = 1;
        }

        alignToWall();

        flag.ultrasonicRead = 0;
    }

	//On button press start debounce
    if (flag.button)
    {
        if(Schedule.debounce.sec == 0 && Schedule.debounce.ms == -1)   	//If debounce not currently occurring
        {
            timeIncrement(&Schedule.debounce, 0, 20);  					//20 ms debounce
        }
        flag.button = 0;
    }
	
	//Drive motor needs to change behaviour
    if (flag.motorDrive)
    {
        motorOutput(&motorDrive);
        flag.motorDrive = 0;
    }

	//Steering motor needs to change behaviour
    if (flag.motorSteer)    //If steering motor needs to change
    {
        motorOutput(&motorSteer);
        flag.motorSteer = 0;
    }

	//Time to change state
    if (flag.stateChange)    //On button press start debounce
    {
        state++;
        if (state == GO)
        {
            //Start driving forward
            motorDrive.direction = FORWARD;
            flag.motorDrive = 1;

            //Initiate first ultrasonic reading
            ultrasonicTrigger(&ultraWall);

        }
        else if (state == STOP)
        {
            motorDrive.direction = OFF;
            flag.motorDrive = 1;
        }
        else
        {
            state = START;
        }
        flag.stateChange = 0;
    }
}

void stateControl()
{
	if(state == Start)
	{
		//Do nothing until button pressed
		
		//On button press do an initial ultrasonic reading
		
		//Schedule state to change to GO for some time after first ultrasonic reading
	}
	
	if(state == GO)
	{
		//Drive forward at start
		
		//Continuously take ultrasonic readings to wall
		
		//Possibly swivel RADAR and take some forward readings simultaneously to detect can
		
		//Use readings to stay aligned with the wall
		
		//Stop when can detected
		
	}
	
	if(state == STOP)
	{
		//Stop all motors and readings
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

void setupButton()
{
    //Setup button for input and interrupt (P1.3)
    P1DIR &= ~BIT3;
    //Pull up so when pressed will go high to low
    P1REN |= BIT3;
    P1OUT |= BIT3;
    P1IE |= BIT3;    //Enable interrupts
    P1IES |= BIT3;   //High to Low transition
    P1IFG &= ~BIT3;  //Clear interrupts
}

void setupTimerSchedule()
{
    if(CLOCK_USED_SCHEDULER == SMCK_FREQ)
    {
        TA0CTL |= TASSEL_2 + MC_1;              //SMCK  so f = 1 MHz, operating in up mode to TA0CCR0
    }
    else
    {
        TA0CTL |= TASSEL_1 + MC_1;              //ACLK  so f = 32768 Hz, operating in up mode to TA0CCR0
    }
    TA0CCTL0 |= 0x10;                       //Interrupt occurs when TA0R reaches TA0CCR0
    TA0CCR0 = CLOCK_USED_SCHEDULER*TIMER_INC_MS/1000; //Set the count to value (5 ms) f*5ms = 5000
    TA0CCTL0 &= ~CCIFG;                     //Clear interrupt flags
}

void setupTimerRADAR()
{
    if(CLOCK_USED_ULTRASONIC == SMCK_FREQ)
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
	
	//Start counting to TA1CCR0 (Defined in servo setup)
    TA1CTL |= MC_1;
}

void alignToWall()
{
    char turnStatePrevious = turnState;
    char i = 0;

    //When state change is based on measured distance to wall
    if(wallDistances[0] < startingDistance-wallTolerance)       //When drifted closer to wall
    {
        turnState = AWAY;
    }
    else if(wallDistances[0] > startingDistance+wallTolerance)  //When drifted further from wall
    {
        turnState = CLOSE;
    }
    else                                                    //If right distance from wall drive straight
    {
        turnState = STRAIGHT;
    }


    //When state change depends on if distance is increasing
    if (turnState == CLOSE | turnState == STRAIGHTEN)
    {
        //Determine distance is increasing
        for(i = 0; i < WALLREADINGS-1; i++)
        {
            if(wallDistances[i] < wallDistances[i + 1]) //Check if distance decreasing
            {
                break;  //If so break out of loop
            }
        }
        if (i == WALLREADINGS-1)    //Distance measured is increasing
        {
            switch(turnState)
            {
            case CLOSE:
                turnState = STRAIGHTEN;
                break;
            case STRAIGHTEN:
                turnState = STRAIGHT;
                break;
            }
        }
    }

    //When in same state as before
    if (turnState == turnStatePrevious)
    {
        //When in same state for long enough but not straight state
        if((++turnStateTime) >= 5 && (turnState != STRAIGHT))
        {
            motorDrive.pwm.aMs = 50;    //Half driving speed
        }
        else
        {
            motorDrive.pwm.aMs = 100;   //Full speed
        }
    }
    else    //When state has changed
    {
        turnStateTime = 0;  //Changed state so reset timer
        motorDrive.pwm.aMs = 100;   //Full speed

        switch(turnState)
        {
        case STRAIGHT:
            motorSteer.direction = STRAIGHT;
            break;
        case AWAY:
            motorSteer.direction = RIGHT;
            break;
        case CLOSE:
            motorSteer.direction = LEFT;
            break;
        case STRAIGHTEN:
            motorSteer.direction = RIGHT;
            break;
        }
        flag.motorSteer = 1;
    }
}

//==============================================================================
// End of File : AutonomousCar/main.c
