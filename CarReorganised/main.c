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

--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
31-MAR-2024 SARK created to restructure AutonomousCar project
01-APR-2024 SARK continued to restructure to make readable
03-APR-2024 SARK added multiple ultrasonic sensors
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

    //To read ultrasonic when result available
    char ultraWallRead;
    char ultraRADARRead;
};

//Structure contains all timings that events occur on
struct Scheduler {
    //Time at which debouncing finishes
    struct Time debounce;

    //Time to start an ultrasonic reading
    struct Time ultraLeftStart;
	//struct Time ultraRightStart;
	struct Time ultraRADARStart;

    //Time at which car state should change
    struct Time stateChange;

    //Time at which motor PWM should change
    struct Time pwmMotorDrive;
    struct Time pwmMotorSteer;
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
// MACROs
//------------------------------------------------------------------------------
//Available clock information
#define SMCK_FREQ      1000000
#define ACLK_FREQ      32768

// Important Ultrasonic Info
#define SOUND_SPEED 343
#define CLOCK_USED_ULTRASONIC   SMCK_FREQ
#define dist2pulse(d)           ((CLOCK_USED_ULTRASONIC/100)*d*2/SOUND_SPEED)     // Converts a distance (cm) to ultrasonic sensor output pulse length

//Scheduler information
#define CLOCK_USED_SCHEDULER    SMCK_FREQ
#define SECOND_COUNT   			1000		//1000 ms = 1s
#define isTime(X) ((currentTime.sec == X.sec) && (currentTime.ms == X.ms))

//Car States
#define START       0
#define GO          1
#define STOP        2

//Wall alignment
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
// CHANGEABLE SETTINGS MACROs
//------------------------------------------------------------------------------
//Scheduler control
#define TIMER_INC_MS    2           //Scheduler interrupts period (2 ms)

//Drive Speeds
#define MOTOR_PWM_PERIOD    100                     //PWM period of motors.
#define SPEED_TOP           1*MOTOR_PWM_PERIOD      //Top speed of the motors as percentage of PWM.
#define SPEED_SLOW          0.5*MOTOR_PWM_PERIOD    //Slower speed when changing direction.
#define READINGS_TO_SLOW    10                      //Number of distance readings taken whilst turning before slowing down.

//Wall readings control
#define WALL_READINGS   2                   //Number of wall readings to decrease before changing state (Possibly not used).
#define WALL_TOLERANCE  dist2pulse(5)      //Distance +- correct distance from wall
#define CAN_DETECT_DIST dist2pulse(15)     //Distance closer then wall

//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------
//Scheduling info
struct Time currentTime     =   {0, 0};    //Running count of time
struct Scheduler Schedule   =   {0};       //Schedule when events needing attended
struct flags flag           =   {0};       //Flag when something ready to be attended

//What car should be doing
char state  =   START;

//DC Motor info (On Port 1)
struct MotorDC motorDrive = {0, BIT5, BIT4, {0, MOTOR_PWM_PERIOD, 0, SPEED_TOP, 1}};
struct MotorDC motorSteer = {0, BIT6, BIT7, {0, MOTOR_PWM_PERIOD, 0, MOTOR_PWM_PERIOD, 1}};

//Wall Ultrasonic info (Port 2)
struct Ultrasonic ultraLeft = {0, {0, 0}, 0, BIT0, BIT2, 2};
//struct Ultrasonic ultraRight = {0, {0, 0}, 0, BIT0, BIT1, 2};

//RADAR Ultrasonic info (Port 1)
struct Ultrasonic ultraRADAR = {0, {0, 0}, 0, BIT0, BIT2, 1};

//Servo info (Port 2)
struct Servo servoA = {BIT4, 50, 0};    //PWM on Port 2.4, change PWM by 50 clock ticks when turned, initially turn anti-clockwise

//Wall alignment info
volatile int leftWall;			//Initial distance to maintain to left wall
//volatile int rightWall;          //Initial distance to maintain to right wall
char turnState = STRAIGHT;				//Wall alignment turning instruction
int  wallDistances[WALL_READINGS] = {0};	//Distance readings to wall
char turnStateTime = 0;					//Time spent turning to correct for wall

//Flags for after main flags are dealt with and then results are used for stateControl()
char buttonPressed = 0;
char ultraRead = 0;
char ultraRADARRead = 0;
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

#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer0_A1_ISR(void)
{
    switch(TA0IV)
    {
        case TA0IV_TACCR1:  //TA0CCR1
            P2OUT &= ~0x2A;
            P2OUT|= RGB_YELLOW;
            ultraRADAR.time[ultraRADAR.timeNumber] = TA0CCR1;
            ultraRADAR.timeNumber++;
            if (ultraRADAR.timeNumber==2)       //After up/down edges of feedback
            {
                ultraRADAR.distance = ultraRADAR.time[1]-ultraRADAR.time[0];
                if (ultraRADAR.distance < 0)    //When timer wrapped
                {
                    ultraRADAR.distance += TA0CCR0;
                }
                flag.ultraRADARRead = 1;
                ultraRADAR.timeNumber=0;
                TA0CCTL1 |= CM_1;   //Capture on rising edge
            }
            else
            {
                TA0CCTL1 |= CM_2;   //Capture on falling edge
            }
            TA0CCTL1 &= ~CCIFG;
            break;

        case TA0IV_TACCR2:  //TA0CCR2
            TA0CCTL2 &= ~CCIFG;
            break;

        case 0xA:   break;
    }
}

//Ultrasonic capture compare
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer1_A1_ISR (void)
{
    switch(TA1IV)
    {
    case 0xA:	//OVERFLOW
        TA1CTL &= ~TAIFG;
        break;
    case TA1IV_TACCR1:	//TA1CCR1 (Wall Ultrasonic)
        if(TA1CCTL1 & CCIS_1)
        {
            P2OUT &= ~RGB_WHITE;
            P2OUT|= RGB_GREEN;
            ultraLeft.time[ultraLeft.timeNumber] = TA1CCR1;
            ultraLeft.timeNumber++;
            if (ultraLeft.timeNumber==2)       //After up/down edges of feedback
            {
                ultraLeft.distance = ultraLeft.time[1]-ultraLeft.time[0];
                if (ultraLeft.distance < 0)    //When timer wrapped
                {
                    ultraLeft.distance += TA1CCR0;
                }
                flag.ultraWallRead = 1;
                ultraLeft.timeNumber=0;
                TA1CCTL1 |= CM_1;   //Capture on rising edge
            }
            else
            {
                TA1CCTL1 |= CM_2;   //Capture on falling edge
            }
            TA1CCTL1 &= ~CCIFG;
        }
        //else
        //{
        //    P2OUT &= ~RGB_WHITE;
        //    P2OUT|= RGB_BLUE;
        //    ultraRight.time[ultraRight.timeNumber] = TA1CCR1;
        //    ultraRight.timeNumber++;
        //    if (ultraRight.timeNumber==2)       //After up/down edges of feedback
        //    {
        //        ultraRight.distance = ultraRight.time[1]-ultraRight.time[0];
        //        if (ultraRight.distance < 0)    //When timer wrapped
        //        {
        //            ultraRight.distance += TA1CCR0;
        //        }
        //        flag.ultraWallRead = 1;
        //        ultraRight.timeNumber=0;
        //        TA1CCTL1 |= CM_1;   //Capture on rising edge
        //    }
        //    else
        //    {
        //        TA1CCTL1 |= CM_2;   //Capture on falling edge
        //    }
        //    TA1CCTL1 &= ~CCIFG;
        //}
        break;
		
    case TA1IV_TACCR2:	//TA1CCR2 (No interrupt as used in servo PWM)
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
    ultrasonicSetup(&ultraLeft);
	//ultrasonicSetup(&ultraRight);
    ultrasonicSetup(&ultraRADAR);
	servoSetup(&servoA);
	setupTimerRADAR();
    setupTimerSchedule();

    //TA0CCTL1 = TA1CCTL1;
    //TA0CCTL1 |= CM_3 +SCCI;

    //Disable schedules
    Schedule.debounce.sec = 0;
    Schedule.debounce.ms = -1;

    Schedule.stateChange.sec = 0;
    Schedule.stateChange.ms = -1;

	Schedule.ultraLeftStart.sec = 0;
    Schedule.ultraLeftStart.ms = -1;
		
	//Schedule.ultraRightStart.sec = 0;
    //Schedule.ultraRightStart.ms = -1;
	
	//Do an ultrasonic RADAR reading
	timeIncrement(&(Schedule.ultraRADARStart), 0, 20);

	//Start DC motor PWM schedules but set both motors output to do nothing
    timeIncrement(&Schedule.pwmMotorDrive, motorDrive.pwm.aSec, motorDrive.pwm.aMs);
    motorDrive.pwm.state = 1;
    flag.motorDrive = 1;
    motorDrive.direction = 0;

    timeIncrement(&Schedule.pwmMotorSteer, motorSteer.pwm.aSec, motorSteer.pwm.aMs);
    motorSteer.pwm.state = 1;
    flag.motorSteer = 1;
    motorSteer.direction = 0;

    //RGB to indicate things
    P2DIR |= 0x2A;
    P1DIR |= BIT0;
    P2OUT &= ~0x2A;
    P1OUT &= ~(BIT0+BIT2);

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
			stateControl();		//Control behaviour of car
        }
    }
}

void checkSchedule()
{
    int incSec = 0;
    int incMs = 0;

	//Time for car to change behaviour
    if(isTime(Schedule.stateChange))
    {
		//Attend to in checkFlags()
        flag.stateChange = 1;
		
		//Disable schedule
        Schedule.stateChange.sec = 0;
        Schedule.stateChange.ms = -1;
    }

    if (isTime(Schedule.ultraLeftStart))
    {
        ultrasonicTrigger(&ultraLeft);

        //Disable schedule
        Schedule.ultraLeftStart.sec = 0;
        Schedule.ultraLeftStart.ms = -1;
    }
	
    //if (isTime(Schedule.ultraRightStart))
    //{
    //    //ultrasonicTrigger(&ultraRight);
	//
    //    //Disable schedule
    //    Schedule.ultraRightStart.sec = 0;
    //    Schedule.ultraRightStart.ms = -1;
    //}
	
	if (isTime(Schedule.ultraRADARStart))
    {
        ultrasonicTrigger(&ultraRADAR);
        //P2OUT &= ~RGB_WHITE;
        //Disable schedule
        Schedule.ultraRADARStart.sec = 0;
        Schedule.ultraRADARStart.ms = -1;
    }
	
	//Time to check button debounce
    if(isTime(Schedule.debounce))
    {
		//Attend to in checkFlags()
        flag.debounce = 1;
		
		//Disable schedule
		Schedule.debounce.sec = 0;
        Schedule.debounce.ms = -1;
    }

	//Control PWM for DC motors
    if(isTime(Schedule.pwmMotorDrive))
    {
        if(motorDrive.pwm.state)	//If PWM high
        {
            //Find time to be LOW based on duration of HIGH and PWM period
            incSec = motorDrive.pwm.sec-motorDrive.pwm.aSec;
            incMs = motorDrive.pwm.ms-motorDrive.pwm.aMs;
			
			//Schedule time to become HIGH again
            timeIncrement(&(Schedule.pwmMotorDrive), incSec, incMs);
			
			//Set PWM low
            motorDrive.pwm.state = 0;
			
			//Attend to in checkFlags() to turn off motor as PWM is low
            flag.motorDrive = 1;
        }
        else						//If PWM low
        {
			//Set time to become LOW again
            timeIncrement(&Schedule.pwmMotorDrive, motorDrive.pwm.aSec, motorDrive.pwm.aMs);
            
			//Set PWM high
			motorDrive.pwm.state = 1;
			
			//Attend to in checkFlags() to turn on motor as PWM is high
            if (motorDrive.pwm.aMs == 0)
            {
				//If PWM is always low do not change as scheduler takes
				//short time to then make low again so motor is on momentarily
                flag.motorDrive = 0;
            }
            else
            {
                flag.motorDrive = 1;
            }
        }
    }

	//See behavioural comments above
    if(isTime(Schedule.pwmMotorSteer))
    {
        if(motorSteer.pwm.state)
        {
            incSec = motorSteer.pwm.sec-motorSteer.pwm.aSec;
            incMs = motorSteer.pwm.ms-motorSteer.pwm.aMs;
            timeIncrement(&(Schedule.pwmMotorSteer), incSec, incMs);
            motorSteer.pwm.state = 0;
            flag.motorSteer = 1;
        }
        else
        {
            timeIncrement(&Schedule.pwmMotorSteer, motorSteer.pwm.aSec, motorSteer.pwm.aMs);
            motorSteer.pwm.state = 1;
            if (motorDrive.pwm.aMs == 0)
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
			///Attend to in stateControl()
			buttonPressed = 1;
        }
        Schedule.debounce.sec = 0;
        Schedule.debounce.ms = -1;
        flag.debounce = 0;
    }

	//Wall ultrasonic has reading ready
    if (flag.ultraWallRead)
    {
		//Update array of wall readings
        for(i = WALL_READINGS-1; i > 0; i--)
        {
            wallDistances[i] = wallDistances[i - 1];
        }
        wallDistances[0] = ultraLeft.distance;
		
		///Attend to in stateControl()
		ultraRead = 1;
		
        flag.ultraWallRead = 0;
    }
	
	//Wall ultrasonic has reading ready
    if (flag.ultraRADARRead)
    {
		///Attend to in stateControl()
		ultraRADARRead = 1;
		
        flag.ultraRADARRead = 0;
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
		//Alter behaviour as has been updated
        motorOutput(&motorDrive);
        flag.motorDrive = 0;
    }

	//Steering motor needs to change behaviour
    if (flag.motorSteer)    //If steering motor needs to change
    {
		//Alter behaviour as has been updated
        motorOutput(&motorSteer);
        flag.motorSteer = 0;
    }

	//Time to change state
    if (flag.stateChange)    //On button press start debounce
    {
		//Increase to next state (May need to become more specific)
        state++;
		
		//Events to occur when changing to GO state
        if (state == GO)
        {
            //Start driving forward
            motorDrive.direction = FORWARD;
            flag.motorDrive = 1;

            //Initiate first ultrasonic reading
            ultrasonicTrigger(&ultraLeft);
        }
        else if (state == STOP)
        {
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
	//On start up state
	if(state == START)
	{
		//Do nothing until button pressed
		//On button press do an initial ultrasonic wall reading 
		//& schedule to next state to start moving
		if (buttonPressed)
		{
			//Trigger next reading in 20 ms
		    //TA1CCTL1 &= ~CCIS_3;
		    //TA1CCTL1 |= CCIS_0;
            timeIncrement(&(Schedule.ultraLeftStart), 0, 20);
            //timeIncrement(&(Schedule.ultraRightStart), 0, 200);
			timeIncrement(&Schedule.stateChange, 1, 0);
			buttonPressed = 0;
		}
		
		//Use initial ultrasonic reading as distance to wall to maintain
		if (ultraRead)
		{
		    if(TA1CCTL1 & CCIS_1)
		    {
		        leftWall = wallDistances[0];
		    }
		    //else
		    //{
		    //    rightWall = wallDistances[0];
		    //}
			ultraRead = 0;
		}
	}
	
	//On follow wall and detect can state
	if(state == GO)
	{
	    P2OUT |= RGB_BLUE;
		//Drive forward at start (Done in state change)
		//Continuously take ultrasonic readings to wall (Started in state change)
		if (ultraRead)
		{
			//When reading is suddenly closer can detected so change state
			if(wallDistances[0] < leftWall-CAN_DETECT_DIST)
			{
				flag.stateChange = 1;	//Change state to stop
			    P2OUT &= ~0x2A;
			    P2OUT |= RGB_CYAN;
			}
			
			//Use latest reading to keep aligned to wall
			alignToWall();

		    //Trigger next reading in 20 ms
		    timeIncrement(&(Schedule.ultraLeftStart), 0, 20);

			ultraRead = 0;
		}
		
		//Possibly swivel RADAR and take some forward readings simultaneously to detect can
		
		//Stop if button pressed (FOR TESTING IF FAILS TO STOP)
		if (buttonPressed)
        {
			state = STOP;
			buttonPressed = 0;
        }
	}
	
	//On stop moving state
	if(state == STOP)
	{
	    //Stop if button pressed (FOR TESTING IF FAILS TO STOP)
	    if (buttonPressed)
	    {
	        state = START;
	        buttonPressed = 0;
	    }

		//Stop all motors and readings
		motorDrive.direction = OFF;
		motorSteer.direction = STRAIGHT;
        flag.motorDrive = 1;
        flag.motorSteer = 1;
	}


	//To occur in all states
	if(ultraRADARRead)
	{
		if (ultraRADAR.distance >= 550)
		{
		    if (TA1CCR2 >= PWM_SERVO_UPPER)
		    {
		        servoA.direction = 0; //Anti-clockwise
		    }
		    else if (TA1CCR2 <= PWM_SERVO_LOWER)
		    {
		        servoA.direction = 1; //Clockwise
		    }
			servoTurn(&servoA);
		}
		timeIncrement(&(Schedule.ultraRADARStart), 0, 20);
		ultraRADARRead = 0;
	}
}


void timeIncrement(struct Time *time, int sec, int ms)
{
    //Always on next increment amount
    ms += TIMER_INC_MS;
    ms -= ms % TIMER_INC_MS;

	//Add to current time and if more then max ms add to seconds
    time->ms = currentTime.ms + ms;
    while(time->ms >= SECOND_COUNT)
    {
        time->ms -= SECOND_COUNT;
        sec++;
    }

	//Add to current time and if more then max s wrap back to 0
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
    TA0CCTL0 |= 0x10;                       			//Interrupt occurs when TA0R reaches TA0CCR0
    TA0CCR0 = CLOCK_USED_SCHEDULER*TIMER_INC_MS/1000; 	//Set the count to schedule time, e.g 1 MHz*5ms = 5000
    TA0CCTL0 &= ~CCIFG;                     			//Clear interrupt flags
	
	TA0CTL &= ~TAIFG;	//Clear interrupt
    TA0CTL &= ~TAIE;	//Disable interrupt on timer edge
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

    TA1CCTL0 &= ~(CCIFG+CCIE);
    TA1CCTL2 &= ~CCIE;
    TA1CCTL2 &= ~CCIFG;
	
	//Count to TA1CCR0 (Defined in servo setup)
    TA1CTL |= MC_1;
}

void alignToWall()
{
    char turnStatePrevious = turnState;

    //When state change is based on measured distance to wall
    if(wallDistances[0] < leftWall-WALL_TOLERANCE)       //When drifted closer to wall
    {
        turnState = AWAY;
    }
    else if(wallDistances[0] > leftWall+WALL_TOLERANCE)  //When drifted further from wall
    {
        turnState = CLOSE;
    }
    else                                                    //If right distance from wall drive straight
    {
        turnState = STRAIGHT;
    }


    //When state change depends on if distance is increasing
    //if (turnState == CLOSE | turnState == STRAIGHTEN)
    //{
    //    //Determine if distance is increasing
    //    for(i = 0; i < WALL_READINGS-1; i++)
    //    {
    //        if(wallDistances[i] < wallDistances[i + 1]-5) //Check if distance ever decreases
    //        {
    //            break;  //If so break out of loop
    //        }
    //    }
	//	//If for loop went to completion
    //    if (i == WALL_READINGS-1)    //Distance measured is increasing
    //    {
    //        switch(turnState)
    //        {
    //        case CLOSE:		//Was turning towards wall so needs to straighten back out
    //            turnState = STRAIGHTEN;
    //            break;
    //        case STRAIGHTEN:	//Was straightening out but over corrected go straight and back to first if correction
    //            turnState = STRAIGHT;
    //            break;
    //        }
    //    }
    //}

    //When in same state as before
    if (turnState == turnStatePrevious)
    {
        //When in same state for long enough but not straight state, 
		//slow down to not overshoot corrections
        if((++turnStateTime) >= READINGS_TO_SLOW && (turnState != STRAIGHT))
        {
            motorDrive.pwm.aMs = SPEED_SLOW;    //Half driving speed
        }
        else
        {
            motorDrive.pwm.aMs = SPEED_TOP;   //Full speed
        }
    }
    else    //When state has changed
    {
        turnStateTime = 0;  //Changed state so reset time to slow
        motorDrive.pwm.aMs = SPEED_TOP;   //Full speed

		//Control steering according to new state
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
