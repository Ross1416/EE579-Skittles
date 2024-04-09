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
05-APR-2024 SARK added multiple ultrasonic sensors
06-APR-2024 SARK fine tuned car to follow wall and stop beside can
07-APR-2024 SARK added RADAR scan to lock on to car
08-APR-2024 SARK fixed bug with ultrasonic readings and timer overflows
08-APR-2024 SARK added code for car to approach can
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
#include "Infrared.h"

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
void	RADAR();
void    findAverage(unsigned char start, unsigned char numOfValues, int *values);

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
#define START       	0
#define SEARCH      	1
#define CAN_ALIGN   	2
#define RADAR_SCAN  	3
#define CAN_APPROACH	4
#define COLOUR_DETECT 	5
#define CAN_HIT			6
#define CAN_AVOID		7
#define STOP        	8

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
#define MOTOR_PWM_PERIOD    50                     //PWM period of motors.
#define SPEED_TOP           0.4*MOTOR_PWM_PERIOD      //Top speed of the motors as percentage of PWM.
#define SPEED_SLOW          0.3*MOTOR_PWM_PERIOD    //Slower speed when changing direction.
#define READINGS_TO_SPEED_CHANGE    5                      //Number of distance readings taken whilst turning before slowing down.

//Wall readings control
#define WALL_READINGS   4                   //Number of wall readings remembered (MAKE EVEN)
#define WALL_TOLERANCE  dist2pulse(2)       //Distance +- correct distance from wall
#define CAN_DETECT_DIST dist2pulse(10)      //Distance closer then wall

//RADAR Reading control
#define RADAR_READINGS   5
#define MAX_FRONT_DETECT 1500

//RADAR SCANNING
#define NUMBER_OF_ANGLES_CHECKED    15
#define MAX_RADAR_DISTANCE          dist2pulse(50)


//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------
//Scheduling info
struct Time currentTime     =   {0, 0};    //Running count of time
struct Scheduler Schedule   =   {0};       //Schedule when events needing attended
struct flags flag           =   {0};       //Flag when something ready to be attended

//What car should be doing
char previousState  =   START;
char state          =   START;
char nextState      =   START;

//DC Motor info (On Port 1)
struct MotorDC motorDrive = {0, BIT5, BIT4, {0, MOTOR_PWM_PERIOD, 0, SPEED_TOP, 1}};
struct MotorDC motorSteer = {0, BIT6, BIT7, {0, MOTOR_PWM_PERIOD, 0, MOTOR_PWM_PERIOD, 1}};

//Wall Ultrasonic info (Port 2)
struct Ultrasonic ultraLeft = {0, {0, 0}, 0, BIT0, BIT2, 2};
//struct Ultrasonic ultraRight = {0, {0, 0}, 0, BIT0, BIT1, 2};

//RADAR Ultrasonic info (Port 1)
struct Ultrasonic ultraRADAR = {0, {0, 0}, 0, BIT0, BIT2, 1};

//Servo info (Port 2)
struct Servo servoA = {BIT4, (PWM_SERVO_UPPER-PWM_SERVO_LOWER)/NUMBER_OF_ANGLES_CHECKED, 0};    //PWM on Port 2.4, angle to be turned, initially turn anti-clockwise

// IR info (Port 2)
struct Infrared IR = {2, BIT3, BIT2};

//Wall alignment info
volatile int leftWall;			//Initial distance to maintain to left wall
//volatile int rightWall;          //Initial distance to maintain to right wall
char turnState = STRAIGHT;				//Wall alignment turning instruction
int  wallDistances[WALL_READINGS] = {[0 ... WALL_READINGS-1] = 2000};	//Distance readings to wall
unsigned char turnStateTime = 0;					//Time spent turning to correct for wall
int avgReading = 0;
int avgOldReading = 0;
int avgNewReading = 0;

int RADARDistances[RADAR_READINGS] = {[0 ... RADAR_READINGS-1] = 2000};

//unsigned int canHorizontalDist = 0;

unsigned int i = 0;

//RADAR Scan Variables
unsigned char readingsOnAngle = 0;
unsigned char anglesChecked = 0;
unsigned char anomalyPositions[NUMBER_OF_ANGLES_CHECKED] = {0};
unsigned char anomalyNumber = 0;
unsigned char canAnomaly = 0;
unsigned char anomalyStart[NUMBER_OF_ANGLES_CHECKED] = {[0 ... NUMBER_OF_ANGLES_CHECKED-1] = 0};;
unsigned char anomalyEnd[NUMBER_OF_ANGLES_CHECKED] = {[0 ... NUMBER_OF_ANGLES_CHECKED-1] = 0};;
int RADARAtEachAngle[NUMBER_OF_ANGLES_CHECKED] = {[0 ... NUMBER_OF_ANGLES_CHECKED-1] = 0};
unsigned int newAngle;

//Approaching can variables
unsigned char lostCan = 0;

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

        //If wrap occurs during ultrasonic pulse
        if (ultraRADAR.timeNumber != 0)
        {
            ultraRADAR.time[1] += TA0CCR0;
        }

        __low_power_mode_off_on_exit();
        TA0CCTL0 &= ~CCIFG;
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer0_A1_ISR(void)
{
    switch(TA0IV)
    {
        case TA0IV_TACCR1:  //TA0CCR1
            ultraRADAR.time[ultraRADAR.timeNumber] += TA0CCR1;
            ultraRADAR.timeNumber++;
            if (ultraRADAR.timeNumber==2)       //After up/down edges of feedback
            {
                ultraRADAR.distance = ultraRADAR.time[1]-ultraRADAR.time[0];
                flag.ultraRADARRead = 1;
                ultraRADAR.time[0] = 0;
                ultraRADAR.time[1] = 0;
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

#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer1_A0_ISR (void)
{
    //If wrap occurs during ultrasonic pulse
    if (ultraLeft.timeNumber != 0)
    {
        ultraLeft.time[1] += TA1CCR0;
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
            //P2OUT &= ~RGB_WHITE;
            //P2OUT|= RGB_GREEN;
            ultraLeft.time[ultraLeft.timeNumber] += TA1CCR1;
            ultraLeft.timeNumber++;
            if (ultraLeft.timeNumber==2)       //After up/down edges of feedback
            {
                ultraLeft.distance = ultraLeft.time[1]-ultraLeft.time[0];
                flag.ultraWallRead = 1;
                ultraLeft.time[0] = 0;
                ultraLeft.time[1] = 0;
                ultraLeft.timeNumber=0;

                TA1CCTL1 |= CM_1;   //Capture on rising edge
            }
            else
            {
                TA1CCTL1 |= CM_2;   //Capture on falling edge
            }
            TA1CCTL1 &= ~CCIFG;
        }
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
	timeIncrement(&(Schedule.ultraRADARStart), 1, 0);

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
        //Update array of wall readings
        for(i = RADAR_READINGS-1; i > 0; i--)
        {
            RADARDistances[i] = RADARDistances[i - 1];
        }
        RADARDistances[0] = ultraRADAR.distance;

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

	//Change state
    if (flag.stateChange)    //On button press start debounce
    {
		//Become next state
        previousState = state;
        state = nextState;
		
		//Events to occur when changing to GO state
		if (state == START)
		{
			
		}
        else if (state == SEARCH)
        {
            //Start driving forward
            motorDrive.direction = FORWARD;
            flag.motorDrive = 1;

            //Centre RADAR
            servoCenter();

            //Initiate first ultrasonic reading
            ultrasonicTrigger(&ultraLeft);
        }
        else if (state == CAN_ALIGN)
        {
			motorDrive.direction = BACK;
            motorDrive.pwm.aMs = 0.6*MOTOR_PWM_PERIOD;
            flag.motorDrive = 1;

            motorSteer.direction = RIGHT;
            flag.motorSteer = 1;
			
			nextState = RADAR_SCAN;
			timeIncrement(&Schedule.stateChange, 1, 0);
        }
        else if (state == RADAR_SCAN)
        {
			//Clear variables used in function
			readingsOnAngle = 0;
			anglesChecked = 0;
			newAngle = 0;
			avgReading = 0;
			anomalyNumber = 0;
			canAnomaly = 0;
			
			for (i=0; i<NUMBER_OF_ANGLES_CHECKED; i++)
			{
			    anomalyStart[i] = 0;
			    anomalyEnd[i] = 0;
				anomalyPositions[i] = 0;
				RADARAtEachAngle[i] = 0;
			}
			
			//Prepare servo to turn anti-clockwise
			servoA.direction = 1;
			
			//Stop driving
			motorDrive.direction = OFF;
			motorSteer.direction = STRAIGHT;
			flag.motorDrive = 1;
			flag.motorSteer = 1;
			
			//Rotate servo full clockwise
            TA1CCR2 = PWM_SERVO_LOWER;
			
			//Start first reading in a second when servo has turned
			timeIncrement(&Schedule.ultraRADARStart, 1, 0);
        }
		else if (state == CAN_APPROACH)
		{
			motorDrive.pwm.aMs = 0.3*MOTOR_PWM_PERIOD;
			motorDrive.direction = FORWARD;
			flag.motorDrive = 1;
			
			lostCan = 0;
			
			timeIncrement(&Schedule.ultraRADARStart, 0, 20);
		}
        else if (state == STOP)
        {
        }
        else
        {
            //state = START;
        }
        flag.stateChange = 0;
    }
}

void stateControl()
{
//ON START UP==============================================================================
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
            nextState = SEARCH;
			timeIncrement(&Schedule.stateChange, 1, 0);
			buttonPressed = 0;
		}
		
		//Use initial ultrasonic reading as distance to wall to maintain
		if (ultraRead)
		{
		    if(TA1CCTL1 & CCIS_1)
		    {
		        leftWall = wallDistances[i];
		    }

		    if (i < WALL_READINGS)
		    {
		        i++;
		        timeIncrement(&(Schedule.ultraLeftStart), 0, 20);
		    }
			ultraRead = 0;
		}

	}
	
//FOLLOW WALL WHILST LOOKING FOR CANS========================================================
	if(state == SEARCH)
	{
		//Drive forward at start (Done in state change)

		//Take ultrasonic readings to wall to stay aligned and if can is past on left
		if (ultraRead)
		{
			findAverage(0, WALL_READINGS, wallDistances);

            if(avgReading < leftWall-CAN_DETECT_DIST)
            {
                nextState = CAN_ALIGN;
                flag.stateChange = 1;
            }
			
			//Use latest reading to keep aligned to wall
			alignToWall();

		    //Trigger next reading in 20 ms
		    timeIncrement(&(Schedule.ultraRADARStart), 0, 20);

			ultraRead = 0;
		}
		
		//Take forward readings to see if can is in front of car
		if (ultraRADARRead)
		{
		    findAverage(0, RADAR_READINGS, RADARDistances);

            if(avgReading < MAX_FRONT_DETECT)
            {
                nextState = RADAR_SCAN;
                flag.stateChange = 1;
            }

            //Trigger next reading in 20 ms
            timeIncrement(&(Schedule.ultraLeftStart), 0, 20);

            ultraRADARRead = 0;
		}
		
		//Stop if button pressed (FOR TESTING IF FAILS TO STOP)
		if (buttonPressed)
        {
			state = STOP;
			buttonPressed = 0;
        }
	}
	
//TURN TO GET CAN IN RADAR RANGE==========================================================
	if(state == CAN_ALIGN)
	{
	    //Only used in state change

	    //Stop if button pressed (FOR TESTING IF FAILS TO STOP)
        if (buttonPressed)
        {
            state = STOP;
            buttonPressed = 0;
        }
	}

//STOP, SCAN & POINT AT CAN===============================================================
	if (state == RADAR_SCAN)
	{
		if (ultraRADARRead)
		{
			RADAR();
			ultraRADARRead = 0;
		}
		
	    //Stop if button pressed (FOR TESTING IF FAILS TO STOP)
        if (buttonPressed)
        {
            state = STOP;
            buttonPressed = 0;
        }
	}

//DRIVE TO CAN WHEN IN RADAR RANGE=======================================================
    if (state == CAN_APPROACH)
    {
		//Drive to heading of RADAR
        if (TA1CCR2 < 1500) //CAN TO THE LEFT?
        {
            motorSteer.direction = RIGHT;
        }
        else if (TA1CCR2 > 1500) //CAN TO THE RIGHT
        {
            motorSteer.direction = LEFT;
        }
        else
        {
            motorSteer.direction = STRAIGHT;
        }
		flag.motorSteer = 1;
		
		//Track can with RADAR
		if (ultraRADARRead)
		{
			//Get an average
			avgOldReading = avgReading;
			findAverage(0, RADAR_READINGS, RADARDistances);
			
			//When reading is close enough can is in front
            if(avgReading < MAX_FRONT_DETECT)
            {
                nextState = COLOUR_DETECT;
                flag.stateChange = 1;
            }
			
			//Determine whether can is being looked at
			if (lostCan)	//If can is lost
			{
				if (avgReading < avgOldReading-100)	//Can refound
				{
					lostCan = 0;
					motorDrive.direction = FORWARD;
				}
			}
			else	//If can is not lost
			{
				if (avgReading > avgOldReading+100)	//Now lost
				{
					lostCan = 1;
					motorDrive.direction = FORWARD;
					
					if (motorSteer.direction == RIGHT)
					{
						servoA.direction = 1;
					}
					else if  (motorSteer.direction == LEFT)
					{
						servoA.direction = 0;
					}
					else
					{
						nextState = RADAR_SCAN;
						flag.stateChange = 1;
					}
				}
			}
			
			if (lostCan)
			{
				servoTurn(&servoA);
				timeIncrement(&Schedule.ultraRADARStart, 0, 200);
			}
			else
			{
				P2OUT &= ~RGB_WHITE;
				P2OUT |= RGB_BLUE;
				timeIncrement(&Schedule.ultraRADARStart, 0, 20);
			}

			ultraRADARRead = 0;
		}

        //Stop if button pressed (FOR TESTING IF FAILS TO STOP)
        if (buttonPressed)
        {
            state = STOP;
            buttonPressed = 0;
        }
    }

//DETECT COLOUR========================================================================
    if (state == COLOUR_DETECT)
    {
		//Stop driving
		motorDrive.direction = OFF;
		motorSteer.direction = STRAIGHT;
		flag.motorDrive = 1;
		flag.motorSteer = 1;
		
		P2OUT &= ~RGB_WHITE;
		P2OUT |= RGB_GREEN;

		// Check IR sensor - get colour
		IRRead(IR);

		// Decide HIT or AVOID depending on BLACK or WHITE can
		if (IR->colour == 0)
		{
		    // Hit can if WHITE
		    state = CAN_AVOID;
		    flag.stateChange = 1;
		}

		if (IR->colour == 1)
        {
		    // Hit can if BLACK
            state = CAN_HIT;
            flag.stateChange = 1;
        }

        //Stop if button pressed (FOR TESTING IF FAILS TO STOP)
        if (buttonPressed)
        {
            state = STOP;
            buttonPressed = 0;
        }
    }

//HIT CAN IF BLACK========================================================================
    if (state == CAN_HIT)
    {





        //Stop if button pressed (FOR TESTING IF FAILS TO STOP)
        if (buttonPressed)
        {
            state = STOP;
            buttonPressed = 0;
        }
    }

//AVOID CAN IF WHITE======================================================================
    if (state == CAN_AVOID)
    {




        //Stop if button pressed (FOR TESTING IF FAILS TO STOP)
        if (buttonPressed)
        {
            state = STOP;
            buttonPressed = 0;
        }
    }

//DO NOTHING==============================================================================
	//On stop moving state
	if(state == STOP)
	{
	    //Stop if button pressed (FOR TESTING IF FAILS TO STOP)
	    if (buttonPressed)
	    {
	        state = START;
	        buttonPressed = 0;
	    }
		
		P2OUT &= ~RGB_BLUE;

		//Stop all motors and readings
		motorDrive.direction = OFF;
		motorSteer.direction = STRAIGHT;
        flag.motorDrive = 1;
        flag.motorSteer = 1;
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
	
    TA1CCTL0 |= CCIE;   //Enable interrupt to know when wraps

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

    //When in same state as before
    if (turnState == turnStatePrevious)
    {
        //When in same state for long enough but not straight state, 
		//slow down to not overshoot corrections
        if((++turnStateTime) >= READINGS_TO_SPEED_CHANGE && (turnState == STRAIGHT))
        {
            motorDrive.pwm.aMs = SPEED_TOP;
        }
    }
    else    //When state has changed
    {
        turnStateTime = 0;  //Changed state so reset time to speed change
        motorDrive.pwm.aMs = SPEED_SLOW;

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

void RADAR()
{
    readingsOnAngle++;
    //When readings full at current angle
    if (readingsOnAngle == RADAR_READINGS)
    {
        //Get average reading for angle set to
        readingsOnAngle = 0;
        avgOldReading = avgReading;
        findAverage(0, RADAR_READINGS, RADARDistances);
        RADARAtEachAngle[anglesChecked] = avgReading;

        //FIND ANOMALIES
        //Is it start or continuation of an anomaly???
        //Only check after first angle
        if (anglesChecked > 0)
        {
            //If anomaly not detected yet
            if (anomalyPositions[anglesChecked-1]==0)
            {
                //When reading suddenly closer
                //if ((avgReading < avgOldReading-200) & (avgReading < MAX_RADAR_DISTANCE))
                if (avgReading < avgOldReading-200)
                {
                    anomalyPositions[anglesChecked] = 1;
                    P2OUT |= RGB_BLUE;
                }
                else
                {
                    anomalyPositions[anglesChecked] = 0;
                    P2OUT &= ~RGB_BLUE;
                }
            }
            else    //If last reading is anomaly
            {
                //When reading suddenly further way
                if (avgReading > avgOldReading+150)
                {
                    anomalyPositions[anglesChecked] = 0;
                    P2OUT &= ~RGB_BLUE;
                }
                else
                {
                    anomalyPositions[anglesChecked] = 1;
                    P2OUT |= RGB_BLUE;
                }
            }
        }

        //Finished dealing with new reading so increment ready for next
        anglesChecked++;

        //Have all angles been checked
        if (anglesChecked < NUMBER_OF_ANGLES_CHECKED)
        {
            //Go to next angle
            servoTurn(&servoA);
            timeIncrement(&Schedule.ultraRADARStart, 0, 200);
        }
        else    //Turn to where anomaly is thought to be after all angles checked
        {
            //Find area of anomalies
            for(i=1;i<NUMBER_OF_ANGLES_CHECKED; i++)
            {
                if (anomalyPositions[i])
                {
                    if (anomalyStart[i-1] == 0)
                    {
                        anomalyStart[anomalyNumber] = i;
                        while((anomalyPositions[i]) & (i < NUMBER_OF_ANGLES_CHECKED))
                        {
                            anomalyEnd[anomalyNumber] = i;
                            i++;
                        }
                        anomalyNumber++;
                    }
                }
            }

            //Find closest anomaly
            avgOldReading = 32000;
            //For each anomaly
            for (readingsOnAngle =0; readingsOnAngle<anomalyNumber; readingsOnAngle++)
            {
                //Average reading of the anomaly
                findAverage(anomalyStart[readingsOnAngle], 1+anomalyEnd[readingsOnAngle], RADARAtEachAngle);

                if (avgReading < avgOldReading)
                {
                    avgOldReading = avgReading;
                    canAnomaly = anomalyStart[readingsOnAngle]+(anomalyEnd[readingsOnAngle]-anomalyStart[readingsOnAngle])/2;
                }
              }

            //If no anomaly
            if (anomalyStart[0] == 0)
            {
                servoCenter();
                nextState = previousState;  //No can, so go back to previous behaviour (search or can align)
                flag.stateChange = 1;
            }
            else    //Set servo to point at centre of can if anomaly found
            {
                newAngle = PWM_SERVO_UPPER;
                for (i=NUMBER_OF_ANGLES_CHECKED;i>canAnomaly;i--)
                {
                    newAngle -= servoA.speed;
                }
                TA1CCR2 = newAngle;
                nextState = CAN_APPROACH;
                flag.stateChange = 1;
            }
        }
    }
    else    //Take another reading at same angle
    {
        timeIncrement(&Schedule.ultraRADARStart, 0, 20);
    }
}

void    findAverage(unsigned char start, unsigned char numOfValues, int *values)
{
    unsigned char increments = 0;
	avgReading = 0;
	avgNewReading = 0;

	for(increments=start;increments<numOfValues;increments++, values++)
	{
		avgNewReading += *values;

		//If value has wrapped set to the maximum
		if (avgNewReading < avgReading)
		{
			avgReading = 32000;
			break;
		}
		avgReading = avgNewReading;
	}
	if (avgReading != 32000)
	{
		avgReading = avgReading/WALL_READINGS;
	}
}
//==============================================================================
// End of File : AutonomousCar/main.c
