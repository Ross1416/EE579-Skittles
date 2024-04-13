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
09-APR-2024 SARK added right ultrasonic code
09-APR-2024 RI added AL's IR sensor functionality
10-APR-2024 SARK debugged to allow car to nudge into a can
11-APR-2024 SARK got car to knock over a can & started looking into colour sensing
12-APR-2024 SARK got colour sensing to work and working on realigning to wall to repeat
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
#include "IndicatorLED.h"
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
    char ultraLeftRead;
	char ultraRightRead;
    char ultraRADARRead;
};

//Structure contains all timings that events occur on
struct Scheduler {
    //Time at which debouncing finishes
    struct Time debounce;

    //Time to start an ultrasonic reading
    struct Time ultraLeftStart;
	struct Time ultraRightStart;
	struct Time ultraRADARStart;

    //Time at which car state should change
    struct Time stateChange;

    //Time at which motor PWM should change
    struct Time pwmMotorDrive;
    struct Time pwmMotorSteer;

    //Time to change movement
    struct Time movementChange;
	
	//Time to start searching for cans
	struct Time searchStart;
};

//==============================================================================
// Function Prototypes
//------------------------------------------------------------------------------
//Interrupt Service Routines
__interrupt void Port1_ISR(void);
__interrupt void Timer0_A0_ISR(void);
__interrupt void Timer0_A1_ISR(void);
__interrupt void Timer1_A0_ISR (void);
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
void    RADARFindWall();
void    canLock();
void    findDistanceAverage(unsigned char start, unsigned char end, unsigned int *values);
void    changingState();
void    circumnavigate();

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
#define SPRINT          1
#define SEARCH      	2
#define CAN_ALIGN   	3
#define RADAR_SCAN  	4
#define CAN_APPROACH	5
#define COLOUR_DETECT 	6
#define REVERSE_CAN     7
#define CAN_HIT			8
#define CIRCUMNAVIGATE  9
#define WALL_REALIGN    10
#define STOP        	11

//Wall alignment
#define STRAIGHT    	0
#define AWAY        	1
#define CLOSE       	2
#define STRAIGHTEN  	3

//==============================================================================
// CHANGEABLE SETTINGS MACROs
//------------------------------------------------------------------------------
//Scheduler control
#define TIMER_INC_MS    2           //Scheduler interrupts period (2 ms)

//Drive Speeds
#define MOTOR_PWM_PERIOD    50                      //PWM period of motors.
#define SPEED_TOP           0.4*MOTOR_PWM_PERIOD    //Top speed of the motors as percentage of PWM.
#define SPEED_SLOW          0.3*MOTOR_PWM_PERIOD    //Slower speed when changing direction.
#define READINGS_TO_SPEED_CHANGE    5               //Number of distance readings taken whilst turning before slowing down.

//Wall readings control
#define WALL_READINGS   5                   //Number of wall readings remembered (MAKE 1+2^x)
#define AVERAGE_SHIFT   2					//log2(WALL_READINGS-1)
#define WALL_TOLERANCE  dist2pulse(2)       //Distance +- correct distance from wall
#define CAN_DETECT_DIST dist2pulse(10)      //Distance closer then wall
#define TIME_BEFORE_SEARCHING   1

//Time to align to can
//#define ALIGNMENT_TIME   850

//RADAR Reading control
#define RADAR_READINGS   5					//Make equal to wall readings
#define MAX_FRONT_DETECT 1500
#define ANOMALY_DISTANCE 500

//RADAR SCANNING
#define NUMBER_OF_ANGLES_CHECKED    15
#define MAX_RADAR_DISTANCE          dist2pulse(50)

//CAN APPROACH
#define FACE_CAN_ANGLE_TOLERANCE    250
#define FACE_CAN_DIST_TOLERANCE     500

//REVERSE TIME FOR RUN UP TO CAN
#define REVERSE_TIME   500
#define REVERSE_SPEED  0.6

//HIT CAN
#define HIT_TIME   900

//CIRCUMNAVIGATE
#define CIRCUMNAVIGATE_SPEED 0.75

//REALIGN TO WALL
#define REALIGN_TIME_S  1
#define REALIGN_TIME_MS 250
#define REALIGN_SPEED   0.4



//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------
//Scheduling variables
struct Time currentTime     =   {0, 0};    //Running count of time
struct Scheduler Schedule   =   {0};       //Schedule when events should
struct flags flag           =   {0};       //Flag when something needed to be attended

//Hardware Variable Instantiation
//Indicator LED (Port 2)
struct IndicateLED LEDTop = {2, BIT5};
//DC Motor info (Port 1)
struct MotorDC motorDrive = {0, BIT5, BIT4, {0, MOTOR_PWM_PERIOD, 0, SPEED_TOP, 1}};
struct MotorDC motorSteer = {0, BIT6, BIT7, {0, MOTOR_PWM_PERIOD, 0, MOTOR_PWM_PERIOD, 1}};
//Wall Ultrasonic info (Port 2)
struct Ultrasonic ultraLeft = {0, {0, 0}, 0, BIT0, BIT2, 2};
struct Ultrasonic ultraRight = {0, {0, 0}, 0, BIT0, BIT1, 2};
//RADAR Ultrasonic info (Port 1)
struct Ultrasonic ultraRADAR = {0, {0, 0}, 0, BIT0, BIT2, 1};
//Servo info (Port 2)
struct Servo servoRADAR = {BIT4, (PWM_SERVO_UPPER-PWM_SERVO_LOWER)/NUMBER_OF_ANGLES_CHECKED, 0};
// IR info (Port 2)
struct Infrared IR = {2, BIT3, 2};

//General Variables
unsigned int avgReading = 0;
unsigned int avgOldReading = 0;
unsigned int avgNewReading = 0;
unsigned int i = 0;
unsigned int j = 0;

//What car should be doing
char previousState  =   START;
char state          =   START;
char nextState      =   START;

//Start state variables
unsigned char fillBufferCount = 0;

//Wall alignment & search info
unsigned int leftWall;			//Initial distance to maintain to left wall
unsigned int rightWall;         //Initial distance to maintain to right wall
char turnState = STRAIGHT;				//Wall alignment turning instruction
unsigned int  wallDistancesLeft[WALL_READINGS] = {[0 ... WALL_READINGS-1] = 0};	//Distance readings to left wall
unsigned int  wallDistancesRight[WALL_READINGS] = {[0 ... WALL_READINGS-1] = 0};	//Distance readings to left wall
unsigned char turnStateTime = 0;					//Time spent turning to correct for wall
unsigned char canPosition = STRAIGHT;
unsigned char search = 0;
unsigned int RADARDistances[RADAR_READINGS] = {[0 ... RADAR_READINGS-1] = 2000};

//Can alignment
unsigned int timeToCanAlign = 0;

//RADAR Scan Variables
unsigned char readingsOnAngle = 0;
unsigned char anglesChecked = 0;
unsigned char anomalyPositions[NUMBER_OF_ANGLES_CHECKED] = {0};
unsigned char anomalyNumber = 0;
unsigned char canAnomaly = 0;
unsigned char anomalyStart[NUMBER_OF_ANGLES_CHECKED] = {[0 ... NUMBER_OF_ANGLES_CHECKED-1] = 0};;
unsigned char anomalyEnd[NUMBER_OF_ANGLES_CHECKED] = {[0 ... NUMBER_OF_ANGLES_CHECKED-1] = 0};;
unsigned int RADARAtEachAngle[NUMBER_OF_ANGLES_CHECKED] = {[0 ... NUMBER_OF_ANGLES_CHECKED-1] = 0};
unsigned int newAngle;

//Approaching can variables
unsigned char lostCan = 0;
unsigned int  lostCount = 0;
unsigned int  avgAnomalyDistance = 64000;
unsigned int  expectedCanDistance = 64000;
unsigned int  avgNewAnomalyDistance = 64000;

//Circumnavigate
unsigned char movement = 0;

//Wall realignment
unsigned char onAngle = 0;
unsigned char minAngle = 0;


//Flags for after event flags are dealt with and then results are used in stateControl()
char buttonPressed = 0;
char ultraReadLeft = 0;
char ultraReadRight = 0;
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
	
	if (ultraRight.timeNumber != 0)
    {
        ultraRight.time[1] += TA1CCR0;
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
        if(TA1CCTL1 & CCIS_1)	//LEFT Ultrasonic being used
        {
            ultraLeft.time[ultraLeft.timeNumber] += TA1CCR1;
            ultraLeft.timeNumber++;
            if (ultraLeft.timeNumber==2)       //After up/down edges of feedback
            {
                ultraLeft.distance = ultraLeft.time[1]-ultraLeft.time[0];
                flag.ultraLeftRead = 1;
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
		else
		{
            ultraRight.time[ultraRight.timeNumber] += TA1CCR1;
            ultraRight.timeNumber++;
            if (ultraRight.timeNumber==2)       //After up/down edges of feedback
            {
                ultraRight.distance = ultraRight.time[1]-ultraRight.time[0];
                flag.ultraLeftRead = 1;
                ultraRight.time[0] = 0;
                ultraRight.time[1] = 0;
                ultraRight.timeNumber=0;

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

    //Setup User Interface
    setupButton();

    //IndicatorLEDSetup
    indicatorLEDSetup(&LEDTop);
    indicatorLEDOff(&LEDTop);
	
	//Car DC Motor Setup
    motorSetup(&motorDrive);
    motorSetup(&motorSteer);
	
	//Setup Ultrasonic Sensors
    ultrasonicSetup(&ultraLeft);
	ultrasonicSetup(&ultraRight);
    ultrasonicSetup(&ultraRADAR);

    //Infrared Setup
    IRSetup(&IR);
	
	//Setup Servo Motors
	servoSetup(&servoRADAR);
	
	//Setup Internal Timers
	setupTimerRADAR();
    setupTimerSchedule();

    //Disable schedules
    Schedule.debounce.sec = 0;
    Schedule.debounce.ms = -1;

    Schedule.stateChange.sec = 0;
    Schedule.stateChange.ms = -1;

	Schedule.ultraLeftStart.sec = 0;
    Schedule.ultraLeftStart.ms = -1;
		
	Schedule.ultraRightStart.sec = 0;
    Schedule.ultraRightStart.ms = -1;

    Schedule.movementChange.sec = 0;
    Schedule.movementChange.ms = -1;

	Schedule.searchStart.sec = 0;
	Schedule.searchStart.ms = -1;

	//Start DC motor PWM schedules but set both motors output to do nothing
    timeIncrement(&Schedule.pwmMotorDrive, motorDrive.pwm.aSec, motorDrive.pwm.aMs);
    motorDrive.pwm.state = 1;
    flag.motorDrive = 1;
    motorDrive.direction = OFF;

    timeIncrement(&Schedule.pwmMotorSteer, motorSteer.pwm.aSec, motorSteer.pwm.aMs);
    motorSteer.pwm.state = 1;
    flag.motorSteer = 1;
    motorSteer.direction = STRAIGHT;

    //CANT REMEMBER WHAT THIS WAS FOR, TRY REMOVING XXX
    P1DIR |= BIT0;
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
            checkFlags();       //Deal with events that occurred
			stateControl();		//Control car behaviour
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
	
	//Time for car to start searching for can and not just driving forward
    if(isTime(Schedule.searchStart))
    {
		//Attend to in checkFlags()
        if (state == SEARCH)
        {
            search = 1;
        }
		
		//Disable schedule
        Schedule.searchStart.sec = 0;
        Schedule.searchStart.ms = -1;
    }

    if (isTime(Schedule.ultraLeftStart))
    {
        ultrasonicTrigger(&ultraLeft);

        //Disable schedule
        Schedule.ultraLeftStart.sec = 0;
        Schedule.ultraLeftStart.ms = -1;
    }
	
    if (isTime(Schedule.ultraRightStart))
    {
        ultrasonicTrigger(&ultraRight);
	
        //Disable schedule
        Schedule.ultraRightStart.sec = 0;
        Schedule.ultraRightStart.ms = -1;
    }
	
	if (isTime(Schedule.ultraRADARStart))
    {
        ultrasonicTrigger(&ultraRADAR);

        //Disable schedule
        Schedule.ultraRADARStart.sec = 0;
        Schedule.ultraRADARStart.ms = -1;
    }
	
	if (isTime(Schedule.movementChange))
    {
        movement++;

        //Disable schedule
        Schedule.movementChange.sec = 0;
        Schedule.movementChange.ms = -1;
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
            if (motorSteer.pwm.aMs == 0)
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

	//Left wall ultrasonic has reading ready
    if (flag.ultraLeftRead)
    {
		//Update array of wall readings
        for(i = WALL_READINGS-1; i > 0; i--)
        {
            wallDistancesLeft[i] = wallDistancesLeft[i - 1];
        }
        wallDistancesLeft[0] = ultraLeft.distance;
		
        if (state != START)
        {
            //Check whether a single reading is an outlier & if so ignore it
            if (((wallDistancesLeft[1] >= wallDistancesLeft[2]+2000) || (wallDistancesLeft[1] <= wallDistancesLeft[2]-2000))
                && ((wallDistancesLeft[1] >= wallDistancesLeft[0]+2000) || (wallDistancesLeft[1] <= wallDistancesLeft[0]-2000)))
            {
                wallDistancesLeft[1] = (wallDistancesLeft[0]+wallDistancesLeft[2]) >> 1;
            }
        }
		
		///Attend to in stateControl()
		ultraReadLeft = 1;
		
        flag.ultraLeftRead = 0;
    }

	//Right wall ultrasonic has reading ready
    if (flag.ultraRightRead)
    {
		//Update array of wall readings
        for(i = WALL_READINGS-1; i > 0; i--)
        {
            wallDistancesRight[i] = wallDistancesRight[i - 1];
        }
        wallDistancesRight[0] = ultraRight.distance;
		
		//Check whether a single reading is an outlier & if so ignore it
		if (((wallDistancesRight[1] >= wallDistancesRight[2]+2000) || (wallDistancesRight[1] <= wallDistancesRight[2]-2000))
			&& ((wallDistancesRight[1] >= wallDistancesRight[0]+2000) || (wallDistancesRight[1] <= wallDistancesRight[0]-2000)))
		{
			wallDistancesRight[1] = (wallDistancesRight[0]+wallDistancesRight[2]) >> 1;
		}
		
		///Attend to in stateControl()
		ultraReadRight = 1;
		
        flag.ultraRightRead = 0;
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

        if ((state != RADAR_SCAN) && (state != CAN_APPROACH) && (state != WALL_REALIGN))
        {
            //Check whether a single reading is an outlier & if so ignore it
            if (((RADARDistances[1] >= RADARDistances[2]+2000) || (RADARDistances[1] <= RADARDistances[2]-2000))
                && ((RADARDistances[1] >= RADARDistances[0]+2000) || (RADARDistances[1] <= RADARDistances[0]-2000)))
            {
                RADARDistances[1] = (RADARDistances[0]+RADARDistances[2]) >> 1;
            }
        }

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
    if (flag.stateChange)
    {
        flag.stateChange = 0;

		//Become next state
        previousState = state;
        state = nextState;
		
		changingState();
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
            //nextState = SEARCH;

            nextState = SEARCH;
			timeIncrement(&Schedule.stateChange, 1, 0);
			buttonPressed = 0;
			
			timeIncrement(&(Schedule.searchStart), 1+TIME_BEFORE_SEARCHING, 0);
		}
		
		//Use initial ultrasonic reading as distance to wall to maintain
		if (ultraReadLeft)
		{
		    if(TA1CCTL1 & CCIS_1)
		    {
		        leftWall = wallDistancesLeft[i];
		    }

		    if (fillBufferCount < WALL_READINGS)
		    {
		        fillBufferCount++;
		        timeIncrement(&(Schedule.ultraLeftStart), 0, 20);
		    }
			ultraReadLeft = 0;
		}

	}
	
//FOLLOW WALL WHILST LOOKING FOR CANS========================================================
	if(state == SEARCH)
	{
		//Drive forward at start (Done in state change)

		//Take ultrasonic readings to wall to stay aligned and if can is past on left
		if (ultraReadLeft)
		{
			findDistanceAverage(1, WALL_READINGS, wallDistancesLeft);

            if((avgReading < leftWall-CAN_DETECT_DIST) && (search))
            {
                canPosition = LEFT;
                //distToCan = avgReading;
                //angleToCan = PWM_SERVO_LOWER;
                timeToCanAlign = 999;
                nextState = CAN_ALIGN;
                flag.stateChange = 1;
                Schedule.stateChange.sec = 0;
                Schedule.stateChange.ms = -1;
            }
			
			//Use latest reading to keep aligned to wall
			alignToWall();

		    //Trigger next reading in 20 ms
		    timeIncrement(&(Schedule.ultraRADARStart), 0, 20);

			ultraReadLeft = 0;
		}
		
		//Take forward readings to see if can is in front of car
		if (ultraRADARRead)
		{
		    findDistanceAverage(1, RADAR_READINGS, RADARDistances);
			
            if(avgReading < MAX_FRONT_DETECT)
            {
				canPosition = STRAIGHT;
				nextState = RADAR_SCAN;
				flag.stateChange = 1;
				Schedule.stateChange.sec = 0;
				Schedule.stateChange.ms = -1;
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
        if (TA1CCR2 < 1500) //CAN TO THE RIGHT
        {
            motorSteer.direction = RIGHT;
        }
        else if (TA1CCR2 > 1500) //CAN TO THE LEFT
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
			canLock();
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
        //Functionality done when changing to state
        IRRead(&IR);

        // Decide HIT or AVOID depending on BLACK or WHITE can
        if (IR.colour)
        {
            indicatorLEDOn(&LEDTop);    //Black if 1
        }
        else
        {
            indicatorLEDOff(&LEDTop);   //White if 0
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
    if (state == CIRCUMNAVIGATE)
    {
		circumnavigate();
		
        //Stop if button pressed (FOR TESTING IF FAILS TO STOP)
        if (buttonPressed)
        {
            state = STOP;
            buttonPressed = 0;
        }
    }

//GET ROUGHLY STRAIGHT WITH WALL AND DRIVE PAST DEALT WITH CAN===========================
    if (state == WALL_REALIGN)
    {
		if (movement == 0)
		{
			//Realign to wall
			motorDrive.direction = BACK;
			motorDrive.pwm.aMs = 0.6*MOTOR_PWM_PERIOD;
			flag.motorDrive = 1;

			motorSteer.direction = canPosition;
			flag.motorSteer = 1;
			
			if (Schedule.movementChange.ms == -1)
            {
			    timeIncrement(&(Schedule.movementChange), 1, 250);
            }
		}
		
		if (movement == 1)
		{
			motorDrive.direction = OFF;
			flag.motorDrive = 1;

			motorSteer.direction = STRAIGHT;
			flag.motorSteer = 1;
			
			if (Schedule.ultraRADARStart.ms == -1)
			{
			    timeIncrement(&Schedule.ultraRADARStart, 0, 250);
			}
		}
		
		if (movement == 2)
		{
			motorDrive.direction = OFF;
			flag.motorDrive = 1;

			motorSteer.direction = STRAIGHT;
			flag.motorSteer = 1;
			
			nextState = SEARCH;

			if (Schedule.ultraLeftStart.ms == -1)
            {
                timeIncrement(&(Schedule.ultraLeftStart), 0, 20);
            }

			if (Schedule.stateChange.ms == -1)
            {
                timeIncrement(&(Schedule.stateChange), 1, 0);
            }
			
			if (Schedule.searchStart.ms == -1)
            {
                timeIncrement(&(Schedule.searchStart), 6, 0);
            }
		}
		
		if (ultraRADARRead)
		{
		    ultraRADARRead = 0;
			RADARFindWall();
		}

        //Use initial ultrasonic reading as distance to wall to maintain
        if (ultraReadLeft)
        {
			/*
            ultraReadLeft = 0;
			if (movement == 2)
			{
				//Turn until roughly straight again
				if ((wallDistancesLeft[0] - wallDistancesLeft[1] < 100) && (wallDistancesLeft[1] - wallDistancesLeft[0] < 100))
				{
					movement = 1;
				}
				timeIncrement(&(Schedule.ultraLeftStart), 0, 40);
			}
			else
			{
				if (fillBufferCount < WALL_READINGS)
				{
					fillBufferCount++;
					timeIncrement(&(Schedule.ultraLeftStart), 0, 20);
				}
				else
				{
					findDistanceAverage(1, WALL_READINGS, wallDistancesLeft);
					leftWall = avgReading;
				}
			}*/
			
            if (fillBufferCount < WALL_READINGS)
            {
                fillBufferCount++;
                timeIncrement(&(Schedule.ultraLeftStart), 0, 20);
            }
            else
            {
                findDistanceAverage(1, WALL_READINGS, wallDistancesLeft);
                leftWall = avgReading;
            }
            ultraReadLeft = 0;
            
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
    if(wallDistancesLeft[0] < leftWall-WALL_TOLERANCE)       //When drifted closer to wall
    {
        turnState = AWAY;
    }
    else if(wallDistancesLeft[0] > leftWall+WALL_TOLERANCE)  //When drifted further from wall
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

void    canLock()
{
    //Get an average
    //findDistanceAverage(1, RADAR_READINGS, RADARDistances);

    //When reading is close enough
    if(RADARDistances[0] < FACE_CAN_DIST_TOLERANCE)
    {
        indicatorLEDOff(&LEDTop);
        if ((TA1CCR2 > 1500-FACE_CAN_ANGLE_TOLERANCE) && (TA1CCR2 < 1500+FACE_CAN_ANGLE_TOLERANCE))   //If facing can
        {
            //Stop driving
            motorDrive.direction = OFF;
            motorSteer.direction = STRAIGHT;
            flag.motorDrive = 1;
            flag.motorSteer = 1;

            nextState = COLOUR_DETECT;
            flag.stateChange = 1;
            Schedule.stateChange.sec = 0;
            Schedule.stateChange.ms = -1;
        }
        else
        {
            nextState = CAN_ALIGN;
            flag.stateChange = 1;
            Schedule.stateChange.sec = 0;
            Schedule.stateChange.ms = -1;
            timeToCanAlign = 850;
        }
    }
    else if (RADARDistances[0] < FACE_CAN_DIST_TOLERANCE+300) //If slightly further away but angle really bad
    {
        if ((TA1CCR2 < 1000) || (TA1CCR2 > 2000))
        {
            nextState = CAN_ALIGN;
            flag.stateChange = 1;
            Schedule.stateChange.sec = 0;
            Schedule.stateChange.ms = -1;
            timeToCanAlign = 850;
        }
    }

    //Behaviours whether can is lost
    if (lostCan)    //If can is lost
    {
        //if (avgReading < expectedCanDistance+400) //Can refound
        if (((RADARDistances[0] < RADARDistances[1]-ANOMALY_DISTANCE) && (RADARDistances[0] < expectedCanDistance + 300))
            || (RADARDistances[0] < expectedCanDistance + 300))
        {
            motorDrive.direction = FORWARD;
            flag.motorDrive = 1;

            expectedCanDistance = RADARDistances[0];
            lostCan = 0;
            lostCount = 0;

            indicatorLEDOn(&LEDTop);
            timeIncrement(&Schedule.ultraRADARStart, 0, 20);
        }
        else    //Can still lost
        {
            lostCan = 1;
            motorDrive.direction = OFF;
            flag.motorDrive = 1;

            if (RADARDistances[0] > RADARDistances[1] + ANOMALY_DISTANCE) //Probably just moved off can
            {
                servoRADAR.direction ^= 1;
            }
            else if (lostCount > 4)
            {
                nextState = RADAR_SCAN;
                flag.stateChange = 1;
                Schedule.stateChange.sec = 0;
                Schedule.stateChange.ms = -1;
                /*
                if (lostCount == 3)
                {
                    servoRADAR.direction ^= 1;
                }
                else if (lostCount == 6)
                {
                    servoRADAR.direction ^= 1;
                }
                else if (lostCount > 4)
                {
                    nextState = RADAR_SCAN;
                    flag.stateChange = 1;
                    Schedule.stateChange.sec = 0;
                    Schedule.stateChange.ms = -1;
                }
                */
            }

            indicatorLEDOff(&LEDTop);
            servoTurn(&servoRADAR);
            timeIncrement(&Schedule.ultraRADARStart, 0, 200);
            lostCount++;
        }
    }
    else    //If can is not lost
    {
        if (RADARDistances[0] > expectedCanDistance+500)    //Now lost
        {
            lostCan = 1;
            motorDrive.direction = OFF;
            flag.motorDrive = 1;

            if (TA1CCR2 >= 1500)
            {
                servoRADAR.direction = 0;
            }
            else if  (TA1CCR2 < 1500)
            {
                servoRADAR.direction = 1;
            }
            else
            {
                /*
                nextState = RADAR_SCAN;
                flag.stateChange = 1;*/
            }

            indicatorLEDOff(&LEDTop);
            servoTurn(&servoRADAR);
            timeIncrement(&Schedule.ultraRADARStart, 0, 200);
            lostCount = 1;
        }
        else    //Still looking at can
        {
            lostCan = 0;
            lostCount = 0;
            motorDrive.direction = FORWARD;
            flag.motorDrive = 1;

            expectedCanDistance = RADARDistances[0]; //Update latest distance to can

            indicatorLEDOn(&LEDTop);
            timeIncrement(&Schedule.ultraRADARStart, 0, 20);
        }
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
        findDistanceAverage(1, RADAR_READINGS, RADARDistances);
        RADARAtEachAngle[anglesChecked] = avgReading;
		//anglesChecked = 0; //XXX

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
                if (avgReading < avgOldReading-ANOMALY_DISTANCE)
                {
                    anomalyPositions[anglesChecked] = 1;
                    anomalyStart[anomalyNumber] = anglesChecked;
                    indicatorLEDOn(&LEDTop);
                }
                else
                {
                    anomalyPositions[anglesChecked] = 0;
                    indicatorLEDOff(&LEDTop);
                }
            }
            else    //If last reading is anomaly
            {
                //When reading suddenly further way
                if (avgReading > avgOldReading+100)
                {
                    anomalyPositions[anglesChecked] = 0;
                    anomalyEnd[anomalyNumber] = anglesChecked;
                    anomalyNumber++;
                    indicatorLEDOff(&LEDTop);
                }
                else if (avgReading < avgOldReading-ANOMALY_DISTANCE) //Start of a new anomaly if in front of old anomaly
                {
                    anomalyPositions[anglesChecked] = 1;
                    anomalyEnd[anomalyNumber] = anglesChecked;
                    anomalyNumber++;
                    anomalyStart[anomalyNumber] = anglesChecked;
                    indicatorLEDToggle(&LEDTop);
                }
                else
                {
                    anomalyPositions[anglesChecked] = 1;
                    indicatorLEDOn(&LEDTop);
                }
            }
        }

        //Finished dealing with new reading so increment ready for next
        anglesChecked++;

        //Have all angles been checked
        if (anglesChecked < NUMBER_OF_ANGLES_CHECKED)
        {
            //Go to next angle
            servoTurn(&servoRADAR);
            timeIncrement(&Schedule.ultraRADARStart, 0, 200);
        }
        else    //Turn to where anomaly is thought to be after all angles checked
        {
            //If no anomaly
            if (anomalyStart[0] == 0)
            {
                servoCenter();
                nextState = SEARCH;  //No can, so go back to searching
                flag.stateChange = 1;
                Schedule.stateChange.sec = 0;
                Schedule.stateChange.ms = -1;
				
				timeIncrement(&Schedule.searchStart, 0, 50);
            }
            else    //Set servo to point at centre of closest anomaly found
            {
                //If last anomaly never ended
                if ((anomalyStart[anomalyNumber] != 0) && (anomalyEnd[anomalyNumber] == 0))
                {
                    anomalyEnd[anomalyNumber] = NUMBER_OF_ANGLES_CHECKED-1;
                }

                /*
                //If anomaly is left-most
                if ((anomalyEnd[anomalyNumber] = NUMBER_OF_ANGLES_CHECKED-1) &
                        ((RADARAtEachAngle[14] < leftWall+100) & (RADARAtEachAngle[14] > leftWall-100)))
                {
                    anomalyStart[anomalyNumber] = 0;
                    anomalyEnd[anomalyNumber] = 0;
                    anomalyNumber--;
                }*/

                //Find closest anomaly
                expectedCanDistance = 65000;
                //For each anomaly
                for (readingsOnAngle = 0; readingsOnAngle<anomalyNumber+1; readingsOnAngle++)
                {
                    //Average reading of the anomaly
                    avgAnomalyDistance = 0;
                    avgNewAnomalyDistance = 0;
                    for(j=anomalyStart[readingsOnAngle];j<1+anomalyEnd[readingsOnAngle];j++)
                    {
                        avgNewAnomalyDistance += RADARAtEachAngle[j];
                        //If value has wrapped set to the maximum
                        if (avgNewAnomalyDistance < avgAnomalyDistance)
                        {
                            avgAnomalyDistance = 65000;
                            break;
                        }
                        avgAnomalyDistance = avgNewAnomalyDistance;
                    }
                    if (avgAnomalyDistance != 65000)
                    {
                        avgAnomalyDistance = avgAnomalyDistance/(1+anomalyEnd[readingsOnAngle]-anomalyStart[readingsOnAngle]);
                    }

                    if (avgAnomalyDistance < expectedCanDistance)
                    {
                        expectedCanDistance = avgAnomalyDistance;
                        canAnomaly = anomalyStart[readingsOnAngle]+((anomalyEnd[readingsOnAngle]-anomalyStart[readingsOnAngle])>>1);
                    }
                }
                readingsOnAngle = 0;

                newAngle = PWM_SERVO_UPPER;
                for (j=NUMBER_OF_ANGLES_CHECKED;j>canAnomaly;j--)
                {
                    newAngle -= servoRADAR.speed;
                }
                TA1CCR2 = newAngle;
                nextState = CAN_APPROACH;
                flag.stateChange = 1;
                Schedule.stateChange.sec = 0;
                Schedule.stateChange.ms = -1;
            }
        }
    }
    else    //Take another reading at same angle
    {
        timeIncrement(&Schedule.ultraRADARStart, 0, 20);
    }
}

void    findDistanceAverage(unsigned char start, unsigned char end, unsigned int *values)
{
    unsigned char increments = 0;
	avgReading = 0;
	avgNewReading = 0;

	for(increments=start;increments<end;increments++, values++)
	{
		avgNewReading += *values >> AVERAGE_SHIFT;

		//If value has wrapped set to the maximum
		if (avgNewReading < avgReading)
		{
			avgReading = 65000;
			break;
		}
		avgReading = avgNewReading;
	}
}

//Events to occur when changing to each state
void    changingState()
{
	if (state == START)
	{
		//Count for wall buffer
		fillBufferCount = 0;
	}

	if (state == SEARCH)
	{
		//Don't search and only follow wall for an amount of time
		search = 0;
		
		//Start driving forward
		motorDrive.direction = FORWARD;
		motorDrive.pwm.aMs = SPEED_TOP;
		flag.motorDrive = 1;

		//Centre RADAR
		servoCenter();

		//Initiate first ultrasonic reading
		ultrasonicTrigger(&ultraLeft);
	}
	
	if (state == CAN_ALIGN)
	{
		motorDrive.direction = BACK;
		motorDrive.pwm.aMs = 0.6*MOTOR_PWM_PERIOD;
		flag.motorDrive = 1;

		motorSteer.direction = RIGHT;
		flag.motorSteer = 1;
		
		nextState = RADAR_SCAN;
		timeIncrement(&Schedule.stateChange, 0, timeToCanAlign);
	}
	
	if (state == RADAR_SCAN)
	{
		//Clear variables used in state
		readingsOnAngle = 0;
		anglesChecked = 0;
		newAngle = 0;
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
		servoRADAR.direction = 1;
		
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
	
	if (state == CAN_APPROACH)
	{
		motorDrive.pwm.aMs = 0.3*MOTOR_PWM_PERIOD;
		motorDrive.direction = FORWARD;
		flag.motorDrive = 1;
		
		lostCan = 0;
		lostCount = 0;
		
		timeIncrement(&Schedule.ultraRADARStart, 0, 20);
	}
	
	if (state == COLOUR_DETECT)
	{
	    motorDrive.direction = OFF;
        flag.motorDrive = 1;

        motorSteer.direction = STRAIGHT;
        flag.motorSteer = 1;

		// Check IR sensor - get colour
		IRRead(&IR);

		// Decide HIT or AVOID depending on BLACK or WHITE can
		if (IR.colour)
		{
			// Hit can if BLACK
			nextState = REVERSE_CAN;
			flag.stateChange = 1;
			Schedule.stateChange.sec = 0;
			Schedule.stateChange.ms = -1;
			indicatorLEDOn(&LEDTop);
		}
		else
		{
			// Avoid can if WHITE
		    if (canPosition  == STRAIGHT)
		    {
		        nextState = CIRCUMNAVIGATE;
		    }
		    else
		    {
		        nextState = WALL_REALIGN;
		    }
			flag.stateChange = 1;
			Schedule.stateChange.sec = 0;
			Schedule.stateChange.ms = -1;
			indicatorLEDOff(&LEDTop);
		}
	}
	
	if (state == REVERSE_CAN)
	{
		motorDrive.direction = BACK;
		motorDrive.pwm.aMs = REVERSE_SPEED*MOTOR_PWM_PERIOD;
		flag.motorDrive = 1;

		motorSteer.direction = STRAIGHT;
		flag.motorSteer = 1;
		
		nextState = CAN_HIT;
		timeIncrement(&Schedule.stateChange, 0, REVERSE_TIME);
	}
	
	if (state == CAN_HIT)
	{
		motorDrive.direction = FORWARD;
		motorDrive.pwm.aMs = MOTOR_PWM_PERIOD;
		flag.motorDrive = 1;

		motorSteer.direction = STRAIGHT;
		flag.motorSteer = 1;
		
		nextState = WALL_REALIGN;
		timeIncrement(&Schedule.stateChange, 0, HIT_TIME);
	}
	
	if (state == CIRCUMNAVIGATE)
	{
	    movement = 0;
	}
	
	if (state == WALL_REALIGN)
	{
	    //XXX For testing
	    //canPosition = LEFT;
	    //TA1CCR2 = 1500 + servoRADAR.speed;

		if (canPosition == LEFT)
		{
			servoRADAR.direction = 1;
		}
		else
		{
			servoRADAR.direction = 0;
		}
		minAngle = NUMBER_OF_ANGLES_CHECKED-((TA1CCR2 - PWM_SERVO_LOWER)/servoRADAR.speed);
		onAngle = minAngle;
		readingsOnAngle = 0;
		
		//Realign to wall
		motorDrive.direction = BACK;
		motorDrive.pwm.aMs = REALIGN_SPEED*MOTOR_PWM_PERIOD;
		flag.motorDrive = 1;

		motorSteer.direction = canPosition;
		flag.motorSteer = 1;
		
		movement = 1;
		
		//timeIncrement(&(Schedule.movementChange), REALIGN_TIME_S, REALIGN_TIME_MS);
		
		//Get new wall distance to maintain after realigning
		fillBufferCount = 0;
        //timeIncrement(&(Schedule.ultraLeftStart), REALIGN_TIME_S, REALIGN_TIME_MS);
		//timeIncrement(&(Schedule.ultraLeftStart), 0, 20);
        
		//timeIncrement(&(Schedule.ultraRADARStart), 0, 20);
		
		//Switch to search state when new wall value obtained
		//nextState = SEARCH;
        //timeIncrement(&Schedule.stateChange, 1+REALIGN_TIME_S, REALIGN_TIME_MS);
        //buttonPressed = 0;
        
		//Drive forward for 5 seconds before looking for cans again
        //timeIncrement(&(Schedule.searchStart), 6+REALIGN_TIME_S, 0);
	}
	
	if (state == STOP)
	{
		indicatorLEDOff(&LEDTop);

		//Stop all motors and readings
		motorDrive.direction = OFF;
		motorSteer.direction = STRAIGHT;
        flag.motorDrive = 1;
        flag.motorSteer = 1;
	}
}

void circumnavigate()
{
	if (movement == 0)
	{
		motorDrive.direction = BACK;
		motorDrive.pwm.aMs = CIRCUMNAVIGATE_SPEED*MOTOR_PWM_PERIOD;
		flag.motorDrive = 1;

		motorSteer.direction = STRAIGHT;
		flag.motorSteer = 1;

		if ((Schedule.movementChange.sec == 0) && (Schedule.movementChange.ms == -1))
		{
			timeIncrement(&Schedule.movementChange, 0, 800);
		}
	}
	else if (movement == 1)
	{
		motorDrive.direction = FORWARD;
		motorDrive.pwm.aMs = CIRCUMNAVIGATE_SPEED*MOTOR_PWM_PERIOD;
		flag.motorDrive = 1;

		motorSteer.direction = LEFT;
		flag.motorSteer = 1;

		if ((Schedule.movementChange.sec == 0) && (Schedule.movementChange.ms == -1))
		{
			timeIncrement(&Schedule.movementChange, 1, 250);
		}
	}
	else if (movement == 2)
	{
		motorDrive.direction = FORWARD;
		motorDrive.pwm.aMs = 0.5*MOTOR_PWM_PERIOD;
		flag.motorDrive = 1;

		motorSteer.direction = RIGHT;
		flag.motorSteer = 1;

		if ((Schedule.movementChange.sec == 0) && (Schedule.movementChange.ms == -1))
		{
			timeIncrement(&Schedule.movementChange, 1, 0);
		}
	}
	else if (movement == 3)
	{
		motorDrive.direction = FORWARD;
		motorDrive.pwm.aMs = 0.4*MOTOR_PWM_PERIOD;
		flag.motorDrive = 1;

		motorSteer.direction = LEFT;
		flag.motorSteer = 1;

		if ((Schedule.movementChange.sec == 0) && (Schedule.movementChange.ms == -1))
		{
			timeIncrement(&Schedule.movementChange, 0, 600);
		}
	}
	else
	{
		nextState = SEARCH;
		flag.stateChange = 1;
		timeIncrement(&Schedule.searchStart, 0, 50);
	}
}

void RADARFindWall()
{
    readingsOnAngle++;
    //When readings full at current angle
    if (readingsOnAngle == RADAR_READINGS)
    {
        //Get average reading for angle set to
        readingsOnAngle = 0;
        avgOldReading = avgReading;
        findDistanceAverage(1, RADAR_READINGS, RADARDistances);
        RADARAtEachAngle[onAngle] = avgReading;

    //RADARAtEachAngle[onAngle] = RADARDistances[0];
        if (TA1CCR2 == PWM_SERVO_LOWER)
        {
            for (i = minAngle; i > 0 ; i--)
            {
                if (RADARAtEachAngle[i-1] < RADARAtEachAngle[minAngle])
                {
                    minAngle = i-1;
                }
            }
            TA1CCR2 = PWM_SERVO_LOWER+(NUMBER_OF_ANGLES_CHECKED-minAngle)*servoRADAR.speed;
            onAngle = minAngle;
            if (minAngle == 0)
            {
                movement = 2;
            }
            else
            {
                movement = 0;
            }
        }
        else if (TA1CCR2 == PWM_SERVO_UPPER)
        {
            for (i = minAngle; i < NUMBER_OF_ANGLES_CHECKED ; i++)
            {
                if (RADARAtEachAngle[i] < RADARAtEachAngle[minAngle])
                {
                    minAngle = i;
                }
                else if (RADARAtEachAngle[i] > RADARAtEachAngle[minAngle]+500)
                {
                    minAngle = i;
                }
                else if (RADARAtEachAngle[i] > RADARAtEachAngle[minAngle]+100)
                {
                    break;
                }
            }
            TA1CCR2 = PWM_SERVO_LOWER+minAngle*servoRADAR.speed;
            onAngle = minAngle;
            if (minAngle == NUMBER_OF_ANGLES_CHECKED-1)
            {
                movement = 2;
            }
            else
            {
                movement = 0;
            }
        }
        else
        {
            servoTurn(&servoRADAR);
            if (servoRADAR.direction)
            {
                onAngle++;
            }
            else
            {
                onAngle--;
            }
            timeIncrement(&Schedule.ultraRADARStart, 0, 200);
        }
    }
    else
    {
        timeIncrement(&Schedule.ultraRADARStart, 0, 20);
    }
}
//==============================================================================
// End of File : AutonomousCar/main.c
