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
07-APR-2024 SARK added SONAR scan to lock on to car
08-APR-2024 SARK fixed bug with ultrasonic readings and timer overflows
08-APR-2024 SARK added code for car to approach can
09-APR-2024 SARK added right ultrasonic code
09-APR-2024 RI added AL's IR sensor functionality
10-APR-2024 SARK debugged to allow car to nudge into a can
11-APR-2024 SARK got car to knock over a can & started looking into colour sensing
12-APR-2024 SARK got colour sensing to work and working on realigning to wall with SONAR
13-APR-2024 SARK working on retracing steps to realign to wall as SONAR unsuccessful
15-APR-2024 SARK implemented realigning to wall with side ultrasonic
16-APR-2024 SARK fixed changing wall position bug by aligning to wall with changes in
            reading instead of absolute distance to wall
22-APR-2024 SARK implemented left and right ultrasonic sensors as well as allowing
            "wall to align to" to be controllable
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
#include "UserInterface.h"
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
    char ultraSONARRead;
};

//Structure contains all timings that events occur on
struct Scheduler {
    //Time at which debouncing finishes
    struct Time debounce;

    //Time to start an ultrasonic reading
    struct Time ultraLeftStart;
	struct Time ultraRightStart;
	struct Time ultraSONARStart;

	struct Time ultraLeftStartBackup;
    struct Time ultraRightStartBackup;
    struct Time ultraSONARStartBackup;

    //Time at which car state should change
    struct Time stateChange;

    //Time at which motor PWM should change
    struct Time pwmMotorDrive;
    struct Time pwmMotorSteer;

    //Time to change movement
    struct Time movementChange;
	struct Time useNumber;
	
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

//Hardware setup
void    setupTimerSchedule();
void    setupTimerSONAR();

//Main body of code
int     main(void);
void    checkFlags();
void	stateControl();

//Scheduler functions
void    checkSchedule();
void    timeIncrement(struct Time *time, int sec, int ms);

//Other Functionality
void    alignToWall();
void	SONAR();
void    SONARFindWall();
void    canLock();
void    findDistanceAverage(unsigned char start, unsigned char end, unsigned int *values);
void    changingState();
void    circumnavigate();
void    updateCarHeading(unsigned char direction, unsigned char angle, unsigned int speed);

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
#define CLOSEST_WALL    0
#define READY       	1
#define SEARCH      	2
#define CAN_ALIGN   	3
#define SONAR_SCAN  	4
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
#define MOTOR_PWM_PERIOD    100                      //PWM period of motors.
#define SPEED_FORWARD       50    //Driving forward speed
#define SPEED_BACK          50    //Driving backward is slower so should have higher PWM to compensate
#define READINGS_TO_SPEED_CHANGE    5               //Number of distance readings taken whilst turning before slowing down.

//Wall readings control
#define WALL_READINGS   5                   //Number of wall readings remembered (MAKE 1+2^x)
#define AVERAGE_SHIFT   2					//log2(WALL_READINGS-1)
#define WALL_TOLERANCE  150                 //Distance +- correct distance from wall
#define CAN_DETECT_DIST 500                 //Distance closer then wall
#define TIME_BEFORE_SEARCHING   2
#define FIND_SPEED    40
#define SPRINT_SPEED  50

//Time to align to can
//#define ALIGNMENT_TIME   850

//SONAR Reading control
#define SONAR_READINGS   5					//Make equal to wall readings
#define MAX_FRONT_DETECT 800
#define ANOMALY_DISTANCE 500

//SONAR SCANNING
#define NUMBER_OF_ANGLES_CHECKED    15

//CAN ALIGN
#define CAN_ALIGN_SPEED 45
#define INITIAL_CAN_ALIGN_SEC   0
#define INITIAL_CAN_ALIGN_MS    900
#define SECONDARY_CAN_ALIGN_SEC 0
#define SECONDARY_CAN_ALIGN_MS  400

//CAN APPROACH
#define FACE_CAN_ANGLE_TOLERANCE    250
#define FACE_CAN_DIST_TOLERANCE     500
#define CAN_APPROACH_SPEED          40

//REVERSE TIME FOR RUN UP TO CAN
#define REVERSE_TIME   500
#define REVERSE_SPEED  40

//HIT CAN
#define HIT_TIME   900

//CIRCUMNAVIGATE
#define CIRCUMNAVIGATE_SPEED    40

//REALIGN TO WALL
#define REALIGN_SPEED   40

//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------
//Scheduling variables
struct Time currentTime     =   {0, 0};    //Running count of time
struct Scheduler Schedule   =   {0};       //Schedule when events should
struct flags flag           =   {0};       //Flag when something needed to be attended

//Hardware Variable Instantiation
//Indicator LED (Port 2)
struct IndicateLED LEDRed = {2, BIT5};
struct IndicateLED LEDBlue = {2, BIT6};
//Starting side switch
struct Switch sideSelect = {1, BIT1, 0};
//Button to control
struct Button startButton = {1, BIT3, 0};
//DC Motor info (Port 1)
struct MotorDC motorDrive = {0, BIT6, BIT7, {0, MOTOR_PWM_PERIOD, 0, 40, 1}};
struct MotorDC motorSteer = {0, BIT4, BIT5, {0, MOTOR_PWM_PERIOD, 0, MOTOR_PWM_PERIOD, 1}};
//Wall Ultrasonic info (Port 2)
struct Ultrasonic ultraLeft = {0, {0, 0}, 0, BIT0, BIT2, 2};
struct Ultrasonic ultraRight = {0, {0, 0}, 0, BIT0, BIT1, 2};
//SONAR Ultrasonic info (Port 1)
struct Ultrasonic ultraSONAR = {0, {0, 0}, 0, BIT0, BIT2, 1};
//Servo info (Port 2)
struct Servo servoSONAR = {BIT4, (PWM_SERVO_UPPER-PWM_SERVO_LOWER)/NUMBER_OF_ANGLES_CHECKED, 0};
// IR info (Port 2)
struct Infrared IR = {2, BIT3, 2};

//General Variables
unsigned int avgReading = 0;
unsigned int avgOldReading = 0;
unsigned int avgNewReading = 0;
unsigned int i = 0;
unsigned int j = 0;
unsigned char cansDealtWith = 0;

//What car should be doing
char previousState  =   CLOSEST_WALL;
char state          =   CLOSEST_WALL;
char nextState      =   CLOSEST_WALL;

//Calibrate Wall Variables
unsigned char fillBufferCount = 0;
unsigned char startSide = STRAIGHT;
unsigned int searchSpeeds = 50;

//Wall alignment & search info
char turnState = STRAIGHT;				//Wall alignment turning instruction
unsigned int  wallDistancesLeft[WALL_READINGS] = {[0 ... WALL_READINGS-1] = 0};	//Distance readings to left wall
unsigned int  wallDistancesRight[WALL_READINGS] = {[0 ... WALL_READINGS-1] = 0};	//Distance readings to left wall
unsigned char canPosition = STRAIGHT;
unsigned char search = 0;
unsigned int SONARDistances[SONAR_READINGS] = {[0 ... SONAR_READINGS-1] = 2000};
unsigned int closestWallLeft = 0;
unsigned int closestWallRight = 0;

//Can alignment
unsigned int timeToCanAlignSec = 0;
unsigned int timeToCanAlignMs = 0;

//SONAR Scan Variables
unsigned char readingsOnAngle = 0;
unsigned char anglesChecked = 0;
unsigned char anomalyPositions[NUMBER_OF_ANGLES_CHECKED] = {0};
unsigned char anomalyNumber = 0;
unsigned char canAnomaly = 0;
unsigned char anomalyStart[NUMBER_OF_ANGLES_CHECKED] = {[0 ... NUMBER_OF_ANGLES_CHECKED-1] = 0};;
unsigned char anomalyEnd[NUMBER_OF_ANGLES_CHECKED] = {[0 ... NUMBER_OF_ANGLES_CHECKED-1] = 0};;
unsigned int SONARAtEachAngle[NUMBER_OF_ANGLES_CHECKED] = {[0 ... NUMBER_OF_ANGLES_CHECKED-1] = 0};
unsigned int newAngle;

//Approaching can variables
unsigned char lostCan = 0;
unsigned int  lostCount = 0;
unsigned int  avgAnomalyDistance = 64000;
unsigned int  expectedCanDistance = 64000;
unsigned int  avgNewAnomalyDistance = 64000;

//Circumnavigate
unsigned char movement = 0;
unsigned char furtherWall = STRAIGHT;

//Wall realignment
unsigned int useNumber = 0;
unsigned int straightTime = 0;

//Flags for after event flags are dealt with and then results are used in stateControl()
char buttonPressed = 0;
char ultraReadLeft = 0;
char ultraReadRight = 0;
char ultraSONARRead = 0;


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
            if(++currentTime.sec == 300) currentTime.sec = 0;
        }
        flag.timerA0 = 1;

        //If wrap occurs during ultrasonic pulse
        if (ultraSONAR.timeNumber != 0)
        {
            ultraSONAR.time[1] += TA0CCR0;
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
            ultraSONAR.time[ultraSONAR.timeNumber] += TA0CCR1;
            ultraSONAR.timeNumber++;
            if (ultraSONAR.timeNumber==2)       //After up/down edges of feedback
            {
                ultraSONAR.distance = ultraSONAR.time[1]-ultraSONAR.time[0];
                flag.ultraSONARRead = 1;
                ultraSONAR.time[0] = 0;
                ultraSONAR.time[1] = 0;
                ultraSONAR.timeNumber=0;
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
                flag.ultraRightRead = 1;
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
    setupButton(&startButton);
    setupSwitch(&sideSelect);
    indicatorLEDSetup(&LEDRed);
    indicatorLEDSetup(&LEDBlue);
    indicatorLEDOff(&LEDRed);
    indicatorLEDOff(&LEDBlue);
	
	//Car DC Motor Setup
    motorSetup(&motorDrive);
    motorSetup(&motorSteer);
	
	//Ultrasonic Sensor Setup
    ultrasonicSetup(&ultraLeft);
	ultrasonicSetup(&ultraRight);
    ultrasonicSetup(&ultraSONAR);

    //Infrared Setup
    IRSetup(&IR);
	
	//Setup Servo Motor
	servoSetup(&servoSONAR);
	
	//Setup Internal Timers
	setupTimerSONAR();
    setupTimerSchedule();

    //Disable Schedules
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

    Schedule.useNumber.sec = 0;
    Schedule.useNumber.ms = -1;

	Schedule.searchStart.sec = 0;
	Schedule.searchStart.ms = -1;


	//Start DC motor PWM schedules but set both motors output to do nothing
    timeIncrement(&Schedule.pwmMotorDrive, motorDrive.pwm.aSec, motorDrive.pwm.aMs);
    motorDrive.pwm.state = 1;
	motorDrive.direction = OFF;
    flag.motorDrive = 1;

    timeIncrement(&Schedule.pwmMotorSteer, motorSteer.pwm.aSec, motorSteer.pwm.aMs);
    motorSteer.pwm.state = 1;
	motorSteer.direction = STRAIGHT;
    flag.motorSteer = 1;

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

        timeIncrement(&Schedule.ultraLeftStartBackup, 0, 100);

        //Disable schedule
        Schedule.ultraLeftStart.sec = 0;
        Schedule.ultraLeftStart.ms = -1;
    }
	
    if (isTime(Schedule.ultraRightStart))
    {
        ultrasonicTrigger(&ultraRight);

        timeIncrement(&Schedule.ultraRightStartBackup, 0, 100);
	
        //Disable schedule
        Schedule.ultraRightStart.sec = 0;
        Schedule.ultraRightStart.ms = -1;
    }
	
	if (isTime(Schedule.ultraSONARStart))
    {
        ultrasonicTrigger(&ultraSONAR);

        timeIncrement(&Schedule.ultraSONARStartBackup, 0, 100);

        //Disable schedule
        Schedule.ultraSONARStart.sec = 0;
        Schedule.ultraSONARStart.ms = -1;
    }

    if (isTime(Schedule.ultraLeftStartBackup))
    {
        ultrasonicTrigger(&ultraLeft);

        //Disable schedule
        Schedule.ultraLeftStart.sec = 0;
        Schedule.ultraLeftStart.ms = -1;
    }

    if (isTime(Schedule.ultraRightStartBackup))
    {
        ultrasonicTrigger(&ultraRight);

        //Disable schedule
        Schedule.ultraRightStart.sec = 0;
        Schedule.ultraRightStart.ms = -1;
    }

    if (isTime(Schedule.ultraSONARStartBackup))
    {
        ultrasonicTrigger(&ultraSONAR);

        //Disable schedule
        Schedule.ultraSONARStart.sec = 0;
        Schedule.ultraSONARStart.ms = -1;
    }
	
	if (isTime(Schedule.movementChange))
    {
        movement++;

        //Disable schedule
        Schedule.movementChange.sec = 0;
        Schedule.movementChange.ms = -1;
    }
	
	if (isTime(Schedule.useNumber))
    {
        useNumber++;

        //Disable schedule
        Schedule.useNumber.sec = 0;
        Schedule.useNumber.ms = -1;
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
		
        if ((state != READY) && (state != WALL_REALIGN))
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
		
		Schedule.ultraLeftStartBackup.ms = -1;

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
		
        if ((state != READY) && (state != WALL_REALIGN))
        {
			//Check whether a single reading is an outlier & if so ignore it
			if (((wallDistancesRight[1] >= wallDistancesRight[2]+2000) || (wallDistancesRight[1] <= wallDistancesRight[2]-2000))
				&& ((wallDistancesRight[1] >= wallDistancesRight[0]+2000) || (wallDistancesRight[1] <= wallDistancesRight[0]-2000)))
			{
				wallDistancesRight[1] = (wallDistancesRight[0]+wallDistancesRight[2]) >> 1;
			}
		}
		
		///Attend to in stateControl()
		ultraReadRight = 1;
		
		Schedule.ultraRightStartBackup.ms = -1;

        flag.ultraRightRead = 0;
    }
	
	//Wall ultrasonic has reading ready
    if (flag.ultraSONARRead)
    {
        //Update array of wall readings
        for(i = SONAR_READINGS-1; i > 0; i--)
        {
            SONARDistances[i] = SONARDistances[i - 1];
        }
        SONARDistances[0] = ultraSONAR.distance;

        if ((state != SONAR_SCAN) && (state != CAN_APPROACH) && (state != WALL_REALIGN))
        {
            //Check whether a single reading is an outlier & if so ignore it
            if (((SONARDistances[1] >= SONARDistances[2]+2000) || (SONARDistances[1] <= SONARDistances[2]-2000))
                && ((SONARDistances[1] >= SONARDistances[0]+2000) || (SONARDistances[1] <= SONARDistances[0]-2000)))
            {
                SONARDistances[1] = (SONARDistances[0]+SONARDistances[2]) >> 1;
            }
        }

		///Attend to in stateControl()
		ultraSONARRead = 1;
		
		Schedule.ultraSONARStartBackup.ms = -1;

        flag.ultraSONARRead = 0;
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
		
		changingState();
    }
}

void stateControl()
{
//CALIBRATE WHERE CLOSEST WALL WILL BE========================================================
    if (state == CLOSEST_WALL)
    {
		readSwitch(&sideSelect);
		if (sideSelect.val == 0)
		{
			startSide = LEFT;
			indicatorLEDOff(&LEDBlue);
			indicatorLEDOn(&LEDRed);
		}
		else
		{
			startSide = RIGHT;
			indicatorLEDOff(&LEDRed);
			indicatorLEDOn(&LEDBlue);
		}

        //Do nothing until button pressed
        //On button press do an initial ultrasonic wall reading
        //& schedule to ready state
        if (buttonPressed)
        {
            //Trigger next reading in 20 ms
            timeIncrement(&(Schedule.ultraLeftStart), 0, 20);

            nextState = READY;
            timeIncrement(&Schedule.stateChange, 1, 0);
            buttonPressed = 0;
        }

        //Use initial ultrasonic reading as distance to wall to maintain
        if (ultraReadLeft)
        {
            if (fillBufferCount < WALL_READINGS)
            {
				//Do nothing
            }
            else
            {
                findDistanceAverage(1, WALL_READINGS, wallDistancesLeft);
                //Use cardboard to calibrate sensors to recognise where closest wall will be
                 closestWallLeft = avgReading-300;
            }
			timeIncrement(&(Schedule.ultraRightStart), 0, 20);
            ultraReadLeft = 0;
        }
		
		//Use initial ultrasonic reading as distance to wall to maintain
        if (ultraReadRight)
        {
            if (fillBufferCount < WALL_READINGS)
            {
                fillBufferCount++;
                timeIncrement(&(Schedule.ultraLeftStart), 0, 20);
            }
            else
            {
                findDistanceAverage(1, WALL_READINGS, wallDistancesRight);
                //Use cardboard to calibrate sensors to recognise where closest wall will be
                 closestWallRight = avgReading-300;
            }
            ultraReadRight = 0;
        }
    }

//ON SECOND BUTTON PRESS START SEARCHING======================================================
	if(state == READY)
	{
	    indicatorLEDOn(&LEDRed);
	    indicatorLEDOn(&LEDBlue);

		//Do nothing until button pressed
		//On button press do an initial ultrasonic wall reading
		//& schedule to next state to start moving
		if (buttonPressed)
		{
            nextState = SEARCH;
			timeIncrement(&Schedule.stateChange, 1, 0);
			buttonPressed = 0;

			timeIncrement(&(Schedule.searchStart), 1+TIME_BEFORE_SEARCHING, 0);
		}
	}

//FOLLOW WALL WHILST LOOKING FOR CANS========================================================
	if(state == SEARCH)
	{
	    if (movement == 1)
	    {
	        movement = 0;
	        updateCarHeading(FORWARD, STRAIGHT, searchSpeeds);
	        turnState = STRAIGHT;

	        //Trigger next reading in 20 ms
	        timeIncrement(&(Schedule.ultraSONARStart), 0, 20);
	    }

		//Take ultrasonic readings to wall to stay aligned and if can is past on left
		if (ultraReadLeft)
		{
		        //Light LEDs to indicate when searching has begun
                if (search)
                {
					searchSpeeds = FIND_SPEED;
                    indicatorLEDOn(&LEDBlue);
                    indicatorLEDOn(&LEDRed);
                }
				else
				{
					searchSpeeds = SPRINT_SPEED;
					indicatorLEDOff(&LEDBlue);
                    indicatorLEDOff(&LEDRed);
				}

                findDistanceAverage(1, WALL_READINGS, wallDistancesLeft);

				//Can detection threshold
                if (avgReading < closestWallLeft-500)
                {
					//If searching then go to can
                    if(search)
                    {
                        canPosition = LEFT;
                        nextState = CAN_ALIGN;
                        flag.stateChange = 1;
                        Schedule.stateChange.sec = 0;
                        Schedule.stateChange.ms = -1;
                    }
                    else	//If not, then start searching soon
                    {
                        timeIncrement(&Schedule.searchStart, 0, 300);
                    }
                }

                if (startSide == LEFT)
                {
                    //Use latest readings to keep aligned to wall
                    alignToWall();
                }

                if ((Schedule.movementChange.ms == -1) && (canPosition != LEFT))
                {
                    //Trigger next reading in 20 ms
                    timeIncrement(&(Schedule.ultraRightStart), 0, 20);
                }
			ultraReadLeft = 0;
		}
		
		
		//Take ultrasonic readings to wall to stay aligned and if can is past on left
		if (ultraReadRight)
		{

                findDistanceAverage(1, WALL_READINGS, wallDistancesRight);

                if (avgReading < closestWallRight-500)
                {
                    if(search)
                    {
                        canPosition = RIGHT;
                        nextState = CAN_ALIGN;
                        flag.stateChange = 1;
                        Schedule.stateChange.sec = 0;
                        Schedule.stateChange.ms = -1;
                    }
                    else
                    {
                        timeIncrement(&Schedule.searchStart, 0, 300);
                    }
                }

                if (startSide == RIGHT)
                {
                    //Use latest readings to keep aligned to wall
                    alignToWall();
                }

                if ((Schedule.movementChange.ms == -1) && (canPosition != RIGHT))
                {
                    //Trigger next reading in 20 ms
                    timeIncrement(&(Schedule.ultraSONARStart), 0, 20);
                }
			ultraReadRight = 0;
		}
		
		//Take forward readings to see if can is in front of car
		if (ultraSONARRead)
		{
		    findDistanceAverage(1, SONAR_READINGS, SONARDistances);
			
            if(avgReading < MAX_FRONT_DETECT)
            {
				canPosition = STRAIGHT;
				nextState = CAN_ALIGN;
                flag.stateChange = 1;
                Schedule.stateChange.sec = 0;
                Schedule.stateChange.ms = -1;
            }
            else
            {
                //Trigger next reading in 20 ms
                timeIncrement(&(Schedule.ultraLeftStart), 0, 20);
            }
			
            ultraSONARRead = 0;
		}
	}

//TURN TO GET CAN IN SONAR RANGE==========================================================
	if(state == CAN_ALIGN)
	{
		if (movement == 1)
		{
			nextState = SEARCH;
			flag.stateChange = 1;
			Schedule.stateChange.sec = 0;
			Schedule.stateChange.ms = -1;

			timeIncrement(&Schedule.searchStart, 0, 50);
		}
		
        if (ultraReadRight)
        {
			ultraReadRight = 0;
			
			//Refill array with new readings whilst reversing
			if (fillBufferCount < WALL_READINGS)
			{
			    updateCarHeading(BACK, STRAIGHT, 45);
				fillBufferCount++;
				timeIncrement(&Schedule.ultraRightStart , 0, 20);
			}
			else
			{
				findDistanceAverage(1, WALL_READINGS, wallDistancesRight);

				if (avgReading < closestWallRight-500)
				{
					Schedule.movementChange.ms = -1;
					
					updateCarHeading(BACK, LEFT, 45);
					
					timeToCanAlignSec = INITIAL_CAN_ALIGN_SEC;
					timeToCanAlignMs =  INITIAL_CAN_ALIGN_MS;
					
					nextState = SONAR_SCAN;
					timeIncrement(&Schedule.stateChange, timeToCanAlignSec, timeToCanAlignMs);
				}
				else
				{
					timeIncrement(&Schedule.ultraRightStart , 0, 20);
				}
			}
        }

        if (ultraReadLeft)
        {
			ultraReadLeft = 0;
			
			//Refill array with new readings whilst reversing
			if (fillBufferCount < WALL_READINGS)
			{
			    updateCarHeading(BACK, STRAIGHT, 45);
				fillBufferCount++;
				timeIncrement(&Schedule.ultraLeftStart , 0, 20);
			}
			else
			{
				findDistanceAverage(1, WALL_READINGS, wallDistancesLeft);

				if (avgReading < closestWallLeft-500)
				{
					Schedule.movementChange.ms = -1;
					
					updateCarHeading(BACK, RIGHT, 45);
					
					timeToCanAlignSec = INITIAL_CAN_ALIGN_SEC;
					timeToCanAlignMs =  INITIAL_CAN_ALIGN_MS;
					
					nextState = SONAR_SCAN;
					timeIncrement(&Schedule.stateChange , timeToCanAlignSec, timeToCanAlignMs);
				}
				else
				{
					timeIncrement(&Schedule.ultraLeftStart , 0, 20);
				}
			}
        }

        if (ultraSONARRead)
        {
			ultraSONARRead = 0;
			
			//Refill array with new readings whilst reversing
			if (fillBufferCount < SONAR_READINGS)
			{
				fillBufferCount++;
				timeIncrement(&Schedule.ultraSONARStart , 0, 20);
			}
			else
			{
				findDistanceAverage(1, SONAR_READINGS, SONARDistances);
				if (avgReading <= MAX_FRONT_DETECT)
				{
					Schedule.movementChange.ms = -1;
					nextState = SONAR_SCAN;
					flag.stateChange = 1;
					Schedule.stateChange.sec = 0;
					Schedule.stateChange.ms = -1;
				}
				else
				{
				    Schedule.movementChange.ms = -1;
					nextState = SEARCH;
					flag.stateChange = 1;
					Schedule.stateChange.sec = 0;
					Schedule.stateChange.ms = -1;
				}
			}
        }
	}

//STOP, SCAN & POINT AT CAN===============================================================
	if (state == SONAR_SCAN)
	{
		if (ultraSONARRead)
		{
		    if (movement == 0)
		    {
		        SONAR();
		    }
			ultraSONARRead = 0;
		}
	}

//DRIVE TO CAN WHEN IN SONAR RANGE=======================================================
    if (state == CAN_APPROACH)
    {
		//Track can with SONAR
		if (ultraSONARRead)
		{
			canLock();
			ultraSONARRead = 0;
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
            indicatorLEDOff(&LEDBlue);
            indicatorLEDOn(&LEDRed);    //Black if 1
        }
        else
        {
            indicatorLEDOn(&LEDBlue);
            indicatorLEDOff(&LEDRed);   //White if 0
        }
    }

//HIT CAN IF BLACK========================================================================
    if (state == CAN_HIT)
    {
        //Functionality in stateChange();
    }

//AVOID CAN IF WHITE======================================================================
    if (state == CIRCUMNAVIGATE)
    {
		if (wallDistancesLeft[0] < wallDistancesRight[0])
		{
			furtherWall = RIGHT;
		}
		else
		{
			furtherWall = LEFT;
		}
		
		circumnavigate();
    }

//GET ROUGHLY STRAIGHT WITH WALL AND DRIVE PAST DEALT WITH CAN===========================
    if (state == WALL_REALIGN)
    {
        if (movement == 0)
        {
            updateCarHeading(BACK, STRAIGHT, 45);

            if (Schedule.movementChange.ms == -1)
            {
                timeIncrement(&Schedule.movementChange, 0, 600);
            }
        }
        else if (movement == 1)
        {
            updateCarHeading(BACK, canPosition, 45);
            if (Schedule.movementChange.ms == -1)
            {
                timeIncrement(&Schedule.movementChange, 0, 700);
            }
        }
        else if (movement == 2)
        {
            movement = 4;
        }
		else if (movement ==3)
		{
			//Let ultrasonics work
		}
        else if (movement == 4)
        {
            movement = 3;
	        updateCarHeading(FORWARD, STRAIGHT, 45);
	        turnState = STRAIGHT;

	        if (startSide == LEFT)
			{
				timeIncrement(&Schedule.ultraLeftStart, 0, 20);
			}
			else
			{
				timeIncrement(&Schedule.ultraRightStart, 0, 20);
			}
        }

        //After dealing with a can and car has guessed wall alignment get straight again
        if (ultraReadLeft)
        {
            indicatorLEDOff(&LEDBlue);
            indicatorLEDOn(&LEDRed);
            if (fillBufferCount < WALL_READINGS)
            {
                fillBufferCount++;
                timeIncrement(&(Schedule.ultraLeftStart), 0, 20);
            }
            else
            {
                if (straightTime > 20)  //Occurs when not aligning to this side and just getting new reading
                {
                    findDistanceAverage(1, WALL_READINGS, wallDistancesLeft);
                    if (avgReading > 1000)
                    {
                        closestWallLeft = 1000;
                    }
                    else
                    {
                        closestWallLeft = avgReading-200;
                    }

                    nextState = SEARCH;
                    flag.stateChange = 1;
                    if (IR.colour == 1) //Can was black
                    {
                        //Search start again once given some time to realign to wall properly
                        timeIncrement(&Schedule.searchStart, 2, 0);
                    }
                    else
                    {
                        //Search start again once past white can so set large where change in search due to seeing white can again
                        timeIncrement(&Schedule.searchStart, 10, 0);
                    }
                }
                else
                {
                    alignToWall();
                    if (turnState == STRAIGHT)
                    {
                        straightTime++;
                        if (straightTime > 20)  //Find wall on right next before searching
                        {
                            fillBufferCount = 0;
                            findDistanceAverage(1, WALL_READINGS, wallDistancesLeft);
                            if (avgReading > 1000)
                            {
                                closestWallLeft = 1000;
                            }
                            else
                            {
                                closestWallLeft = avgReading-200;
                            }
                            timeIncrement(&(Schedule.ultraRightStart), 0, 20);
                        }
                        else
                        {
                            timeIncrement(&(Schedule.ultraLeftStart), 0, 20);
                        }
                    }
                    else
                    {
                        straightTime = 0;
                        timeIncrement(&(Schedule.ultraLeftStart), 0, 20);
                    }
                }
            }
            ultraReadLeft = 0;
        }

        //After dealing with a can and car has guessed wall alignment get straight again
        if (ultraReadRight)
        {
            indicatorLEDOn(&LEDBlue);
            indicatorLEDOff(&LEDRed);
            if (fillBufferCount < WALL_READINGS)
            {
                fillBufferCount++;
                timeIncrement(&(Schedule.ultraRightStart), 0, 20);
            }
            else
            {
                if (straightTime > 10)  //Occurs when not aligning to this side and just getting new reading
                {
                    findDistanceAverage(1, WALL_READINGS, wallDistancesRight);
                    if (avgReading > 1000)
                    {
                        closestWallRight = 1000;
                    }
                    else
                    {
                        closestWallRight = avgReading-200;
                    }
                    movement++;

                    nextState = SEARCH;
                    flag.stateChange = 1;
                    if (IR.colour == 1) //Can was black
                    {
                        //Search start again once given some time to realign to wall properly
                        timeIncrement(&Schedule.searchStart, 2, 0);
                    }
                    else
                    {
                        //Search start again once past white can so set large where change in search due to seeing white can again
                        timeIncrement(&Schedule.searchStart, 10, 0);
                    }
                }
                else
                {
                    alignToWall();
                    if (turnState == STRAIGHT)
                    {
                        straightTime++;
                        if (straightTime > 20)  //Find wall on right next before searching
                        {
                            fillBufferCount = 0;
                            findDistanceAverage(1, WALL_READINGS, wallDistancesRight);
                            if (avgReading > 1000)
                            {
                                closestWallRight = 1000;
                            }
                            else
                            {
                                closestWallRight = avgReading-200;
                            }
                            timeIncrement(&(Schedule.ultraLeftStart), 0, 20);
                        }
                        else
                        {
                            timeIncrement(&(Schedule.ultraRightStart), 0, 20);
                        }
                    }
                    else
                    {
                        straightTime = 0;
                        timeIncrement(&(Schedule.ultraRightStart), 0, 20);
                    }
                }
            }
            ultraReadRight = 0;
        }

    }

//DO NOTHING==============================================================================
	//On stop moving state
	if(state == STOP)
	{
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

	//Add to current time and if more then max wrap back to 0
    time->sec = currentTime.sec + sec;
    while(time->sec >= 300)
    {
        time->sec -= 300;
    }
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

void setupTimerSONAR()
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
    unsigned char increasing = 0;
    unsigned char decreasing = 0;
    unsigned char largeChange = 0;
	unsigned int  *wallDistances = 0;
	

    if (startSide == RIGHT)
    {
        wallDistances = wallDistancesRight;
    }
    else
    {
        wallDistances = wallDistancesLeft;
    }

	//Find reading trends
    for (j=1; j<WALL_READINGS; j++)
    {
        if (*(wallDistances + j) < *(wallDistances + j - 1)-10)
        {
            increasing++;
        }
        else if (*(wallDistances + j) > *(wallDistances + j - 1)+10)
        {
            decreasing++;
        }

        if ((*(wallDistances + j) < *(wallDistances + j - 1)-1500)
                || (*(wallDistances + j) > *(wallDistances + j - 1)+1500))
        {
            largeChange++;
        }
    }

    if ((largeChange > 0) && (largeChange <= 2))   //Probably wall change so ignore for this reading
    {
        turnState = STRAIGHT;
    }
    else if (decreasing >= WALL_READINGS-2) //If results are decreasing turn away from wall
    {
        turnState = AWAY;
    }
    else if (increasing >= WALL_READINGS-2) //If results are increasing turn to the wall
    {
        turnState = CLOSE;
    }
	else    //Anything else go straight
	{
		turnState = STRAIGHT;
	}

    //Control steering according to state
    switch(turnState)
    {
    case STRAIGHT:
        updateCarHeading(FORWARD, STRAIGHT, searchSpeeds);
        break;
    case AWAY:
		if (startSide == LEFT)
		{
			updateCarHeading(FORWARD, RIGHT, searchSpeeds-5);
		}
		else
		{
			updateCarHeading(FORWARD, LEFT, searchSpeeds-5);
		}

		for (j=1; j<WALL_READINGS; j++)
		{
		    *(wallDistances + j) = *(wallDistances);
		}

        timeIncrement(&Schedule.movementChange, 0, 50);
        break;
    case CLOSE:
        if (startSide == LEFT)
		{
			updateCarHeading(FORWARD, LEFT, searchSpeeds-5);
		}
		else
		{
			updateCarHeading(FORWARD, RIGHT, searchSpeeds-5);
		}

        for (j=1; j<WALL_READINGS; j++)
        {
            *(wallDistances + j) = *(wallDistances);
        }

        timeIncrement(&Schedule.movementChange, 0, 50);
        break;
    }
}

void    canLock()
{
    //When reading is close enough
    if(SONARDistances[0] < FACE_CAN_DIST_TOLERANCE)
    {
        indicatorLEDOff(&LEDRed);
        if ((TA1CCR2 > 1500-FACE_CAN_ANGLE_TOLERANCE) && (TA1CCR2 < 1500+FACE_CAN_ANGLE_TOLERANCE))   //If facing can
        {
            //Stop driving
			updateCarHeading(OFF, STRAIGHT, CAN_APPROACH_SPEED);

			Schedule.movementChange.ms = -1;

            nextState = COLOUR_DETECT;
            flag.stateChange = 1;
            Schedule.stateChange.sec = 0;
            Schedule.stateChange.ms = -1;
        }
        else
        {
            if (canPosition == LEFT)
            {
                updateCarHeading(BACK, RIGHT, CAN_APPROACH_SPEED);
            }
            else
            {
                updateCarHeading(BACK, LEFT, CAN_APPROACH_SPEED);
            }

            nextState = SONAR_SCAN;
			timeToCanAlignSec = SECONDARY_CAN_ALIGN_SEC;
            timeToCanAlignMs =  SECONDARY_CAN_ALIGN_MS;
            timeIncrement(&Schedule.stateChange, timeToCanAlignSec, timeToCanAlignMs);
        }
    }
    else if (SONARDistances[0] < FACE_CAN_DIST_TOLERANCE+700) //If slightly further away but angle really bad
    {
        if ((TA1CCR2 < 1000) || (TA1CCR2 > 2000))
        {
            if (canPosition == LEFT)
            {
                updateCarHeading(BACK, RIGHT, CAN_APPROACH_SPEED);
            }
            else
            {
                updateCarHeading(BACK, LEFT, CAN_APPROACH_SPEED);
            }

            if (SONARDistances[0] > 5000)
            {
                updateCarHeading(3, 3, CAN_APPROACH_SPEED + 15);
            }
            else
            {
                updateCarHeading(3, 3, CAN_APPROACH_SPEED);
            }

            nextState = SONAR_SCAN;
            timeToCanAlignSec = SECONDARY_CAN_ALIGN_SEC;
            timeToCanAlignMs = SECONDARY_CAN_ALIGN_MS;
            timeIncrement(&Schedule.stateChange, timeToCanAlignSec, timeToCanAlignMs);
        }
    }

    //Behaviours whether can is lost
    if (lostCan)    //If can is lost
    {
        //Can is found
        //if (((SONARDistances[0] < SONARDistances[1]+ANOMALY_DISTANCE) && (SONARDistances[0] < expectedCanDistance + 500)) ||
        if (SONARDistances[0] < expectedCanDistance + 500)
        {
            if (TA1CCR2 < 1500)         //CAN TO THE RIGHT
            {
                updateCarHeading(FORWARD, RIGHT, CAN_APPROACH_SPEED);
            }
            else if (TA1CCR2 > 1500)    //CAN TO THE LEFT
            {
                updateCarHeading(FORWARD, LEFT, CAN_APPROACH_SPEED);
            }
            else
            {
                updateCarHeading(FORWARD, STRAIGHT, CAN_APPROACH_SPEED);
            }

            if (SONARDistances[0] > 5000)
            {
                updateCarHeading(3, 3, CAN_APPROACH_SPEED + 15);
            }
            else
            {
                updateCarHeading(3, 3, CAN_APPROACH_SPEED);
            }

            expectedCanDistance = SONARDistances[0];
            lostCan = 0;
            lostCount = 0;

            indicatorLEDOn(&LEDRed);
            timeIncrement(&Schedule.ultraSONARStart, 0, 20);
        }
        else    //Can still lost
        {
            lostCan = 1;

            if (SONARDistances[0] > SONARDistances[1] + ANOMALY_DISTANCE) //Probably just moved off can
            {
                servoSONAR.direction ^= 1;
            }
            else if (lostCount > 4)
            {
                nextState = SONAR_SCAN;
                flag.stateChange = 1;
                Schedule.stateChange.sec = 0;
                Schedule.stateChange.ms = -1;
            }

            indicatorLEDOff(&LEDRed);
            servoTurn(&servoSONAR);
            timeIncrement(&Schedule.ultraSONARStart, 0, 200);
            lostCount++;
        }
    }
    else    //If can is not lost
    {
        if (SONARDistances[0] > expectedCanDistance+500)    //Now lost
        {
            lostCan = 1;
			
			updateCarHeading(OFF, 3, CAN_APPROACH_SPEED);

            if (TA1CCR2 >= 1500)
            {
                servoSONAR.direction = 0;
            }
            else if  (TA1CCR2 < 1500)
            {
                servoSONAR.direction = 1;
            }
            else
            {
                /*
                nextState = SONAR_SCAN;
                flag.stateChange = 1;*/
            }

            indicatorLEDOff(&LEDRed);
            servoTurn(&servoSONAR);
            timeIncrement(&Schedule.ultraSONARStart, 0, 200);
            lostCount = 1;
        }
        else    //Still looking at can
        {
            lostCan = 0;
            lostCount = 0;

            expectedCanDistance = SONARDistances[0]; //Update latest distance to can

            indicatorLEDOn(&LEDRed);
            timeIncrement(&Schedule.ultraSONARStart, 0, 20);
        }
    }
}

void SONAR()
{
    readingsOnAngle++;
    //When readings full at current angle
    if (readingsOnAngle == SONAR_READINGS)
    {
        //Get average reading for angle set to
        readingsOnAngle = 0;
        avgOldReading = avgReading;
        findDistanceAverage(1, SONAR_READINGS, SONARDistances);
        SONARAtEachAngle[anglesChecked] = avgReading;

        //FIND ANOMALIES
        //Is it start or continuation of an anomaly???
        //Only check after first angle
        if (anglesChecked > 0)
        {
            //If anomaly not detected yet
            if (anomalyPositions[anglesChecked-1]==0)
            {
                //When reading suddenly closer
                if (avgReading < avgOldReading-ANOMALY_DISTANCE)
                {
                    anomalyPositions[anglesChecked] = 1;
                    anomalyStart[anomalyNumber] = anglesChecked;
                    indicatorLEDOn(&LEDRed);
                }
                else
                {
                    anomalyPositions[anglesChecked] = 0;
                    indicatorLEDOff(&LEDRed);
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
                    indicatorLEDOff(&LEDRed);
                }
                else if (avgReading < avgOldReading-ANOMALY_DISTANCE) //Start of a new anomaly if in front of old anomaly
                {
                    anomalyPositions[anglesChecked] = 1;
                    anomalyEnd[anomalyNumber] = anglesChecked;
                    anomalyNumber++;
                    anomalyStart[anomalyNumber] = anglesChecked;
                    indicatorLEDToggle(&LEDRed);
                }
                else
                {
                    anomalyPositions[anglesChecked] = 1;
                    indicatorLEDOn(&LEDRed);
                }
            }
        }

        //Finished dealing with new reading so increment ready for next
        anglesChecked++;

        //Have all angles been checked
        if (anglesChecked < NUMBER_OF_ANGLES_CHECKED)
        {
            //Go to next angle
            servoTurn(&servoSONAR);
            timeIncrement(&Schedule.ultraSONARStart, 0, 200);
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
                        avgNewAnomalyDistance += SONARAtEachAngle[j];
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
                    newAngle -= servoSONAR.speed;
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
        timeIncrement(&Schedule.ultraSONARStart, 0, 20);
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
	//Become next state
	previousState = state;
	state = nextState;
	
	if (state == READY)
	{
		//Count for wall buffer
		fillBufferCount = 0;
	}

	if (state == SEARCH)
	{
		//Don't search and only follow wall for an amount of time
		search = 0;
		movement = 1;
		turnState = STRAIGHT;
		indicatorLEDOff(&LEDRed);
		indicatorLEDOff(&LEDBlue);
		searchSpeeds = SPRINT_SPEED;
		canPosition = STRAIGHT;

		//Centre SONAR
		servoCenter();

		//Initiate first ultrasonic reading
		timeIncrement(&Schedule.ultraLeftStart, 0, 20);
	}

	if (state == CAN_ALIGN)
	{
		movement = 0;
	    updateCarHeading(OFF, STRAIGHT, 45);
		fillBufferCount = 0;
        switch (canPosition)
        {
            case STRAIGHT:
                indicatorLEDOn(&LEDBlue);
                indicatorLEDOn(&LEDRed);
                timeIncrement(&Schedule.ultraSONARStart, 2, 0);
                break;
            case RIGHT:
                indicatorLEDOff(&LEDRed);
                indicatorLEDOn(&LEDBlue);
                timeIncrement(&Schedule.ultraRightStart , 2, 0);
                break;
            case LEFT:
                indicatorLEDOff(&LEDBlue);
                indicatorLEDOn(&LEDRed);
                timeIncrement(&Schedule.ultraLeftStart , 2, 0);
                break;
        }
		
		timeIncrement(&Schedule.movementChange, 3, 0);
	}
	
	if (state == SONAR_SCAN)
	{
	    indicatorLEDOff(&LEDBlue);
        indicatorLEDOff(&LEDRed);

		//Clear variables used in state
		readingsOnAngle = 0;
		anglesChecked = 0;
		newAngle = 0;
		anomalyNumber = 0;
		canAnomaly = 0;
		movement = 0;
		
		for (i=0; i<NUMBER_OF_ANGLES_CHECKED; i++)
		{
			anomalyStart[i] = 0;
			anomalyEnd[i] = 0;
			anomalyPositions[i] = 0;
			SONARAtEachAngle[i] = 0;
		}
		
		//Prepare servo to turn anti-clockwise
		servoSONAR.direction = 1;
		
		//Stop driving
		updateCarHeading(OFF, STRAIGHT, motorDrive.pwm.aMs);
		
		//Rotate servo full clockwise
		TA1CCR2 = PWM_SERVO_LOWER;
		
		//Start first reading in a second when servo has turned
		timeIncrement(&Schedule.ultraSONARStart, 1, 0);
	}
	
	if (state == CAN_APPROACH)
	{
	    movement = 0;

	    indicatorLEDOff(&LEDBlue);
	    indicatorLEDOff(&LEDRed);

		if (TA1CCR2 < 1500) //CAN TO THE RIGHT
		{
			updateCarHeading(FORWARD, RIGHT, 40);
		}
		else if (TA1CCR2 > 1500) //CAN TO THE LEFT
		{
			updateCarHeading(FORWARD, LEFT, 40);
		}
		else
		{
			updateCarHeading(FORWARD, STRAIGHT, 40);
		}
		
		lostCan = 0;
		lostCount = 0;
		
		timeIncrement(&Schedule.ultraSONARStart, 0, 20);
	}
	
	if (state == COLOUR_DETECT)
	{
	    movement = 0;

		updateCarHeading(OFF, STRAIGHT, 40);

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
			indicatorLEDOn(&LEDRed);
		}
		else
		{
		    indicatorLEDOn(&LEDBlue);
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
			indicatorLEDOff(&LEDRed);
		}
	}
	
	if (state == REVERSE_CAN)
	{
	    movement = 0;

		updateCarHeading(BACK, STRAIGHT, SPEED_BACK);
		
		nextState = CAN_HIT;
		timeIncrement(&Schedule.stateChange, 0, REVERSE_TIME);
	}
	
	if (state == CAN_HIT)
	{
	    movement = 0;

		updateCarHeading(FORWARD, STRAIGHT, 100);
		
		nextState = WALL_REALIGN;
		timeIncrement(&Schedule.stateChange, 0, HIT_TIME);
	}
	
	if (state == CIRCUMNAVIGATE)
	{
	    movement = 0;
	}
	
	if (state == WALL_REALIGN)
	{
	    indicatorLEDOn(&LEDRed);
	    indicatorLEDOn(&LEDBlue);
	    movement = 0;
		Schedule.movementChange.ms = -1;
		
		fillBufferCount = 0;
		straightTime = 0;
	}
	
	if (state == STOP)
	{
		indicatorLEDOff(&LEDRed);

		//Stop all motors and readings
		updateCarHeading(OFF, STRAIGHT, 40);
	}
}

void circumnavigate()
{
	if (movement == 0)
	{
		updateCarHeading(BACK, STRAIGHT, 40);

		if (Schedule.movementChange.ms == -1)
		{
			timeIncrement(&Schedule.movementChange, 0, 800);
		}
	}
	else if (movement == 1)
	{
		updateCarHeading(FORWARD, furtherWall, 60);

		if (Schedule.movementChange.ms == -1)
		{
			timeIncrement(&Schedule.movementChange, 1, 250);
		}
	}
	else if (movement == 2)
	{
		
		if (furtherWall == LEFT)
		{
			updateCarHeading(FORWARD, RIGHT, 50);
		}
		else
		{
			updateCarHeading(FORWARD, LEFT, 50);
		}

		if (Schedule.movementChange.ms == -1)
		{
			timeIncrement(&Schedule.movementChange, 1, 0);
		}
	}
	else if (movement == 3)
	{
		updateCarHeading(FORWARD, furtherWall, 45);

		if (Schedule.movementChange.ms == -1)
		{
			timeIncrement(&Schedule.movementChange, 0, 600);
		}
	}
	else
	{
		nextState = SEARCH;
		flag.stateChange = 1;
		Schedule.stateChange.ms = -1;
		timeIncrement(&Schedule.searchStart, 0, 50);
	}
}

void    updateCarHeading(unsigned char direction, unsigned char angle, unsigned int speed)
{
	//Change heading
	if (direction != 3)
	{
		motorDrive.direction = direction;
		flag.motorDrive = 1;
	}
	if (angle != 3)
	{
		motorSteer.direction = angle;
		flag.motorSteer = 1;
	}

	//Add more speed whilst turning to account for friction
    //Faster if backwards as reverses slower
	motorDrive.pwm.aMs  = speed;
    if (motorDrive.direction == BACK)
    {
        motorDrive.pwm.aMs  += 17;
        if (motorSteer.direction != STRAIGHT)
        {
            motorDrive.pwm.aMs  += 3;
        }

        if (motorDrive.pwm.aMs > 100)
        {
            motorDrive.pwm.aMs = 100;
        }
    }
    flag.motorDrive = 1;
}

//==============================================================================
// End of File : AutonomousCar/main.c
