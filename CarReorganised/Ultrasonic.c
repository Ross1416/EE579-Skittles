/*
Filename    : CarReorganised/Ultrasonic.c
Author      : Samuel Kliskey
Project     : EE579 Skittles Knocking Over Project
Date        : 31/3/24
Description : Ultrasonic Functionality
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
31-MAR-2024 SARK created to restructure AutonomousCar project
--------------------------------------------------------------------------------
*/
#include "Ultrasonic.h"

void ultrasonicSetup(struct Ultrasonic *ultra)
{
	if (ultra->port == 1)
	{
		//Echo pin
		P1SEL |= ultra->echoPin;
		P1DIR &= ~ultra->echoPin;

		//Trig pin
		P1DIR |= ultra->trigPin;
		P1OUT &= ~ultra->trigPin;

		//Set capture compare to the ultrasonic being used
		if (ultra->echoPin == BIT6) //Capture compare input A
		{
			TA0CCTL1 |= CCIS_1;
		}
		//Capture rising, capture mode, enable interrupt, sync to clock rising edge
		TA0CCTL1 |= CM_1 + CAP + CCIE + SCS;
		TA0CCTL1 &= ~(CCIFG+SCCI+CCI);  //Clear interrupts
	}
	
	if (ultra->port == 2)
	{
		//Echo pin
		P2SEL |= ultra->echoPin;

		//Trig pin
		P2DIR |= ultra->trigPin;
		P2OUT &= ~ultra->trigPin;

		//Timer A1 settings
		if (ultra->echoPin == BIT2) //Capture compare input B
		{
			TA1CCTL1 |= CCIS_1;
		}
		else if (ultra->echoPin == BIT1) //Capture compare input A
		{
			TA1CCTL1 |= CCIS_0;
		}
		//Capture rising, capture mode, enable interrupt, sync to clock rising edge
		TA1CCTL1 |= CM_1 + CAP + CCIE + SCS;
		TA1CCTL1 &= ~(CCIFG+SCCI+CCI);  //Clear interrupts
	}
}

void ultrasonicTrigger(struct Ultrasonic *ultra)
{
	if (ultra->port == 1)
	{
		//Set capture compare to the ultrasonic being used
		if (ultra->echoPin == BIT2) //Capture compare input A
		{
			TA0CCTL1 |= CCIS_1;
		}

		//Trigger the ultrasonic
		P1OUT |= ultra->trigPin;
		__delay_cycles(20);
		P1OUT &= ~ultra->trigPin;
	}

	if (ultra->port == 2)
	{
		//Set capture compare to the ultrasonic being used
		if (ultra->echoPin == BIT2) //Capture compare input B
		{
		    //TA1CCTL1 &= ~CCIS_3;
			TA1CCTL1 |= CCIS_1;
		}
		else if (ultra->echoPin == BIT1) //Capture compare input A
		{
		    TA1CCTL1 &= ~CCIS_3;
			TA1CCTL1 |= CCIS_0;
		}

		//Trigger the ultrasonic
		P2OUT |= ultra->trigPin;
		__delay_cycles(20);
		P2OUT &= ~ultra->trigPin;
	}
}

//==============================================================================
// End of File :  CarReorganised/Ultrasonic.h
