#include <msp430.h> 

//RGB colours (P2OUT |= RGB_XX;)
//RGB off command: (P2OUT &= ~0x2A;)
#define RGB_RED     0x02
#define RGB_GREEN   0x08
#define RGB_BLUE    0x20
#define RGB_PURPLE  0x22
#define RGB_CYAN    0x28
#define RGB_YELLOW  0xA
#define RGB_WHITE   0x2A

struct Ultrasonic{
    int distance;
    int time[2];
    char timeNumber;
    char trigPin;
    char echoPin;
    char port;
};

struct Ultrasonic ultraLeft = {0, {0, 0}, 0, BIT0, BIT2, 2};
struct Ultrasonic ultraRADAR = {0, {0, 0}, 0, BIT0, BIT2, 1};
char read = 0;

#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer1_A1_ISR (void)
{
    switch(TA1IV)
    {
    case 0xA:   //OVERFLOW
        TA1CTL &= ~TAIFG;
        break;
    case TA1IV_TACCR1:  //TA1CCR1 (Wall Ultrasonic)

       if((P2OUT & RGB_BLUE) == RGB_BLUE)
       {
           P2OUT &= ~RGB_WHITE;
           P2OUT|= RGB_RED;
       }
       else
       {
           P2OUT &= ~RGB_WHITE;
           P2OUT|= RGB_BLUE;
       }
        ultraLeft.time[ultraLeft.timeNumber] = TA1CCR1;
        ultraLeft.timeNumber++;
        if (ultraLeft.timeNumber==2)       //After up/down edges of feedback
        {
            ultraLeft.distance = ultraLeft.time[1]-ultraLeft.time[0];
            if (ultraLeft.distance < 0)    //When timer wrapped
            {
                ultraLeft.distance += TA1CCR0;
            }
            read = 1;
            ultraLeft.timeNumber=0;
        }
        TA1CCTL1 &= ~CCIFG;
        break;

    case TA1IV_TACCR2:  //TA1CCR2 (No interrupt as used in servo PWM)
        if((P2OUT & RGB_BLUE) == RGB_BLUE)
        {
            P2OUT &= ~RGB_WHITE;
            P2OUT|= RGB_RED;
        }
        else
        {
            P2OUT &= ~RGB_WHITE;
            P2OUT|= RGB_BLUE;
        }
         ultraLeft.time[ultraLeft.timeNumber] = TA1CCR2;
         ultraLeft.timeNumber++;
         if (ultraLeft.timeNumber==2)       //After up/down edges of feedback
         {
             ultraLeft.distance = ultraLeft.time[1]-ultraLeft.time[0];
             if (ultraLeft.distance < 0)    //When timer wrapped
             {
                 ultraLeft.distance += TA1CCR0;
             }
             read = 1;
             ultraLeft.timeNumber=0;
         }
         TA1CCTL2 &= ~CCIFG;
         break;
        break;
    }
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TimerA0(void)
{
    if((P2OUT & RGB_BLUE) == RGB_BLUE)
    {
        P2OUT &= ~RGB_WHITE;
        P2OUT|= RGB_RED;
    }
    else
    {
        P2OUT &= ~RGB_WHITE;
        P2OUT|= RGB_BLUE;
    }
     ultraLeft.time[ultraLeft.timeNumber] = TA0CCR0;
     ultraLeft.timeNumber++;
     if (ultraLeft.timeNumber==2)       //After up/down edges of feedback
     {
         ultraLeft.distance = ultraLeft.time[1]-ultraLeft.time[0];
         if (ultraLeft.distance < 0)    //When timer wrapped
         {
             //ultraLeft.distance += TA0CCR0;
         }
         read = 1;
         ultraLeft.timeNumber=0;
     }
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer0_A1_ISR (void)
{
    switch(TA0IV)
    {
    case 0xA:   //OVERFLOW
        TA0CTL &= ~TAIFG;
        break;
    case TA0IV_TACCR1:  //TA1CCR1 (RADAR Ultrasonic)

       if((P2OUT & RGB_BLUE) == RGB_BLUE)
       {
           P2OUT &= ~RGB_WHITE;
           P2OUT|= RGB_RED;
       }
       else
       {
           P2OUT &= ~RGB_WHITE;
           P2OUT|= RGB_BLUE;
       }
        ultraRADAR.time[ultraRADAR.timeNumber] = TA0CCR1;
        ultraRADAR.timeNumber++;
        if (ultraRADAR.timeNumber==2)       //After up/down edges of feedback
        {
            ultraRADAR.distance = ultraRADAR.time[1]-ultraRADAR.time[0];
            if (ultraRADAR.distance < 0)    //When timer wrapped
            {
                ultraRADAR.distance += TA0CCR0;
            }
            read = 1;
            ultraRADAR.timeNumber=0;
        }
        TA0CCTL1 &= ~CCIFG;
        break;

    case TA0IV_TACCR2:  //TA1CCR2 (No interrupt as used in servo PWM)
        TA0CCTL2 &= ~CCIFG;
        break;
    }
}


/**
 * main.c
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	int i = 0;
	int array[10] = {0};

	//Trig pin
	P2DIR |= BIT0;
	P2OUT &= ~BIT0;

	//RGB for status
	P2DIR |= 0x2A;
	P2OUT &= ~0x2A;

    //OPTION 1---------------------------------------------------------------------------------------
    //Capture compare 0 for timer 0 on input A (1.1)
    //P1SEL |= BIT1;
    //P1REN |= BIT1;
    //P1OUT &= ~BIT1;
    //TA0CCTL0 |= CM_3 | CCIS_0 | CCIE | CAP | SCS;
    //TA0CTL |= TASSEL_2 | MC_2 | TACLR;

	//Timer1 settings-------------------------------------------------------------------------------
	TA1CTL |= TASSEL_2;         // f = 1 MHz
    TA1CTL &= ~TAIFG;           //Clear interrupt
    TA1CTL &= ~TAIE;            //Disable interrupt on timer edge
    TA1CCTL0 &= ~(CCIFG+CCIE);
    TA1CCTL1 &= ~(CCIFG+CCIE);
    TA1CCTL2 &= ~(CCIFG+CCIE);
    TA1CCR0 = 27000;
    TA1CTL |= MC_1;             //Count to TA1CCR0

    //Timer0 settings-------------------------------------------------------------------------------
    TA0CTL |= TASSEL_2;         // f = 1 MHz
    TA0CTL &= ~TAIFG;           //Clear interrupt
    TA0CTL &= ~TAIE;            //Disable interrupt on timer edge
    TA0CCTL0 &= ~(CCIFG+CCIE);
    TA0CCTL1 &= ~(CCIFG+CCIE);
    TA0CCTL2 &= ~(CCIFG+CCIE);
    TA0CCR0 = 27000;
    TA0CTL |= MC_1;             //Count to TA1CCR0

    //OPTION 2---------------------------------------------------------------------------------------
    //Capture compare 1 for timer 0 on input A (1.2)
    TA0CCTL1 |= CCIS_0 + CM_3 + CAP + CCIE + SCS;
    P1SEL |= BIT2;
    P1REN |= BIT2;
    P1OUT &= ~BIT2;
    TA0CCTL1 &= ~(CCIFG+SCCI+CCI);  //Clear interrupts

    //OPTION 5---------------------------------------------------------------------------------------
	//Capture compare 1 for timer 1 on input A (2.1)
	//TA1CCTL1 |= CCIS_0 + CM_3 + CAP + CCIE + SCS;
    //P2SEL |= BIT1;
    //P2SEL |= BIT1;
    //P2REN |= BIT1;
    //P2OUT &= ~BIT1;
    //TA1CCTL1 &= ~(CCIFG+SCCI+CCI);  //Clear interrupts

    //OPTION 6---------------------------------------------------------------------------------------
    //Capture compare 1 for timer 1 on input B (2.2)
    TA1CCTL1 |= CCIS_1 + CM_3 + CAP + CCIE + SCS;
    P2SEL |= BIT2;
    P2REN |= BIT2;
    P2OUT &= ~BIT2;
    TA1CCTL1 &= ~(CCIFG+SCCI+CCI);  //Clear interrupts

    //OPTION 7---------------------------------------------------------------------------------------
    //Capture compare 2 for timer 1 on input A (2.4)
    //TA1CCTL2 |= CCIS_0 + CM_3 + CAP + CCIE + SCS;
    //P2SEL |= BIT4;
    //P2REN |= BIT4;
    //P2OUT &= ~BIT4;
    //TA1CCTL2 &= ~(CCIFG+SCCI+CCI);  //Clear interrupts

    //OPTION 8---------------------------------------------------------------------------------------
    //Capture compare 2 for timer 1 on input B (2.5)
    //TA1CCTL2 |= CCIS_1 + CM_3 + CAP + CCIE + SCS;
    //P2SEL |= BIT5;
    //P2REN |= BIT5;
    //P2OUT &= ~BIT5;
    //TA1CCTL2 &= ~(CCIFG+SCCI+CCI);  //Clear interrupts

    //OPTION NO HOPE---------------------------------------------------------------------------------------
    //Capture compare 1 for timer 0 on input B (1.6)
    //TA0CCTL1 |= CCIS_1 + CM_3 + CAP + CCIE + SCS;
    //P1SEL |= BIT6;

	//TA0CCTL1 &= ~(CCIFG+SCCI+CCI);  //Clear interrupts

	read = 1;
	__bis_SR_register(GIE);

	while(1)
	{
	    if(read == 1)
	    {
	        if (++i > 8)
	        {
	            i = 0;
	        }
	        array[i] = ultraLeft.distance;
	        array[i+1] = ultraRADAR.distance;

	        read = 0;

	        __delay_cycles(20000);

	        P2OUT |= BIT0;  //Trigger sensor
            __delay_cycles(20);
            P2OUT &= ~BIT0; //Stop Trigger
	    }
	}
}
