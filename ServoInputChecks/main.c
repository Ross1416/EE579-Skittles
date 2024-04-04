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

       if(P2OUT & RGB_BLUE)
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
            TA1CCTL1 |= CM_1;   //Capture on rising edge
        }
        else
        {
            TA1CCTL1 |= CM_2;   //Capture on falling edge
        }
        TA1CCTL1 &= ~CCIFG;
        break;

    case TA1IV_TACCR2:  //TA1CCR2 (No interrupt as used in servo PWM)
        TA1CCTL2 &= ~CCIFG;
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

	//Echo pin
	P2SEL |= BIT1;
	//P2SEL |= BIT2;

	//Trig pin
	P2DIR |= BIT0;
	P2OUT &= ~BIT0;

	//RGB for status
	P2DIR |= 0x2A;

	//Timer 1 settings
	TA1CTL |= TASSEL_2;         // f = 1 MHz
    TA1CTL &= ~TAIFG;           //Clear interrupt
    TA1CTL &= ~TAIE;            //Disable interrupt on timer edge

    TA1CCTL0 &= ~(CCIFG+CCIE);
    TA1CCTL1 &= ~(CCIFG+CCIE);
    TA1CCTL2 &= ~(CCIFG+CCIE);

    TA1CCR0 = 20000;

    TA1CTL |= MC_1;             //Count to TA1CCR0

	//Capture compare 1 for timer 1 on input A (2.1)
	TA1CCTL1 |= CCIS_0 + CM_1 + CAP + CCIE + SCS;

    //Capture compare 1 for timer 1 on input B (2.2)
    //TA1CCTL1 |= CCIS_1 + CM_1 + CAP + CCIE + SCS;




	TA1CCTL1 &= ~(CCIFG+SCCI+CCI);  //Clear interrupts

	read = 1;
	__bis_SR_register(GIE);

	while(1)
	{
	    if(read == 1)
	    {
	        if (++i > 10)
	        {
	            i = 0;
	        }
	        array[i] = ultraLeft.distance;

	        read = 0;

	        __delay_cycles(20000);

	        P2OUT |= BIT0;  //Trigger sensor
            __delay_cycles(20);
            P2OUT &= ~BIT0; //Stop Trigger
	    }
	}
}
