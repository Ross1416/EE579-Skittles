#include <msp430.h> 

/* Connections:
 *
 * Ultrasonic       MSP
 * Vcc              5V
 * Trig             P2.0
 * Echo             P2.2 (through voltage divider R1=220,R2=470; to take 5V -> ~ 3.4V)
 * GND              GND
 *
 * Servo            MSP
 * Vcc              5V
 * Control          P2.1
 * GND              GND
 */

//const int servo_lower_limit_count = 18; // 0 degrees
//const int servo_upper_limit_count = 74; // 180 degrees

//const unsigned int period = 640; // 50 Hz

volatile int vals[2];
volatile int i = 0;
volatile float diff = 0;

volatile int calc_flag = 0;
float distance=0;

int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	//setup trig
	P2DIR |= BIT0;

	P1DIR |= BIT6;

//	// setup servo
//	P2DIR |= BIT1;  // set servo pin output
//	P2SEL |= BIT1;
//
//	P2SEL |= BIT2; // set echo pin
//
//	TA1CCR0 = period;
//    TA1CCR1 = 24;

    // Setup timer for capture
    TA1CTL |= TASSEL_1 + TAIE; //ACLK, enable interrupts
    TA1CTL &= ~TAIFG;
//    TA1CCTL1 |= OUTMOD_7 + CAP + CM_3 + SCS + CCIS_1 + CCIE;// Reset-set, output mode, capture mode, capture on both rising and falling edges, sync, in CCI1B, enable interrupts
    TA1CCTL1 |= CAP + CM_3 + SCS + CCIS_1 + CCIE;

    TA1CCTL1 &= ~CCIFG;

    TA1CTL |= MC_1; // up mode

    __enable_interrupt();

    // Send pulse
    P2OUT |= BIT0;
    __delay_cycles(20);         // 12/1 Mhz = 12 uS (min is 10uS)
    P2OUT &= ~BIT0;

//    while(1)
//        {
//            if(calc_flag)
//            {
//                distance=((343.0*diff)/32000)*100/2;
//                calc_flag=0;
//            }
//        }

}

// Timer A0 interrupt service routine
// Where tasks occur
#pragma vector=TIMER1_A1_VECTOR
__interrupt void Timer_A (void)
{
    P1OUT |= BIT6;
//    vals[i] = TA1CCR1;
//    TA1CCR1 = 24;
//    i++;
//    if (i==2)
//    {
//        diff = vals[1]-vals[0];
//        calc_flag=1;
//        i=0;
//    }
    TA1CCTL1 &= ~CCIFG;
}
