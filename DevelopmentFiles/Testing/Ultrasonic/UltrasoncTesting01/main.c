#include <msp430.h> 


/* Connections
 *
 * Ultrasonic       MSP
 * Vcc              5V
 * Trig             P2.0
 * Echo             P1.1 (through voltage divider R1=220,R2=470; to take 5V -> ~ 3.4V)
 * GND              GND
 */


volatile int vals[2];
volatile int i = 0;
volatile float diff = 0;

volatile int calc_flag = 0;
float distance=0;

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

    // Setup timer for capture
    TA0CTL |= TASSEL_2 + TAIE; //SMCLK, enable interrupts
    TA0CTL &= ~TAIFG;
    TA0CCTL0 |= CM_3 + CCIS_0 + CAP + CCIE + SCS; //Capture on both rising and falling edges, capture input CCI0A, (leave async), capture mode, enable interrupt, synchronous recommended to stop race conditions
    TA0CCTL0 &= ~CCIFG;

    TA0CTL |= MC_2; //continuous up mode


    P1SEL |= BIT1;
    P1SEL2 &= ~ BIT1;
    P1DIR &= ~BIT1;

    //Setup trigger pinz
    P2DIR |= BIT0;
    P2OUT = 0;

//    // setup led pin
//    P1DIR |= BIT6;
//    P1OUT &= ~BIT6;

    // Enable Btn interrupts and set high to low edge
    P1IE |= BIT3;
    P1IES |= BIT3;
    P1IFG &= ~BIT3; //clear interrupt flag

    __enable_interrupt();

//    __delay_cycles(1000);

//    // Send pulse
//    P2OUT |= BIT0;
//    __delay_cycles(20);         // 12/1 Mhz = 12 uS (min is 10uS)
//    P2OUT &= ~BIT0;

    while(1)
    {
        if(calc_flag)
        {
            distance=((343.0*diff)/1000000)*100/2;
            calc_flag=0;
        }
    }

}


// Timer A0 interrupt service routine
// Where tasks occur
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
//    P1OUT |= BIT6;
    vals[i] = TA0CCR0;
    i++;
    if (i==2)
    {
        diff = vals[1]-vals[0];
        calc_flag=1;
        i=0;
    }
    TA0CCTL0 &= ~CCIFG;
}

// Port 1 interrupt service routine
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_ISR (void)
{
    // Send pulse
    P2OUT |= BIT0;
    __delay_cycles(20);         // 12/1 Mhz = 12 uS (min is 10uS)
    P2OUT &= ~BIT0;

    P1IFG &= ~BIT3;                                  // Clear switch interrupt flag

}
