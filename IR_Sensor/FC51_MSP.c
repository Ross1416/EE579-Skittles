#include <msp430.h>

#define currentTime(X) ((currentTime.sec == X.sec) && (currentTime.ms == X.ms))

#define GREEN_LED BIT0
#define RED_LED BIT6
#define irSensorValue BIT3

#define ClockPeriod 980 //Interrupt will trigger every ~1 ms

volatile int refreshRate = 25; // 25ms

struct Time {
    int sec;
    int ms;
};

struct Time currentTime = {0,0};        // Soft clock
struct Time colourDetect = {0,1};       // For detecting colour

struct Time Schedule (int duration)
{
    struct Time futureTime;
    futureTime.sec = currentTime.sec;
    futureTime.ms = currentTime.ms+duration;               // Add the duration in ms
    while ((futureTime.ms-=1000) >= 0){                    // Subtract seconds until ms are negative
      if (futureTime.sec++ == 60){                         // Add a second each time
        futureTime.sec = 0;                                // If a minute has gone then set the seconds to 0
      }                                                    // Don't need to account for minutes
    }
    futureTime.ms+=1000;                                   // Add 1000 ms to account for negating 1000 from ms in the while statement
    return futureTime;                                     // Return the time
}

int main(void)
{
    WDTCTL = WDTPW + WDTHOLD; // Stop WDT
    
    /* Setup Ports */
    P1DIR |= GREEN_LED + RED_LED;               // Set green (P1.0) and red (P1.6) outputs.
    
    /* Timer A0 Setup - determine PWM & frequency*/
    TA1CTL = TASSEL_2 + MC_1;                   // SMCLK (1 MHz), upmode
    TA1CCR0 = ClockPeriod;                      // Set CCR0 such that timer triggers every 1 ms
    TA1CCTL0 = CCIE;                            // Enable CCR0 interrupt
    
    __bis_SR_register(LPM0_bits + GIE); // Enter LPM0 w/ interrupt
}

// Timer A0 interrupt service routine
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer1_A0 (void)
{
  /* Update soft clock*/
  if (currentTime.ms++ == 1000){                // Has a second gone?
    currentTime.ms = 0;                         // Reset ms and count 1s if it has
    if (currentTime.sec++ == 60){               // Has a minute gona?
      currentTime.sec = 0;                      // Reset the seconds to 0 if a minute has gone
    }
  }
  
  if (currentTime(colourDetect)){               
    if (P1IN & irSensorValue){
      P1OUT |= GREEN_LED;
      P1OUT &= ~ RED_LED;
    }
    else{
      P1OUT &= ~ GREEN_LED; 
      P1OUT |= RED_LED; 
    } colourDetect = Schedule(refreshRate);
  }
}