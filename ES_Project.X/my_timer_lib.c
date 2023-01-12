/*
 * File:   my_timer_lib.c
 * Authors: Carlone Matteo, Maragliano Matteo, Musumeci Mattia, Sani Ettore
 *
 * Created on 4 ottobre 2022, 11.55
 */


#include "xc.h"
#include "my_timer_lib.h"
#include <p30F4011.h>

#define FCY_1000 1843.2

void trm1_setup(int pr_value, short prescaler)
{
    // setup timer 1
    
    IFS0bits.T1IF = 0;           // put the timer flag down
    TMR1 = 0;                    // reset timer counter
    PR1 = pr_value;              // set the number of clock step of the counter
    T1CONbits.TCKPS = prescaler; // set the prescaler
    T1CONbits.TON = 1;           // start the timer
}

void trm2_setup(int pr_value, short prescaler)
{
    // setup timer 2
    
    IFS0bits.T2IF = 0;           // put the timer flag down
    TMR2 = 0;                    // reset timer counter
    PR2 = pr_value;              // set the number of clock step of the counter
    T2CONbits.TCKPS = prescaler; // set the prescaler
    T2CONbits.TON = 1;           // start the timer
}

void trm3_setup(int pr_value, short prescaler)
{
    // setup timer 3
    
    IFS0bits.T3IF = 0;           // put the timer flag down
    TMR3 = 0;                    // reset timer counter
    PR3 = pr_value;              // set the number of clock step of the counter
    T3CONbits.TCKPS = prescaler; // set the prescaler
    T3CONbits.TON = 1;           // start the timer
}

void trm4_setup(int pr_value, short prescaler)
{
    // setup timer 4
    
    IFS1bits.T4IF = 0;           // put the timer flag down
    TMR4 = 0;                    // reset timer counter
    PR4 = pr_value;              // set the number of clock step of the counter
    T4CONbits.TCKPS = prescaler; // set the prescaler
    T4CONbits.TON = 1;           // start the timer
}

void trm5_setup(int pr_value, short prescaler)
{
    // setup timer 5
    
    IFS1bits.T5IF = 0;           // put the timer flag down
    TMR5 = 0;                    // reset timer counter
    PR5 = pr_value;              // set the number of clock step of the counter
    T5CONbits.TCKPS = prescaler; // set the prescaler
    T5CONbits.TON = 1;           // start the timer
}

void tmr_setup_period(int timer, int ms)
{
    // setup the timer with a provided period
    
    // depending on the number of ms that the timer must count, set the correct
    // prescaler and the relative divisor

    // default value (prescaler value 256)
    short prescaler = 3;
    int divisor = 256;
    
    if (ms <= 35)
    {
        prescaler = 0; // prescaler value 1
        divisor = 1;
    }
    else if (ms <= 284 )
    {
        prescaler = 1; // prescaler value 8
        divisor = 8;
    }
    else if (ms <= 2275) 
    {
        prescaler = 2; // prescaler value 64
        divisor = 64;
    }
    
    // compute the number of clock step of the counter
    int pr_value = (int)(FCY_1000 * ((float)ms / divisor));
    
    // depending on the selected timer, call the relative setup function
    switch(timer)
    {
        case 1:
            trm1_setup(pr_value, prescaler);
            break;
            
        case 2:
            trm2_setup(pr_value, prescaler);
            break;

        case 3:
            trm3_setup(pr_value, prescaler);
            break;

        case 4:
            trm4_setup(pr_value, prescaler);
            break;
            
        case 5:
            trm5_setup(pr_value, prescaler);
            break;
        
        default:
           
           break;
    } 
}


void tmr_wait_period(int timer)
{
    // perform a busy waiting on the timer flag until it is raised
    
    // wait on the selected timer, then put the flag down and return
    switch (timer)
    {
        case 1 :
            while (!IFS0bits.T1IF);
            IFS0bits.T1IF = 0;
            break;

        case 2:
            while (!IFS0bits.T2IF);
            IFS0bits.T2IF = 0;
            break;

        case 3:
            while (!IFS0bits.T3IF);
            IFS0bits.T3IF = 0;
            break;

        case 4:
            while (!IFS1bits.T4IF);
            IFS1bits.T4IF = 0;
            break;

        case 5:
            while (!IFS1bits.T5IF);
            IFS1bits.T5IF = 0;
            break;
            
        default:
            break;
    }
}

void tmr_wait_ms(int timer, int ms)
{
    // wait the number of ms on the specified timer
    
    tmr_setup_period(timer, ms); // setup the timer
    tmr_wait_period(timer);      // wait on the timer
}
