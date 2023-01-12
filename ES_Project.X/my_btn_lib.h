/* 
 * File: my_btn_lib.h
 * Authors: Carlone Matteo, Maragliano Matteo, Musumeci Mattia, Sani Ettore
 * Comments:
 * Revision history: 
 */

#ifndef MY_BUTTON_LIB_H
#define MY_BUTTON_LIB_H
 
// The timer lib is necessary for this
#include "my_timer_lib.h"
 
// DEFINING THE BUTTON INTERRUPT
// 1. Always reset the related interrupt flag
// 2. Disabling the interrupt so that it so not called again (prevent bouncing)
// 3. Starting the timer to read the state of the button after the bouncing
 
 
// DEFINING THE TIMER INTERRUPT
// 1. Always reset the related interrupt flag
// 2. Disable the timer
// 3. Invoke the button handler if the button is still released
// NB. The following two instruction is necessary that are executed in order
// otherwise the interrupt might fire again.
// 4. Resetting the interrupt flag of the related button interrupt
// 5. Enabling the interrupt for the related button interrupt
 
#if defined(TIMER_FOR_BUTTON_S5)
 
    // The function that will be called when the button is pressed
    void (*_buttonS5CallbackPtr)(void);
 
    void init_btn_s5(void(*callback)(void))
    {
        // Storing callback
        _buttonS5CallbackPtr = callback;
 
        // Set the button S5 pin as input
        TRISEbits.TRISE8 = 1;
        // Enables interrupt related to button S5
        IEC0bits.INT0IE = 1;
 
        // Enabling the related timer
        if(TIMER_FOR_BUTTON_S5 == TIMER1)
            IEC0bits.T1IE = 1;
        else if(TIMER_FOR_BUTTON_S5 == TIMER2)
            IEC0bits.T2IE = 1;
        else if(TIMER_FOR_BUTTON_S5 == TIMER3)
            IEC0bits.T3IE = 1;
        else if(TIMER_FOR_BUTTON_S5 == TIMER4)
            IEC1bits.T4IE = 1;
        else if(TIMER_FOR_BUTTON_S5 == TIMER5)
            IEC1bits.T5IE = 1;
    }
 
    // DEFINING THE BUTTON INTERRUPT
    void __attribute__((__interrupt__, __auto_psv__)) _INT0Interrupt() 
    {
        IFS0bits.INT0IF = 0;
        IEC0bits.INT0IE = 0;
 
        tmr_setup_period(TIMER_FOR_BUTTON_S5, 20);
    }
 
    // DEFINING THE TIMER INTERRUPT
    #if TIMER_FOR_BUTTON_S5 == TIMER1
        void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt() 
        {
            IFS0bits.T1IF = 0;
            T1CONbits.TON = 0;
 
            if(PORTEbits.RE8 == 1)
                (*_buttonS5CallbackPtr)();
 
            IFS0bits.INT0IF = 0;
            IEC0bits.INT0IE = 1;
        }
    #elif TIMER_FOR_BUTTON_S5 == TIMER2
        void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt() 
        {
            IFS0bits.T2IF = 0;
            T2CONbits.TON = 0;
 
            if(PORTEbits.RE8 == 1)
                (*_buttonS5CallbackPtr)();
 
            IFS0bits.INT0IF = 0;
            IEC0bits.INT0IE = 1;
        }
    #elif TIMER_FOR_BUTTON_S5 == TIMER3
        void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt() 
        {
            IFS0bits.T3IF = 0;
            T3CONbits.TON = 0;
 
            if(PORTEbits.RE8 == 1)
                (*_buttonS5CallbackPtr)();
 
            IFS0bits.INT0IF = 0;
            IEC0bits.INT0IE = 1;
        }
    #elif TIMER_FOR_BUTTON_S5 == TIMER4
        void __attribute__((__interrupt__, __auto_psv__)) _T4Interrupt() 
        {
            IFS1bits.T4IF = 0;
            T4CONbits.TON = 0;
 
            if(PORTEbits.RE8 == 1)
                (*_buttonS5CallbackPtr)();
 
            IFS0bits.INT0IF = 0;
            IEC0bits.INT0IE = 1;
        }
    #elif TIMER_FOR_BUTTON_S5 == TIMER5
        void __attribute__((__interrupt__, __auto_psv__)) _T5Interrupt() 
        {
            IFS1bits.T5IF = 0;
            T5CONbits.TON = 0;
 
            if(PORTEbits.RE8 == 1)
                (*_buttonS5CallbackPtr)();
 
            IFS0bits.INT0IF = 0;
            IEC0bits.INT0IE = 1;
        }
    #endif
 
#endif //TIMER_FOR_BUTTON_S5
 
#if defined(TIMER_FOR_BUTTON_S6)
 
    void (*_buttonS6CallbackPtr)(void);
 
    void init_btn_s6(void(*callback)(void))
    {
        // Storing callback
        _buttonS6CallbackPtr = callback;
 
        // Set the button S5 pin as input
        TRISDbits.TRISD0 = 1;
        // Enables interrupt related to button S5
        IEC1bits.INT1IE = 1;
 
        // Enabling the related timer
        if(TIMER_FOR_BUTTON_S6 == TIMER1)
            IEC0bits.T1IE = 1;
        else if(TIMER_FOR_BUTTON_S6 == TIMER2)
            IEC0bits.T2IE = 1;
        else if(TIMER_FOR_BUTTON_S6 == TIMER3)
            IEC0bits.T3IE = 1;
        else if(TIMER_FOR_BUTTON_S6 == TIMER4)
            IEC1bits.T4IE = 1;
        else if(TIMER_FOR_BUTTON_S6 == TIMER5)
            IEC1bits.T5IE = 1;
    }
 
    // DEFINING THE BUTTON INTERRUPT
    void __attribute__((__interrupt__, __auto_psv__)) _INT1Interrupt() 
    {
        IFS1bits.INT1IF = 0;
        IEC1bits.INT1IE = 0;
        tmr_setup_period(TIMER_FOR_BUTTON_S6, 20);
    }
 
    // DEFINING THE TIMER INTERRUPT
    #if TIMER_FOR_BUTTON_S6 == TIMER1
        void __attribute__((__interrupt__, __auto_psv__)) _T1Interrupt() 
        {
            IFS0bits.T1IF = 0;
            T1CONbits.TON = 0;
 
            if(PORTDbits.RD0 == 1)
                (*_buttonS6CallbackPtr)();
 
            IFS1bits.INT1IF = 0;
            IEC1bits.INT1IE = 1;
        }
    #elif TIMER_FOR_BUTTON_S6 == TIMER2
        void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt() 
        {
            IFS0bits.T2IF = 0;
            T2CONbits.TON = 0;
 
            if(PORTDbits.RD0 == 1)
                (*_buttonS6CallbackPtr)();
 
            IFS1bits.INT1IF = 0;
            IEC1bits.INT1IE = 1;
        }
    #elif TIMER_FOR_BUTTON_S6 == TIMER3
        void __attribute__((__interrupt__, __auto_psv__)) _T3Interrupt() 
        {
            IFS0bits.T3IF = 0;
            T3CONbits.TON = 0;
 
            if(PORTDbits.RD0 == 1)
                (*_buttonS6CallbackPtr)();
 
            IFS1bits.INT1IF = 0;
            IEC1bits.INT1IE = 1;
        }
    #elif TIMER_FOR_BUTTON_S6 == TIMER4
        void __attribute__((__interrupt__, __auto_psv__)) _T4Interrupt() 
        {
            IFS1bits.T4IF = 0;
            T4CONbits.TON = 0;
 
            if(PORTDbits.RD0 == 1)
                (*_buttonS6CallbackPtr)();
 
            IFS1bits.INT1IF = 0;
            IEC1bits.INT1IE = 1;
        }
    #elif TIMER_FOR_BUTTON_S6 == TIMER5
        void __attribute__((__interrupt__, __auto_psv__)) _T45nterrupt() 
        {
            IFS1bits.T5IF = 0;
            T5CONbits.TON = 0;
 
            if(PORTDbits.RD0 == 1)
                (*_buttonS6CallbackPtr)();
 
            IFS1bits.INT1IF = 0;
            IEC1bits.INT1IE = 1;
 
        }
    #endif
 
#endif //TIMER_FOR_BUTTON_S6
 
#endif //MY_BUTTON_LIB_H