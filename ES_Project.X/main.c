
// DSPIC30F4011 Configuration Bit Settings

// 'C' source line config statements

// FOSC
#pragma config FPR = XT                 // Primary Oscillator Mode (XT)
#pragma config FOS = PRI                // Oscillator Source (Primary Oscillator)
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_64          // POR Timer Value (64ms)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_ON          // PBOR Enable (Enabled)
#pragma config LPOL = PWMxL_ACT_HI      // Low-side PWM Output Polarity (Active High)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN       // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#define TIMER_FOR_BUTTON_S5 3
#define TIMER_FOR_BUTTON_S6 4

#include <xc.h>
#include "parser.h"
#include "my_timer_lib.h"
#include "my_print_lib.h"
#include "my_circular_buffer_lib.h"
#include "my_btn_lib.h"
#include <stdio.h>
#include <string.h>

#define TASK_COUNT 8

void handleUARTReading();
void handleUARTWriting();

typedef struct
{
    int n;  // number of periods elapsed from last task call
    int N;  // after how many periods should the task be called
    void (*task)(void);  // the task
} Heartbeat;

typedef struct{
    float linear;
    float angular;
    float rpm_right;
    float rpm_left;
} mcref_data;

typedef struct{
    float temp;
    int n;
} mcfbk_data;

// The buffer that will contain data received from the UART
volatile circular_buffer in_buffer;
// The buffer that will contain data to send to the UART
volatile circular_buffer out_buffer;

Heartbeat schedInfo[TASK_COUNT];

parser_state pstate;
mcref_data state_sdata = {0};
mcfbk_data out_sdata = {0};

volatile short flagSafeMode = 0;
volatile short flagTimeOutMode = 0;
volatile short flagRpmLimits = 0;
volatile short flagPage = 0;
volatile short state = 0;

char MCALE[21];


void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt()
{
    // Resetting the interrupt flag
    IFS0bits.T2IF = 0;
    
    // Setting the timeout mode
    flagTimeOutMode = 1;
    // Stopping the motors
    state_sdata.rpm_left = 0;
    state_sdata.rpm_right = 0;
    // Writing state on the LCD
    lcd_write(8, "T");
    // set the state to timeout value
    state = 1;
}

// This is triggered when the receiver UART buffer is 3/4 full
void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt()
{
    // Reset the interrupt flag
    IFS1bits.U2RXIF = 0;
    // Handle the reading of the buffer
    handleUARTReading();
}

void handleUARTReading()
{
    // Check if there is something to read from UART
    while(U2STAbits.URXDA == 1)
        // Put the data in the circular buffer
        cb_push_back(&in_buffer, U2RXREG);
}

// This is triggered when the transmitter UART buffer becomes empty
void __attribute__((__interrupt__, __auto_psv__)) _U2TXInterrupt()
{
    // Reset the interrupt flag
    IFS1bits.U2TXIF = 0;
    // Handle the writing on the buffer
    handleUARTWriting();
}

void handleUARTWriting()
{
    char word;
    // Trasmit data if the UART transmission buffer is not full and 
    // there is actually something to transmit in the output buffer
    while (U2STAbits.UTXBF == 0 && out_buffer.count != 0){
        cb_pop_front(&out_buffer, &word);
        U2TXREG = word;
    }
}

void handleUARTOverflow()
{
    // Overflow did not occur, do nothing
    if(U2STAbits.OERR == 0)
        return;
    
    // Waiting for the current LCD transmittion to end and
    // then writing 2 on the LDC for debugging
    while (SPI1STATbits.SPITBF == 1);
    // Handle the UART overflow by storing all the available data
    handleUARTReading();
    // Clear the UART overflow flag
    U2STAbits.OERR = 0;
}

void scheduler()
{
    for(int i=0; i<TASK_COUNT; ++i)
    {
        if(++schedInfo[i].n < schedInfo[i].N)
            continue;
        schedInfo[i].task();
        schedInfo[i].n = 0;
    }
}

void led_working()
{
    // blinking led D3 at 1 Hz
    LATBbits.LATB0 = !LATBbits.LATB0;
}

void led_timeout()
{
    // blinking led D4 at 5 Hz
    if(flagTimeOutMode)
        LATBbits.LATB1 = !LATBbits.LATB1;
    else
        LATBbits.LATB1 = 0;
}

void acquire_temp()
{
    // Starts the sampling
    ADCON1bits.SAMP = 1;
    // Waiting for new sampled data
    while(!ADCON1bits.DONE);
    // Registering another temperature data sempling
    out_sdata.temp += ADCBUF0 * 0.48828125 - 50;
    out_sdata.n++;
}

void send_temp()
{
    // Creating message feedback for temperature
    char msg[13];
    float temp = out_sdata.temp/out_sdata.n;
    // 5 chars for ###.#°C (considering also 100+ temperature)
    char buff[] = "     ";
    char* str_temp = float_to_string(temp, buff, 1);
    strcpy(msg, "$MCTEM,");
    strcat(msg, str_temp);
    strcat(msg,"*");
    // Sending message
    if(cb_push_back_string(&out_buffer, msg) == -1)
        return;
    IEC1bits.U2TXIE = 0;
    handleUARTWriting();
    IEC1bits.U2TXIE = 1;
    // Reinit the data struct
    out_sdata.temp = 0;
    out_sdata.n = 0;
}

void update_rpm()
{
    PDC1 = (state_sdata.rpm_right/60 + 1) * PTPER;
    PDC2 = (state_sdata.rpm_left/60 + 1) * PTPER; 
}

void send_mcale()
{
    if(flagRpmLimits)
    {
        if(cb_push_back_string(&out_buffer, MCALE) == -1)
            return;
        
        IEC1bits.U2TXIE = 0;
        handleUARTWriting();
        IEC1bits.U2TXIE = 1;
    }
}

void send_fbk()
{
    char fbk_buff[] = "$MCFBK,zzzzz,zzzzz,z*";
    /*char rpm_right_buff[] = "      ";
    char rpm_left_buff[] = "      ";
    char state_buff[] = " ";
    char* rpm_right_str = float_to_string(state_sdata.rpm_right, rpm_right_buff, 1);
    char* rpm_left_str = float_to_string(state_sdata.rpm_left, rpm_left_buff, 1);
    char* state_str = float_to_string(state, state_buff, 0);
    strcat(fbk_buff, rpm_right_str);
    strcat(fbk_buff, ",");
    strcat(fbk_buff, rpm_left_str);
    strcat(fbk_buff, ",");
    strcat(fbk_buff, state_str);
    strcat(fbk_buff, "*");*/
    if (cb_push_back_string(&out_buffer, fbk_buff) == -1 )
        return;
    IEC1bits.U2TXIE = 0;
    handleUARTWriting();
    IEC1bits.U2TXIE = 1;
}

short clamp(float* value, float min, float max)
{
    if(*value < min)
    {
        *value = min;
        return 1;
    }
    if(*value > max)
    {
        *value = max;
        return 1;
    }
    return 0;
}

float clamp2(float value, float min, float max)
{
    if(value < min)
        return min;
    if(value > max)
        return max;
    return value;
}

void controller()
{
    // Temporarely disable the UART interrupt to read data
    // This does not cause problems if data arrives now since we are empting the buffer
    IEC1bits.U2RXIE = 0;
    // Handle the reading of the buffer
    handleUARTReading();
    // Enable UART interrupt again
    IEC1bits.U2RXIE = 1;
    // Check if there was an overflow in the UART buffer
    handleUARTOverflow();
        
    // Handling all the data in the input buffer
    char word;
    while (in_buffer.count != 0)
    {
        cb_pop_front(&in_buffer, &word);
        // Check if there is a new message to handle
        if(parse_byte(&pstate, word) != NEW_MESSAGE)
            continue;
            
        // Handling message type
        if (strcmp(pstate.msg_type, "HLREF") == 0)
        {
            // Ignoring HLREF if safe mode is enabled
            if(flagSafeMode)
                return;
            
            // Resetting the timeout timer
            TMR2 = 0;
            flagTimeOutMode = 0;
            // Setting the state as C (Controlled)
            lcd_write(8, "C");
            
            char* payload = pstate.msg_payload;
            // Parsing angular and linear velocities from payload string
            state_sdata.angular = extract_float(&payload);
            state_sdata.linear = extract_float(&payload);
            // Computing rpm for left and right wheel (coverted from rad/s to rpm)
            state_sdata.rpm_right = (5*state_sdata.linear - 1.25*state_sdata.angular) * 9.54929658551;
            state_sdata.rpm_left = (5*state_sdata.linear + 1.25*state_sdata.angular) * 9.54929658551;
            // Clamping velocities
            float prev_rpm_right = clamp2(state_sdata.rpm_right, -999.9, 999.9);
            float prev_rpm_left = clamp2(state_sdata.rpm_left, -999.9, 999.9);
            short limit_right = clamp(&state_sdata.rpm_right, -50, 50);
            short limit_left = clamp(&state_sdata.rpm_left, -50, 50);
            flagRpmLimits = limit_right || limit_left;
            
            if(flagRpmLimits)
            {
                char rpm_right_buff[] = "      ";
                char rpm_left_buff[] = "      ";
                char* rpm_right_str = float_to_string(prev_rpm_right, rpm_right_buff, 1);
                char* rpm_left_str = float_to_string(prev_rpm_left, rpm_left_buff, 1);
                
                strcpy(MCALE, "$MCALE,");
                strcat(MCALE, rpm_right_str);
                strcat(MCALE, ",");
                strcat(MCALE, rpm_left_str);
                strcat(MCALE, "*");
            }
        }
        else if (strcmp(pstate.msg_type, "HLENA") == 0)
        {
            // Exiting safe mode
            flagSafeMode = 0;
            // Writing debug message on lcd
            lcd_write(8, " ");
            // Resetting the timer
            TMR2 = 0;
            // Stopping the timeout timer
            T2CONbits.TON = 1;
            // Sending ACK message to PC
            char out[] = "$MCACK,ENA,1*";
            if (cb_push_back_string(&out_buffer, out)== -1)
                break;
            IEC1bits.U2TXIE = 0;
            handleUARTWriting();
            IEC1bits.U2TXIE = 1;
        }
        else
        {
            // Waiting for the current LCD transmittion to end and
            // then writing 3 on the LDC for debugging
            while (SPI1STATbits.SPITBF == 1);
            SPI1BUF = '4';
        }
    }
    if(!flagPage)
    {
        char rpm_right_buff[] = "      ";
        char rpm_left_buff[] = "      ";
        char lcd_second_row_buff[13];
        char* rpm_right_str = float_to_string(state_sdata.rpm_right, rpm_right_buff, 1);
        char* rpm_left_str = float_to_string(state_sdata.rpm_left, rpm_left_buff, 1);
        strcpy(lcd_second_row_buff, rpm_left_str);
        strcat(lcd_second_row_buff, ";");
        strcat(lcd_second_row_buff, rpm_right_str);
        lcd_clear(19, 31);
        lcd_write(19, lcd_second_row_buff);
    }
    else
    {
        char angular_buff[] = "      ";
        char linear_buff[] = "      ";
        char lcd_second_row_buff2[13];
        char* angular_str = float_to_string(state_sdata.angular, angular_buff, 1);
        char* linear_str = float_to_string(state_sdata.linear, linear_buff, 1);
        strcpy(lcd_second_row_buff2, angular_str);
        strcat(lcd_second_row_buff2, ";");
        strcat(lcd_second_row_buff2, linear_str);
        lcd_clear(19, 31);
        lcd_write(19, lcd_second_row_buff2);
    }
}

void on_button_s5_released()
{
    // Entering safe mode
    flagSafeMode = 1;
    // Stopping the motors
    state_sdata.rpm_left = 0;
    state_sdata.rpm_right = 0;
    // Writing debug message on lcd
    lcd_write(8, "H");
    // Stopping the timeout timer
    T2CONbits.TON = 0;
    // Resetting an eventual timeout mode
    flagTimeOutMode = 0;
    // set the state to safemode value
    state = 2;
}

void on_button_s6_released()
{
    flagPage = !flagPage;
}

int main(void) {
    // Parser initialization
	pstate.state = STATE_DOLLAR;
	pstate.index_type = 0; 
	pstate.index_payload = 0;
    // Set pins of leds D3 and D4 as output
    TRISBbits.TRISB0 = 0;
    TRISBbits.TRISB1 = 0;
    // Init schedInfo struct, the heartbeat is set to 5ms
    schedInfo[0] = (Heartbeat){0, 10, led_working};
    schedInfo[1] = (Heartbeat){0, 1, acquire_temp};
    schedInfo[2] = (Heartbeat){0, 10, send_temp};
    schedInfo[3] = (Heartbeat){0, 1, controller};
    schedInfo[4] = (Heartbeat){0, 2, led_timeout};
    schedInfo[5] = (Heartbeat){0, 1, update_rpm};
    schedInfo[6] = (Heartbeat){0, 10, send_mcale};
    schedInfo[7] = (Heartbeat){0, 2, send_fbk};
    // Init circular buffers to read and write on the UART
    char in[50];
    char out[200];
    cb_init(&in_buffer, in, 50);     // init the circular buffer to read from UART (4,8 max character every 5 ms, 8 for safety)
    cb_init(&out_buffer, out, 200);  // init the circular buffer to write on UART (20 max character, 24 for safety)
    init_uart();                 // init the UART
    init_spi();                  // init the SPI (for debug pourposes, it prints error messages)
    tmr_wait_ms(TIMER1, 1500);   // wait 1.5s to start the SPI correctly
    tmr_setup_period(TIMER1, 100); // initialize heatbeat timer 10Hz = 100ms
    // Init PWM to set the voltage to the armature of the DC motor
    PTCONbits.PTMOD = 0;    // free running mode
    PTCONbits.PTCKPS = 0b0; // prescaler
    PWMCON1bits.PEN1H = 1;  // output high bit for PWM1
    PWMCON1bits.PEN1L = 1;  // output low bit for PWM1
    PWMCON1bits.PEN2H = 1;  // output high bit for PWM2
    PWMCON1bits.PEN2L = 1;  // output low bit for PWM2
    PTPER = 1842;           // time period
    PDC1 = 0;               // duty cycle, at the beginning the motor is still
    PDC2 = 0;               // duty cycle, at the beginning the motor is still
    PTCONbits.PTEN = 1;     // enable the PWM
    // Init the ADC converter in manual sampling and automatic conversion
    ADCON3bits.ADCS = 8;      // Tad = 4.5 Tcy
    ADCON1bits.ASAM = 0;      // start
    ADCON1bits.SSRC = 7;      // end
    ADCON3bits.SAMC = 16;     // auto sampling time
    ADCON2bits.CHPS = 0;    // selecting the channel to convert
    ADCHSbits.CH0SA = 0b0011; // choosing the positive input of the channels
    ADPCFG = 0xfff7;          // select the AN3 pins as analogue for reading
    ADCON1bits.ADON = 1;      // turn the ADC on
    // Initializing buttons
    init_btn_s5(&on_button_s5_released);
    init_btn_s6(&on_button_s6_released);
    
    // Intializing the necessary chars on the LCD
    lcd_write( 0, "STATUS:         ");
    lcd_write(16, "R:              ");
    // Enabling the interrupt related to TIMER2
    IEC0bits.T2IE = 1;
    // Setting the timer for the timeout check
    tmr_setup_period(TIMER2, 5000);
    
    // main loop
    while (1) {
        scheduler();
        tmr_wait_period(TIMER1);
    }

    return 0;
}