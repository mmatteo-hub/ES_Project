
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

#define TASK_COUNT 8

#include <xc.h>
#include "parser.h"
#include "my_utils.h"
#include "my_timer_lib.h"
#include "my_lcd_lib.h"
#include "my_uart_lib.h"
#include "my_circular_buffer_lib.h"
#include "my_btn_lib.h"
#include "my_scheduling_lib.h"
#include <stdio.h>
#include <string.h>

// All the data related to the motors
typedef struct{
    float angular;
    float linear;
    float rpm_right;
    float rpm_left;
    short is_clamped;
} motors_data;

// The data for the MCTEM message
typedef struct{
    float temp;
    int n;
} mctem_data;

// The buffer that will contain data received from the UART
volatile circular_buffer in_buffer;
// The buffer that will contain data to send to the UART
volatile circular_buffer out_buffer;

// The variables necessary for handling multiple states
typedef enum {CONTROLLED, TIMEOUT, SAFE} WORKING_MODE;
volatile WORKING_MODE current_mode = 0;

// The message parser state
parser_state pstate;
// The data related to the motors
motors_data motors = {0};
// The data for MCTEM has to be stored because it is a mean
mctem_data MCTEM_data = {0};

// The second lines on the lcd
char debug_line_0[13] = "0.0; 0.0";
char debug_line_1[13] = "0.0; 0.0";
short current_lcd_page = -1;
short should_update_lcd = 1;

// The variables necessary for messages 
char MCALE[21] = "$MCALE,";
char MCTEM[14] = "$MCTEM,";
char MCFBK[21] = "$MCFBK,0.0,0.0,0*";
char MCACK[13] = "$MCACK,ENA,1*";


char* create_mcale(float rpm_right_req, float rpm_left_req);
char* create_mctem();
char* create_mcfbk();
void store_rpm(float angular, float linear);
void recreate_lcd_lines();
void enter_working_mode(int mode);

char* create_mcale(float rpm_right_req, float rpm_left_req)
{
    // The message is not created if the values are not clamped
    if(!motors.is_clamped)
        return MCALE;

    char value_buff_right[] = "      ";
    char value_buff_left[] = "      ";
    char* value_str_right;
    char* value_str_left;
    
    // The values are clamped to the maximum range
    rpm_right_req = clamp(rpm_right_req, -999.9, 999.9);
    rpm_left_req = clamp(rpm_left_req, -999.9, 999.9);
    
    value_str_left = float_to_string(rpm_left_req, value_buff_left, 1);
    value_str_right = float_to_string(rpm_right_req, value_buff_right, 1);
    strscat(5, &MCALE[7], value_str_left, ",", value_str_right, "*");

    return MCALE;
}

char* create_mctem()
{
    // Computing the temperature and clamping just to be sure
    float temp = clamp(MCTEM_data.temp/MCTEM_data.n, -999.9, 999.9);

    // 6 chars for -###.# temperature
    char temp_buff[] = "     ";
    char* temp_str;
    
    temp_str = float_to_string(temp, temp_buff, 1);
    strscat(3, &MCTEM[7], temp_str, "*");
    
    // Reinit the data struct since the message has been created
    MCTEM_data.temp = 0;
    MCTEM_data.n = 0;

    return MCTEM;
}

char* create_mcfbk()
{
    char value_buff_right[] = "      ";
    char value_buff_left[] = "      ";
    char* value_str_right;
    char* value_str_left;
    char mode_str[] = " \0";

    value_str_right = float_to_string(motors.rpm_right, value_buff_right, 1);
    value_str_left = float_to_string(motors.rpm_left, value_buff_left, 1);
    mode_str[0] = current_mode+'0';
    strscat(7, &MCFBK[7], value_str_left, ",", value_str_right, ",", mode_str, "*");
    
    return MCFBK;
}

void store_rpm(float angular, float linear)
{
    // The values are clamped into a reasonable range
    motors.angular = angular;
    motors.linear = linear;
    // Computing the rpms for the right and left motors
    float rpm_right_req = (5*linear - 1.25*angular) * 9.54929658551;
    float rpm_left_req = (5*linear + 1.25*angular) * 9.54929658551;
    // Storing the rmps
    motors.rpm_right = rpm_right_req;
    motors.rpm_left = rpm_left_req;
    // Performing a post-clmaping in the motors safe range
    short is_right_clamped = clamp_inplace(&motors.rpm_right, -50, 50);
    short is_left_clamped = clamp_inplace(&motors.rpm_left, -50, 50);
    motors.is_clamped = is_right_clamped || is_left_clamped;
    
    // Recreating the lcd lines now that we have new data
    recreate_lcd_lines();
    // The mcale message needs to be creates only when the rmp are updated
    create_mcfbk();
    // The mcale message needs to be creates only when the rmp are updated
    create_mcale(rpm_right_req, rpm_left_req);
}

void recreate_lcd_lines()
{
    char buff_value_left[]  = "     ";
    char buff_value_right[] = "     ";
    char* value_str_left;
    char* value_str_right;
    // Creating the first line of the LCD
    value_str_right = float_to_string(motors.rpm_right, buff_value_right, 1);
    value_str_left  = float_to_string(motors.rpm_left, buff_value_left, 1);
    strscat(4, debug_line_0, value_str_left, "; ", value_str_right); 
    // Creating the second line of the LCD
    value_str_left  = float_to_string(clamp(motors.angular, -99.9, 99.9), buff_value_left, 1);
    value_str_right = float_to_string(clamp(motors.linear, -99.9, 99.9), buff_value_right, 1);
    strscat(4, debug_line_1, value_str_left, "; ", value_str_right);
    // Marking the lcd for update
    should_update_lcd = 1;
}

void enter_working_mode(int mode)
{
    // Storing the current mode
    current_mode = mode;
    
    // Handling the initialization of every mode
    if(current_mode == CONTROLLED)
    {
        // Enabling the TIMEOUT timer
        T2CONbits.TON = 1;
        // The TIMEOUT timer is reset for when the SAFE is exited
        TMR2 = 0;
        // Writing the debug message on the LCD
        lcd_write(8, "C");
    }
    else if(current_mode == TIMEOUT)
    {
        // Resetting the motors
        store_rpm(0.0, 0.0);
        // Writing the debug message on the LCD
        lcd_write(8, "T");
    }
    else if(current_mode == SAFE)
    {
        // Stopping the TIMEOUT timer
        T2CONbits.TON = 0;
        // The TIMEOUT timer is reset for when the SAFE is exited
        TMR2 = 0;
        // Resetting the motors
        store_rpm(0.0, 0.0);
        // Writing the debug message on the LCD
        lcd_write(8, "H");
    }
}

// The Timer2 interrupt fires when no message is received for 5 seconds.
// In that case, the TIMEOUT mode logic must be executed.
void __attribute__((__interrupt__, __auto_psv__)) _T2Interrupt()
{
    // Resetting the interrupt flag
    IFS0bits.T2IF = 0;
    
    // Switching to timeout mode
    enter_working_mode(TIMEOUT);
}

void led_working()
{
    // blinking led D3 at 1 Hz
    LATBbits.LATB0 = !LATBbits.LATB0;
}

void led_timeout()
{
    // If we are in TIMEOUT mode, we need to blink the led D4 at 5Hz
    if(current_mode == TIMEOUT)
    {
        LATBbits.LATB1 = !LATBbits.LATB1;
        return;
    }

    // Otherwise, we turn off the led D4
    LATBbits.LATB1 = 0;
}

void acquire_temp()
{
    // Starts the sampling
    ADCON1bits.SAMP = 1;
    // Waiting for new sampled data
    while(!ADCON1bits.DONE);
    // Registering another temperature data sempling
    MCTEM_data.temp += ADCBUF0 * 0.48828125 - 50;
    MCTEM_data.n++;
}

void controller()
{
    // The UART logic that needs to be executed in the main loop
    uart_main_loop();
        
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
            if(current_mode == SAFE)
                return;
            
            // Handling the new values
            char* payload = pstate.msg_payload;
            float angular = extract_float(&payload);
            float linear = extract_float(&payload);
            // We are not in CONTROLLED mode because we are reading a new message
            enter_working_mode(CONTROLLED);
            store_rpm(angular, linear);
        }
        else if (strcmp(pstate.msg_type, "HLENA") == 0)
        {
            // Cannot exit safe mode if we are not in safe mode
            if(current_mode != SAFE)
                return;
            
            // We are exiting SAFE mode and entering normal execution
            enter_working_mode(CONTROLLED);
            // Sending ACK message to PC
            uart_send(MCACK);
        }
    }

    if(should_update_lcd)
    {
        lcd_clear(27, 4);
        if(current_lcd_page == 0)
            lcd_write(19, debug_line_0);
        else
            lcd_write(19, debug_line_1);
        should_update_lcd = 0;
    }
}

void update_pwm()
{
    PDC1 = (motors.rpm_right/60 + 1) * PTPER;
    PDC2 = (motors.rpm_left/60 + 1) * PTPER; 
}

void send_mctem()
{
    uart_send(create_mctem());
}

void send_mcale()
{
    if(!motors.is_clamped)
        return;
    
    uart_send(MCALE);
}

void send_mcfbk()
{
    uart_send(MCFBK);
}

void on_button_s5_released()
{
    // Whent the button s5 is released, the device must enter
    // the safe mode.
    enter_working_mode(SAFE);
}

void on_button_s6_released()
{
    // Setting the LCD for update at the next iteration
    // of the controller cycle.
    current_lcd_page = (current_lcd_page+1)%2;
    should_update_lcd = 1;
}

int main(void) 
{
    // Scheduling initialization
    scheduling_init(TIMER1, 100);                    // The scheduling is executed every 100 ms
    // The scheduling of the tasks is offsetted in the period so that one
    // heartbeat is not overloaded on executing all the tasks.
    scheduling_add_task(led_working, 1000, 0);
    scheduling_add_task(send_mctem,  1000, -3);
    scheduling_add_task(send_mcale,  1000, -6);
    scheduling_add_task(send_mcfbk,   200, 0);
    scheduling_add_task(led_timeout,  200, -1);
    scheduling_add_task(acquire_temp, 100, 0);
    scheduling_add_task(update_pwm,   100, 0);
    scheduling_add_task(controller,   100, 0);
    // SPI Initialization
    spi_init();
    tmr_wait_ms(TIMER1, 1500);                      // wait 1.5s to start the SPI correctly
    lcd_write( 0, "STATUS:         ");
    lcd_write(16, "R:              ");
    // UART Initialization
    char in[50];    // we read at most 48 chars
    char out[70];   // we write at most 69 chars
    cb_init(&in_buffer, in, 50);                    // Init UART input buffer ()
    cb_init(&out_buffer, out, 70);                 // Init UART output buffer ()
    uart_init(4800, &in_buffer, &out_buffer);             // Init UART with buffers
    // Buttons initialization
    init_btn_s5(&on_button_s5_released);
    init_btn_s6(&on_button_s6_released);
    // Parser initialization
	pstate.state = STATE_DOLLAR;
	pstate.index_type = 0; 
	pstate.index_payload = 0;

    // Set pins of leds D3 and D4 as output
    TRISBbits.TRISB0 = 0;
    TRISBbits.TRISB1 = 0;
    // Init PWM to set the voltage to the armature of the DC motor
    PTCONbits.PTMOD = 0;        // free running mode
    PTCONbits.PTCKPS = 0b0;     // prescaler
    PWMCON1bits.PEN1H = 1;      // output high bit for PWM1
    PWMCON1bits.PEN1L = 1;      // output low bit for PWM1
    PWMCON1bits.PEN2H = 1;      // output high bit for PWM2
    PWMCON1bits.PEN2L = 1;      // output low bit for PWM2
    PTPER = 1842;               // time period
    PDC1 = 0;                   // duty cycle, at the beginning the motor is still
    PDC2 = 0;                   // duty cycle, at the beginning the motor is still
    DTCON1bits.DTAPS = 0b00;    // dead time prescaler
    DTCON1bits.DTA = 6;         // dead time
    PTCONbits.PTEN = 1;         // enable the PWM
    // Init the ADC converter in manual sampling and automatic conversion
    ADCON3bits.ADCS = 8;        // Tad = 4.5 Tcy
    ADCON1bits.ASAM = 0;        // start
    ADCON1bits.SSRC = 7;        // end
    ADCON3bits.SAMC = 16;       // auto sampling time
    ADCON2bits.CHPS = 0;        // selecting the channel to convert
    ADCHSbits.CH0SA = 0b0011;   // choosing the positive input of the channels
    ADPCFG = 0xfff7;            // select the AN3 pins as analogue for reading
    ADCON1bits.ADON = 1;        // turn the ADC on
    
    // Device initializtion
    // Enabling the interrupt related to TIMER2
    IEC0bits.T2IE = 1;
    // Setting the timer for the timeout check
    tmr_setup_period(TIMER2, 5000);
    // Initializng the controlled execution mode
    enter_working_mode(CONTROLLED);

    // Executing the scheduling in a loop
    scheduling_loop();

    return 0;
}