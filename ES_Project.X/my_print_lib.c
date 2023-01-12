/*
 * File:   my_print_lib.c
 * Authors: Carlone Matteo, Maragliano Matteo, Musumeci Mattia, Sani Ettore
 *
 * Created on 3 novembre 2022, 11.15
 */

#include "xc.h"
#include "my_print_lib.h"
#include <p30F4011.h>
#include <string.h>

void init_spi()
{
    // Function to init the SPI
    SPI1CONbits.MSTEN = 1;  // master mode 
    SPI1CONbits.MODE16 = 0; // 8 bit mode 
    SPI1CONbits.PPRE = 3;   // primary prescaler 
    SPI1CONbits.SPRE = 6;   // secondary prescaler 
    SPI1STATbits.SPIEN = 1; // enable SPI
}

void init_uart()
{
    // Function to init the UART, it will trigger the interrupt when 3/4 of the
    // UART buffer is full
    U2BRG = 11;               // (7372800 / 4) / (16 * 9600)
    U2MODEbits.UARTEN = 1;    // enable UART 
    U2STAbits.UTXEN = 1;      // enable U1TX (must be after UARTEN)
    U2STAbits.URXISEL = 0b10; // set the receiver interrupt when buffer is 3/4 full
    U2STAbits.UTXISEL = 1;    // set the transmitter interrupt when buffer is empty
    IEC1bits.U2RXIE = 1;      // enable UART receiver interrupt
    IEC1bits.U2TXIE = 1;      // enable UART transmitter interrupt
}

void lcd_clear(short start, short amount)
{
    // Filling an array with spaces
    char chars[amount];
    for(int i=0; i<amount; ++i)
        chars[i] = ' ';
    // Writing the spaces on the LCD
    lcd_write(start, chars);
}

void lcd_write(short start, char chars[])
{
    // Moving the cursor to the correct position
    lcd_move_cursor(start);
 
    // Actually writing on the LCD
    for(int i = 0; chars[i] != '\0'; i++)
    {
        while (SPI1STATbits.SPITBF == 1);
        SPI1BUF = chars[i];
    }
}

void lcd_move_cursor(short position)
{
    // NB. The arg "position" goes from 0 to 31, after 15 we need to write on the second line
    // The value 0x80 moves the cursor at the beginning of the first line
    // The value 0xC0 moves the cursor at the beginning of the second line
 
    while (SPI1STATbits.SPITBF == 1);
    SPI1BUF = ((position<16) ? 0x80 : 0xC0) + (position%16);
}

void uart_write(char chars[])
{
    for(int i = 0; chars[i] != '\0'; i++)
        U2TXREG = chars[i];
}

void charcounter_to_str(volatile unsigned int counter, volatile short int is_overflow, char str[])
{
    // If the number is over the max printable number, printing error
    if(is_overflow)
    {
        str[0] = 'O';
        str[1] = 'V';
        str[2] = 'R';
        str[3] = 'F';
        str[4] = 'L';
    }
    else
    {
        int i = 4;
        do
        {
            // Getting each digit from the number
            str[i--] = (char)((counter%10)+48);
            counter /= 10;
        } while(counter > 0);
        
        for(int j=0; j<=i; ++j)
            str[j] = ' ';
    }
}

char* float_to_string(float x, char *p, short decimals)
{
    // Going to the end of the buffer
    char *s = p + strlen(p);
    // tens = 10^decimals
    int tens = 1;
    for(int i=0; i<decimals; ++i)
        tens *= 10;

    // The sign of the number
    short sign = x<0 ? -1 : 1;
    // Storing the decimals as integers
    unsigned short decms = (int)(sign*x*tens) % tens;
    // Storing the integer part without decimals
    int units = (int)(sign*x);

    // Converting the decimals
    for(int i=0; i < decimals; i++)
    {
        *--s = (decms % 10) + '0';
        decms /= 10;
    }
    // Adding point after decimals
    *--s = '.';
    // Converting int
    while (units > 0) 
    {
        *--s = (units % 10) + '0';
        units /= 10;
    }
    // Adding sign
    if (x < 0) 
        *--s = '-';

    return s;
}